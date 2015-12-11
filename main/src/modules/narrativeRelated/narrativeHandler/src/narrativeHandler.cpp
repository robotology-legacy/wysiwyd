/*
* Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Grégoire Pointeau
* email:   gregoire.pointeau@inserm.fr
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* wysiwyd/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include "narrativeHandler.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

bool narrativeHandler::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("narrativeHandler")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << ": finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "narrativeHandler", "narrativeHandler.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    if (!iCub->connect())
    {
        yInfo() << "iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    dThresholdDiffStory = rf.check("dThresholdDiffStory", Value(15.)).asDouble();
    iThresholdSizeStory = rf.check("iThresholdSizeStory", Value(6)).asInt();

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    abm = true;
    if (!iCub->getABMClient())
    {
        abm = false;
        yWarning() << " WARNING ABM NOT CONNECTED, MODULE CANNOT START";
    }

    yInfo() << " dThresholdDiffStory: " << dThresholdDiffStory;
    yInfo() << " iThresholdSizeStory: " << iThresholdSizeStory;


    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";


    findStories();
    cout << endl;
    initializeStories();

    //for (auto it = listStories.begin(); it != listStories.end(); it++){
    //    tellingStory(*it);
    //}

    return abm;
}


bool narrativeHandler::interruptModule() {
    rpcPort.interrupt();

    yInfo() << "--Interrupting the synchronized yarp ports module...";
    return true;
}

bool narrativeHandler::close() {
    iCub->close();
    delete iCub;

    rpcPort.interrupt();
    rpcPort.close();
    return true;
}


bool narrativeHandler::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    }
    else{
        reply.addString(helpMessage);
    }

    rpcPort.reply(reply);

    return true;
}

/* Called periodically every getPeriod() seconds */
bool narrativeHandler::updateModule() {

    return true;
}

void narrativeHandler::findStories(int iInstance)
{
    story currentStory;

    //    int iCurrentInstance = iInstance;

    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE instance > " << iInstance << " ORDER by instance";
    Bottle  bAllInstances = iCub->getABMClient()->requestFromString(osRequest.str());
    Bottle bMessenger;
    int numberSentence = bAllInstances.size();

    myTimeStruct timeBegin;
    myTimeStruct timeEnd;

    vector<int> vError;
    yInfo() << "\t" << "found " << numberSentence << " sentence(s)";
    double mDiff;
    for (int j = 1; j < (numberSentence); j++)
    {

        int Id = atoi(bAllInstances.get(j - 1).asList()->get(0).toString().c_str());
        int Id2 = atoi(bAllInstances.get(j).asList()->get(0).toString().c_str());

        osRequest.str("");
        osRequest << "SELECT time, begin FROM main WHERE instance = " << Id;
        bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        string sT1 = bMessenger.get(0).asList()->get(0).toString();

        osRequest.str("");
        osRequest << "SELECT time, begin FROM main WHERE instance = " << Id2;
        bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        string sT2 = bMessenger.get(0).asList()->get(0).toString();

        myTimeStruct m1 = string2Time(sT1),
            m2 = string2Time(sT2);

        mDiff = timeDiff(m1, m2);

        if (bMessenger.get(0).asList()->get(1).toString() != "f")
        {
            if (mDiff > dThresholdDiffStory){
                if (currentStory.viInstances.size() > iThresholdSizeStory)
                {
                    listStories.push_back(currentStory);
                }
                currentStory.viInstances.clear();
                currentStory.viInstances.push_back(Id2);
            }
            else{
                currentStory.viInstances.push_back(Id2);
            }
        }
        else{
            currentStory.viInstances.push_back(Id2);
        }
    }

    if (currentStory.viInstances.size() > iThresholdSizeStory)
    {
        listStories.push_back(currentStory);
    }

    int ii = 1;

    cout << listStories.size() << " stories found" << endl;
    for (std::vector<story>::iterator itSt = listStories.begin(); itSt != listStories.end(); itSt++)
    {
        cout << "Story " << ii << ": ";
        for (std::vector<int>::iterator itIn = itSt->viInstances.begin(); itIn != itSt->viInstances.end(); itIn++)
        {
            cout << *itIn << " ";
        }
        cout << endl;
        osRequest.str("");
        osRequest << "SELECT subject, verb, object FROM relation WHERE instance = " << *itSt->viInstances.begin() << " AND verb != 'isAtLoc'";
        bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        if (bMessenger.toString() != "NULL")   cout << "before: " << bMessenger.toString() << endl;

        osRequest.str("");
        osRequest << "SELECT subject, verb, object FROM relation WHERE instance = " << itSt->viInstances[itSt->viInstances.size() - 1] << " AND verb != 'isAtLoc'";
        bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        if (bMessenger.toString() != "NULL")   cout << "after : " << bMessenger.toString() << endl;

        ii++;
    }
}

// return the diff between two actions in seconds
double narrativeHandler::timeDiff(myTimeStruct tm1, myTimeStruct tm2, bool bPrint)
{
    //  struct tm diffTime;
    long int iYears,
        iMonth,
        iDays,
        iHours,     // number of hours of differences
        iMinutes,
        iSecond,
        iMilliSec;   // number of minutes of differences

    iYears = tm2.m_tm.tm_year - tm1.m_tm.tm_year;
    if (bPrint) cout << " iYears: " << iYears;
    iMonth = iYears * 12 + (tm2.m_tm.tm_mon - tm1.m_tm.tm_mon);
    if (bPrint) cout << " iMonth: " << iMonth;
    iDays = iMonth * 30 + (tm2.m_tm.tm_mday - tm1.m_tm.tm_mday);
    if (bPrint) cout << " iDays: " << iDays;
    iHours = iDays * 24 + (tm2.m_tm.tm_hour - tm1.m_tm.tm_hour);
    if (bPrint) cout << " iHours: " << iHours;
    iMinutes = iHours * 60 + (tm2.m_tm.tm_min - tm1.m_tm.tm_min);
    if (bPrint) cout << " iMinutes: " << iMinutes;
    iSecond = iMinutes * 60 + (tm2.m_tm.tm_sec - tm1.m_tm.tm_sec);
    if (iSecond > dThresholdDiffStory) return 10000.;
    if (bPrint) cout << " iSecond: " << iSecond;
    iMilliSec = iSecond * 1000 + (tm2.iMilliSec - tm1.iMilliSec);
    if (bPrint) cout << " iMilliSec: " << iMilliSec << endl;

    return (0.001*iMilliSec);
}

myTimeStruct  narrativeHandler::string2Time(string sTime)
{
    char *cBuffer;
    cBuffer = (char*)sTime.c_str();
    unsigned int i = 0;
    int iLevel = 0;
    //  int iHH,iMM,iSS; //iYear,iMonth,iDay,
    string sYear, sMonth, sDay, sHH, sMM, sMS, sSS = "";
    //  bool bYear,bMonth,bDay,bHH,bMM = false;
    while (cBuffer[i] != '\0')
    {
        char cTemp = cBuffer[i];
        if (cTemp == ' ' || cTemp == '-' || cTemp == ':' || cTemp == '+' || cTemp == '.')
        {
            iLevel++;
        }
        else if (cTemp != '"')
        {
            switch (iLevel)
            {
            case 0:
                sYear += cTemp;
                break;
            case 1:
                sMonth += cTemp;
                break;
            case 2:
                sDay += cTemp;
                break;
            case 3:
                sHH += cTemp;
                break;
            case 4:
                sMM += cTemp;
                break;
            case 5:
                sSS += cTemp;
                break;
            case 6:
                sMS += cTemp;
                break;
            }
        }
        i++;
    }
    struct tm tOutput;
    time_t myTime;
    time(&myTime);

    tOutput = *localtime(&myTime);
    tOutput.tm_hour = atoi(sHH.c_str());
    tOutput.tm_min = atoi(sMM.c_str());
    tOutput.tm_sec = atoi(sSS.c_str());
    tOutput.tm_year = atoi(sYear.c_str());
    tOutput.tm_mon = atoi(sMonth.c_str());
    tOutput.tm_mday = atoi(sDay.c_str());
    mktime(&tOutput);

    myTimeStruct mtsOut;
    mtsOut.m_tm = tOutput;
    (sMS.size() == 2) ? mtsOut.iMilliSec = atoi(sMS.c_str()) * 10 : mtsOut.iMilliSec = atoi(sMS.c_str());

    return mtsOut;
}


void narrativeHandler::tellingStory(story st){

    ostringstream osRequest;
    Bottle bMessenger,
        bRelations,
        bActivity,
        bArguments;
    int ii = 0;

    string previous_activitytype = "none";
    string current_activitytype = "none";
    string previous_activityname = "none";
    string current_activityname = "none";
    bool previous_begin = false;
    bool current_begin = false;

    string P = "none",
        A = "none",
        O = "none",
        R = "none",
        reason = "none",
        status = "success",
        adv1 = "none",
        adv2 = "none";

    Bottle bPreviousRelation;

    cout << "--------------------------------------------------------------" << endl;

    for (std::vector<int>::iterator it = st.viInstances.begin(); it != st.viInstances.end(); it++){

        cout << ii << " " <<  st.timeBegin.toString() << " to " << st.timeEnd.toString() << endl;
        osRequest.str("");
        osRequest << "SELECT subject, verb, object FROM relation WHERE instance = " << *it << " AND verb != 'isAtLoc'";
        bRelations = iCub->getABMClient()->requestFromString(osRequest.str());
        //cout << "Relations: " << bMessenger.toString() << endl;

        osRequest.str("");
        osRequest << "SELECT activityname, activitytype, begin FROM main WHERE instance = " << *it;
        bActivity = iCub->getABMClient()->requestFromString(osRequest.str());
        //        cout << "activity info: " << bActivity.toString() << endl;

        osRequest.str("");
        osRequest << "SELECT argument, role FROM contentarg WHERE instance = " << *it;
        bArguments = iCub->getABMClient()->requestFromString(osRequest.str());
        //      cout << "Arguments: ";

        // initial situation
        if (ii == 0){
            cout << "\t\t\t" << "At the beggining, ";
            for (int jj = 0; jj < bRelations.size(); jj++){
                if (jj != 0){
                    cout << " and ";
                }
                cout << bRelations.get(jj).toString();
            }
            cout << endl;
        }
        // end initial situation

        previous_activitytype = current_activitytype;
        current_activitytype = (bActivity.get(0).asList())->get(1).toString();

        previous_activityname = current_activityname;
        current_activityname = (bActivity.get(0).asList())->get(0).toString();

        previous_begin = current_begin;
        current_begin = (bActivity.get(0).asList())->get(2).toString() == "t";

        P = "none";
        A = "none";
        O = "none";
        R = "none";
        status = "success";
        adv1 = "none";
        adv2 = "none";

        for (int kk = 0; kk < bArguments.size(); kk++){
            Bottle bTemp = *bArguments.get(kk).asList();

            if (bTemp.get(1).toString() == "predicate"
                || bTemp.get(1).toString() == "action"
                || bTemp.get(1).toString() == "action2"
                || bTemp.get(1).toString() == "action3"
                || bTemp.get(1).toString() == "verb1"
                || bTemp.get(1).toString() == "verb2"
                || bTemp.get(1).toString() == "verb3") P = bTemp.get(0).toString();
            else if (bTemp.get(1).toString() == "agent"
                || bTemp.get(1).toString() == "agent1"
                || bTemp.get(1).toString() == "agent2")     A = bTemp.get(0).toString();
            else if (bTemp.get(1).toString() == "object"
                || bTemp.get(1).toString() == "object1"
                || bTemp.get(1).toString() == "object2")    O = bTemp.get(0).toString();
            else if (bTemp.get(1).toString() == "recipient"
                || bTemp.get(1).toString() == "spatial"
                || bTemp.get(1).toString() == "spatial1"
                || bTemp.get(1).toString() == "spatial2") R = bTemp.get(0).toString();
            else if (bTemp.get(1).toString() == "status") status = bTemp.get(0).toString();
            else if (bTemp.get(1).toString() == "reason") reason = bTemp.get(0).toString();
            else if (bTemp.get(1).toString() == "adv1") reason = bTemp.get(0).toString();
            else if (bTemp.get(1).toString() == "adv2") reason = bTemp.get(0).toString();

        }

        // if it is an ACTION
        if (current_activitytype == "action"){

            if (P == "none"){
                P = current_activityname;
            }
            // if the action begin
            if (current_begin){

                // if the previous instance wasn't already an action
                if (current_activitytype != previous_activitytype || previous_begin != current_begin){
                    cout << "\t\t\t" << A << " tries to " << P << " the " << O;
                    if (R != "none")  cout << " " << R;
                    if (adv1 != "none")  cout << " " << adv1;
                    if (adv2 != "none")  cout << " " << adv2;

                    cout << endl;
                }
            }
            // the action ends
            else{
                // if previous instance was not a beggining of action
                if (previous_begin || previous_activitytype != "action"){
                    if (status == "failed"){
                        cout << "\t\t\t" << "But it failed";
                        if (reason != "none"){
                            cout << " because " << reason;
                        }
                        cout << "." << endl;

                    }
                    else {
                        cout << "\t\t\t" << "And it worked." << endl;
                    }
                }
                else{
                    for (int kk = 0; kk < bArguments.size(); kk++){
                        Bottle bTemp = *bArguments.get(kk).asList();
                        if (bTemp.get(1).toString() == "reason") cout << "\t\t\t" << "Because " << bTemp.get(0).toString() << endl;
                    }
                }

            }
        }
        else if (current_activityname == "sentence") {
            string speaker = "none",
                addressee = "none",
                sentence = "none";
            for (int kk = 0; kk < bArguments.size(); kk++){
                Bottle bTemp = *bArguments.get(kk).asList();
                if (bTemp.get(1).toString() == "speaker") speaker = bTemp.get(0).toString();
                else if (bTemp.get(1).toString() == "addressee")     addressee = bTemp.get(0).toString();
                else if (bTemp.get(1).toString() == "sentence")    sentence = bTemp.get(0).toString();
            }

            cout << "\t\t\t" << speaker << " says to " << addressee << ": " << sentence << endl;
        }
        // if not actino or sentence
        else if (current_activitytype == "reasoning"){
            for (int kk = 0; kk < bArguments.size(); kk++){
                Bottle bTemp = *bArguments.get(kk).asList();
                cout << bTemp.toString() << endl;
            }
            // if the action begin
            if (current_begin){

                // if the previous instance wasn't already an action
                if (current_activitytype != previous_activitytype || previous_begin != current_begin){
                    cout << "\t\t\t" << A << " tries to " << current_activityname << endl;

                    for (int kk = 0; kk < bArguments.size(); kk++){
                        Bottle bTemp = *bArguments.get(kk).asList();
                        if (bTemp.get(1).toString() == "goal"){
                            Bottle bUnfolded = unfoldGoal(bTemp.get(0).toString());
                            cout << "\t\t\t" << "The goal was that: " << bUnfolded.find("agent").toString() << " tries to " << bUnfolded.find("predicate").toString() << " the " << bUnfolded.find("object").toString();
                            if (bUnfolded.find("recipient").toString() != "")  cout << " " << bUnfolded.find("recipient").toString();
                            cout << endl;
                        }
                    }
                }
            }
            // the action ends
            else{
                // if previous instance was not a beggining of action
                if (previous_begin || previous_activitytype != "action"){
                    if (status == "failed"){
                        cout << "\t\t\t" << "But it failed";
                        if (reason != "none"){
                            cout << " because " << reason;
                        }
                        cout << "." << endl;

                    }
                    else {
                        cout << "\t\t\t" << "And it worked." << endl;
                    }
                }
                else{
                    for (int kk = 0; kk < bArguments.size(); kk++){
                        Bottle bTemp = *bArguments.get(kk).asList();
                        if (bTemp.get(1).toString() == "reason") cout << "\t\t\t" << "Because " << bTemp.get(0).toString() << endl;
                    }
                    if (reason != "none"){
                        cout << " because " << reason;
                    }
                    cout << "." << endl;
                }
            }
        }
        else {
            for (int kk = 0; kk < bArguments.size(); kk++){
                Bottle bTemp = *bArguments.get(kk).asList();
                //                cout << bTemp.toString() << endl;
            }
            // if the action begin
            if (current_begin){

                // if the previous instance wasn't already an action
                if (current_activitytype != previous_activitytype || previous_begin != current_begin){
                    cout << "\t\t\t" << A << " tries to " << current_activityname << endl;
                }
            }
            // the action ends
            else{
                // if previous instance was not a beggining of action
                if (previous_begin || previous_activitytype != "action"){
                    if (status == "failed"){
                        cout << "\t\t\t" << "But it failed";
                        if (reason != "none"){
                            cout << " because " << reason;
                        }
                        cout << "." << endl;

                    }
                    else {
                        cout << "\t\t\t" << "And it worked." << endl;
                    }
                }
                else{
                    for (int kk = 0; kk < bArguments.size(); kk++){
                        Bottle bTemp = *bArguments.get(kk).asList();
                        if (bTemp.get(1).toString() == "reason") cout << "\t\t\t" << "Because " << bTemp.get(0).toString() << endl;
                    }
                    if (reason != "none"){
                        cout << " because " << reason;
                    }
                    cout << "." << endl;
                }
            }
        }


        // final situation
        if (ii != st.viInstances.size() && ii != 0){

            if (bRelations != bPreviousRelation)
            {
                if (bRelations.toString() != "NULL"){
                    cout << "\t\t\t" << "And now, ";
                    for (int jj = 0; jj < bRelations.size(); jj++){
                        if (jj != 0){
                            cout << " and ";
                        }
                        cout << bRelations.get(jj).toString();
                    }
                    cout << endl;
                }
            }
        }
        bPreviousRelation = bRelations;

        ii++;


        // final situation
        if (ii == st.viInstances.size()){
            cout << "\t\t\t" << "In the end, ";
            for (int jj = 0; jj < bRelations.size(); jj++){
                if (jj != 0){
                    cout << " and ";
                }
                cout << bRelations.get(jj).toString();
            }
            cout << endl;
        }
    }

    cout << endl << endl;

}


Bottle narrativeHandler::unfoldGoal(string goal)
{
    bool bVerbose = false;
    Bottle bOutput;

    if (bVerbose) cout << endl << "Starting to unfold: " << goal << endl;

    bool isRole = true;
    bool bIsFirst = true;
    istringstream iss(goal);
    Bottle bTemp;
    do
    {
        string sub;
        iss >> sub;
        if (sub[0] == '(') sub = sub.erase(0, 1);
        if (sub[sub.size() - 1] == ')')   sub = sub.erase(sub.size() - 1);
        if (bVerbose) cout << "Substring: " << sub << endl;
        if (isRole){
            if (!bIsFirst) bOutput.addList() = bTemp;
            bTemp.clear();
            bTemp.addString(sub);
        }
        else{
            bTemp.addString(sub);
        }
        bIsFirst = false;
        isRole = !isRole;

    } while (iss);

    if (bVerbose) cout << "bOutput: " << bOutput.toString() << endl;

    if (bVerbose) cout << "find is: " << bOutput.find("predicate").toString() << endl;

    return bOutput;
}


void narrativeHandler::initializeStories()
{
    for (auto itSt = listStories.begin(); itSt != listStories.end(); itSt++){

        itSt->vEvents.clear();
        ostringstream osRequest;
        osRequest.str("");
        osRequest << "SELECT time FROM main WHERE instance = " << *(itSt->viInstances.begin());
        Bottle bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        itSt->timeBegin = string2Time(bMessenger.toString());

        osRequest.str("");
        osRequest << "SELECT time FROM main WHERE instance = " << itSt->viInstances[itSt->viInstances.size()-1];
        bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        itSt->timeEnd = string2Time(bMessenger.toString());

        for (auto itInst = itSt->viInstances.begin(); itInst != itSt->viInstances.end(); itInst++){


            ostringstream osRequest;

            osRequest.str("");
            osRequest << "SELECT subject, verb, object FROM relation WHERE instance = " << *itInst << " AND verb != 'isAtLoc'";
            Bottle bRelations = iCub->getABMClient()->requestFromString(osRequest.str());
            //cout << "Relations: " << bMessenger.toString() << endl;

            osRequest.str("");
            osRequest << "SELECT activityname, activitytype, begin FROM main WHERE instance = " << *itInst;
            Bottle bActivity = iCub->getABMClient()->requestFromString(osRequest.str());
            //        cout << "activity info: " << bActivity.toString() << endl;

            osRequest.str("");
            osRequest << "SELECT argument, role FROM contentarg WHERE instance = " << *itInst;
            Bottle bArguments = iCub->getABMClient()->requestFromString(osRequest.str());


            itSt->vEvents.push_back(evtStory(*itInst, bActivity, bArguments, bRelations));
        }

        cout << "story initialized" << endl;

        itSt->displayNarration();

        cout << endl << endl;

    }


}
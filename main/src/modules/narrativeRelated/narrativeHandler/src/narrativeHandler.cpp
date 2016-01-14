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
#include "wrdac/subsystems/subSystem_LRH.h"

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

    if (!iCub->getLRH())
    {
        yWarning() << " WARNING LRH NOT CONNECTED";
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

    return false;
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
        osRequest << "SELECT time FROM main WHERE instance = " << itSt->viInstances[itSt->viInstances.size() - 1];
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

            evtStory evtTemp;
            vector<string> tempOCW = initializeEVT(evtTemp, *itInst, bActivity, bArguments, bRelations);

            itSt->vEvents.push_back(evtTemp);
            itSt->addOCW(tempOCW);
        }

        cout << "story initialized" << endl;

        itSt->displayNarration();

        cout << endl << endl;

    }
}



void narrativeHandler::updateScoreStory(story &st){

    st.mapScore.clear();

    unsigned int _size = st.vEvents.size();

    // create the map of each OCW with a vector of double for the score at each instance of the story
    for (auto itS = st.vOCW.begin(); itS != st.vOCW.end(); itS++){
        vector<double> vTemp(_size);
        st.mapScore[*itS] = vTemp;
    }

    // for each instance update the score of each OCW corresponding to a few rules
    for (auto itE = st.vEvents.begin(); itE != st.vEvents.end(); itE++){




    }
}




vector<string> narrativeHandler::initializeEVT(evtStory &evt, int _instance, Bottle bActivity, Bottle bArguments, Bottle _bRelations){
    evt.instance = _instance;
    evt.bRelations = _bRelations;
    vector<string>   vOCW;

    vector<string> vPredicate{ "predicate", "action", "action1", "action2", "action3", "verb", "verb1", "verb2", "verb3" };
    vector<string> vAgent{ "agent", "agent1", "agent2", "agent3" };
    vector<string> vObject{ "object", "object1", "object2", "object2" };
    vector<string> vRecipient{ "recipient", "spatial", "spatial1", "spatial2", "spatial3" };

    vector<string>  vNoPAOR{ "subsystem", "provider", "vector", "sentence" };

    evt.activity_type = (bActivity.get(0).asList())->get(1).toString();
    evt.activity_name = (bActivity.get(0).asList())->get(0).toString();

    if (evt.activity_type == ""){
        yWarning() << " in narrativeHandler::evtStory::evtStory no activity_type.";
    }
    if (evt.activity_name == ""){
        yWarning() << " in narrativeHandler::evtStory::evtStory no activity_type.";
    }

    evt.begin = (bActivity.get(0).asList())->get(2).toString() == "t";

    for (int kk = 0; kk < bArguments.size(); kk++){
        if (bArguments.get(kk).isList()) {
            Bottle bTemp = *bArguments.get(kk).asList();

            if (evt.isIn(vPredicate, bTemp.get(1).toString())) evt.predicate = bTemp.get(0).toString();
            else if (evt.isIn(vAgent, bTemp.get(1).toString())) evt.agent = bTemp.get(0).toString();
            else if (evt.isIn(vObject, bTemp.get(1).toString()))    evt.object = bTemp.get(0).toString();
            else if (evt.isIn(vRecipient, bTemp.get(1).toString())) evt.recipient = bTemp.get(0).toString();
            else {
                pair<string, string> ptemp(bTemp.get(1).toString(), bTemp.get(0).toString());
                evt.vArgument.push_back(ptemp);
            }

            if (!evt.isIn(vNoPAOR, bTemp.get(1).toString())) vOCW.push_back(bTemp.get(0).asString());
        }
    }

    if (evt.activity_name == "sentence"){
        evt.predicate = "say";
        for (int kk = 0; kk < bArguments.size(); kk++){
            if (bArguments.get(kk).isList()) {
                Bottle bTemp = *bArguments.get(kk).asList();
                if (bTemp.get(1).toString() == "speaker") evt.agent = bTemp.get(0).toString();
                else if (bTemp.get(1).toString() == "addressee") evt.recipient = bTemp.get(0).toString();
                else if (bTemp.get(1).toString() == "sentence") evt.object = bTemp.get(0).toString();
            }
        }
    }


    if (_bRelations.toString() != "NULL"){
        for (int kk = 0; kk < _bRelations.size(); kk++){
            Bottle bTemp = *_bRelations.get(kk).asList();
            for (int jj = 0; jj != bTemp.size(); jj++){
                vOCW.push_back(bTemp.get(jj).toString());
            }
        }
    }

    if (evt.activity_type == "action"){
        if (evt.predicate == "none" || evt.predicate == ""){
            evt.predicate = evt.activity_name;
            vOCW.push_back(evt.predicate);
        }
    }

    return vOCW;

}

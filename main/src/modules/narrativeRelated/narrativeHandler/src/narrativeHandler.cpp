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
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "narrativeHandler", "narrativeHandler.ini", isRFVerbose);
    //iCub->opc->isVerbose &= true;

    // get grammar file
    GrammarNarration = rf.findFileByName(rf.check("GrammarNarration", Value("GrammarNarration.xml")).toString());
    GrammarYesNo = rf.findFileByName(rf.check("GrammarYesNo", Value("nodeYesNo.xml")).toString());

    if (!iCub->connect())
    {
        yInfo() << "iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    dThresholdDiffStory = rf.check("dThresholdDiffStory", Value(15.)).asDouble();
    iThresholdSizeStory = rf.check("iThresholdSizeStory", Value(6)).asInt();
    iThresholdSentence = rf.check("iThresholdSentence", Value(6)).asInt();
    iMinInstance = rf.check("instanceStart", Value(0)).asInt();
    narrator = rf.check("narrator", Value("Narrator")).asString().c_str();
    lrh = rf.find("lrh").asInt() == 1;

	cursorStories = 0;

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    if (!iCub->getABMClient())
    {
        yWarning() << " WARNING ABM NOT CONNECTED, MODULE CANNOT START";
        return false;
    }

    if (!iCub->getLRH())
        yWarning(" WARNING LRH NOT CONNECTED");
    else
        iCub->getLRH()->bForwardABM = (rf.find("forwardABM").asInt() == 1);

    if (!iCub->getRecogClient())
    {
        iCub->say("Proactive Tagging warning speech recognizer not connected");
        yWarning() << "WARNING SPEECH RECOGNIZER NOT CONNECTED";
    }

	std::string ttsOptions = rf.check("ttsOptions", yarp::os::Value("iCub")).toString();
	if (ttsOptions != "iCub") {
		if (iCub->getSpeechClient())
			iCub->getSpeechClient()->SetOptions(ttsOptions);
	}


    yInfo() << " dThresholdDiffStory: " << dThresholdDiffStory;
    yInfo() << " iThresholdSizeStory: " << iThresholdSizeStory;

    counter = 0;

	findStories(iMinInstance);
	for (auto st : listStories){
		st.displayNarration();
	}


    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";




    //    cout << iCub->getLRH()->SentenceToMeaning("if I could ask you to give it to me") << endl;
    //    cout << iCub->getLRH()->SentenceToMeaning("then you would give it to me") << endl;

    return true;
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
		" setNarrator + name: \n" +
		" askNarrate: \n" +
		" narrate: \n" +
		" commands are: \n" +
		" quit \n";

	yInfo() << " rpc command received: " << command.toString();

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    }
	else if (command.get(0).asString() == "setNarrator"){
		if (command.size() == 2){
			narrator = command.get(1).asString();
			reply.addString("narrator set to " + narrator);
		}
		else{
			reply.addString("error in narrativeHandler::setNarrator wrong size of input: should be: (setNarrator narrator)");
		}
	}
	else if (command.get(0).asString() == "askNarrate"){
		if (askNarrate()){
			reply.addString("ack");
		}
		else{
			reply.addString("nack");
		}
	}
	else if (command.get(0).asString() == "narrate"){
		if (narrate()){
			reply.addString("ack");
		}
		else{
			reply.addString("nack");
		}
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
	yInfo() << " begin findStories from: " << iInstance << " while iMinInstance is: " << iMinInstance;
    cout << "Starting findStories begin: "<< iInstance <<endl;

    story currentStory;
    currentStory.iThresholdSentence = iThresholdSentence;

    //    int iCurrentInstance = iInstance;

    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE instance > " << iInstance << " ORDER by instance";
    Bottle  bAllInstances = iCub->getABMClient()->requestFromString(osRequest.str());
    Bottle bMessenger;
    int numberInstances = bAllInstances.size();

    vector<int> vError;
    yInfo() << "\t" << "found " << numberInstances << " instance(s)";
    double mDiff;
    for (int j = 1; j < (numberInstances); j++)
    {

        int Id = atoi(bAllInstances.get(j - 1).asList()->get(0).toString().c_str());
        int Id2 = atoi(bAllInstances.get(j).asList()->get(0).toString().c_str());

		iMinInstance = Id2;

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
                    currentStory.counter = counter;
                    counter++;
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
        currentStory.counter = counter;
        counter++;
        listStories.push_back(currentStory);
    }

    int ii = 1;

    for (auto& itSt : listStories)
    {
        cout << "Story " << ii << ": ";
        for (auto& itIn : itSt.viInstances)
        {
            cout << itIn << " ";
        }
        cout << endl;
        osRequest.str("");
        osRequest << "SELECT subject, verb, object FROM relation WHERE instance = " << *itSt.viInstances.begin() << " AND verb != 'isAtLoc'";
        bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        if (bMessenger.toString() != "NULL")   cout << "before: " << bMessenger.toString() << endl;

        osRequest.str("");
        osRequest << "SELECT subject, verb, object FROM relation WHERE instance = " << itSt.viInstances[itSt.viInstances.size() - 1] << " AND verb != 'isAtLoc'";
        bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        if (bMessenger.toString() != "NULL")   cout << "after : " << bMessenger.toString() << endl;

        ii++;
    }

	initializeStories();
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
    mtsOut.iMilliSec = long(atoi(sMS.c_str()) * pow(10, 3 - sMS.size()));

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
	yInfo() << " initializeStories from: " << cursorStories;
    cout << "begin initializating stories ";
    
	vector<int>    toDelete; // vector of the stories to delete from the list.
	int iSto = 0;
	for (int jj = cursorStories; jj < listStories.size(); jj++){

		story &itSt = listStories[jj];
        itSt.vEvents.clear();
        ostringstream osRequest;
        osRequest.str("");
        osRequest << "SELECT time FROM main WHERE instance = " << *(itSt.viInstances.begin());
        Bottle bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        itSt.timeBegin = string2Time(bMessenger.toString());

        osRequest.str("");
        osRequest << "SELECT time FROM main WHERE instance = " << itSt.viInstances[itSt.viInstances.size() - 1];
        bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        itSt.timeEnd = string2Time(bMessenger.toString());

        for (auto& itInst : itSt.viInstances){
            ostringstream osRequest;

            osRequest.str("");
            osRequest << "SELECT subject, verb, object FROM relation WHERE instance = " << itInst << " AND verb != 'isAtLoc'";
            Bottle bRelations = iCub->getABMClient()->requestFromString(osRequest.str());
            //cout << "Relations: " << bMessenger.toString() << endl;

            osRequest.str("");
            osRequest << "SELECT activityname, activitytype, begin FROM main WHERE instance = " << itInst;
            Bottle bActivity = iCub->getABMClient()->requestFromString(osRequest.str());
            //        cout << "activity info: " << bActivity.toString() << endl;

            osRequest.str("");
            osRequest << "SELECT argument, role, subtype FROM contentarg WHERE instance = " << itInst;
            Bottle bArguments = iCub->getABMClient()->requestFromString(osRequest.str());

            evtStory evtTemp;
            vector<string> tempOCW = initializeEVT(evtTemp, itInst, bActivity, bArguments, bRelations);

            itSt.vEvents.push_back(evtTemp);
            itSt.addOCW(tempOCW);
        }

        cout << ".";

        createNarration(itSt);
		if (itSt.sentenceStory.size() < 2){
			toDelete.push_back(iSto);
		}
		iSto++;
    }
	

	for (auto ii : toDelete){
		listStories[ii] = listStories.back();
		listStories.pop_back();
	}
	cursorStories = listStories.size();

	cout << "End of initialisation of stories" << endl;
	cout << listStories.size() << " stories found" << endl;
	cout << endl;

}


void narrativeHandler::updateScoreStory(story &st){

    st.mapScore.clear();

    unsigned int _size = st.vEvents.size();

    // create the map of each OCW with a vector of double for the score at each instance of the story
    for (auto& itS : st.vOCW){
        vector<double> vTemp(_size);
        st.mapScore[itS] = vTemp;
    }

    // for each instance update the score of each OCW corresponding to a few rules
    //for (auto& itE : st.vEvents){
    //    // TODO
    //}
}


vector<string> narrativeHandler::initializeEVT(evtStory &evt, int _instance, Bottle bActivity, Bottle bArguments, Bottle _bRelations){
    evt.instance = _instance;
    evt.isNarration = false;
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
            else if (evt.isIn(vPredicate, bTemp.get(2).toString())) evt.predicate = bTemp.get(0).toString();
            else if (evt.isIn(vAgent, bTemp.get(1).toString())) evt.agent = bTemp.get(0).toString();
            //       else if (evt.isIn(vAgent, bTemp.get(2).toString())) evt.agent = bTemp.get(0).toString();
            else if (evt.isIn(vObject, bTemp.get(1).toString()))    evt.object = bTemp.get(0).toString();
            //       else if (evt.isIn(vObject, bTemp.get(2).toString()))    evt.object = bTemp.get(0).toString();
            else if (evt.isIn(vRecipient, bTemp.get(1).toString())) evt.recipient = bTemp.get(0).toString();
            else if (evt.isIn(vRecipient, bTemp.get(2).toString())) evt.recipient = bTemp.get(0).toString();
            else {
                pair<string, string> ptemp(bTemp.get(1).toString(), bTemp.get(0).toString());
                evt.vArgument.push_back(ptemp);
            }

            if (!evt.isIn(vNoPAOR, bTemp.get(1).toString())) vOCW.push_back(bTemp.get(0).asString());
            else if (!evt.isIn(vNoPAOR, bTemp.get(2).toString())) vOCW.push_back(bTemp.get(0).asString());
        }
    }

    if (evt.activity_name == "production" || evt.activity_name == "comprehension" || evt.activity_name == "sentence"){
        evt.predicate = "say";
        for (int kk = 0; kk < bArguments.size(); kk++){
            if (bArguments.get(kk).isList()) {
                Bottle bTemp = *bArguments.get(kk).asList();
                if (bTemp.get(1).toString() == "speaker") evt.agent = bTemp.get(0).toString();
                else if (bTemp.get(1).toString() == "addressee") evt.recipient = bTemp.get(0).toString();
                else if (bTemp.get(1).toString() == "sentence") evt.object = bTemp.get(0).toString();
            }
        }

        string presentAgent = "partner";
        ostringstream osRequest;
        osRequest << "SELECT name FROM agent WHERE instance = " << _instance << " AND presence = true";
        Bottle bMessenger = iCub->getABMClient()->requestFromString(osRequest.str());
        if (bMessenger.toString() != "NULL"){
            if (bMessenger.size() !=0){
                for (int ll = 0 ; ll < bMessenger.size() ; ll++){
                    if (bMessenger.get(ll).toString() != "partner"){
                        presentAgent = bMessenger.get(ll).toString();
                    }
                }
            }
        }

        if (evt.activity_type == "recog"){
            if (evt.agent == "" || evt.agent == "none" || evt.agent == "partner"){
                evt.agent = presentAgent;
            }
        }
        if (evt.activity_type =="say"){
            evt.agent = "iCub";
            if (evt.recipient == "" || evt.recipient == "none" || evt.recipient == "partner"){
                evt.recipient = presentAgent;
            }
        }

        evt.isNarration = (evt.agent == narrator);
    }

    if (evt.activity_type == "drives"){
        evt.object = evt.agent;
        evt.agent = "iCub";
        evt.predicate = "wants to";
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

    evt.addUnderscore();

    return vOCW;

}


void narrativeHandler::compareNarration(story &target){
    cout << "Starting to compare Narration from target: " << target.counter << endl;
    for (auto& currentStory : listStories){
        comparator.clear();
        if (currentStory.counter != target.counter){ // not comparing the target to itself
            if (target.vEvents.size() <= currentStory.vEvents.size()){ //if there is not more in the target than in the current
                unsigned int K = 0; // possibility to pass narration
                bool stillOk = true;
                unsigned int cursor = 0;

                for (auto& tarEvt : target.vEvents){  // for each event of the target
                    K = cursor;
                    bool found = false;
                    if (stillOk){
                        for (unsigned int j = K; j < currentStory.vEvents.size(); j++){
                            evtStory evt = currentStory.vEvents[j];
                            if (!evt.isNarration && !found){

                                vector<string>  vOriginal, vCopy;
                                vOriginal.push_back(evt.agent);
                                vOriginal.push_back(evt.predicate);
                                vOriginal.push_back(evt.object);
                                vOriginal.push_back(evt.recipient);
                                vCopy.push_back(tarEvt.agent);
                                vCopy.push_back(tarEvt.predicate);
                                vCopy.push_back(tarEvt.object);
                                vCopy.push_back(tarEvt.recipient);

                                bool isEqual = checkListPAOR(vOriginal, vCopy);

                                if (isEqual){
                                    found = true;
                                    cursor = j + 1;
                                }
                            }
                        }
                        stillOk &= found;
                    }
                }
                if (stillOk){
                    yInfo("I found I matching story.");
                    for (auto mean : currentStory.meaningStory){
                        target.meaningStory.push_back(adaptMeaning(mean));
                    }
                }
            }
        }
    }
}



void narrativeHandler::sayNarrationSimple(story target){

    cout << "Start Narration Simple : " << endl;

    vector<string>    sentencesFromLRH;

    for (auto evt : target.vEvents){
        cout << "\t A:" << evt.agent;
        cout << "\t P:" << evt.predicate;
        cout << "\t O:" << evt.object;
        cout << "\t R:" << evt.recipient << endl;
        string meaning = createMeaning(evt.agent, evt.predicate, evt.object, evt.recipient);
        string sentence = iCub->getLRH()->meaningToSentence(meaning);
        sentencesFromLRH.push_back(sentence);
    }


    cout << endl << "Narration simple using LRH" << endl;
    // story comming from LRH is:
    for (auto sen : sentencesFromLRH){
        cout << sen << endl;
    }
}


void narrativeHandler::createNarration(story &sto)
{
    bool VERBOSE = false;

    vector<string>  vsOutput;

    ostringstream  osCurrent;

    tuple<string, string, string, string>  PAOR;

    //Bottle bPreviousRelation;

    osCurrent << "story: " << sto.timeBegin.toString() << " (" << sto.viInstances[0] << ")  to " << sto.timeEnd.toString() << " (" << sto.viInstances[sto.viInstances.size() - 1] << ")" << endl;

    vsOutput.push_back(osCurrent.str());
    int cursor = 0;
    for (unsigned int currentElement = 0; currentElement != sto.vEvents.size(); currentElement++){
        evtStory currentEvent = sto.vEvents[currentElement];
        currentEvent.addUnderscore();
        bool addEvt = true;        
        if (!currentEvent.isNarration)
        {
            osCurrent.str("");
            if (VERBOSE) cout << currentElement << "...";

            // initial situation
            if (cursor == 0){
                if (currentEvent.bRelations.toString() != "NULL"){
                    osCurrent << "\t\t\t" << "At the beginning, ";
                    for (int jj = 0; jj < currentEvent.bRelations.size(); jj++){
                        if (jj != 0){
                            osCurrent << " and ";
                        }
                        if (lrh && currentEvent.bRelations.get(jj).isList()){
                            string meaning = createMeaning(currentEvent.bRelations.get(jj).asList()->get(0).toString(),
                                                           currentEvent.bRelations.get(jj).asList()->get(1).toString(),
                                                           currentEvent.bRelations.get(jj).asList()->get(2).toString());
                            string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                            osCurrent << tmpSentence;
                        }
                        else{
                            osCurrent << currentEvent.bRelations.get(jj).toString();
                        }
                    }
                    osCurrent << endl;
                }
            }
            // end initial situation


            // if it is an ACTION
            if (currentEvent.activity_type == "action"){
                // if the action begin
                if (currentEvent.begin){

                    if (cursor == 0){
                        if (lrh){
                            string meaning = createMeaning(currentEvent.agent,
                                                           currentEvent.predicate,
                                                           currentEvent.object,
                                                           currentEvent.recipient);
                            string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                            osCurrent << "\t\t\t" << tmpSentence;
                        }
                        else{
                            osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate << " the " << currentEvent.object;
                        }
                        if (currentEvent.recipient != "none" && currentEvent.recipient != "")  osCurrent << " " << currentEvent.recipient;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "adv1" || iarg->first == "adv2") osCurrent << " " << iarg->second;
                        }
                        osCurrent << endl;
                    }
                    else{
                        // if the previous instance wasn't already an action
                        if (currentEvent.activity_type != sto.vEvents[currentElement - 1].activity_type || currentEvent.begin != sto.vEvents[currentElement - 1].begin){
                            if (lrh){
                                string meaning = createMeaning(currentEvent.agent,
                                                               currentEvent.predicate,
                                                               currentEvent.object,
                                                               currentEvent.recipient);
                                string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                                osCurrent << "\t\t\t" << tmpSentence;
                            }
                            else{
                                osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate << " the " << currentEvent.object;
                            }
                            if (currentEvent.recipient != "none" && currentEvent.recipient != "")  osCurrent << " " << currentEvent.recipient;
                            for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                                if (iarg->first == "adv1" || iarg->first == "adv2") osCurrent << " " << iarg->second;
                            }
                            osCurrent << endl;
                        }
                    }
                }
                // the action ends
                else{
                    if (cursor == 0){
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed." << endl;
                            }
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second << "." << endl;
                            }
                        }
                    }
                    else{
                        if (VERBOSE) cout << endl;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (VERBOSE) cout << iarg->first << " " << iarg->second << " - " << endl;
                        }
                        // if previous instance was not a beggining of action
                        if (sto.vEvents[currentElement - 1].begin || sto.vEvents[currentElement - 1].activity_type != "action"){
                            for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                                if (iarg->first == "status" && iarg->second == "failed"){
                                    osCurrent << "\t\t\t" << "But it failed." << endl;
                                }
                            }
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << "\t\t\t" << "Because " << iarg->second << "." << endl;
                            }
                        }
                    }
                }
            }
            else if (currentEvent.activity_name == "sentence"
                     || currentEvent.activity_name == "comprehension"
                     || currentEvent.activity_name == "production") {
                string speaker = currentEvent.agent,
                        addressee = currentEvent.recipient,
                        sentence = "none";
                for (auto& iarg : currentEvent.vArgument){
                    if (iarg.first == "speaker") speaker = iarg.second;
                    else if (iarg.first == "addressee")     addressee = iarg.second;
                    else if (iarg.first == "sentence")    sentence = iarg.second;
                }
                if (speaker=="none" && currentEvent.activity_type=="say"){
                    speaker = "iCub";
                }
                if (speaker != "icub" && speaker != "iCub"){
                    if (addressee == "none"){
                        addressee = "iCub";
                    }
                }
                if (sentence=="none" || sentence == ""){
                    addEvt = false;
                }
                else{
                    currentEvent.addUnderscoreString(sentence);
                }
                if (lrh){
                    string meaning = createMeaning(speaker, "says", sentence, addressee);
                    string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                    currentEvent.removeUnderscoreString(tmpSentence);
                    osCurrent << "\t\t\t" << tmpSentence;
                }
                else{
                    osCurrent << "\t\t\t" << speaker << " says to " << addressee << ": " << sentence;
                }
                osCurrent << endl;
            }
            // if not action or sentence
            else if (currentEvent.activity_type == "reasoning"){
                /* for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                osCurrent << iarg->second << " ";
                }*/
                // if the action begin
                if (currentEvent.begin){

                    // if the previous instance wasn't already an action
                    //if (current_activitytype != previous_activitytype || previous_begin != current_begin){
                    if (lrh){
                        string meaning = createMeaning(currentEvent.agent, "tries", currentEvent.activity_name);
                        string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                        osCurrent << "\t\t\t" << tmpSentence;
                    }
                    else{
                        osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.activity_name;
                    }
                    osCurrent << endl;
                    for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                        if (iarg->first == "goal"){
                            Bottle bUnfolded = unfoldGoal(iarg->second);
                            osCurrent << "\t\t\t" << "The goal was that: ";
                            if (lrh){
                                string meaning = createMeaning(bUnfolded.find("agent").toString(),
                                                               "try",
                                                               bUnfolded.find("predicate").toString(),
                                                               bUnfolded.find("object").toString());
                                string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                                osCurrent << tmpSentence;

                            }
                            else{
                                osCurrent << bUnfolded.find("agent").toString() << " " << bUnfolded.find("predicate").toString() << " the " << bUnfolded.find("object").toString();
                            }
                            if (bUnfolded.find("recipient").toString() != "")  osCurrent << " " << bUnfolded.find("recipient").toString();
                            osCurrent << endl;
                        }
                    }
                }
                // the action ends
                else{
                    if (cursor == 0){
                        bool bStatusFound = false;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed";
                                bStatusFound = true;
                            }
                        }
                        if (!bStatusFound){
                            osCurrent << "\t\t\t" << "And it worked." << endl;
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second;
                            }
                        }
                        osCurrent << "." << endl;
                    }
                    else if (sto.vEvents[currentElement - 1].begin || sto.vEvents[currentElement - 1].activity_type != "action"){// if previous instance was not a beggining of action
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed." << endl;
                            }
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second << "." << endl;
                            }
                        }
                    }
                    else{
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second << "." << endl;
                            }
                        }
                    }
                }
            }
            // nor action/reasoning/sentence
            else {
                //for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                //    osCurrent << iarg->first << " " << iarg->second << "; ";
                //    //                osCurrent << bTemp.toString() << endl;
                //}
                //osCurrent << endl;
                // if the action begin
                if (currentEvent.begin){
                    bool hasPredicate = false;
                    if (currentEvent.predicate != "" && currentEvent.predicate!="none"){
                        hasPredicate = true;
                    }
                    if (cursor == 0){
                        if (lrh){
                            string meaning = createMeaning(currentEvent.agent,
                                                           "tries",
                                                           currentEvent.activity_name,
                                                           currentEvent.object);
                            string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                            osCurrent << "\t\t\t" << tmpSentence;
                        }
                        else{
                            if (hasPredicate){
                                osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate;
                            }
                            else{
                                if (currentEvent.agent != "" && currentEvent.agent != "none"){
                                    osCurrent << "\t\t\t" << currentEvent.agent << " tries to " << currentEvent.activity_name;
                                }
                            }
                        }
                        if (currentEvent.object != "") osCurrent << " the " << currentEvent.object;
                        if (currentEvent.recipient != "") osCurrent << " the " << currentEvent.recipient;
                        osCurrent << endl;
                    }
                    // if the previous instance wasn't already an action
                    else if (currentEvent.activity_type != sto.vEvents[currentElement - 1].activity_type || currentEvent.begin != sto.vEvents[currentElement - 1].begin){
                        if (lrh){
                            string meaning = createMeaning(currentEvent.agent,
                                                           "tries",
                                                           currentEvent.activity_name,
                                                           currentEvent.object);
                            string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                            osCurrent << "\t\t\t" << tmpSentence;
                        }
                        else{
                            if (hasPredicate){
                                osCurrent << "\t\t\t" << currentEvent.agent << " " << currentEvent.predicate;
                            }
                            else{
                                osCurrent << "\t\t\t" << currentEvent.agent << " tries to " << currentEvent.activity_name;
                            }
                        }
                        if (currentEvent.object != "") osCurrent << " the " << currentEvent.object;
                        if (currentEvent.recipient != "") osCurrent << " the " << currentEvent.recipient;
                        osCurrent << endl;
                    }
                }
                // the action ends
                else {
                    if (cursor == 0){
                        bool bStatusFound = false;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed";
                                bStatusFound = true;
                            }
                        }
                        if (!bStatusFound){
                            osCurrent << "\t\t\t" << "And it worked." << endl;
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second << "." << endl;
                            }
                        }
                    }
                    else if (sto.vEvents[currentElement - 1].begin || sto.vEvents[currentElement - 1].activity_type != "action"){// if previous instance was not a beggining of action
                        bool bStatusFound = false;
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed";
                                bStatusFound = true;
                            }
                        }
                        if (!bStatusFound){
                            osCurrent << "\t\t\t" << "And it worked." << endl;
                        }
                        for (auto iarg = currentEvent.vArgument.begin(); iarg != currentEvent.vArgument.end(); iarg++){
                            if (iarg->first == "reason"){
                                osCurrent << " because " << iarg->second << "." << endl;
                            }
                        }
                    }
                }
            }


            // changes in the relations:
            if (currentElement != sto.vEvents.size() && cursor != 0){

                if (currentEvent.bRelations != sto.vEvents[currentElement - 1].bRelations)
                {
                    if (currentEvent.bRelations.toString() != "NULL"){
                        osCurrent << "\t\t\t" << "And now, ";
                        for (int jj = 0; jj < currentEvent.bRelations.size(); jj++){
                            if (jj != 0){
                                osCurrent << " and ";
                            }
                            osCurrent << currentEvent.bRelations.get(jj).toString();
                        }
                        osCurrent << endl;
                    }
                }
            }

            // final situation
            if (currentElement == sto.vEvents.size() - 1){
                osCurrent << "\t\t\t" << "In the end, ";
                for (int jj = 0; jj < currentEvent.bRelations.size(); jj++){
                    if (jj != 0){
                        osCurrent << " and ";
                    }
                    if (lrh && currentEvent.bRelations.get(jj).isList()){
                        string meaning = createMeaning(currentEvent.bRelations.get(jj).asList()->get(0).toString(),
                                                       currentEvent.bRelations.get(jj).asList()->get(1).toString(),
                                                       currentEvent.bRelations.get(jj).asList()->get(2).toString());
                        string tmpSentence = iCub->getLRH()->meaningToSentence(meaning);
                        osCurrent << tmpSentence;
                    }
                    else{
                        osCurrent << currentEvent.bRelations.get(jj).toString();
                    }
                }
                osCurrent << "." << endl;
            }

            if (((currentEvent.predicate == "" || currentEvent.predicate == "none")
                && (currentEvent.activity_name == "" || currentEvent.activity_name == "none"))
                || (currentEvent.agent == "" || currentEvent.agent == "none"))
            {
                addEvt = false;
            }

            // Add only one appearance and one dissapearance per object
            if (currentEvent.predicate == "appear"){
                for (unsigned int prev = 0; prev < currentElement; prev++){
                    if (sto.vEvents[prev].predicate == "appear"
                        && sto.vEvents[prev].agent == currentEvent.agent){
                        addEvt = false;
                    }
                }
            }
            if (currentEvent.predicate == "dissappear"){
                for (unsigned int prev = 0; prev < currentElement; prev++){
                    if (sto.vEvents[prev].predicate == "dissappear"
                        && sto.vEvents[prev].agent == currentEvent.agent){
                        addEvt = false;
                    }
                }
            }
            addEvt &= osCurrent.str() != "";

            if (VERBOSE) cout << osCurrent.str();
            if (addEvt){
                string sentenceWithoutUnderscore = osCurrent.str();
                currentEvent.removeUnderscoreString(sentenceWithoutUnderscore);
                vsOutput.push_back(sentenceWithoutUnderscore);
            }
            cursor++;
        }
        // is narrration
        else {
            for (auto arg : currentEvent.vArgument){
                if (arg.first == "meaning"){
                    sto.meaningStory.push_back(arg.second);
                }
            }
        }
    }

    if (VERBOSE) cout << endl << endl;
    sto.sentenceStory = vsOutput;
}


string narrativeHandler::createMeaning(string agent, string predicate, string object, string recipient){
    ostringstream osMeaning;
    bool bO = object != "";
    bool bR = recipient != "";

    // at least agent and predicate:
    osMeaning << ", " << predicate << " " << agent << " ";
    if (bO) osMeaning << object << " ";
    if (bR) osMeaning << recipient << " ";

    osMeaning << "<o> [_-_-_-_-_-_-_-_][A-P-";
    bO ? osMeaning << "O-" : osMeaning << "_-";
    bR ? osMeaning << "R-" : osMeaning << "_-";
    osMeaning << "_-_-_-_][_-_-_-_-_-_-_-_] <o>";
    return osMeaning.str();
}


// compare a list of PAOR to the ones that could be used in another story
bool narrativeHandler::checkListPAOR(vector<string> vOriginal, vector<string> vCopy){

    if (vOriginal.size() != vCopy.size()){
        yInfo(" Error in narrativeHandler::checkListPAOR - different sizes of input");
        return false;
    }

    for (unsigned int i = 0; i < vOriginal.size(); i++){
        bool found = false;
        for (auto &PAIR : comparator){
            if (!found){
                if (PAIR.first == vOriginal[i]){
                    found = true;
                    if (PAIR.second != vCopy[i]){
                        return false;
                    }
                }
            }
        }
        if (!found){
            pair<string, string>  pTmp(vOriginal[i], vCopy[i]);
            comparator.push_back(pTmp);
        }
    }

    return true;
}


string narrativeHandler::adaptMeaning(string meaning){
    //    cout << "meaning: " << meaning;
    string str = meaning;
    for (unsigned int jj = 0; jj < comparator.size(); jj++){
        if (comparator[jj].first != ""){
            //            cout << " first: " << comparator[jj].first << " second: " << comparator[jj].second;
            size_t pos = 0;
            while ((pos = str.find(comparator[jj].first, pos)) != std::string::npos){
                str.replace(pos, comparator[jj].first.length(), comparator[jj].second);
                pos += comparator[jj].second.length();
            }
            //cout << "str is:" << str << endl;
        }
    }
    return str;
}


bool narrativeHandler::tellingStoryFromMeaning(story target){

    yInfo() << " start telling story from meaning";
    vector<string>    tmpStory;
    for (auto mean : target.meaningStory){
        tmpStory.push_back(iCub->getLRH()->meaningToSentence(mean));
    }
	yInfo() << " tmpStory size is: " << tmpStory.size();
	if (tmpStory.size() < 2)
	{
		return false;
	}


    for (auto sen : tmpStory){
        cout << "\t\t" << sen << endl;
    }
	return true;
}

bool narrativeHandler::narrate(){
	yInfo(" begin narrate");

	findStories(iMinInstance);

	story target = listStories[listStories.size() - 1];

	target.displayNarration();
	narrationToSpeech(target);

	yInfo(" Narration finished.");
	
	return true;
}

bool narrativeHandler::askNarrate(){
	yInfo(" begin askNarrate");

	findStories(iMinInstance);
	
	addNarrationToStory(listStories[listStories.size() - 1], true);
	
	yInfo() << " size of narration of story: " << listStories[listStories.size() - 1].meaningStory.size();
	yInfo() << "telling:";
	tellingStoryFromMeaning(listStories[listStories.size() - 1]);
	yInfo("display");
	listStories[listStories.size() - 1].displayNarration();



	return true;
}



bool narrativeHandler::narrationToSpeech(story target){

	cout << "narration to speech: " << target.humanNarration.size() << endl;

	if (target.humanNarration.size() > 2){
		cout << "begin narration to speech from human:" << endl;
		for (auto ii : target.humanNarration){
			cout << "\t to speech: " << ii << endl;
			iCub->say(ii);
		}
	}
	else{
		if (target.sentenceStory.size() > iThresholdSentence){
			cout << endl << "begin display narration to speech of story: " << counter << " with " << target.vEvents.size() << " events and " << target.sentenceStory.size() << " sentence." << endl;

			bool removeFirst = true;
			for (auto itSt : target.sentenceStory){
				cout << "\t to speech: " << itSt;
				if (!removeFirst) iCub->say(itSt);
				removeFirst = false;
			}


			cout << endl << endl;
		}
	}


	return true;
}
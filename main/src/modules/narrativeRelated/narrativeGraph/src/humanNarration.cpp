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
using namespace discourseform;
using namespace storygraph;


/*
*  Add through the speechRecog a human spoken narration to an existing story
*
*/
void narrativeHandler::addNarrationToStory(story &target, bool overWrite){
    if (target.meaningStory.size() != 0){
        if (overWrite){
            yWarning(" in narrativeHandler::addNarrationToStory - already an existing narration - overWriting is on");
        }
        else{
            yWarning(" in narrativeHandler::addNarrationToStory - already an existing narration - overWriting is off | aborting");
            return;
        }
    }

    bool storyOnGoing = true;
    ostringstream osError;
    Bottle bOutput;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning; // send the information of recall to the abmReasoning

    vector<string> vNewStory;


    while (storyOnGoing){
        // while the narration is not finished
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarNarration), 20, false, true);

        if (bRecognized.get(0).asInt() == 0)
        {
            yError() << " error in narrativeHandler::addNarrationToStory | Error in speechRecog";
            return;
        }

        bAnswer = *bRecognized.get(1).asList();
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)
        string sSentence = bAnswer.get(0).asString();

        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo("stop called");
            storyOnGoing = false;
        }
        else{
            cout << "confirmation: " << sSentence << endl;

            bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarYesNo), 20, false, false);

            if (bRecognized.get(1).asList()->get(0).toString() == "yes"){
                vNewStory.push_back(sSentence);
            }
        }
    }

    cout << endl << "human said: " << endl;
    for (auto &phrase : vNewStory){
        cout << phrase << endl;
    }

    target.humanNarration = vNewStory;

    yInfo(" End of human narration, starting to create meaning with LRH");
    if (true){   // to change to lrh
        for (auto sentence : target.humanNarration){
            removeUnderscoreString(sentence);
            string meaning = iCub->getLRH()->SentenceToMeaning(sentence);
            enrichMeaning(meaning, sentence);
            target.meaningStory.push_back(meaning);
        }
    }
    yInfo(" human narration meanings are:");
    for (auto meani : target.meaningStory){
        cout << "\t" << meani << endl;
    }

    // recording narration
    recordNarrationABM(target);

    yInfo(" END add narration");

}


void narrativeHandler::addTextNarration(story &target, bool overWrite){
    cout << " add text narration " << endl;
}


void narrativeHandler::addAutoNarration(story &target, int iScena, bool overWrite){
    cout << " add auto narration, scenario:  " << iScena << endl;

    target.humanNarration = listAutoScenarios[iScena];

    yInfo(" End of human narration, starting to create meaning with LRH");
    if (true){   // to change to lrh
        for (auto sentence : target.humanNarration){
            removeUnderscoreString(sentence);
            string meaning = iCub->getLRH()->SentenceToMeaning(sentence);
            enrichMeaning(meaning, sentence);
            target.meaningStory.push_back(meaning);
        }
    }
    yInfo(" human narration meanings are:");
    for (auto meani : target.meaningStory){
        cout << "\t" << meani << endl;
    }

    // recording narration
    //    recordNarrationABM(target);

}


// take a meaning under the format: OCW, OCW OCW, OCW OCW, P1 P2 A2 O3 ... and return: , OCW1 OCW2 , OCW3 OCW4 OCW5 <o> [_-_-_-_-_-_-_-_][A-P-_-_-_-_-_-_][A-_-P-O-_-_-_-_] <o>
void narrativeHandler::enrichMeaning(string &meaning, string sentence){
    //if,could I,ask I John,give John it me,P1 P2 A2 P3 A3 O3 P4 A4 O4 R4
    //if I could ask John to give it to me
    //[P-_-_-_-_-_-_-_][_ - A - P - _ - _ - _ - _ - _][_ - A - _ - P - R - _ - _ - _][_ - A - _ - _ - _ - P - O - R]


    // decompose the sentence into words
    istringstream iss(sentence);
    vector<string> wordsSentence;
    copy(istream_iterator<string>(iss),
        istream_iterator<string>(),
        back_inserter(wordsSentence));

    //    cout << "         BEGIN ENRICH         " << endl;

    //yInfo() << " sentence: " << sentence;
    //yInfo() << " meaning: " << meaning;

    // This is to print the vector

    //yInfo(" sentence decomposed: ");
    //for (auto iter : wordsSentence)
    //{
    //    cout << iter << " ";
    //}
    //cout << endl;

    string delimiter = ",";
    size_t pos = 0;
    string token;
    string s = meaning;
    ostringstream osMeaning;
    int iPAORWordsInProp = 0;
    bool isFirst = true;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        if (!isFirst) { osMeaning << " , "; }
        osMeaning << token;
        iPAORWordsInProp++;
        s.erase(0, pos + delimiter.length());
        //cout << "\t meaning is: " << osMeaning.str() << " - iPAORWordsInProp " << iPAORWordsInProp << endl;
        isFirst = false;
    }
    //cout << s << endl;

    vector<string>  meaningParsed;
    // meaning parsed by spaces and colon
    stringstream stringStream(meaning);
    string line;
    while (std::getline(stringStream, line))
    {
        std::size_t prev = 0, pos;
        while ((pos = line.find_first_of(" ,", prev)) != std::string::npos)
        {
            if (pos > prev)
                meaningParsed.push_back(line.substr(prev, pos - prev));
            prev = pos + 1;
        }
        if (prev < line.length())
            meaningParsed.push_back(line.substr(prev, std::string::npos));
    }

    // remove PAOR from meaningParsed:
    vector<string> meaningWords = vector<string>(meaningParsed.begin(), meaningParsed.begin() + meaningParsed.size() / 2);
    vector<string> meaningPAOR = vector<string>(meaningParsed.begin() + meaningParsed.size() / 2, meaningParsed.end());

    //yInfo() << " meaning meaningWords is:";
    //for (auto it : meaningWords){
    //    cout << it << " ";
    //}
    //cout << endl;
    //yInfo() << " meaning meaningPAOR is:";
    //for (auto it : meaningPAOR){
    //    cout << it << " ";
    //}


    // get the number of OCW:
    vector<string> OCW;
    for (auto word : wordsSentence){
        if (isIn(meaningParsed, word)){
            OCW.push_back(word);
        }
    }
    //cout << endl;
    //yInfo() << " OCW is:";
    //for (auto it : OCW){
    //    cout << it << " ";
    //}
    int iPAOR = meaningPAOR.size();
    osMeaning << " <o> ";

    int iCurrentPAOR = 1;
    vector<string> currentProp;
    for (auto kk : OCW){
        currentProp.push_back("_");
    }

    for (int ii = 0; ii < iPAOR; ii++){

        // if we change the proposition
        string current = meaningPAOR[ii];
        if (int(current.at(1) - '0') != iCurrentPAOR){

            // Fill the meaning
            osMeaning << "[";
            bool isFirst = true;
            for (auto kk : currentProp){
                if (!isFirst) osMeaning << "-";
                osMeaning << kk;
                isFirst = false;
            }
            osMeaning << "]";

            // clear the current one
            currentProp.clear();
            for (auto kk : OCW){
                currentProp.push_back("_");
            }
        }
        // Find the position in the OCW and add the role in the current prop
        for (unsigned int jj = 0; jj < OCW.size(); jj++){
            if (meaningWords[ii] == OCW[jj]){
                currentProp[jj] = current.at(0);
            }
        }
        iCurrentPAOR = int(current.at(1) - '0');
        //yInfo() << " osMeaning: " << osMeaning.str();
    }
    osMeaning << "[";
    isFirst = true;
    for (auto kk : currentProp){
        if (!isFirst) osMeaning << "-";
        osMeaning << kk;
        isFirst = false;
    }

    osMeaning << "] <o>";


    // if no narrative word
    if (iPAORWordsInProp == iCurrentPAOR - 1){
        //yInfo("   adding a coma");
        meaning = "," + osMeaning.str();
    }
    else{
        meaning = osMeaning.str();
    }


    //cout << "meaning is: " << meaning << endl;
    //cout << "result is: " <<  iCub->getLRH()->meaningToSentence(meaning) << endl;

    //meaning = osMeaning.str();
    //cout << endl;

}


/*
* Get the human narration and record it into ABM and link it to the corresponding story
*/
void narrativeHandler::recordNarrationABM(story &target){

    // each sentence is recorded with the corresponding instance from the target and the corresponding order

    yInfo(" BEGIN recording in ABM");
    if (target.humanNarration.size() != target.meaningStory.size()){
        yWarning(" in narrativeHandler::recordNarrationABM : meaning and narration have different size. Won't be recorded");
        return;
    }

    for (unsigned int ii = 0; ii < target.humanNarration.size(); ii++){
        if (iCub->getABMClient()->Connect())
        {
            std::list<std::pair<string, string> > lArgument;
            lArgument.push_back(pair<string, string>(target.meaningStory[ii], "meaning"));
            lArgument.push_back(pair<string, string>(target.humanNarration[ii], "sentence"));
            lArgument.push_back(pair<string, string>(to_string(target.viInstances[0]), "story"));
            lArgument.push_back(pair<string, string>(to_string(ii), "rank"));
            iCub->getABMClient()->sendActivity("action",
                "narration",
                "narration",
                lArgument,
                true);
        }
    }
    yInfo(" END recording in ABM");

}




/*
* Goes from a form to a meaning
* Input is a narrative story
* Check if another similar story exists
*/
bool narrativeHandler::narrationToMeaning(story &target){

    cout << "------------------------------" << endl;
    cout << "BEGIN NARRATION TO MEANING" << endl;

    // story between Robert and Larry
    //vector<string> storyToTest;
    //storyToTest.push_back("Robert wanted to get the hat");
    //storyToTest.push_back("but he failed to grasp it");
    //storyToTest.push_back("because it laid outofreach");
    //storyToTest.push_back("so he found a different action");
    //storyToTest.push_back("if he could ask Larry to give it to him");
    //storyToTest.push_back("then Larry would give it to him");
    //storyToTest.push_back("so he asked Larry to give it to him");
    //storyToTest.push_back("and Larry gave it to him");
    //storyToTest.push_back("Robert has now the hat");

    //vector<string> meaningToTest;
    //meaningToTest.push_back(", wanted Robert , get Robert hat <o> [_-_-_-_-_-_-_-_][A-P-_-_-_-_-_-_][A-_-P-O-_-_-_-_] <o>");
    //meaningToTest.push_back("but , failed Robert , grasp Robert it <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-O-_-_-_] <o>");
    //meaningToTest.push_back("because , laid it outofreach <o> [P-_-_-_-_-_-_-_][_-A-P-R-_-_-_-_] <o>");
    //meaningToTest.push_back("so , found Robert action different <o> [P-_-_-_-_-_-_-_][_-A-P-R-O-_-_-_] <o>");
    //meaningToTest.push_back("if , could Robert , ask Robert Larry , give Larry it him <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-R-_-_-_][_-A-_-_-_-P-O-R] <o>");
    //meaningToTest.push_back("then , would Larry , give Larry it him <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-O-R-_-_] <o>");
    //meaningToTest.push_back("so , asked Robert Larry , give Larry it him <o> [P-_-_-_-_-_-_-_][_-A-P-R-_-_-_-_][_-_-_-A-P-O-R-_] <o>");
    //meaningToTest.push_back("and, gave Larry it him <o> [_-_-_-_-_-_-_-_][A-P-O-R-_-_-_-_] <o>");
    //meaningToTest.push_back("now , have Robert hat <o> [_-_-P-_-_-_-_-_][A-P-_-O-_-_-_-_] <o>");


    //    target.humanNarration = storyToTest;
    //    target.meaningStory = meaningToTest;

    //    target.viInstances.push_back(5000);

    //    yInfo() << "BEGIN compareNarration: start to compare Narration from target: " << target.counter;
    for (auto& currentStory : listStories){
        comparator.clear();
        if (currentStory.counter != target.counter){ // not comparing the target to itself
            if (target.meaningStory.size() <= currentStory.meaningStory.size()){ //if there is not more in the target than in the current
                unsigned int K = 0; // possibility to pass narration
                bool stillOk = true;
                unsigned int cursor = 0;

                for (auto& tarMea : target.meaningStory){  // for each event of the target
                    K = cursor;
                    bool found = false;
                    if (stillOk){
                        for (unsigned int j = K; j < currentStory.meaningStory.size(); j++){
                            if (!found){
                                string curMea = currentStory.meaningStory[j];
                                vector<string>  vOriginal, vCopy;

                                string delimiter = "<o>";

                                // filling vOriginal:
                                string curMeaLimited = curMea.substr(0, curMea.find(delimiter));
                                string tarMeaLimited = tarMea.substr(0, tarMea.find(delimiter));

                                string token;
                                // current story
                                size_t pos = 0;
                                size_t prev = 0;
                                while ((pos = curMeaLimited.find_first_of(" ,", prev)) != string::npos)
                                {
                                    if (pos > prev)
                                        vOriginal.push_back(curMeaLimited.substr(prev, pos - prev));
                                    prev = pos + 1;
                                }
                                if (prev < curMeaLimited.length())
                                    vOriginal.push_back(curMeaLimited.substr(prev, string::npos));


                                //target story
                                pos = 0;
                                prev = 0;
                                while ((pos = tarMeaLimited.find_first_of(" ,", prev)) != string::npos)
                                {
                                    if (pos > prev)
                                        vCopy.push_back(tarMeaLimited.substr(prev, pos - prev));
                                    prev = pos + 1;
                                }
                                if (prev < tarMeaLimited.length())
                                    vCopy.push_back(tarMeaLimited.substr(prev, string::npos));

                                //  cout << curMeaLimited << " - " << tarMeaLimited << endl;

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
                    if (currentStory.humanNarration.size() > 0){
                        yInfo("I found a matching story.");
                        cout << "CURRENT MATCHING: " << currentStory.counter << " starts at: " << currentStory.viInstances[0] << endl;
                        currentStory.displayNarration();
                        cout << " COMPARATOR " << endl;
                        for (auto co : comparator){
                            if (co.first != co.second){
                                cout << co.first << " - " << co.second << endl;
                            }
                        }
                        target.vEvents = currentStory.vEvents;
                        target.iBasedOn = currentStory.counter;
                        for (auto& tt : target.vEvents){
                            adaptMeaning(tt);
                        }
                        //target.displayNarration();
                    }
                }
            }
        }
    }

    cout << "TARGET STORY IS: " << endl;
    target.displayNarration();

    imagineStory(target);

    yInfo() << "\nEND compareNarration";




    return true;
}


evtStory narrativeHandler::adaptMeaning(evtStory& evtInput){

    // If PAOR are simple words
    for (unsigned int jj = 0; jj < comparator.size(); jj++){
        if (comparator[jj].first == "I") { comparator[jj].first = "iCub"; }

        if (comparator[jj].first == evtInput.predicate){
            //cout << "PREDICATE: first: " << comparator[jj].first << " second: " << comparator[jj].second << endl;
            evtInput.predicate = comparator[jj].second;
        }
        if (comparator[jj].first == evtInput.agent){
            //cout << "AGENT    : first: " << comparator[jj].first << " second: " << comparator[jj].second << endl;
            evtInput.agent = comparator[jj].second;
        }
        if (comparator[jj].first == evtInput.object){
            //cout << "OBJECT   : first: " << comparator[jj].first << " second: " << comparator[jj].second << endl;
            evtInput.object = comparator[jj].second;
        }
        if (comparator[jj].first == evtInput.recipient){
            //cout << "RECIPIENT: first: " << comparator[jj].first << " second: " << comparator[jj].second << endl;
            evtInput.recipient = comparator[jj].second;
        }

        // if object is a sentence
        size_t pos = evtInput.object.find(comparator[jj].first);

        if (pos != string::npos){
            //            cout << "OBJECT BEFORE: " << evtInput.object << endl;
            evtInput.object.erase(pos, comparator[jj].first.length());
            evtInput.object.insert(pos, comparator[jj].second);
            //            cout << "OBJECT NOW: " << evtInput.object << endl;
        }
    }

    // also for relations
    Bottle bNewRelations;
    // for each relation
    //    cout << "RELATION BEFORE: " << evtInput.bRelations.toString() << endl;
    for (int iRel = 0; iRel < evtInput.bRelations.size(); iRel++){
        Bottle bTemp = *evtInput.bRelations.get(iRel).asList();
        Bottle bNewTemp;

        // for each element of the relation
        for (int iSub = 0; iSub < bTemp.size(); iSub++){
            bool bFound = false;
            for (unsigned int jj = 0; jj < comparator.size(); jj++){
                if (comparator[jj].first == bTemp.get(iSub).asString()){
                    bFound = true;
                    bNewTemp.addString(comparator[jj].second);
                }
            }
            if (!bFound){
                bNewTemp.addString(bTemp.get(iSub).asString());
            }
        }
        bNewRelations.addList() = bNewTemp;
    }

    evtInput.bRelations = bNewRelations;
    //    cout << "RELATION AFTER: " << evtInput.bRelations.toString() << endl;



    return evtInput;
}

/*
* Imagine a story in the MOPC
*
*/
void narrativeHandler::imagineStory(story& target)
{
    cout << "-----------------------------------" << endl;
    yInfo(" Starting to imagine story.");
    cout << "STORY BASED ON: " << target.iBasedOn << endl;

    if (target.iBasedOn == -1){
        /*
        * Comes from a real story
        * Only need to imagine it through ABMReasoning
        */
        for (auto inst : target.vEvents)
        {
            if (!Network::isConnected(Port2abmReasoning.getName(), "/abmReasoning/rpc")){
                Network::connect(Port2abmReasoning.getName(), "/abmReasoning/rpc");
            }
            Bottle bSendReasoning;
            bSendReasoning.addString("imagine");
            bSendReasoning.addInt(inst.instance);
            Port2abmReasoning.write(bSendReasoning);
            Time::delay(1.0);
        }
        return;
    }
    // ELSE need to adapt memory to reconstruct story
    else {
        story originalStory;

        // if the iCub is not on of the agent, translate the scene of 1.5m
        bool bIcubInvolved = false;
        vector<int>  vInstanceTarget;

        for (auto& currentStory : listStories){
            if (currentStory.counter == target.iBasedOn){ // not comparing the target to itself
                originalStory = currentStory;
            }
        }
        cout << "CHECK ORIGINAL: counter " << originalStory.counter << " starts at: " << originalStory.viInstances[0] << endl;

        ostringstream osRequestEntities;
        osRequestEntities << "select distinct object.instance, object.name, object.position, contentopc.subtype from object, contentopc where (object.presence = 't' and object.instance in (";
        ostringstream osRequestRelations;
        osRequestRelations << "select distinct subject, verb, object, instance from relation where instance in (";
        bool bFirst = true;
        for (auto inst : originalStory.viInstances){
            if (bFirst){
                bFirst = false;
            }
            else{
                osRequestRelations << ",";
                osRequestEntities << ",";
            }
            osRequestRelations << inst;
            osRequestEntities << inst;
        }
        osRequestEntities << ") and object.instance = contentopc.instance and object.opcid = contentopc.opcid) order by object.instance";
        osRequestRelations << ") order by instance";

        Bottle bAllInstances = iCub->getABMClient()->requestFromString(osRequestEntities.str());
        Bottle bAllRelations = iCub->getABMClient()->requestFromString(osRequestRelations.str());

        Bottle bNewOpc;
        for (int iRel = 0; iRel < bAllInstances.size(); iRel++){
            Bottle bTemp = *bAllInstances.get(iRel).asList();
            Bottle bNewTemp;
            // for each element of the relation
            for (int iSub = 0; iSub < bTemp.size(); iSub++){

                string stmp = bTemp.get(iSub).asString();
                for (unsigned int jj = 0; jj < comparator.size(); jj++){
                    if (comparator[jj].first == bTemp.get(iSub).asString()){
                        stmp = comparator[jj].second;
                    }
                }
                bIcubInvolved |= (stmp == "iCub" || stmp == "icub");
                bNewTemp.addString("iCub");
            }
            bNewOpc.addList() = bNewTemp;

            cout << bNewTemp.toString() << endl;

            //add the instance
            if (find(vInstanceTarget.begin(), vInstanceTarget.end(), atoi(bNewTemp.get(0).asString().c_str())) == vInstanceTarget.end()) {
                // someName not in name, add it
                vInstanceTarget.push_back(atoi(bNewTemp.get(0).asString().c_str()));
            }
        }


        cout << "bIcubInvolved: " << bIcubInvolved << endl;

        // THRESHOLD in -X
        double dThresholdX;
        bIcubInvolved ? dThresholdX = 0.0 : dThresholdX = -0.7;
        cleanMental();

        vector<string> vInvolvedAgents;
        vector<string> vInvolvedObjects;

        for (int ii = 0; ii < bNewOpc.size(); ii++){
            Bottle bTemp = *bNewOpc.get(ii).asList();

            if (bTemp.get(3).toString() == "agent"){
                if (bTemp.get(1).asString() != "iCub" && bTemp.get(1).asString() != "icub"){
                    if (find(vInvolvedAgents.begin(), vInvolvedAgents.end(), bTemp.get(1).asString()) == vInvolvedAgents.end()) {
                        // someName not in name, add it
                        vInvolvedAgents.push_back(bTemp.get(1).asString().c_str());
                    }
                }
            }

            if (bTemp.get(3).toString() == "object"){
                if (find(vInvolvedObjects.begin(), vInvolvedObjects.end(), bTemp.get(1).asString()) == vInvolvedObjects.end()) {
                    // someName not in name, add it
                    vInvolvedObjects.push_back(bTemp.get(1).asString().c_str());
                }
            }
        }

        for (auto age : vInvolvedAgents){
            Agent *ag = mentalOPC->addEntity<Agent>(age);
            ag->m_present = 0;
            ag->m_color[0] = Random::uniform(0, 80);
            ag->m_color[1] = Random::uniform(180, 250);
            ag->m_color[2] = Random::uniform(80, 180);
            mentalOPC->commit(ag);
        }

        for (auto obj : vInvolvedObjects){
            Object *ob = mentalOPC->addEntity<Object>(obj);
            ob->m_present = 0;
            ob->m_color[0] = Random::uniform(100, 180);
            ob->m_color[1] = Random::uniform(0, 80);
            ob->m_color[2] = Random::uniform(180, 250);
            mentalOPC->commit(ob);
        }


        for (auto inst : vInstanceTarget){
            cout << "INSTANCE: " << inst << endl;

            for (auto itSt : target.vEvents){
                if (itSt.instance == inst){
                    itSt.removeUnderscore();
                    cout << "\t A:" << itSt.agent;
                    cout << "\t P:" << itSt.predicate;
                    cout << "\t O:" << itSt.object;
                    cout << "\t R:" << itSt.recipient << endl;
                }
            }

            mentalOPC->checkout();

            for (int ii = 0; ii < bNewOpc.size(); ii++){
                Bottle bTemp = *bNewOpc.get(ii).asList();

                // if it is the good instance
                if (inst == atoi(bTemp.get(0).asString().c_str())){


                    vector<double> position;

                    string spos = bTemp.get(2).toString();
                    spos.erase(remove(spos.begin(), spos.end(), '\"'), spos.end());
                    spos.erase(remove(spos.begin(), spos.end(), '}'), spos.end());
                    spos.erase(remove(spos.begin(), spos.end(), '{'), spos.end());

                    string token;
                    // current story
                    size_t pos = 0;
                    while ((pos = spos.find(",")) != string::npos) {
                        token = spos.substr(0, pos);
                        position.push_back(atof(token.c_str()));
                        spos.erase(0, pos + 1);
                    }
                    position.push_back(atof(spos.c_str()));

                    // add the entity:
                    if (bTemp.get(3).toString() == "agent"){
                        Agent *ag = dynamic_cast<Agent*>(mentalOPC->getEntity(bTemp.get(1).toString()));

                        ag->m_ego_position[0] = position[0] + dThresholdX;
                        ag->m_ego_position[1] = position[1];

                        ag->m_present = 1;

                        mentalOPC->commit(ag);
                    }
                    // add the entity:
                    if (bTemp.get(3).toString() == "object"){
                        Object *ob = dynamic_cast<Object*>(mentalOPC->getEntity(bTemp.get(1).toString()));

                        ob->m_ego_position[0] = position[0] + dThresholdX;
                        ob->m_ego_position[1] = position[1];
                        ob->m_ego_position[2] = position[2];

                        ob->m_present = 1;

                        mentalOPC->commit(ob);
                    }
                }
                mentalOPC->commit();
            }

            Time::delay(1.5);
        }
    }
}

/*
* Clean the mentalOPC and the GUI
*/
void narrativeHandler::cleanMental(){
    mentalOPC->checkout();
    //clean GUI :
    list<Entity*> lMental = mentalOPC->EntitiesCache();
    for (list<Entity*>::iterator it_E = lMental.begin(); it_E != lMental.end(); it_E++)
    {
        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_OBJECT)   {
            Object *Ob = dynamic_cast<Object*>(*it_E);
            Ob->m_present = 0.0;
        }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_AGENT)    {
            Agent *Ag = dynamic_cast<Agent*>(*it_E);
            Ag->m_present = 0.0;
        }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_RTOBJECT) {
            RTObject *Rt = dynamic_cast<RTObject*>(*it_E);
            Rt->m_present = 0.0;
        }
    }
    mentalOPC->commit();
    Time::delay(0.1);
    mentalOPC->clear();
}



void narrativeHandler::listeningStory(){
    story target;
    target.counter = counter;
    counter++;

    addNarrationToStory(target);

    narrationToMeaning(target);
}




/*
* Get already written stories in a conf folder
* (bypass the speech part)
*/
void narrativeHandler::initializeScenarios(Bottle bNarrations, ResourceFinder &rf){

    for (int ii = 0; ii < bNarrations.size(); ii++){
        cout << "narration number: " << ii + 1 << ": " << bNarrations.get(ii).asString() << endl;
        string currentNarration = rf.findFileByName(bNarrations.get(ii).asString());

        ifstream infile;
        string currentLine;
        infile.open(currentNarration);
        vector<string>  vsNarration;
        while (!infile.eof()){
            getline(infile, currentLine);
            if (currentLine.find("#")){
                cout << currentLine << endl;
                vsNarration.push_back(currentLine);
            }
        }
        infile.close();

        listAutoScenarios[ii + 1] = vsNarration;
        cout << endl;
    }
}


/*
* Get already written meaning for stories in a conf folder
* (bypass the speech part then the lrh part)
*/
void narrativeHandler::initializeMeaning(Bottle bMeaning, ResourceFinder &rf){

    for (int ii = 0; ii < bMeaning.size(); ii++){
        int count = 0;
        string currentNarration = rf.findFileByName(bMeaning.get(ii).asString());
        ifstream infile;
        string currentLine;
        infile.open(currentNarration);
        vector<string>  vsMeaning;
        while (!infile.eof()){
            getline(infile, currentLine);
            if (currentLine.find("#")){
                count++;
                //cout << currentLine << endl;
                vsMeaning.push_back(currentLine);
            }
        }
        infile.close();
        cout << "meaning number: " << ii + 1 << ": " << bMeaning.get(ii).asString() << " ... " << count << " meaning(s) found." << endl;

        listAutoMeaning[ii + 1] = vsMeaning;
    }

    yInfo() << "Initialisation of meaning: " << bMeaning.size() << " found.";
}

/*
* Test if the scenarios in the conf folder can be understood by lrh
*
*/
void narrativeHandler::checkScenarios(int iScena){
    if (iScena == -1){
        vector<pair<int, int>> vpiRes;
        for (map<int, vector<string>>::iterator itMp = listAutoScenarios.begin(); itMp != listAutoScenarios.end(); itMp++){
            cout << endl << "narration number: " << itMp->first << endl;
            int iSuccss = 0, iTot = 0;
            for (vector<string>::iterator itLi = itMp->second.begin(); itLi != itMp->second.end(); itLi++){
                //cout << "\t" << *itLi;
                //                string meaning = iCub->getLRH()->SentenceToMeaning(*itLi);
                if (iCub->getLRH()->SentenceToMeaning(*itLi) != "") { iSuccss++; }
                iTot++;
                //cout << "\t" << meaning << endl;
            }
            vpiRes.push_back(pair<int, int>(iSuccss, iTot));
        }
        cout << "Summary of scenarios check:" << endl;
        int ii = 1;
        for (vector<pair<int, int>>::iterator itV = vpiRes.begin();
            itV != vpiRes.end();
            itV++){
            cout << "\tScenario: " << ii << ": " << itV->first << "/" << itV->second << endl;
            ii++;
        }
    }
    else {
        if (iScena < listAutoScenarios.size())
        {
            int iSuccss = 0, iTot = 0;
            cout << endl << "narration number: " << iScena << endl;
            for (vector<string>::iterator itLi = listAutoScenarios[iScena].begin();
                itLi != listAutoScenarios[iScena].end();
                itLi++){
                //                string meaning = iCub->getLRH()->SentenceToMeaning(*itLi);
                if (iCub->getLRH()->SentenceToMeaning(*itLi) != "") { iSuccss++; }
                iTot++;
            }
            cout << "Summary of scenarios check:" << endl;
            cout << "\tScenario: " << iScena << ": " << iSuccss << "/" << iTot << endl;
        }
    }
}


/*
* Try to link the meaning of a narration to a SM
*
*/
void narrativeHandler::linkNarrationScenario(int iNarration, int iScenario){

    // check sizes:
    if (iNarration > listAutoScenarios.size() || iScenario > listStories.size()){
        yWarning(" in narrativeHandler::linkNarrationScenario - index out or range.");
        return;
    }

    loadSM(iScenario);

    // getting narration
    if (iNarration < listAutoScenarios.size())
    {
        int iSuccss = 0, iTot = 0;
        cout << endl << "Narration number: " << iNarration << endl;
        for (vector<string>::iterator itLi = listAutoScenarios[iNarration].begin();
            itLi != listAutoScenarios[iNarration].end();
            itLi++){
            //                string meaning = iCub->getLRH()->SentenceToMeaning(*itLi);
            if (iCub->getLRH()->SentenceToMeaning(*itLi) != "") { iSuccss++; }
            iTot++;
        }
        cout << "Summary of scenarios check:" << endl;

        cout << "\tScenario: " << iNarration << ": " << iSuccss << "/" << iTot << endl;
    }
}


/*
* Try to link the meaning of a narration to a SM
*
*/
string narrativeHandler::linkMeaningScenario(int iMeaning, int iScenario){

    // check sizes:
    if (iMeaning > listAutoMeaning.size() || iScenario > listStories.size()){
        yWarning(" in narrativeHandler::linkMeaningScenario - index out or range.");
        return " in narrativeHandler::linkMeaningScenario - index out or range.";
    }

    bool display = false;


    listStories.at(iScenario).displayNarration();


    // getting scenario
    loadSM(iScenario);

    cout << "in linkMeaningScenario: " << endl;
    int iLost = 0;

    //    sm.displayEvent();

    ofstream IGARFfile;
    IGARFfile.open(sIGARFfile);
    sm.ABMtoSM(listStories.at(iScenario), IGARFfile);
    IGARFfile.close();
    // getting narration
    if (iMeaning <= listAutoMeaning.size())
    {
        cout << endl << "meaning number: " << iMeaning << endl;
        for (vector<string>::iterator itLi = listAutoMeaning[iMeaning].begin();
            itLi != listAutoMeaning[iMeaning].end();
            itLi++){
            //                string meaning = iCub->getLRH()->SentenceToMeaning(*itLi);
            cout << *itLi << endl;
        }
        cout << "initialized." << endl;
    }

    meaningDiscourse MD;
    // creatino of the MD from the discourse
    string sReturn = MD.meaningToDiscourseForm(listAutoMeaning[iMeaning]);

    int indice = 0;
    // check for each proposition, if it can be asociated to a event of the Scenario

    int count = 0;

    if (display){

        cout << "DL" << endl;
        for (vector<sDiscourseLink>::iterator itDL = sm.vDiscourseLinks.begin(); itDL != sm.vDiscourseLinks.end(); itDL++){
            cout << "[" << count << "] [" << itDL->word << "] [" << itDL->fromEvt.iIGARF << "-" << itDL->fromEvt.cPart << "-" << itDL->fromEvt.iRel << "] to "
                << "[" << itDL->toEvt.iIGARF << "-" << itDL->toEvt.cPart << "-" << itDL->toEvt.iRel << "]" << endl;
            count++;
        }

        count = 0;
        cout << "actions: " << endl;
        for (vector < sActionEvt >::iterator itIG = sm.vActionEvts.begin(); itIG != sm.vActionEvts.end(); itIG++)
        {
            cout << "[" << count << "] [" << itIG->predicate << "-" << itIG->agent << "-" << itIG->object << "-" << itIG->recipient << "]" << endl;
            count++;
        }

        cout << "Relation: " << endl;

        for (vector < sRelation >::iterator itIG = sm.vRelations.begin(); itIG != sm.vRelations.end(); itIG++)
        {
            cout << "[" << count << "] [" << itIG->subject << "-" << itIG->object << "-" << itIG->verb << "]" << endl;
            count++;
        }
        count = 0;
        cout << "init finished" << endl;
    }

    for (vector<meaningSentence>::iterator level1 = MD.meanings.vDiscourse.begin();
        level1 != MD.meanings.vDiscourse.end();
        level1++
        )
    { // for each sentence of the discourse
        DFW *currentDFW;
        bool isMultiple = false; // if a sentence is multiple (with a DFW)
        int iPreposition = 0;   // get the order of the preposition in the sentence
        int iNbPreposition = level1->vSentence.size();  // nb of preposition in the sentence
        bool isDFW = level1->vSentence[0].vOCW.size() == 1;
        vector<EVT_IGARF> singleIGARF;
        vector<EVT_IGARF>  doubleBefore;
        vector<EVT_IGARF>  doubleAfter;

        bool bAllAction = true;  //depend of the score of findBest

        if (display) cout << "---------------------------------------------------------------------\n" << "Sentence full is size: " << iNbPreposition << " and contain DFW: " << isDFW << endl;


        for (vector<meaningProposition>::iterator level2 = level1->vSentence.begin();
            level2 != level1->vSentence.end();
            level2++){  // for each preposition of the sentence
            if (true){
                cout << " [ ";
                for (int iWord = 0; iWord < level2->vOCW.size(); iWord++){

                    cout << level2->vOCW[iWord] << " ";
                }
                cout << "]  ";
            }

            int iScore = 0;
            if (isDFW && iPreposition == 0){ // only one OCW: DFW
                //cout << "\t\t\t sentence has a DFW." << endl;
                string nameDFW = level2->vOCW[0];
                isMultiple = true;
                bool found = false;
                for (vector<DFW>::iterator itDFW = vDFW.begin(); itDFW != vDFW.end(); itDFW++){
                    if (!found && itDFW->sName == nameDFW){
                        found = true;
                        currentDFW = &(*itDFW);
                        if (display) cout << " found existing DFW: " << currentDFW->sName << endl;
                    }
                }
                if (!found) {
                    currentDFW = new DFW(nameDFW);
                    vDFW.push_back(*currentDFW);
                    currentDFW = &vDFW[vDFW.size() - 1];
                    if (display) cout << " creating new DFW: " << currentDFW->sName << endl;
                }
            }
            else{
                vector<sKeyMean> vkTmp = sm.findBest(level2->vOCW, iScore);

                if (vkTmp.size() == 0){
                    iLost++;
                    yWarning() << " in narrativeGraph::humanNarration.cpp::linkMeaningScenario:: findBest : no target found.";
                    yWarning() << level1->getSentence();
                }

                bAllAction &= !(iScore <= iThresholdScoreIGARFPAOR && iPreposition != 0);   // all action except the fisrt one need to be found

                if (bAllAction){     // if found;
                    for (int kk = 0; kk < vkTmp.size(); kk++)
                    {
                        sKeyMean kTmp = vkTmp[kk];
                        if (display) cout << "\t result find: " << (kTmp.toString());
                        int iIg, iL;
                        if (kTmp.iIGARF != -1){
                            //currentIGARF = sm.vIGARF[kTmp.iIGARF];
                            iIg = sm.vIGARF[kTmp.iIGARF].iAction;
                            iL = sm.vIGARF[kTmp.iIGARF].iLevel;

                            if (display){

                                //vIGARF.at(j).vGoal.at(k)
                                if (kTmp.cPart == 'A'){
                                    cout << "\t  " << iIg << " - " << iL
                                        << " [" << sm.vActionEvts[iIg].agent
                                        << "-" << sm.vActionEvts[iIg].predicate
                                        << "-" << sm.vActionEvts[iIg].object
                                        << "-" << sm.vActionEvts[iIg].recipient << "]" << endl;
                                }
                                else if (kTmp.cPart == 'G'){
                                    cout << "\t  " << iIg << " - " << iL
                                        << " [" << sm.vRelations[sm.vIGARF[kTmp.iIGARF].vGoal[kTmp.iRel]].subject
                                        << "-" << sm.vRelations[sm.vIGARF[kTmp.iIGARF].vGoal[kTmp.iRel]].verb
                                        << "-" << sm.vRelations[sm.vIGARF[kTmp.iIGARF].vGoal[kTmp.iRel]].object << "]" << endl;
                                }
                                else if (kTmp.cPart == 'I'){
                                    cout << "\t  " << iIg << " - " << iL
                                        << " [" << sm.vRelations[sm.vIGARF[kTmp.iIGARF].vInitState[kTmp.iRel]].subject
                                        << "-" << sm.vRelations[sm.vIGARF[kTmp.iIGARF].vInitState[kTmp.iRel]].verb
                                        << "-" << sm.vRelations[sm.vIGARF[kTmp.iIGARF].vInitState[kTmp.iRel]].object << "]" << endl;
                                }
                                else if (kTmp.cPart == 'F'){
                                    cout << "\t  " << iIg << " - " << iL
                                        << " [" << sm.vRelations[sm.vIGARF[kTmp.iIGARF].vFinalState[kTmp.iRel]].subject
                                        << "-" << sm.vRelations[sm.vIGARF[kTmp.iIGARF].vFinalState[kTmp.iRel]].verb
                                        << "-" << sm.vRelations[sm.vIGARF[kTmp.iIGARF].vFinalState[kTmp.iRel]].object << "]" << endl;
                                }
                                else if (kTmp.cPart == 'R'){
                                    cout << "\t  " << iIg << " - " << iL
                                        << " [" << sm.vActionEvts[sm.vIGARF[kTmp.iIGARF].iResult].agent
                                        << "-" << sm.vActionEvts[sm.vIGARF[kTmp.iIGARF].iResult].predicate
                                        << "-" << sm.vActionEvts[sm.vIGARF[kTmp.iIGARF].iResult].object
                                        << "-" << sm.vActionEvts[sm.vIGARF[kTmp.iIGARF].iResult].recipient << "]" << endl;
                                }
                            }
                        }

                        // double dI = iIg / (sm.vChronoIgarf.size() *1.0);
                        storygraph::EVT_IGARF evtKM(kTmp, iIg, iL);
                        sm.checkEVTIGARF(evtKM);

                        cout << "KM is: " << evtKM.toString() << endl;

                        if (iNbPreposition > 2){
                            if (iPreposition < 2){
                                doubleBefore.push_back(evtKM);
                            }
                            else{
                                doubleAfter.push_back(evtKM);
                            }
                        }
                        else{
                            singleIGARF.push_back(evtKM);
                        }

                    }
                }
            }
            //else {
            //    cout << "Action not recognized" << endl;
            //}

            iPreposition++;
        }  // end preposition

        if (bAllAction){
            if (isDFW && isMultiple){
                if (iNbPreposition <= 2){
                    //cout << "filling single vector ...";
                    for (auto Single : singleIGARF)
                    {
                        currentDFW->vSingleIGARF.push_back(Single);
                    }
                    //cout << " done !" << endl;
                }
                else{
                    if (display) cout << "filling double vector ... " << doubleBefore.size() << "*" << doubleAfter.size() << " ";
                    int doku = 0;
                    for (int iFirst = 0; iFirst < doubleBefore.size(); iFirst++){
                        pair<EVT_IGARF, EVT_IGARF>  kTmpDouble;
                        kTmpDouble.first = doubleBefore[iFirst];
                        for (int iSecond = 0; iSecond < doubleAfter.size(); iSecond++){
                            kTmpDouble.second = doubleAfter[iSecond];
                            currentDFW->vDoubleIGARF.push_back(kTmpDouble);
                            doku++;
                        }
                    }
                }
            }
        }
        //cout << "\t\t\t\t\end of sentence" << endl;
    } // end sentence
    //cout << "end of the loop, starting to display" << endl;

    displayDFW();

    //MD.print();
    sReturn += to_string(iLost) + " sentences lost.";
    return sReturn;

}




/*
* Get sentence from naive subject
*/
void narrativeHandler::initializeNaives(Bottle bNaives, ResourceFinder &rf){

    for (int ii = 0; ii < bNaives.size(); ii++){
        cout << "naive number: " << ii + 1 << ": " << bNaives.get(ii).asString() << " ... " << endl;;
        string currentNarration = rf.findFileByName(bNaives.get(ii).asString());

        ifstream infile;
        string currentLine;
        infile.open(currentNarration);
        vector<string>  vsNaives;
        while (!infile.eof()){
            getline(infile, currentLine);
            if (currentLine.find("#")){
                cout << currentLine << endl;
                if (currentLine != ""){
                    vsNaives.push_back(currentLine);
                }
            }
        }
        infile.close();

        listAutoNaives[ii + 1] = vsNaives;
    }

    yInfo() << "Initialisation of naives: " << bNaives.size() << " found.";
}


/*
* Send the sentence from the naive subjects to lrh to get it as PAOR
*/
void narrativeHandler::NaiveToPAOR(){
    vector<int> vFound;
    string tmpResp = "";
    yInfo(" Starting to change sentence from naive to PAOR");
    // for each scenario
    for (auto scenario : listAutoNaives){
        yInfo() << " New scenario: " << scenario.first;
        int count = 0;
        // for each sentence for the given scenario
        for (auto &sentence : scenario.second){
            string sBef = sentence;
            //send the sentence to lrh and get the result as PAOR
            //            cout << sentence;
            sentence = iCub->getLRH()->SentenceToMeaning(sentence, false);
            //           cout << "\t" << sentence << endl;
            if (sentence != "" && sentence != "none" && sentence != tmpResp){
                count++;
                cout << sBef << "\t->\t" << sentence << endl;
            }
            tmpResp = sentence;
        }
        vFound.push_back(count);
    }


    for (int ii = 0; ii < vFound.size(); ii++){
        cout << "scenario " << ii + 1 << " found: " << vFound[ii] << " meaning" << endl;
    }
}












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
        unsigned int pos = evtInput.object.find(comparator[jj].first);

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

    if (!mentalOPC->isConnected()){
        yWarning(" in Imagine story: mentalOPC not connected");
        return;
    }

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
                bNewTemp.addString(stmp);
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
    if (!mentalOPC->isConnected()){
        yWarning(" in cleanMental: mentalOPC not connected");
        return;
    }
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


    // story between Robert and Larry
    vector<string> storyToTest;
    storyToTest.push_back("Robert wanted to get the hat");
    storyToTest.push_back("but he failed to grasp it");
    storyToTest.push_back("because it laid outofreach");
    storyToTest.push_back("so he found a different action");
    storyToTest.push_back("if he could ask Larry to give it to him");
    storyToTest.push_back("then Larry would give it to him");
    storyToTest.push_back("so he asked Larry to give it to him");
    storyToTest.push_back("and Larry gave it to him");
    storyToTest.push_back("Robert has now the hat");

    vector<string> meaningToTest;
    meaningToTest.push_back(", wanted Robert , get Robert hat <o> [_-_-_-_-_-_-_-_][A-P-_-_-_-_-_-_][A-_-P-O-_-_-_-_] <o>");
    meaningToTest.push_back("but , failed Robert , grasp Robert it <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-O-_-_-_] <o>");
    meaningToTest.push_back("because , laid it outofreach <o> [P-_-_-_-_-_-_-_][_-A-P-R-_-_-_-_] <o>");
    meaningToTest.push_back("so , found Robert action different <o> [P-_-_-_-_-_-_-_][_-A-P-R-O-_-_-_] <o>");
    meaningToTest.push_back("if , could Robert , ask Robert Larry , give Larry it him <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-R-_-_-_][_-A-_-_-_-P-O-R] <o>");
    meaningToTest.push_back("then , would Larry , give Larry it him <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-O-R-_-_] <o>");
    meaningToTest.push_back("so , asked Robert Larry , give Larry it him <o> [P-_-_-_-_-_-_-_][_-A-P-R-_-_-_-_][_-_-_-A-P-O-R-_] <o>");
    meaningToTest.push_back("and, gave Larry it him <o> [P-_-_-_-_-_-_-_][P-A-P-O-R-_-_-_] <o>");
    meaningToTest.push_back("now , have Robert hat <o> [_-_-P-_-_-_-_-_][A-P-_-O-_-_-_-_] <o>");


    target.humanNarration = storyToTest;
    target.meaningStory = meaningToTest;

    //    addNarrationToStory(target);
    narrationToMeaning(target);

    imagineStory(target);
}
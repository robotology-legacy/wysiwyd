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


///< file for function concerning the Human-Robot Interaction

#include "narrativeHandler.h"
#include <stack>
#include <set>
#include <map>


using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;
using namespace discourseform;
using namespace storygraph;

/*
* Simple speech confirmation: yes is: "yes" "ok"
*
*/
bool narrativeHandler::speechConfirmation(){
    Bottle bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarYesNo), 20, false, false);

    if (bRecognized.get(0).asInt() == 0)
    {
        yError() << " error in narrativeHandler::questionHRI_DFW | Error in speechRecog";
        return false;
    }

    Bottle bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

    if (bAnswer.size() != 2){
        yError() << " error in narrativeHandler::questionHRI_DFW | Wrong size of response from recog ";
        return false;
    }

    return (bAnswer.get(0).toString() == "yes");
}




/*
*
*
*
*/
Bottle narrativeHandler::questionHRI_DFW(){
    yInfo("starting QUESTIONHRI_DFW");

    iCub->getRecogClient()->listen(false);

    Bottle bReturn;

    vector<string> vBegin;
    vBegin.push_back("I'll be glad to talk about what we did!");
    vBegin.push_back("Sure, what do you want to know?");


    vector<string> vConfuse;
    vConfuse.push_back("I don't know, sorry...");
    vConfuse.push_back("That's hard, I give up...");
    vConfuse.push_back("I have no idea, sorry");

    unsigned int randomIndex = rand() % vBegin.size();
    iCub->say(vBegin[randomIndex], false);

    vector < hriResponse > vResponses;
    vector < PAOR > vSaid;
    vector < tuple <Bottle, PAOR > >  vQuestions;

    vector<string> vConfirmation;
    vConfirmation.push_back("okay.");
    vConfirmation.push_back("I get it.");
    vConfirmation.push_back("I understand.");
    vConfirmation.push_back("Yes.");

    bool remember = false; // if asking to recall a situation
    bool listening = false; // is asked to the robot to listen to the human
    bool exit = false;

    while (!exit){
        yInfo() << "exit: " << exit;
        Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic; // semantic information of the content of the recognition
        bool getAnswer = false;

        while (!getAnswer && !exit){
            iCub->opc->checkout();
            iCub->lookAtPartner();
            cout << "Remember: " << remember << " | scenario: " << scenarioToRecall << endl;
            bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarQuestionDFW), 20, false, true);
            if (bRecognized.get(0).asInt() == 0)
            {
                yError() << " error in narrativeHandler::questionHRI_DFW | Error in speechRecog";
                bReturn.addString(" error in narrativeHandler::questionHRI_DFW | Error in speechRecog");
                return bReturn;
            }

            bAnswer = *bRecognized.get(1).asList();
            // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

            if (bAnswer.size() != 2){
                yError() << " error in narrativeHandler::questionHRI_DFW | Wrong size of response from recog ";
                bReturn.addString(" error in narrativeHandler::questionHRI_DFW | Wrong size of response from recog");
                return bReturn;
            }

            string sSentence = bAnswer.get(0).asString();
            bSemantic = *bAnswer.get(1).asList();

            if (bAnswer.get(0).asString() == "stop")
            {
                yInfo("stop called");
                iCub->say("You're welcome!");
                iCub->home();
                exit = true;
            }
            else{
                yInfo() << " confirmation: " << sSentence;
                getAnswer = true; // speechConfirmation();
                if (getAnswer){
                    ///<
                    if (bSemantic.get(0).asString() == "stop" && !listening){
                        exit = true;
                        cout << "Okay, bye !" << endl;
                        iCub->say("You're welcome!");
                        iCub->home();
                    }
                    else if (bSemantic.get(0).asString() == "Else"){
                        remember = false;
                        string picked = pickResponse(vResponses, vSaid);
                        if (picked == "none"){
                            iCub->say("Nothing else, sorry.");
                        }
                        else{
                            PAOR sPAOR;
                            vQuestions.push_back(tuple<Bottle, PAOR >(bSemantic, sPAOR));
                            iCub->say(picked);
                        }
                    }
                    else if (bSemantic.get(0).asString() == "Tell_you"){
                        remember = false;
                        listening = true;
                        iCub->say("Ok, tell me !");
                    }
                    else if (remember == true){
                        if (doYouRemember(sSentence)){
                            iCub->say("Yes, of course I remember !", false);

                            createNarration(vQuestions, scenarioToRecall, vSaid);
                            vResponses.clear();
                        }
                        else{
                            iCub->say("Hum, no, it doesn't ring me a bell.");
                        }
                        remember = false;

                    }
                    else if (listening){
                        if (bSemantic.get(0).asString() == "PAORsimple"
                            || bSemantic.get(0).asString() == "PAORdouble"){
                            // randomly pick a reaction.
                            randomIndex = rand() % vConfirmation.size();
                            iCub->say(vConfirmation[randomIndex]);
                        }
                        else if (bSemantic.get(0).asString() == "stop"){
                            listening = false;
                        }
                    }
                    else if (bSemantic.get(0).asString() == "When"){
                        yInfo() << "Setting remember true !";
                        iCub->say("hum hum?");
                        remember = true;
                    }
                    else{
                        remember = false;
                        //case sentence simple with DFW:
                        vResponses.clear();

                        PAOR sPAOR;
                        if (bSemantic.get(0).asString() == "What_happen_DFW_simple"){
                            vResponses = what_DFW_Simple(bSemantic, scenarioToRecall);
                        }
                        else if (bSemantic.get(0).asString() == "What_happen_DFW_double"){
                            vResponses = what_DFW_Double(bSemantic, sPAOR, scenarioToRecall);
                        }
                        else if (bSemantic.get(0).asString() == "Why"){
                            vResponses = whyPAOR(bSemantic, sPAOR, scenarioToRecall);
                        }
                        else if (bSemantic.get(0).asString() == "Why_is_that"){
                            if (vSaid.size()!=0){
                                vResponses = whyIsThat(vSaid[vSaid.size()-1], scenarioToRecall);
                            }

                        }
                        vQuestions.push_back(tuple<Bottle, PAOR>(bSemantic, sPAOR));

                        string picked = pickResponse(vResponses, vSaid);
                        if (picked == "none"){
                            int randomIndex = rand() % vConfuse.size();
                            iCub->say(vConfuse[randomIndex], true);
                        }
                        else{
                            iCub->say(picked, true);
                        }
                        yInfo() << "---- next question ----";
                    }
                }
            }
        }
    }
    yInfo("leaving HRI");
    iCub->getRecogClient()->listen(true);
    bReturn.addString("exited HRI nicely");
    return bReturn;
}

/*
* get a simple question form from recog, treat the question and return the result as Bottle
* bInput: ( What_happen_DFW_simple ( dfw_simple "dfw"))
* ie: (What_happen_DFW_simple(dfw_simple then))
*/
vector < hriResponse > narrativeHandler::what_DFW_Simple(Bottle bInput, int iScenario){
    yInfo() << " what_DFW_Simple launched: " << bInput.toString() << ", scenario: " << iScenario;
    vector < hriResponse > vResponses;

    // check input Bottle:
    if (bInput.size() != 2){
        yError() << " error in narrativeHandler::what_DFW_simple | Wrong size of response from recog ";
        return vResponses;
    }

    Bottle bDFW = *bInput.get(1).asList();

    if (bDFW.size() != 2){
        yError() << " error in narrativeHandler::what_DFW_simple | Wrong size of response from recog ";
        return vResponses;
    }

    string sdfw = bDFW.get(1).toString();

    cout << "extracted DFW from recog: " << sdfw << endl;
    PAOR paor = PAOR();
    vResponses = useDFW(iScenario, sdfw, paor, true);

    cout << "returning: " << endl << vResponses.size() << endl;
    iCub->say("Hum, what " + sdfw + " ?", false);

    return vResponses;
}


/*
* get a complex question form from recog, treat the question and return the result as Bottle
* bInput:  (What_happen_DFW_double ((dfw_double "dfw") (agent "name") (predicate "predicate") opt: (CCW "ccw") (object "obj")))
* ie: (What_happen_DFW_double ((dfw_double because) (agent Sam) (predicate remove) (CCW the) (object box)))
*/
vector < hriResponse > narrativeHandler::what_DFW_Double(Bottle bInput, PAOR &sPAOR, int iScenario){
    vector < hriResponse > vResponses;
    yInfo() << " what_DFW_Double launched: " << bInput.toString() << ", scenario: " << iScenario << ", PAOR: " << sPAOR.toString();


    // check input Bottle:
    if (bInput.size() != 2){
        yError() << " error in narrativeHandler::what_DFW_Double | Wrong size of response from recog ";
        return vResponses;
    }
    Bottle bWords = *bInput.get(1).asList();

    if (bWords.size() == 0){
        yError() << " error in narrativeHandler::what_DFW_Double | Wrong size of response from recog ";
        return vResponses;
    }

    // if inpout sPAOR is empty: get from the bottle:
    cout << "bWords: " << bWords.toString() << endl;

    string agent = bWords.find("agent").toString();
    string predicate = bWords.find("predicate").toString();
    string object = bWords.find("object").toString();
    string recipient = bWords.find("recipient").toString();


    string sdfw = bWords.find("dfw_double").toString();

    cout << "extracted OCW from recog: " << sdfw
        << " - " << agent
        << " - " << predicate
        << " - " << object
        << " - " << recipient
        << "." << endl;

    if (sPAOR.nbElm() == 0){

        sPAOR.P = predicate;
        sPAOR.A = agent;
        sPAOR.O = object;
        sPAOR.R = recipient;
    }


    cout << "extracted DFW from recog: " << sdfw << endl;
    vResponses = useDFW(iScenario, sdfw, sPAOR, true);
    cout << "returning: " << vResponses.size() << endl;
    iCub->say("Hum, " + sdfw + " ?", false);

    return vResponses;
}


/*
* get a PAOR from recog, treat the as "because" and return the result as Bottle
* bInput:  (What_happen_DFW_double ((dfw_double "dfw") (agent "name") (predicate "predicate") opt: (CCW "ccw") (object "obj")))
* ie: (What_happen_DFW_double ((dfw_double because) (agent Sam) (predicate remove) (CCW the) (object box)))
*/
vector < hriResponse > narrativeHandler::whyPAOR(Bottle bInput, PAOR &sPAOR, int iScenario){
    vector < hriResponse > vResponses;
    yInfo() << " whyPAOR launched: " << bInput.toString() << ", scenario: " << iScenario << ", PAOR: " << sPAOR.toString();

    // check input Bottle:
    if (bInput.size() != 2){
        yError() << " error in narrativeHandler::whyPAOR | Wrong size of response from recog ";
        return vResponses;
    }
    Bottle bWords = *bInput.get(1).asList();

    if (bWords.size() == 0){
        yError() << " error in narrativeHandler::whyPAOR | Wrong size of response from recog ";
        return vResponses;
    }

    cout << "bWords: " << bWords.toString() << endl;

    string agent = bWords.find("agent").toString();
    string predicate = bWords.find("predicate").toString();
    string object = bWords.find("object").toString();
    string recipient = bWords.find("recipient").toString();

    string sdfw = "because";

    cout << "extracted OCW from recog: " << sdfw
        << " - " << agent
        << " - " << predicate
        << " - " << object
        << " - " << recipient
        << "." << endl;

    if (sPAOR.nbElm() == 0){

        sPAOR.P = predicate;
        sPAOR.A = agent;
        sPAOR.O = object;
        sPAOR.R = recipient;
    }
    vResponses = useDFW(iScenario, sdfw, sPAOR, false);

    cout << "returning: " << endl << vResponses.size() << endl;
    iCub->say("Hum, why ?", false);

    return vResponses;
}

/*
* bInput:  (What_happen_DFW_double ((dfw_double "dfw") (agent "name") (predicate "predicate") opt: (CCW "ccw") (object "obj")))
* ie: (What_happen_DFW_double ((dfw_double because) (agent Sam) (predicate remove) (CCW the) (object box)))
*/
vector < hriResponse > narrativeHandler::whyIsThat(PAOR &sPAOR, int iScenario){
    vector < hriResponse > vResponses;
    yInfo() << " whyIsThat launched, scenario: " << iScenario << ", PAOR: " << sPAOR.toString();
    string sdfw = "because";

    vResponses = useDFW(iScenario, sdfw, sPAOR, false);

    cout << "returning: " << endl << vResponses.size() << endl;
    return vResponses;
}




string narrativeHandler::pickResponse(vector < hriResponse > &vResponses, vector<PAOR> &vSaid){
    string sReturn = "none";


    removeDoubleMeaning(vResponses);

    if (vResponses.size() == 0){
        return sReturn;
    }

    yInfo() << "Starting picking response from " << vResponses.size() << " elements";

    double sumProb = 0.;
    yInfo() << "Choice btw: ";
    for (auto pa : vResponses){
        yInfo() << pa.toString();
        sumProb += pa.score*pa.score;
    }

    double p = Random::uniform() * sumProb;

    cout << "\tsumProb = " << sumProb << ", p = " << p << endl;

    bool found = false;
    int toRemove = 0;
    for (auto pa : vResponses){
        bool threshold = (p -= pa.score*pa.score) < 0;
        if (!found) {
            toRemove++;
        }
        if (threshold && !found){
            sReturn = iCub->getLRH()->meaningToSentence(pa.sentence);
            vSaid.push_back(pa.paor);
            yInfo() << "\t\tpicked is: " << sReturn << " found: " << found << " p: " << p;
            found = true;
        }
    }

    vResponses.erase(vResponses.begin() + toRemove - 1);

    string X = "_X_";
    
    for (unsigned int loop = 0; loop < 4; loop++) {
    string::size_type i = sReturn.find(X);
        if (i != std::string::npos)
        sReturn.erase(i, X.length());
    }

    return sReturn;
}


bool narrativeHandler::doYouRemember(string sInput){

    yInfo() << "in DO YOU REMEMBER: sInput is: " << sInput;
    bool display = false;
    vector<string>  vsMeaning;
    string meaning = iCub->getLRH()->SentenceToMeaning(sInput);

    vsMeaning.push_back(meaning);
    meaningDiscourse MD;
    // creation of the MD from the discourse
    string sReturn = MD.meaningToDiscourseForm(vsMeaning);

    int bestScore = -1;
    int iToReturn = -1;

    // FOR EACH SM:

    for (unsigned int iSM = 0; iSM < listStories.size(); iSM++){
        ofstream IGARFfile;
        IGARFfile.open(sIGARFfile);
        sm.ABMtoSM(listStories[iSM], IGARFfile);

        int iCurrentScore = 0;

        //search the events;

        for (vector<meaningSentence>::iterator level1 = MD.meanings.vDiscourse.begin();
            level1 != MD.meanings.vDiscourse.end();
            level1++
            )
        {

            for (auto &prep : level1->vSentence){
                cout << prep.toString() << endl;
                if (prep.A == "you"){
                    prep.A = "iCub";

                }
                if (prep.R == "you"){
                    prep.R = "iCub";

                }
            }
            // for each sentence of the discourse
            DFW *currentDFW;
            int iPreposition = 0;   // get the order of the preposition in the sentence
            int iNbPreposition = level1->vSentence.size();  // nb of preposition in the sentence
            bool isDFW = level1->vSentence[0].A == "";
            vector<EVT_IGARF> singleIGARF;
            vector<EVT_IGARF>  doubleBefore;
            vector<EVT_IGARF>  doubleAfter;

            bool bAllAction = true;  //depend of the score of findBest

            if (display) cout << "---------------------------------------------------------------------\n" << "Sentence full is size: " << iNbPreposition << " and contain DFW: " << isDFW << endl;


            for (vector<PAOR>::iterator level2 = level1->vSentence.begin();
                level2 != level1->vSentence.end();
                level2++){  // for each preposition of the sentence
                if (true){
                    cout << " [ " << level2->toString() << "]  ";
                }

                int iScore = 0;
                if (isDFW && iPreposition == 0){ // only one OCW: DFW
                    //cout << "\t\t\t sentence has a DFW." << endl;
                    string nameDFW = level2->P;
                    bool found = false;
                    for (vector<DFW>::iterator itDFW = vDFW.begin(); itDFW != vDFW.end(); itDFW++){
                        if (!found && itDFW->sName == nameDFW){
                            found = true;
                            currentDFW = &(*itDFW);
                            if (display) cout << " found existing DFW: " << currentDFW->sName << endl;
                        }
                    }
                    if (!found) {
                        if (display) cout << " DFW not found: " << currentDFW->sName << endl;
                        return false;
                    }
                }
                else{
                    vector<sKeyMean> vkTmp = sm.findBest(*level2, iScore);
                    iCurrentScore += iScore;
                    if (vkTmp.size() == 0){
                        yWarning() << " in narrativeGraph::humanNarration.cpp::linkMeaningScenario:: findBest : no target found.";
                        yWarning() << level1->getSentence();
                    }

                    bAllAction &= !(iScore <= (int)iThresholdScoreIGARFPAOR && iPreposition != 0);   // all action except the fisrt one need to be found

                    if (bAllAction){     // if found;
                        for (unsigned int kk = 0; kk < vkTmp.size(); kk++)
                        {
                            sKeyMean kTmp = vkTmp[kk];
                            if (display) cout << "\t result find: " << (kTmp.toString());
                            int iIg = -1,
                                iL = -1;
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

                            //cout << "KM is: " << evtKM.toString() << endl;

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
        }

        if (iCurrentScore >= bestScore){
            bestScore = iCurrentScore;
            iToReturn = iSM;
        }


    }

    if (bestScore > 0){
        yInfo() << "To return: " << iToReturn;

        scenarioToRecall = iToReturn;
        return true;
    }
    else{
        return false;
    }
}



bool narrativeHandler::createNarration(vector<tuple <Bottle, PAOR > > vQuestions, int iScenario, vector < PAOR > vResponsesSaid){

    yInfo("Starting to create narration.");
    if (vQuestions.size() == 0){
        yWarning("narrativeGraph cannot create narration, no question in memory.");
        return true;
    }
    bool canUsePreviousResponses = vQuestions.size() == vResponsesSaid.size();
    if (!canUsePreviousResponses){
        yWarning("in narrativeGraph::hri.cpp narrativeHandler::crerateNarration - vQuestions.size != vResponses.size");
    }
    else {
        yInfo("in narrativeGraph::hri.cpp canUsePreviousResponses");
    }

    int doku = 0;
    vector<PAOR> vSaid;
    vector < hriResponse > vInternalResponses;

    for (auto question : vQuestions){

        Bottle quest = get<0>(question);
        yInfo() << "Current question: " << quest.get(0).asString() << ", PAOR: " << get<1>(question).toString() << ",  Response: " << vResponsesSaid[doku].toString();
        if (quest.get(0).asString() == "Else"){
            Time::delay(1.0);
            string picked = pickResponse(vInternalResponses, vSaid);
            if (picked == "none"){
                iCub->say("Nothing else, sorry.");
            }
            else{
                iCub->say(picked, true);
            }
        }
        else {
            if (quest.get(0).asString() == "What_happen_DFW_simple"){
                cout << "in is using simple" << endl;
                vInternalResponses = what_DFW_Simple(quest, scenarioToRecall);
            }
            else if (quest.get(0).asString() == "What_happen_DFW_double" && canUsePreviousResponses){
                cout << "in is using double" << endl;
                if (doku > 0){
                    // if current question is about previous answer
                    if (get<1>(question) == vResponsesSaid[doku - 1]){
                        // adapt question from previous responses.
                        vInternalResponses = what_DFW_Double(quest, vSaid[doku - 1], scenarioToRecall);
                    }
                    else{
                        cout << (get<1>(question)).toString() << " - " << vResponsesSaid[doku - 1].toString() << endl;
                        yWarning("Question is not previous response");
                    }
                }
                else{
                    yWarning("cannot use double as first question");
                }
            }
            else if (quest.get(0).asString() == "Why" && canUsePreviousResponses){
                cout << "in is using why" << endl;
                if (doku > 0){
                    // if current question is about previous answer
                    if (get<1>(question) == vResponsesSaid[doku - 1]){
                        // adapt question from previous responses.
                        vInternalResponses = whyPAOR(quest, vSaid[doku - 1], scenarioToRecall);
                    }
                    else{
                        cout << (get<1>(question)).toString() << " - " << vResponsesSaid[doku - 1].toString() << endl;
                        yWarning("Question is not previous response");
                    }
                }
                else{
                    yWarning("cannot use double as first question");
                }
            }
            else if (quest.get(0).asString() == "Why_is_that"){
                if ((int) vSaid.size() > doku && doku > 0){
                    vInternalResponses = whyIsThat(vSaid[doku - 1], scenarioToRecall);
                }

            }

            string picked = pickResponse(vInternalResponses, vSaid);
            if (picked == "none"){
                iCub->say("I don't know, sorry.");
            }
            else{
                iCub->say(picked, true);
            }
        }
        doku++;
    }
    return true;
}


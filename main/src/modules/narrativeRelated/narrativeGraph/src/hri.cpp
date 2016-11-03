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
    cout << "starting QUESTIONHRI_DFW" << endl;
    Bottle bReturn;

    vector < pair < string, double > > vResponses;

    bool keepInteracting = true;
    bool remember = false; // if asking to recall a situation

    while (keepInteracting){
        Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic, // semantic information of the content of the recognition
            bSendReasoning; // send the information of recall to the abmReasoning
        bool getAnswer = false;

        while (!getAnswer){
            cout << "Remember: " << remember << " | scenario: " << scenarioToRecall << endl;
            bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarQuestionDFW), 20, false, false);
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
            }
            else{
                cout << "confirmation: " << sSentence << endl;
                getAnswer = speechConfirmation();
                if (getAnswer){
                    ///<
                    if (bSemantic.get(0).asString() == "stop"){
                        keepInteracting = false;
                        cout << "Okay, bye !" << endl;
                    }
                    else if (bSemantic.get(0).asString() == "Else"){
                        remember = false;
                        string picked = pickResponse(vResponses);
                        if (picked == "none"){
                            iCub->say("Nothing else, sorry.");
                        }
                        else{
                            iCub->say(picked);
                        }
                    }
                    else if (remember == true){
                        if (doYouRemember(sSentence)){
                            iCub->say("Yes, of course I remember !");
                        }
                        else{
                            iCub->say("Hum, no, it doesn't ring me a bell.");
                        }
                        remember = false;

                    }
                    else if (bSemantic.get(0).asString() == "When"){
                        cout << "Setting remember true !" << endl;
                        remember = true;
                    }
                    else{
                        remember = false;
                        //case sentence simple with DFW:
                        vResponses.clear();
                        if (bSemantic.get(0).asString() == "What_happen_DFW_simple"){
                            bReturn = what_DFW_Simple(bSemantic, scenarioToRecall);
                        }
                        else if (bSemantic.get(0).asString() == "What_happen_DFW_double"){
                            bReturn = what_DFW_Double(bSemantic, scenarioToRecall);
                        }
                        else if (bSemantic.get(0).asString() == "Why"){
                            bReturn = whyPAOR(bSemantic, scenarioToRecall);
                        }
                        // Pick random answer according to probability        
                        for (int ii = 0; ii < bReturn.size(); ii++){
                            if (bReturn.get(ii).isList()){
                                Bottle bTmp = *bReturn.get(ii).asList();
                                if (bTmp.size() == 2){
                                    if (bTmp.get(1).isDouble()){
                                        pair<string, double> pTmp(bTmp.get(0).toString(), bTmp.get(1).asDouble());
                                        vResponses.push_back(pTmp);
                                    }
                                }
                            }
                        }

                        string picked = pickResponse(vResponses);
                        iCub->say(picked);
                    }
                }
            }
            cout << "keepInteracting: " << keepInteracting << endl;
        }

    }

    return bReturn;
}

/*
* get a simple question form from recog, treat the question and return the result as Bottle
* bInput: ( What_happen_DFW_simple ( dfw_simple "dfw"))
* ie: (What_happen_DFW_simple(dfw_simple then))
*/
Bottle narrativeHandler::what_DFW_Simple(Bottle bInput, int iScenario){
    Bottle bReturn;
    cout << " what_DFW_Simple launched: " << bInput.toString() << ", scenario: " << iScenario << endl;

    // check input Bottle:
    if (bInput.size() != 2){
        yError() << " error in narrativeHandler::what_DFW_simple | Wrong size of response from recog ";
        bReturn.addString(" error in narrativeHandler::what_DFW_simple | Wrong size of response from recog");
        return bReturn;
    }

    Bottle bDFW = *bInput.get(1).asList();

    if (bDFW.size() != 2){
        yError() << " error in narrativeHandler::what_DFW_simple | Wrong size of response from recog ";
        bReturn.addString(" error in narrativeHandler::what_DFW_simple | Wrong size of response from recog");
        return bReturn;
    }

    string sdfw = bDFW.get(1).toString();

    cout << "extracted DFW from recog: " << sdfw << endl;
    Bottle bInternal;
    bInternal.addString("useDFW");
    bInternal.addInt(iScenario);
    bInternal.addString(sdfw);
    bInternal.addInt(1);

    bReturn = useDFW(bInternal);

    cout << "returning: " << endl << bReturn.toString() << endl;

    return bReturn;
}



/*
* get a complex question form from recog, treat the question and return the result as Bottle
* bInput:  (What_happen_DFW_double ((dfw_double "dfw") (agent "name") (predicate "predicate") opt: (CCW "ccw") (object "obj")))
* ie: (What_happen_DFW_double ((dfw_double because) (agent Sam) (predicate remove) (CCW the) (object box)))
*/
Bottle narrativeHandler::what_DFW_Double(Bottle bInput, int iScenario){
    Bottle bReturn;
    cout << " what_DFW_Double launched: " << bInput.toString() << ", scenario: " << iScenario << endl;

    // check input Bottle:
    if (bInput.size() != 2){
        yError() << " error in narrativeHandler::what_DFW_Double | Wrong size of response from recog ";
        bReturn.addString(" error in narrativeHandler::what_DFW_Double | Wrong size of response from recog");
        return bReturn;
    }
    Bottle bWords = *bInput.get(1).asList();

    if (bWords.size() == 0){
        yError() << " error in narrativeHandler::what_DFW_Double | Wrong size of response from recog ";
        bReturn.addString(" error in narrativeHandler::what_DFW_Double | Wrong size of response from recog");
        return bReturn;
    }

    cout << "bWords: " << bWords.toString() << endl;

    string agent = bWords.find("agent").toString();
    string predicate = bWords.find("predicate").toString();
    string object = bWords.find("object").toString();
    string recipient = bWords.find("recipient").toString();

    string meaning;
    ostringstream os;
    os << predicate << " " << agent << " " << object << " " << recipient << ", P1 A1 ";
    if (object != "") {
        os << "O1 ";
    }
    if (recipient != "") {
        os << "R1 ";
    }

    meaning = os.str();

    string sdfw = bWords.find("dfw_double").toString();

    cout << "extracted OCW from recog: " << sdfw
        << " - " << agent
        << " - " << predicate
        << " - " << object
        << " - " << recipient
        << "." << endl;

    Bottle bInternal;
    bInternal.addString("useDFW");
    bInternal.addInt(iScenario);
    bInternal.addString(sdfw);
    bInternal.addString(meaning);

    bReturn = useDFW(bInternal);

    cout << "returning: " << endl << bReturn.toString() << endl;

    return bReturn;
}


/*
* get a PAOR from recog, treat the as "because" and return the result as Bottle
* bInput:  (What_happen_DFW_double ((dfw_double "dfw") (agent "name") (predicate "predicate") opt: (CCW "ccw") (object "obj")))
* ie: (What_happen_DFW_double ((dfw_double because) (agent Sam) (predicate remove) (CCW the) (object box)))
*/
Bottle narrativeHandler::whyPAOR(Bottle bInput, int iScenario){
    Bottle bReturn;
    cout << " whyPAOR launched: " << bInput.toString() << ", scenario: " << iScenario << endl;

    // check input Bottle:
    if (bInput.size() != 2){
        yError() << " error in narrativeHandler::whyPAOR | Wrong size of response from recog ";
        bReturn.addString(" error in narrativeHandler::whyPAOR | Wrong size of response from recog");
        return bReturn;
    }
    Bottle bWords = *bInput.get(1).asList();

    if (bWords.size() == 0){
        yError() << " error in narrativeHandler::whyPAOR | Wrong size of response from recog ";
        bReturn.addString(" error in narrativeHandler::whyPAOR | Wrong size of response from recog");
        return bReturn;
    }

    cout << "bWords: " << bWords.toString() << endl;

    string agent = bWords.find("agent").toString();
    string predicate = bWords.find("predicate").toString();
    string object = bWords.find("object").toString();
    string recipient = bWords.find("recipient").toString();

    string meaning;
    ostringstream os;
    os << predicate << " " << agent << " " << object << " " << recipient << ", P1 A1 ";
    if (object != "") {
        os << "O1 ";
    }
    if (recipient != "") {
        os << "R1 ";
    }

    meaning = os.str();

    string sdfw = "because";

    cout << "extracted OCW from recog: " << sdfw
        << " - " << agent
        << " - " << predicate
        << " - " << object
        << " - " << recipient
        << "." << endl;

    Bottle bInternal;
    bInternal.addString("useDFW");
    bInternal.addInt(iScenario);
    bInternal.addString(sdfw);
    bInternal.addString(meaning);
    bInternal.addInt(1);

    bReturn = useDFW(bInternal);

    cout << "returning: " << endl << bReturn.toString() << endl
        << "Size: " << bReturn.size() << endl;

    return bReturn;
}


string narrativeHandler::pickResponse(vector < pair < string, double > > &vResponses){
    string sReturn = "none";

    //removeDoubleMeaning(vResponses);

    if (vResponses.size() == 0){
        return sReturn;
    }

    cout << "Starting picking response from " << vResponses.size() << " elements" << endl;

    double sumProb = 0.;
    cout << "Choice btw: " << endl;
    for (auto pa : vResponses){
        cout << "\t" << pa.first << " " << pa.second << endl;
        sumProb += pa.second;
    }

    double p = Random::uniform() * sumProb;

    cout << "\tsumProb = " << sumProb << ", p = " << p << endl;

    bool found = false;
    int toRemove = 0;
    for (auto pa : vResponses){
        bool threshold = (p -= pa.second) < 0;
        if (!found) {
            toRemove++;
        }
        if (threshold && !found){
            sReturn = pa.first;
            cout << "\t\tpicked is: " << sReturn << " found: " << found << " p: " << p << endl;
            found = true;
        }
    }

    vResponses.erase(vResponses.begin() + toRemove - 1);




    return sReturn;
}


bool narrativeHandler::doYouRemember(string sInput){

    cout << "in DO YOU REMEMBER: sInput is: " << sInput << endl;
    bool display = true;
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
                for (unsigned int ii = 0 ; ii < prep.vOCW.size() ; ii++){
                    cout << prep.vRole[ii] <<"   " << prep.vOCW[ii] <<endl;
                    if (prep.vOCW[ii] == "you"){
                        prep.vOCW[ii] = "iCub";
                    }
                }
            }
            // for each sentence of the discourse
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
                    for (unsigned int iWord = 0; iWord < level2->vOCW.size(); iWord++){

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
                        if (display) cout << " DFW not found: " << currentDFW->sName << endl;
                        return false;
                    }
                }
                else{
                    vector<sKeyMean> vkTmp = sm.findBest(level2->vOCW, iScore);
                    iCurrentScore += iScore;
                    if (vkTmp.size() == 0){
                        yWarning() << " in narrativeGraph::humanNarration.cpp::linkMeaningScenario:: findBest : no target found.";
                        yWarning() << level1->getSentence();
                    }

                    bAllAction &= !(iScore <= iThresholdScoreIGARFPAOR && iPreposition != 0);   // all action except the fisrt one need to be found

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
        cout << "To return: " << iToReturn << endl;

        scenarioToRecall = iToReturn;
        return true;
    }
    else{
        return false;
    }
}



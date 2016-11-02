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
    while (keepInteracting){
        Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic, // semantic information of the content of the recognition
            bSendReasoning; // send the information of recall to the abmReasoning
        bool getAnswer = false;

        while (!getAnswer){
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
                        string picked = pickResponse(vResponses);
                        if (picked == "none"){
                            iCub->say("Nothing else, sorry.");
                        }
                        else{
                            iCub->say(picked);
                        }
                    }
                    else{
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
    cout << " what_DFW_Simple launched: " << bInput.toString() << endl;

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
    cout << " what_DFW_Double launched: " << bInput.toString() << endl;

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
    cout << " whyPAOR launched: " << bInput.toString() << endl;

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

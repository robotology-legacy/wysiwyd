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
* Gets a bottle as input with:
* 0: useDFW
* 1: id scenario (int)
* 2: dfw
* 3: PAOR = NULL
* 4: first or second element of double dfw (0/1) default = first
*/
vector < hriResponse > narrativeHandler::useDFW(int iScenario, string sdfw, PAOR& paor, bool isFirst){

    yInfo() << "useDFW launched: " << iScenario << " " << sdfw << " " << paor.toString() << " " << isFirst;
    vector < hriResponse > vResponses;

    // check if PAOR
    bool hasPAOR = paor.P != "";   ///< check if dfw will have to be double
    if (paor.A == "you"){
        paor.A = "iCub";
    }
    if (paor.R == "you"){
        paor.R = "iCub";
    }

    // CHECK DFW
    analyseDFW();
    storygraph::DFW dfw = foundDFW(sdfw);

    if (dfw.sName == "none"){
        yWarning() << " in narrativeGraph::usedDFW dfw not found: " << sdfw;
        return vResponses;
    }
    // DFW CHECKED


    map<string, int>  dict = {
        { "I", 0 },
        { "G", 1 },
        { "A", 2 },
        { "R", 3 },
        { "F", 4 }
    };

    // LOAD SCENARIO
    loadSM(iScenario);

    yInfo() << "SCENARIO LOADED: " << iScenario;
    // SCENARIO LOADED

    // IF HASN'T PAOR:
    if (!hasPAOR){
        // THEN GET THE SENTENCE THE BEST ADAPTED

        // DOES THIS DFW CAN BE USE WITH ONE SENTENCE ONLY:
        if (dfw.vSingleIGARF.size() == 0){
            yWarning("DFW about 2 preposition. At least one needed.");
            return vResponses;
        }

        // get a score for each sentence: esperance of IGARF * esperence of the time in HISTO
        double dScore = 0;

        vector<pair<int, double>> vpScore; // vector with each event and associated score
        int range = 0;
        double best = 0;

        // for each evenement calcultate a score
        for (auto evt : sm.vChronoEvent){

            // score is distribution of the role of IGARF according to the dfw
            dScore = dfw.simpleIGARF[dict.find(evt.second)->second];

            int histoSize = dfw.vTimeSimple.size();
            double stepSize = 1. / (histoSize*1.);

            double dPos = (range*1.0) / (sm.vChronoEvent.size() *1.0);

            //cout << "evt: " << range <<
            //    ", IGARF: " << sm.vChronoEvent[range].first << " " << sm.vChronoEvent[range].second
            //    << ", pos: " << dPos << ", hist: ";

            // find position in the histo and multiply score by esperence
            for (int step = 0; step < histoSize; step++){
                if (dPos >= (step*stepSize)
                    && dPos < ((step + 1)*stepSize)){
                    dScore += dfw.vTimeSimple[step];
                }
            }

            //cout << ", score: " << dScore << endl;
            if (dScore > best){
                best = dScore;
            }
            vpScore.push_back(pair<int, double>(range, dScore));
            range++;
        }

        // find bests elements with score dScore
        vector<pair <int, string> > AddedEvt;
        vector<tuple <meaningSentence, double, PAOR> > vMeaningScore; // vector of the meaning of event with their scores and position in the IGARF or the response
        for (auto posibilities : vpScore){
            // I can talk of this element
            if (posibilities.second > 0.5 * best){

                int iIGARF = sm.vChronoEvent[posibilities.first].first;
                string sIGARF = sm.vChronoEvent[posibilities.first].second;

                pair<int, string> currentPair(iIGARF, sIGARF);
                bool toAdd = true;
                for (auto passed : AddedEvt){
                    if (currentPair == passed){
                        toAdd = false;
                    }
                }
                meaningSentence meaning = evtToMeaning(sIGARF, iIGARF);
                if (toAdd && meaning.vSentence.size() != 0){
                    //cout << "I should talk about evt: " << iIGARF << " " << sIGARF << endl;
                    vMeaningScore.push_back(tuple <meaningSentence, double, PAOR>(meaning, posibilities.second, meaning.vSentence[0]));
                    AddedEvt.push_back(currentPair);
                }
            }
        }

        // REMOVE DOUBLES
        removeDoubleMeaning(vMeaningScore);

        for (auto &toSend : vMeaningScore){

            // if several relation at same evt
            for (auto &prop : get<0>(toSend).vSentence)
            {

                hriResponse CurrentResponse;
                //set quadratic score
                CurrentResponse.score = get<1>(toSend)*get<1>(toSend);

                // set the PAOR:
                CurrentResponse.paor = prop;

                string preparedMeaning = prepareMeaningForLRH(sdfw, prop);

                if (preparedMeaning != "none" && prop.P != "home")
                {
                    if (!(sdfw == "finally" && prop.P == "want")){
                        CurrentResponse.sentence = (preparedMeaning);
                        cout << "Adding: " << CurrentResponse.toString() << " - " << preparedMeaning << endl;
                        vResponses.push_back(CurrentResponse);
                    }
                }
            }
        }
    }

    // ELSE IS DFW HAS A PAOR AS INPUT:
    if (hasPAOR){

        // FIND THE CORREPONDANT EVT
        int iScore = 0;
        vector<storygraph::sKeyMean> vKM = sm.findBest(paor, iScore);

        // ADD THE RELATED PAOR TO OUTPUT


        // IF NO EVENT:
        if (vKM.size() == 0){
            yWarning(" in narrativeGraph::useDFW::hasPAOR - cannot recognize event");
            return vResponses;
        }

        // DOES THIS DFW CAN BE USE WITH ONE SENTENCE ONLY:
        if (dfw.vDoubleIGARF.size() == 0){
            yWarning("DFW about 1 preposition only. 2 given.");
            return vResponses;
        }

        vector<pair<int, double>> vpScore; // vector with each event and associated score
        vector<tuple <meaningSentence, double, PAOR> > vMeaningScore;
        int range = 0;
        double best = 0;

        vector<PAOR> addResp;

        for (auto km : vKM){
            range = 0;
            // for each evenement calcultate a score
            for (auto evt : sm.vChronoEvent){

                // CHECK IF EVENT IS NOT THE EVENT WE RELIE TO

                int iIGARF = sm.vChronoEvent[range].first;
                string sIGARF = sm.vChronoEvent[range].second;
                ostringstream ss;
                ss << km.cPart;

                if (iIGARF != km.iIGARF || sIGARF != ss.str()){

                    storygraph::EVT_IGARF evtKM(km, sm.vIGARF[km.iIGARF].iAction, sm.vIGARF[km.iIGARF].iLevel);
                    sm.checkEVTIGARF(evtKM);

                    double dScore;

                    // if the PAOR given is the first of the two elements
                    if (isFirst){
                        dScore = dfw.corIGARF[dict.find(evt.second)->second][dict.find(ss.str())->second];
                    }
                    else{
                        dScore = dfw.corIGARF[dict.find(ss.str())->second][dict.find(evt.second)->second];
                    }

                    // get timing between events:
                    double dPos = range / (1.0*sm.vChronoEvent.size());
                    if (isFirst){
                        dPos = dPos - evtKM.dIGARF;
                    }
                    else{
                        dPos = evtKM.dIGARF - dPos;
                    }

                    int histoSize = dfw.vTimeDouble.size();
                    double stepSize = 2. / (histoSize*1.);

                    for (int step = 0; step < histoSize; step++){
                        if (dPos >= (step*stepSize - 1)
                            && dPos < ((step + 1)*stepSize - 1)){
                            dScore += dfw.vTimeDouble[step];
                        }
                    }


                    vpScore.push_back(pair<int, double>(range, dScore));
                    meaningSentence meaning = evtToMeaning(sIGARF, iIGARF);

                    if (meaning.vSentence.size() != 0){
                        //cout << "I should talk about evt: " << iIGARF << " " << sIGARF << endl;
                        vMeaningScore.push_back(tuple <meaningSentence, double, PAOR>(meaning, dScore, meaning.vSentence[0]));
                        for (auto &prop : meaning.vSentence){
                            string preparedMeaning;

                            hriResponse CurrentResponse;
                            //set quadratic score
                            CurrentResponse.score = dScore*dScore;

                            // set the PAOR:
                            CurrentResponse.paor = prop;

                            if (isFirst){
                                preparedMeaning = prepareMeaningForLRH(sdfw, paor, prop, isFirst);
                            }
                            else{
                                preparedMeaning = prepareMeaningForLRH(sdfw, prop, paor, isFirst);
                            }

                            if (preparedMeaning != "none" && prop.P != "home")
                            {
                                bool bFound = false;
                                for (auto resp : addResp){
                                    if (resp == CurrentResponse.paor){
                                        bFound = true;
                                    }
                                }
                                if (!bFound){
                                    addResp.push_back(CurrentResponse.paor);
                                    CurrentResponse.sentence = (preparedMeaning);
                                    vResponses.push_back(CurrentResponse);
                                    cout << "Adding: " << CurrentResponse.toString() << " - " << preparedMeaning << endl;
                                    //cout << ", dScore: " << dScore << endl;
                                    if (CurrentResponse.score > best){
                                        best = CurrentResponse.score;
                                    }
                                }
                            }
                        }

                    }

                }
                range++;
            }
        }


        // REMOVE DOUBLES
        removeDoubleMeaning(vResponses);

        // keep only 0.5 * best score;

        vector < hriResponse > vTemp;
        for (auto resp : vResponses){
            if (resp.score > 0.25 * best){
                vTemp.push_back(resp);
            }
        }

        vResponses = vTemp;

    }


    yInfo() << "dfwRelated::useDFW return size: " << vResponses.size();

    for (auto resp : vResponses){
        yInfo() << "\t" << resp.toString();
    }
    return vResponses;
}




string narrativeHandler::exportDFW(){
    string dfw_file_path = contextPath + "/data.csv";
    ofstream file(dfw_file_path.c_str(), ios::out | ios::trunc);  // erase previous contents of file
    file << "name\tsimple\tdouble\tfrom\tto\trange1\trange2\n";

    for (auto dfw : vDFW){
        for (auto evt : dfw.vSingleIGARF){

            file << dfw.sName << "\t" << evt.dIGARF << "\t" << -1 << "\t" << evt.km.cPart << "\t" << 'Z' << "\t" << evt.rangeIGARF << "\t" << -1 << endl;
        }
        for (auto evt : dfw.vDoubleIGARF){

            file << dfw.sName << "\t" << evt.first.dIGARF << "\t" << evt.second.dIGARF << "\t" << evt.first.km.cPart << "\t" << evt.second.km.cPart << "\t" << evt.first.rangeIGARF << "\t" << evt.second.rangeIGARF << endl;
        }

        dfw.analyseCorr();
        dfw.printCorMatrix();

        cout << endl;
    }

    return ("file " + dfw_file_path + " written");
}



void narrativeHandler::displayDFW(){

    cout << endl << "Displaying DFW: " << vDFW.size() << endl;

    for (auto itD : vDFW){
        cout << "\t" << itD.sName << endl;

        cout << "\t\t double:" << endl;
        for (unsigned int jj = 0; jj < itD.vDoubleIGARF.size(); jj++){
            cout << itD.vDoubleIGARF[jj].first.toString() << "\t" << itD.vDoubleIGARF[jj].second.toString() << endl;
        }

        cout << "\t\t simple: " << endl;
        for (auto itI : itD.vSingleIGARF){
            cout << "\t" << itI.toString() << endl;
        }
    }
}


void narrativeHandler::analyseDFW(){
    ///< for each DFW:
    for (auto &dfw : vDFW){
        dfw.analyseCorr();
        dfw.createHistDouble();
        dfw.createHistSimple();
    }
}


storygraph::DFW narrativeHandler::foundDFW(string sdfw){

    storygraph::DFW dfw("none");
    bool bFound = false;
    for (auto itDFW : vDFW){
        if (itDFW.sName == sdfw){
            yInfo() << " DFW found: " << sdfw;
            dfw = itDFW;
            bFound = true;
        }
    }

    if (!bFound){
        yWarning() << " in narrativeGraph::foundDFW cannot find dfw: " << sdfw;
    }
    return dfw;
}


bool checkMeaning(tuple <meaningSentence, double, PAOR> M1, tuple <meaningSentence, double, PAOR> M2){
    return (get<0>(M1).getSentence() == get<0>(M2).getSentence());
}

bool checkMeaningSort(tuple <meaningSentence, double, PAOR> M1, tuple <meaningSentence, double, PAOR > M2){
    return (get<0>(M1).getSentence() < get<0>(M2).getSentence());
}


bool checkMeaningSortString(hriResponse M1, hriResponse M2){
    return (M1.sentence < M2.sentence);
}

void narrativeHandler::removeDoubleMeaning(vector<tuple <meaningSentence, double, PAOR> > &vec){

    // check for each meaning if already present and remove them.
    sort(vec.begin(), vec.end(), checkMeaningSort);
    vec.erase(unique(vec.begin(), vec.end(), checkMeaning), vec.end());
}

void narrativeHandler::removeDoubleMeaning(vector < hriResponse > &vec){

    // check for each meaning if already present and remove them.
    sort(vec.begin(), vec.end(), checkMeaningSortString);
    vec.erase(unique(vec.begin(), vec.end()), vec.end());
}


string narrativeHandler::prepareMeaningForLRH(string dfw, PAOR M1, PAOR M2, bool DFWAB){
    string sReturn;
    ostringstream osFocus,
        osOCW;


    // FOR BETTER HRI
    if (M1.R == "iCub"){
        M1.R = "me";
    }
    if (M1.A == "iCub"){
        M1.A = "I";
    }

    if (M2.R == "iCub"){
        M2.R = "me";
    }
    if (M2.A == "iCub"){
        M2.A = "I";
    }


    // if order is: M2 DFW M1
    if (!DFWAB){
        if (M1.toString() == M2.toString()){
            yWarning(" in narrativeHandler::prepareMeaningForLRH meanings identical");
            return "none";
        }

        // ADD OCW SENTENCE 1 & 2
        osOCW << dfw << ", " << M1.toString() << " , " << M2.toString() << " <o>[";


        // First bracket
        // ONLY THE DFW
        for (int ii = 0; ii < M2.nbElm(); ii++){
            osFocus << "_-";
        }
        osFocus << "P";
        // blanck evt for sentence 2
        for (int ii = 0; ii < M1.nbElm(); ii++){
            osFocus << "-_";
        }
        osFocus << "][";

        // Second bracket for M1
        for (int ii = 0; ii < M2.nbElm(); ii++){
            osFocus << "_-";
        }
        // add the effect of DFW
        osFocus << "_";

        // blanck evt for sentence 2
        osFocus << "-A-P";
        if (M1.O != ""){
            osFocus << "-O";
        }
        //if recipient exist
        if (M1.R != ""){
            osFocus << "-R";
        }
        osFocus << "][";

        // Third bracket M2
        // add the role of the first sentence with order: A P O R
        osFocus << "A-P-";
        // IF OBJECT EXIST
        if (M2.O != ""){
            osFocus << "O-";
        }
        //if recipient exist
        if (M2.R != ""){
            osFocus << "R-";
        }
        // add the effect of DFW
        osFocus << "_";

        // blanck evt for M1
        for (int ii = 0; ii < M1.nbElm(); ii++){
            osFocus << "-_";
        }
        osFocus << "]<o>";

        sReturn = osOCW.str() + osFocus.str();
    }
    // if order is DFW M1 M2
    else {

        if (M1.toString() == M2.toString()){
            yWarning(" in narrativeHandler::prepareMeaningForLRH meanings identical");
            return "none";
        }

        // ADD OCW SENTENCE 1 & 2
        osOCW << dfw << ", " << M1.toString() << " , " << M2.toString() << " <o>[P";

        // First bracket
        // ONLY THE DFW
        for (int ii = 0; ii < M1.nbElm(); ii++){
            osFocus << "-_";
        }
        // blanck evt for sentence 2
        for (int ii = 0; ii < M2.nbElm(); ii++){
            osFocus << "-_";
        }


        // SECOND BRACKET IS M1 SECOND ELEMENT OF SENTENCE
        osFocus << "][_";
        osFocus << "-A-P";
        // IF OBJECT EXIST
        if (M1.O != ""){
            osFocus << "-O";
        }
        //if recipient exist
        if (M1.R != ""){
            osFocus << "-R";
        }
        // blanck evt for sentence 2
        for (int ii = 0; ii < M2.nbElm(); ii++){
            osFocus << "-_";
        }
        osFocus << "][_";


        // blanck evt for sentence 2
        for (int ii = 0; ii < M1.nbElm(); ii++){
            osFocus << "-_";
        }
        // add the role of the first sentence with order: A P O R
        osFocus << "-A-P";
        // IF OBJECT EXIST
        if (M2.O != ""){
            osFocus << "-O";
        }
        //if recipient exist
        if (M2.R != ""){
            osFocus << "-R";
        }
        osFocus << "]<o>";


        sReturn = osOCW.str() + osFocus.str();
    }

    return sReturn;
}



string narrativeHandler::prepareMeaningForLRH(string dfw, PAOR M1){
    string sReturn;
    ostringstream osFocus,
        osOCW;

    // FOR BETTER HRI
    if (M1.R == "iCub"){
        M1.R = "me";
    }
    if (M1.A == "iCub"){
        M1.A = "I";
    }


    if (M1.toString() == ""){
        yWarning("in narrativeHandler::prepareMeaningForLRH  meaning is empty.");
        return "none";
    }

    osOCW << dfw << ", " << M1.toString() << " <o> [P";

    // First bracket
    // add the empty for each OCW
    for (int ii = 0; ii < M1.nbElm(); ii++){
        osFocus << "-_";
    }

    // Second bracket for DFW
    // add the effect of DFW
    osFocus << "][_-A-P";

    // IF OBJECT EXIST
    if (M1.O != ""){
        osFocus << "-O";
    }
    //if recipient exist
    if (M1.R != ""){
        osFocus << "-R";
    }
    osFocus << "] <o>";

    sReturn = osOCW.str() + osFocus.str();

    return sReturn;
}

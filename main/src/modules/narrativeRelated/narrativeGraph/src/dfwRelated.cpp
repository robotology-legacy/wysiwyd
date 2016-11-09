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



/*
* Gets a bottle as input with:
* 0: useDFW
* 1: id scenario (int)
* 2: dfw
* 3: PAOR = NULL
* 4: first or second element of double dfw (0/1) default = first
*/
Bottle narrativeHandler::useDFW(Bottle bInput){

    cout << "useDFW launched: " << bInput.toString() << endl;

    Bottle bRet;
    bRet.addVocab(Vocab::encode("many"));


    if (bInput.size() < 3){
        yWarning("in narrativeGraph::useDFW wrong size of input (min 2)");
        bRet.addString("in narrativeGraph::useDFW wrong size of input (min 2)");
        return bRet;
    }

    bool hasPAOR = false;   ///< check if dfw will have to be double
    bool isFirst = true;    ///< if dfw is double, does PAOR sent has to be first or second (first by default)


    // CHECK DFW
    analyseDFW();
    string sdfw = bInput.get(2).asString();
    storygraph::DFW dfw = foundDFW(sdfw);

    if (dfw.sName == "none"){
        yWarning() << " in narrativeGraph::usedDFW dfw not found: " << sdfw;
        bRet.addString(" in narrativeGraph::usedDFW dfw not found: " + sdfw + ")");
        return bRet;
    }
    // DFW CHECKED


    // CHECK IF PAOR
    meaningSentence meaning;
    if (bInput.size() > 3){
        string tmpPAOR = bInput.get(3).asString();
        meaning = sentenceToEvent(tmpPAOR);
        cout << "Meaning before: " << meaning.getSentence() << endl;
        for (auto &prep : meaning.vSentence){
            for (unsigned int ii = 0 ; ii < prep.vOCW.size() ; ii++){
                cout << prep.vRole[ii] <<"   " << prep.vOCW[ii] <<endl;
                if (prep.vOCW[ii] == "you"){
                    prep.vOCW[ii] = "iCub";
                }
            }
        }
        cout << "Meaning after: " << meaning.getSentence() << endl;
    }
    hasPAOR = meaning.vSentence.size() != 0;
    // PAOR CHECKED


    // IF PAOR, IS FIRST ?
    if (hasPAOR){
        if (bInput.size() == 5){
            isFirst = bInput.get(4).asInt() == 1;
        }
    }
    // FIRST OR SECOND DONE

    map<string, int>  dict = {
        { "I", 0 },
        { "G", 1 },
        { "A", 2 },
        { "R", 3 },
        { "F", 4 }
    };

    // LOAD SCENARIO
    int iScenario = bInput.get(1).asInt();
    loadSM(iScenario);

    cout << "SCENARIO LOADED: " << iScenario << endl;
    // SCENARIO LOADED

    // IF HASN'T PAOR:
    if (!hasPAOR){
        // THEN GET THE SENTENCE THE BEST ADAPTED

        // DOES THIS DFW CAN BE USE WITH ONE SENTENCE ONLY:
        if (dfw.vSingleIGARF.size() == 0){
            yWarning("DFW about 2 preposition. At least one needed.");
            bRet.addString("DFW about 2 preposition. At least one needed.");
            return bRet;
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
        ostringstream os;

        vector<pair <int, string> > AddedEvt;
        vector<tuple <meaningSentence, double, string> > vMeaningScore; // vector of the meaning of event with their scores and position in the IGARF or the response
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

                if (toAdd){
                    //cout << "I should talk about evt: " << iIGARF << " " << sIGARF << endl;
                    meaningSentence meaning = evtToMeaning(sIGARF, iIGARF);
                    vMeaningScore.push_back(tuple <meaningSentence, double, string>(meaning, posibilities.second, meaning.getSentence()));
                    AddedEvt.push_back(currentPair);
                }
            }
        }

        // REMOVE DOUBLES
        removeDoubleMeaning(vMeaningScore);

        for (auto &toSend : vMeaningScore){
            for (auto &prep : get<0>(toSend).vSentence){
                for (unsigned int ii = 0 ; ii < prep.vOCW.size() ; ii++){
                    cout << prep.vRole[ii] <<"   " << prep.vOCW[ii] <<endl;
                    if (prep.vOCW[ii] == "iCub"){
                        prep.vOCW[ii] = "I";
                    }
                }
            }

            meaningSentence meaningTemp;
            // if several relation at same evt
            for (auto prop : get<0>(toSend).vSentence)
            {
                meaningTemp.vSentence.clear();
                meaningTemp.vSentence.push_back(prop);

                string preparedMeaning = prepareMeaningForLRH(sdfw, meaningTemp);

                if (preparedMeaning != "none")
                {
                    cout << preparedMeaning << " : " << get<1>(toSend) << endl;
                    Bottle bTemp;
                    bTemp.addString(iCub->getLRH()->meaningToSentence(preparedMeaning, true));
                    bTemp.addDouble(get<1>(toSend));
                    bTemp.addString(meaningTemp.getSentence());
                    cout << bTemp.toString() << endl;
                    bRet.addList() = bTemp;
                }
            }
        }
    }

    // ELSE IS DFW HAS A PAOR AS INPUT:
    if (hasPAOR){

        // FIND THE CORREPONDANT EVT
        int iScore = 0;
        vector<storygraph::sKeyMean> vKM = sm.findBest(meaning.vSentence[0].vOCW, iScore);

        // ADD THE RELATED PAOR TO OUTPUT


        // IF NO EVENT:
        if (vKM.size() == 0){
            yWarning(" in narrativeGraph::useDFW::hasPAOR - cannot recognize event");
            bRet.addString("none - cannot recognize event");
            return bRet;
        }

        // DOES THIS DFW CAN BE USE WITH ONE SENTENCE ONLY:
        if (dfw.vDoubleIGARF.size() == 0){
            bRet.addString("DFW about 1 preposition only. 2 given.");
            return bRet;
        }

        vector<pair<int, double>> vpScore; // vector with each event and associated score

        int range = 0;
        double best = 0;

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
                        dScore = dfw.corIGARF[dict.find(ss.str())->second][dict.find(evt.second)->second];
                    }
                    else{
                        dScore = dfw.corIGARF[dict.find(evt.second)->second][dict.find(ss.str())->second];
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

                    //cout << ", dScore: " << dScore << endl;
                    if (dScore > best){
                        best = dScore;
                    }
                    vpScore.push_back(pair<int, double>(range, dScore));
                }
                range++;
            }
        }

        ostringstream os;

        vector<pair <int, string> > AddedEvt;
        vector<tuple <meaningSentence, double, string> > vMeaningScore; // vector of the meaning of event with their scores
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

                if (toAdd){
                    //cout << "I should talk about evt: " << iIGARF << " " << sIGARF << endl;
                    meaningSentence meaning = evtToMeaning(sIGARF, iIGARF);
                    vMeaningScore.push_back(tuple <meaningSentence, double, string>(meaning, posibilities.second, meaning.getSentence()));
                    AddedEvt.push_back(currentPair);
                }
            }
        }

        // REMOVE DOUBLES
        removeDoubleMeaning(vMeaningScore);

        for (auto &toSend : vMeaningScore){
            for (auto &prep : get<0>(toSend).vSentence){
                for (unsigned int ii = 0 ; ii < prep.vOCW.size() ; ii++){
                    cout << prep.vRole[ii] <<"   " << prep.vOCW[ii] <<endl;
                    if (prep.vOCW[ii] == "iCub" && prep.vRole[ii][0] == 'R'){
                        prep.vOCW[ii] = "me";
                    }
                    if (prep.vOCW[ii] == "iCub" && prep.vRole[ii][0] == 'A'){
                        prep.vOCW[ii] = "I";
                    }
                }
            }

            for (auto &prep : meaning.vSentence){
                for (unsigned int ii = 0 ; ii < prep.vOCW.size() ; ii++){
                    cout << prep.vRole[ii] <<"   " << prep.vOCW[ii] <<endl;
                    if (prep.vOCW[ii] == "iCub" && prep.vRole[ii][0] == 'R'){
                        prep.vOCW[ii] = "me";
                    }
                    if (prep.vOCW[ii] == "iCub" && prep.vRole[ii][0] == 'A'){
                        prep.vOCW[ii] = "I";
                    }
                }
            }


            meaningSentence meaningTemp;
            // if several relation at same evt
            for (auto prop : get<0>(toSend).vSentence)
            {
                string preparedMeaning;

                meaningTemp.vSentence.clear();
                meaningTemp.vSentence.push_back(prop);

                if (isFirst){
                    preparedMeaning = prepareMeaningForLRH(sdfw, meaning, meaningTemp, !isFirst);
                }
                else{
                    preparedMeaning = prepareMeaningForLRH(sdfw, meaningTemp, meaning, !isFirst);
                }

                if (preparedMeaning != "none")
                {
                    cout << preparedMeaning << " : " << get<1>(toSend) << endl;
                    Bottle bTemp;
                    bTemp.addString(iCub->getLRH()->meaningToSentence(preparedMeaning, true));
                    bTemp.addDouble(get<1>(toSend));
                    bTemp.addString(meaningTemp.getSentence());
                    cout << bTemp.toString() << endl;
                    bRet.addList() = bTemp;
                }
            }            
        }
    }



    return bRet;
}




void narrativeHandler::exportDFW(){
    string dfw_file_path = "C:/Users/rclab/data.csv";
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

    yInfo() << "\t" << "file " << dfw_file_path << " written";
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


bool checkMeaning(tuple <meaningSentence, double, string> M1, tuple <meaningSentence, double, string> M2){
    return (get<0>(M1).getSentence() == get<0>(M2).getSentence());
}

bool checkMeaningSort(tuple <meaningSentence, double, string> M1, tuple <meaningSentence, double, string > M2){
    return (get<0>(M1).getSentence() < get<0>(M2).getSentence());
}

bool checkMeaningString(tuple < string, double, string >  M1, tuple < string, double, string > M2){
    return (get<0>(M1) == get<0>(M2));
}

bool checkMeaningSortString(tuple < string, double, string > M1, tuple < string, double, string > M2){
    return (get<0>(M1) < get<0>(M2));
}

void narrativeHandler::removeDoubleMeaning(vector<tuple <meaningSentence, double, string> > &vec){

    // check for each meaning if already present and remove them.
    sort(vec.begin(), vec.end(), checkMeaningSort);
    vec.erase(unique(vec.begin(), vec.end(), checkMeaning), vec.end());
}

void narrativeHandler::removeDoubleMeaning(vector < tuple < string, double, string > > &vec){

    // check for each meaning if already present and remove them.
    sort(vec.begin(), vec.end(), checkMeaningSortString);
    vec.erase(unique(vec.begin(), vec.end(), checkMeaningString), vec.end());
}


string narrativeHandler::prepareMeaningForLRH(string dfw, meaningSentence M1, meaningSentence M2, bool DFWAB){
    string sReturn;
    ostringstream osFocus,
        osOCW;

    // if order is: M1 DFW M2
    if (!DFWAB){
        if (M1.vSentence.size() == 0 || M2.vSentence.size() == 0){
            yWarning("in narrativeHandler::prepareMeaningForLRH  one of the meaning is empty.");
            return "none";
        }

        if (M1.getSentence() == M2.getSentence()){
            yWarning(" in narrativeHandler::prepareMeaningForLRH meanings identical");
            return "none";
        }

        osOCW << dfw << ", ";

        // ADD OCW SENTENCE 1
        for (auto ocw : M1.vSentence[0].vOCW){
            osOCW << ocw << " ";
        }
        osOCW << ", ";

        // ADD OCW SENTENCE 2
        for (auto ocw : M2.vSentence[0].vOCW){
            osOCW << ocw << " ";
        }

        osFocus << " <o>[";

        // First bracket
        // ONLY THE DFW
        for (auto foc : M1.vSentence[0].vRole){
            osFocus << "_-";
        }
        osFocus << "P";
        // blanck evt for sentence 2
        for (auto foc : M2.vSentence[0].vRole){
            osFocus << "-_";
        }
        osFocus << "][";

        // Second bracket for DFW
        // add the role of the first sentence with order: A P O R
        osFocus << "A-P-";
        // IF OBJECT EXIST
        if (M1.vSentence[0].vRole.size() > 2){
            osFocus << "O-";
        }
        //if recipient exist
        if (M1.vSentence[0].vRole.size() > 3){
            osFocus << "R-";
        }
        // add the effect of DFW
        osFocus << "_";

        // blanck evt for sentence 2
        for (auto foc : M2.vSentence[0].vRole){
            osFocus << "-_";
        }
        osFocus << "][";

        // Third bracket
        for (auto foc : M1.vSentence[0].vRole){
            osFocus << "_-";
        }
        // add the effect of DFW
        osFocus << "_";

        // blanck evt for sentence 2
        osFocus << "-A-P";
        if (M2.vSentence[0].vRole.size() > 2){
            osFocus << "-O";
        }
        //if recipient exist
        if (M2.vSentence[0].vRole.size() > 3){
            osFocus << "-R";
        }
        osFocus << "]<o>";

        sReturn = osOCW.str() + osFocus.str();
    }
    // if order is DFW M2 M1
    else {
        if (M1.vSentence.size() == 0 || M2.vSentence.size() == 0){
            yWarning("in narrativeHandler::prepareMeaningForLRH  one of the meaning is empty.");
            return "none";
        }

        if (M1.getSentence() == M2.getSentence()){
            yWarning(" in narrativeHandler::prepareMeaningForLRH meanings identical");
            return "none";
        }

        osOCW << dfw << ", ";

        // ADD OCW SENTENCE 1
        for (auto ocw : M1.vSentence[0].vOCW){
            osOCW << ocw << " ";
        }
        osOCW << ", ";

        // ADD OCW SENTENCE 2
        for (auto ocw : M2.vSentence[0].vOCW){
            osOCW << ocw << " ";
        }

        osFocus << " <o>[P";

        // First bracket
        // ONLY THE DFW
        for (auto foc : M1.vSentence[0].vRole){
            osFocus << "-_";
        }
        // blanck evt for sentence 2
        for (auto foc : M2.vSentence[0].vRole){
            osFocus << "-_";
        }


        // SECOND BRACKET IS M1 THIRD ELEMENT OF SENTENCE
        osFocus << "][_";
        // blanck evt for sentence 2
        for (auto foc : M2.vSentence[0].vRole){
            osFocus << "-_";
        }
        // add the role of the first sentence with order: A P O R
        osFocus << "-A-P";
        // IF OBJECT EXIST
        if (M1.vSentence[0].vRole.size() > 2){
            osFocus << "-O";
        }
        //if recipient exist
        if (M1.vSentence[0].vRole.size() > 3){
            osFocus << "-R";
        }

        osFocus << "][_";
        // THIRD BRACKET IS M2 SECOND ELEMENT
        osFocus << "-A-P";
        // IF OBJECT EXIST
        if (M2.vSentence[0].vRole.size() > 2){
            osFocus << "-O";
        }
        //if recipient exist
        if (M2.vSentence[0].vRole.size() > 3){
            osFocus << "-R";
        }
        // blanck evt for sentence 2
        for (auto foc : M2.vSentence[0].vRole){
            osFocus << "-_";
        }
        osFocus << "]<o>";

        sReturn = osOCW.str() + osFocus.str();
    }

    return sReturn;
}



string narrativeHandler::prepareMeaningForLRH(string dfw, meaningSentence M1){
    string sReturn;
    ostringstream osFocus,
        osOCW;

    if (M1.vSentence.size() == 0){
        yWarning("in narrativeHandler::prepareMeaningForLRH  meaning is empty.");
        return "none";
    }

    osOCW << dfw << ", ";

    // ADD OCW SENTENCE 1
    for (auto ocw : M1.vSentence[0].vOCW){
        osOCW << ocw << " ";
    }

    osFocus << " <o> [P";

    // First bracket
    // add the empty for each OCW
    for (auto foc : M1.vSentence[0].vRole){
        osFocus << "-_";
    }

    // Second bracket for DFW
    // add the effect of DFW
    osFocus << "][_-A-P";

    // IF OBJECT EXIST
    if (M1.vSentence[0].vRole.size() > 2){
        osFocus << "-O";
    }
    //if recipient exist
    if (M1.vSentence[0].vRole.size() > 3){
        osFocus << "-R";
    }
    osFocus << "] <o>";

    sReturn = osOCW.str() + osFocus.str();

    return sReturn;
}

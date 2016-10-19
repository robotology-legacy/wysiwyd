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



/*
* Gets a bottle as input with:
* 0: useDFW
* 1: id scenario (int)
* 2: dfw
* 3: PAOR = NULL
* 4: first or second element of double dfw (0/1) default = first
*/
Bottle narrativeHandler::useDFW(Bottle bInput){
    Bottle bRet;
    bRet.addVocab(Vocab::encode("many"));


    if (bInput.size() < 3){
        yWarning("in narrativeGraph::useDFW wrong size of input (min 2)");
        bRet.addString("in narrativeGraph::useDFW wrong size of input (min 2)");
        return bRet;
    }

    bool hasPAOR = false;   ///< check if dfw will have to be double
    bool isFirst = true;    ///< if dfw is double, does PAOR sent has to be first or second (first by default)

    // LOAD SCENARIO
    int iScenario = bInput.get(1).asInt() - 1;
    loadSM(iScenario);
    // SCENARIO LOADED


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
    discourseform::meaningSentence meaning;
    if (bInput.size() > 3){
        string tmpPAOR = bInput.get(3).asString();
        meaning = meaningToEvent(tmpPAOR);
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

    // IF HASN'T PAOR:
    if (!hasPAOR){
        // THEN GET THE SENTENCE THE BEST ADAPTED


        // DOES THIS DFW CAN BE USE WITH ONE SENTENCE ONLY:
        if (dfw.vTimeSimple.size() == 0){
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
            //  ", IGARF: " << sm.vChronoEvent[range].first << " " << sm.vChronoEvent[range].second
            //  << ", pos: " << dPos << ", hist: ";

            // find position in the histo and multiply score by esperence
            for (int step = 0; step < histoSize; step++){
                if (dPos >= (step*stepSize)
                    && dPos < ((step + 1)*stepSize)){
                    dScore *= dfw.vTimeSimple[step];
                    cout << step;
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

        for (auto posibilities : vpScore){
            // I can talk of this element
            if (posibilities.second == best){
                int iIGARF = sm.vChronoEvent[posibilities.first].first;
                string sIGARF = sm.vChronoEvent[posibilities.first].second;
                os << "I should talk about evt: " << iIGARF << " " << sIGARF << " : ";

                // INIT
                if (sIGARF == "I"){
                    for (auto init : sm.vIGARF[iIGARF].vInitState){
                        os << sm.vRelations[init].subject
                            << " " << sm.vRelations[init].verb
                            << " " << sm.vRelations[init].object
                            << " ";
                    }
                }
                // FINAL
                if (sIGARF == "F"){
                    for (auto i : sm.vIGARF[iIGARF].vFinalState){
                        os << sm.vRelations[i].subject
                            << " " << sm.vRelations[i].verb
                            << " " << sm.vRelations[i].object
                            << " ";
                    }
                }
                // GOAL
                if (sIGARF == "G"){
                    for (auto i : sm.vIGARF[iIGARF].vGoal){
                        os << sm.vRelations[i].subject
                            << " " << sm.vRelations[i].verb
                            << " " << sm.vRelations[i].object
                            << " ";
                    }
                }
                // ACTION
                if (sIGARF == "A"&& sm.vIGARF[iIGARF].iAction >= 0){
                    os << sm.vActionEvts[sm.vIGARF[iIGARF].iAction].agent
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iAction].predicate
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iAction].object
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iAction].recipient;
                }
                // RESULT
                if (sIGARF == "R" && sm.vIGARF[iIGARF].iResult >= 0){
                    os << sm.vActionEvts[sm.vIGARF[iIGARF].iResult].agent
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iResult].predicate
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iResult].object
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iResult].recipient;
                }
                os << endl;
            }
        }
        bRet.addString(os.str());
        cout << os.str();
    }

    // ELSE IS DFW HAS A PAOR AS INPUT:
    if (hasPAOR){

        dfw.printCorMatrix();

        // FIND THE CORREPONDANT EVT
        int iScore = 0;
        vector<storygraph::sKeyMean> vKM = sm.findBest(meaning.vSentence[0].vOCW, iScore);

        // IF NO EVENT:
        if (vKM.size() == 0){
            yWarning(" in narrativeGraph::useDFW::hasPAOR - cannot recognize event");
            bRet.addString("none - cannot recognize event");
            return bRet;
        }

        // DOES THIS DFW CAN BE USE WITH ONE SENTENCE ONLY:
        if (dfw.vTimeDouble.size() == 0){
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

                    //cout << "evt: " << range <<
                    //    ", IGARF: " << sm.vChronoEvent[range].first << " " << sm.vChronoEvent[range].second
                    //    << ", pos: " << dPos << ", hist: ";

                    // find position in the histo and multiply score by esperence
                    for (int step = 0; step < histoSize; step++){
                        if (dPos >= (step*stepSize - 1)
                            && dPos < ((step + 1)*stepSize - 1)){
                            dScore *= dfw.vTimeDouble[step];
                        }
                    }

                    //cout << ", scoreIG: " << dScoreIGARF << ", scoreTiming: " << dScoreTiming << ", dScore: " << dScore << endl;
                    if (dScore > best){
                        best = dScore;
                    }
                    vpScore.push_back(pair<int, double>(range, dScore));
                }
                range++;
            }
        }

        ostringstream os;


        for (unsigned int i = 0; i < vpScore.size(); i++)
        {
            for (unsigned int j = 0; j < vpScore.size(); j++)
            {
                if (i != j)
                {
                    if (vpScore[i] == vpScore[j])
                    {
                        vpScore.erase(vpScore.begin() + i);
                    }
                }
            }
        }


        for (auto posibilities : vpScore){
            // I can talk of this element
            if (posibilities.second == best){
                int iIGARF = sm.vChronoEvent[posibilities.first].first;
                string sIGARF = sm.vChronoEvent[posibilities.first].second;
                os << "I should talk about evt: " << iIGARF << " " << sIGARF << " : ";

                // INIT
                if (sIGARF == "I"){
                    for (auto init : sm.vIGARF[iIGARF].vInitState){
                        os << sm.vRelations[init].subject
                            << " " << sm.vRelations[init].verb
                            << " " << sm.vRelations[init].object
                            << " ";
                    }
                }
                // FINAL
                if (sIGARF == "F"){
                    for (auto i : sm.vIGARF[iIGARF].vFinalState){
                        os << sm.vRelations[i].subject
                            << " " << sm.vRelations[i].verb
                            << " " << sm.vRelations[i].object
                            << " ";
                    }
                }
                // GOAL
                if (sIGARF == "G"){
                    for (auto i : sm.vIGARF[iIGARF].vGoal){
                        os << sm.vRelations[i].subject
                            << " " << sm.vRelations[i].verb
                            << " " << sm.vRelations[i].object
                            << " ";
                    }
                }
                // ACTION
                if (sIGARF == "A"&& sm.vIGARF[iIGARF].iAction >= 0){
                    os << sm.vActionEvts[sm.vIGARF[iIGARF].iAction].agent
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iAction].predicate
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iAction].object
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iAction].recipient;
                }
                // RESULT
                if (sIGARF == "R" && sm.vIGARF[iIGARF].iResult >= 0){
                    os << sm.vActionEvts[sm.vIGARF[iIGARF].iResult].agent
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iResult].predicate
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iResult].object
                        << " " << sm.vActionEvts[sm.vIGARF[iIGARF].iResult].recipient;
                }
                os << endl;
            }
        }
        bRet.addString(os.str());
        cout << os.str();
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

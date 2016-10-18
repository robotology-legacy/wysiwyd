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
        bRet.addString(" in narrativeGraph::usedDFW dfw not found: " + sdfw);
        return bRet;
    }
    // DFW CHECKED


    // CHECK IF PAOR
    discourseform::meaningSentence meaning;
    if (bInput.size() < 3){
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

        // get a score for each sentence: esperance of IGARF * esperence of the time in HISTO
        double dScore = 0;

        vector<pair<int, double>> vpScore; // vector with each event and associated score
        int range = 0;
        int best = 0;
        for (auto evt : sm.vChronoEvent){
            dScore = dfw.simpleIGARF[dict.find(evt.second)->second];

            int histoSize = dfw.vTimeSimple.size();
            double stepSize = 1. / (histoSize*1.);

            double dPos = (evt.first*1.0) / (sm.vChronoEvent.size() *1.0);

            // find position in the histo and multiply score by esperence
            for (int step = 0; step < histoSize; step++){
                if (dPos >=(step*stepSize)
                    && dPos < ((step + 1)*stepSize)){
                    dScore *= dfw.vTimeSimple[step];
                }
            }
            if (dScore > best){
                best = dScore;
            }
            vpScore.push_back(pair<int, double>(range, dScore));
            range++;
        }

        // find bests elements with score dScore

        for (auto posibilities : vpScore){
            // I can talk of this element
            if (posibilities.second == best){
                cout << "I should talk about evt: " << sm.vChronoEvent[posibilities.first].first << " " << sm.vChronoEvent[posibilities.first].second << endl;
//                pair<int, string> pTmp(IGA_Input.km.iIGARF, ss.str());

//                int pos = find(vChronoEvent.begin(), vChronoEvent.end(), pTmp) - vChronoEvent.begin();

            }
        }
    }
    //



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

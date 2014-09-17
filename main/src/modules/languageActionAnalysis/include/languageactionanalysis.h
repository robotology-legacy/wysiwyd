/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Anne-Laure MEALIER
 * email:   anne.laure.mealier@gmail.com
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

#ifndef LANGUAGEACTIONANALYSIS_H
#define LANGUAGEACTIONANALYSIS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <list>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;


class LanguageActionAnalysis : public RFModule {
private:
    string moduleName;
    string sKeyWord;

    string handlerPortName;
    string port2SpeechRecogName;
    string port2iSpeakName;

    string nameGrammarNode;

    string fvector;
    string pythonPath;

    ifstream file;
    string svector,sanswer;
    string sagent;
    int iquestion;

    Port handlerPort;				// a port to handle messages
    Port Port2SpeechRecog;		// a port to send grammar to the speech recog
    Port Port2iSpeak;		// a port to send grammar to the speech recog

    bool isAwake;
    ICubClient *iCub;

    int iCurrentInstance;					// instance of the current request
    int inbsentence;
    string sCurrentNode;
    string sCurrentGrammarFile;
    string sCurrentActivity;

    pair<string, string> psCurrentComplement;

    int languageNode();
    bool mainNode();
    bool grammarNode();
    list<int> nbCaracters(string ssequence);
    string	grammarToString(string sPath);

public:
    LanguageActionAnalysis(ResourceFinder &rf);
    ~LanguageActionAnalysis();

    bool configure(ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod();
    bool updateModule();
};

#endif // LANGUAGEACTIONANALYSIS_H



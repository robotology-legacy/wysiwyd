/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Anne-Laure MEALIER
 * email:   anne-laure.mealier@inserm.fr
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

#ifndef _RESERVOIRHANDLER_H_
#define _RESERVOIRHANDLER_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <list>

#include "boost/python.hpp"
using namespace boost::python;
//#include <python2.7/Python.h>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

class reservoirHandler : public RFModule {
private:
    string moduleName;
    string sKeyWord;

    string handlerPortName;
    string port2SpeechRecogName;
    string port2iSpeakName;

    string nameGrammarNodeType;
    string nameGrammarNodeModality;
    string nameGrammarNodeTrainAP;
    string nameGrammarNodeTestAP;
    string nameGrammarNodeTrainSD;

    string fileAPimputS;
    string fileAPoutputM;
    string fileSRinputM;
    string fileSRoutputS;
    string fileXavierTrain;
    string fileXavierTrainAP;
    string fileSD;
    string fileAP;

    string pythonPath;

    Port handlerPort;				// a port to handle messages
    Port Port2SpeechRecog;		// a port to send grammar to the speech recog
    Port Port2iSpeak;		// a port to send grammar to the speech recog

    bool isAwake;
    ICubClient *iCub;

    int iCurrentInstance;					// instance of the current request
    int inbsentence;
    string sCurrentActivity;
    string sCurrentType;
    string sCurrentNode;
    string sGrammarYesNo;
    string sCurrentCanonical;
    string sCurrentGrammarFile;
    string sLastSentence;
    string sSentence_type;
    string sSentence;
    string sdataTestSD;
    // last sentence said (in case of a repeat)
    pair<string, string> psCurrentComplement;

    string sentence;
    std::list<string> lMeaningsSentences;
    Vector coordinates;

    bool nodeYesNo();
    bool nodeType();
    bool nodeModality();
    bool nodeTrainAP();
    bool nodeTestAP();
    bool nodeTrainSD();
    bool callReservoir(string fPython);
    string	grammarToString(string sPath);

    int copyTrainData(const char* fileNameIn, const char* fileNameOut);
    int copyPastFile(const char* in, const char* fileNameOut);
    int trainSaveMeaningSentence(const char *filename);
    int createTestwithTrainData(const char* filename, string sMeaningSentence);
    string openResult(const char* fileNameIn);
    int AREactions(vector<string> seq);
    vector<string> extractVocabulary(string sequence);


public:
    /**
     * document your methods too.
     */
    reservoirHandler(ResourceFinder &rf);
    ~reservoirHandler();

    bool configure(ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod();
    bool updateModule();
};


#endif // __RESERVOIRHANDLER_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

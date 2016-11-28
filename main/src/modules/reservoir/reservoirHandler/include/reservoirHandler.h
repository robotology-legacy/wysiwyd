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

#include "wrdac/clients/icubClient.h"

class reservoirHandler : public yarp::os::RFModule {
private:

    std::string testObject;
    std::string testAction;
    std::string testLocation;

    double offsetGrasp;

    std::string moduleName;
    std::string sKeyWord;

    std::string handlerPortName;
    std::string port2SpeechRecogName;
    std::string port2iSpeakName;

    std::string nameGrammarNodeType;
    std::string nameGrammarNodeModality;
    std::string nameGrammarNodeTrainAP;
    std::string nameGrammarNodeTestAP;
    std::string nameGrammarNodeTrainSD;
    std::string nameGrammarYesNo;

    std::string fileAPimputS;
    std::string fileAPoutputM;
    std::string fileSRinputM;
    std::string fileSRoutputS;
    std::string fileXavierTrain;
    std::string fileXavierTrainAP;
    std::string fileSD;
    std::string fileAP;

    std::string pythonPath;

    std::string sHand;
    std::string bMode;
    double ZRTObjects;


    yarp::os::Port handlerPort;               // a port to handle messages
    yarp::os::Port Port2SpeechRecog;      // a port to send grammar to the speech recog
    yarp::os::Port Port2iSpeak;       // a port to send grammar to the speech recog

    bool isAwake;
    wysiwyd::wrdac::ICubClient *iCub;

    int iCurrentInstance;                   // instance of the current request
    int inbsentence;
    std::string sCurrentActivity;
    std::string sCurrentType;
    std::string sCurrentNode;
    std::string sCurrentCanonical;
    std::string sCurrentGrammarFile;
    std::string sLastSentence;
    std::string sSentence_type;
    std::string sSentence;
    std::string sdataTestSD;
    // last sentence said (in case of a repeat)
    std::pair<std::string, std::string> psCurrentComplement;

    std::string sentence;
    std::list<std::string> lMeaningsSentences;
    //Vector coordinates;

    std::ofstream fileVectorAP;
    std::ifstream fileVectorAPRead;
    std::string nameGrammarNodeInteraction;
    std::string svector, sanswer;
    std::string fvector;
    std::string sVectorFileAP;
    std::string sagent;
    int iquestion;
    std::list<int> nbCaracters(std::string ssequence);
    std::string sConstrualLocation;
    std::string sobjectFocusChanged;
    std::map<std::string, std::string> mAssociation;


    bool nodeType();
    bool nodeModality();
    bool nodeTrainAP();
    bool nodeTestAP();
    bool nodeTestSD();
    bool nodeYesNo();
    bool nodeYesNoInteraction();

    int languageNodeInteraction();
    bool languageNodeInteractionSD();
    bool grammarNodeInteraction();
    bool mainNodeInteraction();

    bool createVectorFile(std::string sVectorFile);
    bool spatialRelation();
    bool launchSpatialRelation();

    bool callReservoir(std::string fPython);
    std::string  grammarToString(std::string sPath);

    int copyTrainData(const char* fileNameIn, const char* fileNameOut);
    int copyPastFile(const char* in, const char* fileNameOut);
    int trainSaveMeaningSentence(const char *filename);
    int createTestwithTrainData(const char* filename, std::string sMeaningSentence);
    std::string openResult(const char* fileNameIn);
    bool AREactions(std::vector<std::string> seq);
    std::vector<std::string> extractVocabulary(std::string sequence);


    // For simulator
    bool populateOPC();
    bool testARE();

public:
    /**
     * document your methods too.
     */

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();
};


#endif // __RESERVOIRHANDLER_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

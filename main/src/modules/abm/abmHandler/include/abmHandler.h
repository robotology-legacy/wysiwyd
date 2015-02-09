/* 
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
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

#ifndef _ABMHANDLER_H_
#define _ABMHANDLER_H_

#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"


class abmHandler : public yarp::os::RFModule {
private:
    std::string moduleName;
    std::string sKeyWord;

    std::string handlerPortName;
    std::string port2SpeechRecogName;
    std::string port2abmName;
    std::string port2abmReasoningName;
    std::string port2iSpeakName;
    std::string port2OPCmanagerName;
    std::string port2BodySchemaName;

    std::string nameGrammarNode1;
    std::string nameGrammarNode2;
    std::string nameGrammarNode3;
    std::string nameGrammarNode4;

    yarp::os::Port handlerPort;             // a port to handle messages 
    yarp::os::Port Port2SpeechRecog;        // a port to send grammar to the speech recog
    yarp::os::Port Port2ABM;                // a port to communicate with autobiographicalMemory
    yarp::os::Port Port2abmReasoning;       // a port to communicate with the reasoning module
    yarp::os::Port Port2OPCManager;         // a port to communicate with the OPC manager to imagine the memory
    yarp::os::Port Port2iSpeak;             // a port to communicate with the speech synthesis
    yarp::os::Port Port2BodySchema;         // a port to communicate with the Body Schema module

    bool isAwake;
    wysiwyd::wrdac::ICubClient *iCub;

    int iCurrentInstance;                   // instance of the current request
    std::string sCurrentActivity;
    std::string sCurrentPronoun;
    std::string sCurrentNode;
    std::string sCurrentGrammarFile;
    std::string sLastSentence;                   // last sentence said (in case of a repeat)
    std::pair<std::string, std::string> psCurrentComplement;


    yarp::os::Bottle node1();
    yarp::os::Bottle node2();
    yarp::os::Bottle node3();

    std::string dateToSpeech(std::string sDate);

    std::string  grammarToString(std::string sPath);

public:
    /** 
    * document your methods too.
    */
    ~abmHandler();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};


#endif // __ABMHANDLER_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------


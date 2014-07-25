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

#include <iostream>
#include <fstream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <time.h>

#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace yarp::sig;


class abmHandler : public RFModule {
private:
    string moduleName;
    string sKeyWord;

    string handlerPortName;
    string port2SpeechRecogName;
    string port2abmName;
    string port2abmReasoningName;
    string port2iSpeakName;
    string port2OPCmanagerName;

    string nameGrammarNode1;
    string nameGrammarNode2;
    string nameGrammarNode3;
    string nameGrammarNode4;

    yarp::os::Port handlerPort;             // a port to handle messages 
    yarp::os::Port Port2SpeechRecog;        // a port to send grammar to the speech recog
    yarp::os::Port Port2ABM;                // a port to communicate with autobiographicalMemory
    yarp::os::Port Port2abmReasoning;       // a port to communicate with the reasoning module
    yarp::os::Port Port2OPCManager;         // a port to communicate with the OPC manager to imagine the memory
    yarp::os::Port Port2iSpeak;             // a port to communicate with the speech synthesis

    bool isAwake;
    ICubClient *iCub;

    int iCurrentInstance;                   // instance of the current request
    string sCurrentActivity;
    string sCurrentPronom;
    string sCurrentNode;
    string sCurrentGrammarFile;
    string sLastSentence;                   // last sentence said (in case of a repeat)
    pair<string, string> psCurrentComplement;


    Bottle node1();
    Bottle node2();
    Bottle node3();

    string dateToSpeech(string sDate);

    string  grammarToString(string sPath);

public:
    /** 
    * document your methods too.
    */
    abmHandler(ResourceFinder &rf);
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


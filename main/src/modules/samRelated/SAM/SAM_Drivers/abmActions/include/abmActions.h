// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Uriel Martinez, Luke Boorman
* email:   uriel.martinez@sheffield.ac.uk
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* $WYSIWYD_ROOT/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#ifndef __ABMACTIONS_H__
#define __ABMACTIONS_H__

/*
* \defgroup speechInteraction
* @ingroup wysiwyd_modules
*
*
* Receives recognised words and triggers the corresponding behaviour
*
* \author Uriel Martinez, Luke Boorman
*
* Copyright (C) 2015 WYSIWYD Consortium\n
* CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
*
*/


#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <ctime>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include "wrdac/clients/icubClient.h"
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/knowledge/object.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::dev;
using namespace std;
using namespace wysiwyd::wrdac;


class abmActions: public RFModule
{
    private:
        ICubClient *iCub;
            bool ABMconnected;
            SubSystem_ABM* SubABM;

        vector<string> inputVocabs;
        vector<string> outputVocabs;
        
	    BufferedPort<Bottle> outputPort;
	    BufferedPort<Bottle> inputPort;
	    BufferedPort<Bottle> triggerBehaviourPort;

    std::string m_masterName;

            yarp::os::RpcClient rpcPort;

        Port rpc;
        string GrammarAskNamePerson;
        string moduleName;

	    string inputPortName;	   	    
	    string outputPortName;	   	    
        string triggerBehaviourPortName;
        bool outputOpen;
        bool inputOpen;
        bool behaviourPortOpen;
        int nVocabs;
        int speechType;

    string          img_provider_port ;
    string          agentName ;
    string          resume ;
    int             rememberedInstance;
    int instanceIMG;

    public:
        abmActions();
//        abmInteraction()
//        {
//            speechType = 0;
//        }

        ~abmActions();
        bool updateModule();
        bool configure(ResourceFinder &);
        bool interruptModule();
        double getPeriod();
        bool matchVocab(string, int *);
        void sendSpeech(int);
        void triggerBehaviour(int);
        bool close();
};

#endif // __ABMACTIONS_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------


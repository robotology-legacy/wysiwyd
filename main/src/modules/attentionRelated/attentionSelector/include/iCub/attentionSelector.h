// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Stéphane Lallée, moved from EFAA by Maxime Petit
* email:   stephane.lallee@gmail.com
* website: http://wysiwyd.upf.edu/ 
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

#ifndef _ATTENTIONSELECTOR_MODULE_H_
#define _ATTENTIONSELECTOR_MODULE_H_

/** 
*
* \defgroup attentionSelector
* @ingroup wysiwyd_modules
*
* Allow to control the gaze using objects names. Provide a random autonomous switch of attention between the present objects.

The commands it is possible to send are:

@b track "objectname" track an object based on its name
@b track id track an object based on its opc ID
@b track (x y z) : look at a position directly expressed in the robot reference frame
@b auto : turn on the autonomous switch of attention
@b sleep : turn off the autonomous switch of attention
@b look (x y z) : sleep + gaze at xyz

* \section lib_sec Libraries
*
* YARP, iCub.
*
* \section parameters_sec Parameters
* 
* <b>Command-line Parameters</b> 
* 
* The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
* (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
*
* - \c from \c attentionSelector.ini \n 
*   specifies the configuration file
*
* - \c name \c attentionSelector \n   
*   specifies the name of the module (used to form the stem of module port names)  
*
* <b>Configuration File Parameters </b>
*
* The following key-value pairs can be specified as parameters in the configuration file 
* (they can also be specified as command-line parameters if you so wish). 
* The value part can be changed to suit your needs; the default values are shown below. 
*   
* - \c opcName \c OPC \n    
*   specifies the opc database name
* 
* \section tested_os_sec Tested OS
*
* Linux, windows
*
* \section example_sec Example Instantiation of the Module
* 
* <tt>attentionSelector --name attentionSelector --context
* attentionSelector --from attentionSelector.ini</tt>
*
* \author Stéphane Lallée
* 
* Copyright (C) 2014 WYSIWYD Consortium\n
* CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
* 
*/

#include <string>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <wrdac/clients/opcClient.h>
#include <wrdac/clients/icubClient.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

#define ACCELERATION_COEFFICIENT 1.0/1.0
//Define the quantity of saliency lost every update for the tracked object
#define FOCUS_LEAKING 0.1

enum AttentionState { s_waiting, s_exploring, s_tracking };

struct ObjectModel
{
    Object o;
    double speed;
    double acceleration;
};

/**
* Module in charge of polling the OPC and updating icubGUI
*/
class attentionSelectorModule : public RFModule {
    string moduleName;
    string opcName;    

    OPCClient *opc;             //retrieve information from the OPC
    ICubClient *icub_client;    //it serves to open up the icub client
    SubSystem_ARE *are;         //command the gaze through ARE subsystem
    Port handlerPort;           //a port to handle messages 

    Agent* icub;
    map<string, ObjectModel> presentObjectsLastStep;
    vector<string>           presentObjects;
    AttentionState aState;
    string trackedObject;
    int store_context_id;
    bool autoSwitch;                //Defines if attention autonomously switch between present objects

    double timeLastSwitch;
    double trackSwitchingPeriod;

protected:
    void exploring();
    bool isFixationPointSafe(const Vector &fp);

    double x_coord;
    double y_coord;
    double z_coord;
    bool trackedCoordinates;

public:

    bool configure(ResourceFinder &rf);             // configure all the module parameters and return true if successful
    bool interruptModule();                         // interrupt, e.g., the ports 
    bool close();                                   // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};

#endif


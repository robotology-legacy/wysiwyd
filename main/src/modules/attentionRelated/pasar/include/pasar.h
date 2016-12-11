// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2014 EFAA Consortium, European Commission FP7 Project IST-270490
* Authors: Stéphane Lallée, Grégoire Pointeau
* email:   stephane.lallee@gmail.com, gregoire.pointeau@inserm.fr
* website: http://efaa.upf.edu/
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* $WYSIWYD/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#ifndef _PASAR_MODULE_H_
#define _PASAR_MODULE_H_

#include <wrdac/clients/clients.h>

using namespace wysiwyd::wrdac;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

struct ObjectModel
{
    Object o;
    double speed;
    double acceleration;
    int restingSteps;
    double lastTimeSeen;
    bool present;
};


/**
* Module in charge of polling the OPC and updating icubGUI
*/
class PasarModule : public yarp::os::RFModule {
    //Parameter
    double pTopDownAppearanceBurst;         // score of saliency for an appereance
    double pTopDownDisappearanceBurst;      // score of saliency for an diappereance
    double pTopDownAccelerationCoef;        // score of saliency for an acceleration detected
    double pExponentialDecrease;            // Speed of the decrease of the saliency over the time (should be less than 1)
    double pTopDownWaving;                  // increase of saliency if waving
	double thresholdMovementAccelAgent;          // minimum acceleration to detect an agent
	double thresholdMovementAccelObject;          // minimum acceleration to detect an object
	double thresholdWaving;                 // minimum waving detected
	double thresholdPointing;                 // minimum pointing detected
	double rangeHaving;                 // minimum range to have an object detected
	double persistenceHaving;                 // minimum life time of a relation "having"
	double thresholdSaliency;
    double dthresholdAppear;
    double dthresholdDisappear;
    double dBurstOfPointing;

    double lastTimeWaving;
    double lastTimePointing;

	bool checkWaving;
	bool checkHaving;
	bool checkPointing;
    ICubClient  *iCub;

    yarp::os::Port handlerPort;      //a port to handle messages 

    list<shared_ptr<Entity>> entities;

    yarp::sig::Vector rightHandt1;  // position of right at t1
    yarp::sig::Vector rightHandt2;  // position of right at t2
    yarp::sig::Vector leftHandt1;   // position of left hand at t1
    yarp::sig::Vector leftHandt2;   // position of left hand at t2

    pair<bool, bool> presentRightHand;
    pair<bool, bool> presentLeftHand;

    map<int, ObjectModel>  presentObjectsLastStep;
    map<int, pair<double, double> > presentLastSpeed;
    map<int, pair<double, double> > presentCurrentSpeed;
    map<int, ObjectModel>  OPCEntities;


    std::string trackedObject;

    bool isPointing; // if the human is pointing
    bool isWaving; // if if the human is waving
    double initTime;

protected:
    void saliencyTopDown();
    void saliencyNormalize();
    void saliencyLeakyIntegration();
    bool saliencyPointing();
    bool saliencyWaving();
    void initializeMapTiming();
	void updateMapTiming();
	void checkAgentHaving();

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();
};

#endif

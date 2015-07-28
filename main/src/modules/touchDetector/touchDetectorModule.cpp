/* 
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Bertand Higy
 * email:  bertrand.higy@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "touchDetectorModule.h"

using namespace std;
using namespace yarp::os;

bool TouchDetectorModule::configure(ResourceFinder &rf)
{    
	// Initialize parameters
	initializeParameters(rf);
	
	// open ports
	if (!openPorts())
		return false;

   	// create the thread and pass pointers to the module parameters
	thread = new TouchDetectorThread(&torsoPort, &leftArmPort, &rightArmPort, &leftForearmPort, &rightForearmPort, &leftHandPort, &rightHandPort, &touchPort, period, &clustersConfFilepath, threshold);
	// now start the thread to do the work
	thread->start(); // this calls threadInit() and it if returns true, it then calls run()

   	return true ;
}

bool TouchDetectorModule::updateModule()
{
	return true;
}

bool TouchDetectorModule::interruptModule()
{
	torsoPort.interrupt();
	leftArmPort.interrupt();
	rightArmPort.interrupt();
	leftForearmPort.interrupt();
	rightForearmPort.interrupt();
	leftHandPort.interrupt();
	rightHandPort.interrupt();
	touchPort.interrupt();
	return true;
}

bool TouchDetectorModule::close()
{
	/* stop the thread */
	thread->stop();

	torsoPort.close();
	leftArmPort.close();
	rightArmPort.close();
	leftForearmPort.close();
	rightForearmPort.close();
	leftHandPort.close();
	rightHandPort.close();
	touchPort.close();
	return true;
}

void TouchDetectorModule::initializeParameters(ResourceFinder &rf)
{
	moduleName = rf.check("name", Value("touchDetector"), "Module name (string)").asString();
	setName(moduleName.c_str());
	period = rf.check("period", Value(1000 / 30), "Thread period (string)").asInt();
	threshold = rf.check("threshold", Value(50), "Activation threshold (string)").asInt();
	rf.setDefault("clustersConfFile", Value("clustersConfig.ini"));
	clustersConfFilepath = rf.findFile("clustersConfFile");
	
	// get the name of the input and output ports, automatically prefixing the module name by using getName()
	torsoPortName = "/";
	torsoPortName += getName(rf.check("torsoPort", Value("/torso:i"), "Torso input port (string)").asString());
	leftArmPortName = "/";
	leftArmPortName += getName(rf.check("leftArmPort", Value("/left_arm:i"), "Left arm input port (string)").asString());
	rightArmPortName = "/";
	rightArmPortName += getName(rf.check("rightArmPort", Value("/right_arm:i"), "Right arm input port (string)").asString());
	leftForearmPortName = "/";
	leftForearmPortName += getName(rf.check("leftForearmPort", Value("/left_forearm:i"), "Left forearm input port (string)").asString());
	rightForearmPortName = "/";
	rightForearmPortName += getName(rf.check("rightForearmPort", Value("/right_forearm:i"), "Right forearm input port (string)").asString());
	leftHandPortName = "/";
	leftHandPortName += getName(rf.check("leftHandPort", Value("/left_hand:i"), "Left hand input port (string)").asString());
	rightHandPortName = "/";
	rightHandPortName += getName(rf.check("rightHandPort", Value("/right_hand:i"), "Right hand input port (string)").asString());
	touchPortName = "/";
	touchPortName += getName(rf.check("touchPort", Value("/touch:o"), "Touch output port (string)").asString());
}

bool TouchDetectorModule::openPorts()
{
	if (!torsoPort.open(torsoPortName.c_str())) {
		cerr << getName() << ": unable to open port " << torsoPortName << endl;
		return false;
	}
	if (!leftArmPort.open(leftArmPortName.c_str())) {
		cerr << getName() << ": unable to open port " << leftArmPortName << endl;
		return false;
	}
	if (!rightArmPort.open(rightArmPortName.c_str())) {
		cerr << getName() << ": unable to open port " << rightArmPortName << endl;
		return false;
	}
	if (!leftForearmPort.open(leftForearmPortName.c_str())) {
		cerr << getName() << ": unable to open port " << leftForearmPortName << endl;
		return false;
	}
	if (!rightForearmPort.open(rightForearmPortName.c_str())) {
		cerr << getName() << ": unable to open port " << rightForearmPortName << endl;
		return false;
	}
	if (!leftHandPort.open(leftHandPortName.c_str())) {
		cerr << getName() << ": unable to open port " << leftHandPortName << endl;
		return false;
	}
	if (!rightHandPort.open(rightHandPortName.c_str())) {
		cerr << getName() << ": unable to open port " << rightHandPortName << endl;
		return false;
	}
	if (!touchPort.open(touchPortName.c_str())) {
		cerr << getName() << ": unable to open port " << touchPortName << endl;
		return false;
	}
	return true;
}


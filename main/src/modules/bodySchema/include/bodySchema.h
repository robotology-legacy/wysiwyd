/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Martina Zambelli
 * email:   m.zambelli13@imperial.ac.uk
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

#ifndef _BODYSCHEMA_H_
#define _BODYSCHEMA_H_

#include <iostream>
#include <fstream> 
#include <sstream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"
#include "otl.h"
#include "otl_oesgp.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::sig::draw;
using namespace wysiwyd::wrdac;
using namespace OTL;

class bodySchema : public RFModule {
private:
  string moduleName;
  int isVerbose;
	
	string handlerPortName;
	string portImageInRightName;	
	string portImageInLeftName;
	string portRightArmStateName;
        string portLeftArmStateName;
	string portPredictionsName;
	string portPredictionErrorsName;
	string portReadPredictionsName;
	string portReadPredictionErrorsName;
	string portInDataName;
	string portOutDataName;
	string portReadInDataName;
	string portReadOutDataName;
	
        Port handlerPort;// a port to handle messages 
	Port portImageInRight;	
	Port portImageInLeft;
	Port portRightArmState;
        Port portLeftArmState;
	BufferedPort<Bottle> portPredictions;
	BufferedPort<Bottle> portPredictionErrors;
	BufferedPort<Bottle> portReadPredictions;
	BufferedPort<Bottle> portReadPredictionErrors;
	BufferedPort<Bottle> portInData;
	BufferedPort<Bottle> portOutData;
	BufferedPort<Bottle> portReadInData;
	BufferedPort<Bottle> portReadOutData;


	ofstream encodersData; 
	ofstream velocitiesData;
	ofstream imageData;
	ofstream commandData;
	ofstream dataDumperRecJointsPos;
	
	Vector encoders, cmd;
	double cmdRightArm;
  	IPositionControl* pos;
  	IVelocityControl* vel;
  	IEncoders* encs;
	IControlMode2 *ictrl;
    	IInteractionMode *iint;
 	
	Vector encodersHead, commandHead;
  	IPositionControl* posHead;
  	IVelocityControl* velHead;
  	IEncoders* encsHead;
  
	PolyDriver* leftArmDev; 
	PolyDriver* rightArmDev;  
	PolyDriver* robotHeadDev;
	
	
	OESGP oesgp;
	OESGP oesgp2;

	string fileIn;
	string fileOut;

    int nInputs;
    int nOutputs;	

    double initTime;
	
    bool isLearning;
    bool isBabblingLearning;
	
	
//  ICubClient *iCub;

	
	//YARP_OS_API void getName(const yarp::os::ConstString& str);

public:
    /** 
     * document your methods too.
     */
    bodySchema(ResourceFinder &rf);
    ~bodySchema();

    bool configure(ResourceFinder &rf); // configure 
    bool interruptModule();        // interrupt, e.g., the ports 
    bool close();               // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod(); 
    bool updateModule();
    

    bool learn();
    bool learn(string &fileNameIn, string &fileNameOut);
//    bool learn(BufferedPort<Bottle> &portIn, BufferedPort<Bottle> &portOut);

    
private:

    string portImageLeftName;	
    string portImageTargetLeftName;
    BufferedPort<ImageOf<PixelRgb> > portImageLeft;
    BufferedPort<Vector> portImageTargetLeft;
    
    double prev_x;
    double prev_y;
    double prev_xMean;
    double prev_yMean;
    Vector setpoints;
    
    void find_image();

    bool init_iCub(string &part);
};


#endif // __BODYSCHEMA_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

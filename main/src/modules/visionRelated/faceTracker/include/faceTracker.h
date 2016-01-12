// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Hyung Jin Chang
* email:   hj.chang@imperial.ac.uk
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

#ifndef _FACETRACKER_MODULE_H_
#define _FACETRACKER_MODULE_H_

/*
* \defgroup faceTracker
* @ingroup wysiwyd_modules
*
*
* Allow to control the gaze using objects names. Provide a random autonomous switch of attention between the present objects.

* Face tracking module for iCub head and gaze using OpenCV2.X functions

* Using the face detection functions of OpenCV, the iCub is detecting faces every frame. Then, it tries to located the biggest face in the middle of the view by moving head and eye simultaneously.
	- Image sequences from left eye are used for tracking (not both eyes).
	- There are five modes for the tracking internally: 'set position mode', 'panning mode', 'face tracking mode, 'smooth stopping mode' and 'face searching mode'.
*
* \author Hyung Jin Chang
*
* Copyright (C) 2014 WYSIWYD Consortium\n
* CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
*
*/

#include <stdio.h>
#include <iostream>
#include <ctime>

#ifdef _WIN32
#include <io.h>
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep(x)
#endif

#include <cmath>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/Drivers.h>

#include <wrdac/clients/opcClient.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;

using namespace wysiwyd::wrdac;

/**
* Module in charge of polling the OPC and updating icubGUI
*/
class faceTrackerModule : public yarp::os::RFModule {
    std::string moduleName;
    std::string opcName;

    std::string faceTrackerPortName;
    std::string handlerPortName;

    OPCClient *opc;					 //retrieve information from the OPC
    yarp::os::Port handlerPort;      //a port to handle messages

    Agent* icub;

    BufferedPort<ImageOf<PixelRgb> > imagePortLeft; 	// make a port for reading left images
    //BufferedPort<ImageOf<PixelRgb> > imagePortRight; 	// make a port for reading right images

    // ==================================================================
	// robot
	IPositionControl *pos;
	IVelocityControl *vel;
	IEncoders *enc;

    PolyDriver *robotHead;

	Vector setpoints;
	Vector cur_encoders;
	Vector prev_encoders;

    IControlMode2 *ictrl;

protected:
    //void exploring();

	int counter;
	double x_buf;
	double y_buf;

	int mode; // 0: going to a set position, 1: face searching, 2: face tracking, 3: face stuck,
	int setpos_counter;
	int panning_counter;
	int stuck_counter;
	int tracking_counter;

	// ------------------------------
	// random motion
	int tilt_target;
	int pan_target;

	double pan_r, tilt_r;
	int seed;
	int pan_max;
	int tilt_max;

    int jnts;

    cv::CascadeClassifier face_classifier_left;
	IplImage *cvIplImageLeft;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();
};


#endif // __FACETRACKER_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------


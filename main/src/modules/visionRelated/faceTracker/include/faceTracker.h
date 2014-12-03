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

/**
* @ingroup icub_module
*
* \defgroup modules faceTracker
*
* Allow to control the gaze using objects names. Provide a random autonomous switch of attention between the present objects.

The commands it is possible to send are:

@b track "objectname" track an object based on its name
@b track id track an object based on its opc ID
@b track (x y z) : look at a position directly expressed in the robot reference frame
@b auto : turn on the autonomous switch of attention
@b sleep : turn off the autonomous switch of attention
@b look (x y z) : sleep + gaze at xyz
@b waitMotionDone : blocking call until the motion is done (map to iKinGazeCtrl waitMotionDone() )
@b getFixationPoint : return a bottle filled with the current fixation point (map to iKinGazeCtrl getFixationPoint() )
@b getHeadPose : return a bottle filled with the current head pose (map to iKinGazeCtrl getHeadPose() )

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
* - \c from \c faceTracker.ini \n
*   specifies the configuration file
*
* - \c name \c faceTracker \n
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
* <tt>faceTracker --name faceTracker --context
* faceTracker --from faceTracker.ini</tt>
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

//#include <opencv/cv.h>
//#include <opencv/cvaux.h>
//#include <opencv/highgui.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/stitching/stitcher.hpp>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
//#include <highgui/highgui.hpp>
//#include <stitching/stitcher.hpp>

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

protected:
    //void exploring();


	int counter;
	int x_buf;
	int y_buf;

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


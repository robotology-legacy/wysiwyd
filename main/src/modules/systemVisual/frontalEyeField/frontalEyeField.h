// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 WYSIWYD Consortium
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com
 * website: http://efaa.upf.edu/ 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * $EFAA_ROOT/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef FRONTAL_EF
#define FRONTAL_EF

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/Drivers.h>
#include "cvz/core/all.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class LeakyIntegrator
{
public:

	double x;
	double leak;
	double inputFactor;

	LeakyIntegrator()
	{
		x = 0.0;
		leak = 0.0;
		inputFactor = 1000.0;
	}

	void update(double input)
	{
		x = input;// -leak*x + inputFactor*input;
		//x = min(max(0.0, x), 1.0);
	}
};

class FrontalEyeField : public yarp::os::RFModule 
{
private:
    std::string moduleName;
    std::string retinaName;
    std::string gazePortName;

    PolyDriver clientGazeCtrl;
	IGazeControl *igaze;
	int store_context_id;
	int cameraUsed;
	double timeNextSaccade;
	vector < vector<BufferedPort<Bottle> * > >  retinaInput;
	vector < vector< LeakyIntegrator > > errorMap;
	cvz::core::ThreadedCvz* mmcmErrorPrediction;
	int retinaW, retinaH;

	double tau;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    double getPeriod(); 
    bool updateModule();

	void connectErrorInput(std::string splitterPrefix, std::string splitterSuffix);
	void resetErrorMap();
};

#endif
//----- end-of-file --- ( next line intentionally left blank ) ------------------


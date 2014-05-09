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

#include "frontalEyeField.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/************************************************************************/
bool FrontalEyeField::configure(yarp::os::ResourceFinder &rf) {

	//Get conf parameters
	moduleName = rf.check("name",Value("frontalEyeField")).asString();
	setName(moduleName.c_str());
	tau = rf.check("tau", Value(0.1)).asDouble();
	retinaW = rf.check("retinaW", Value(3)).asInt();
	retinaH = rf.check("retinaH", Value(3)).asInt();
	string nameSourcePrefix = rf.check("nameSplitterPrefix", Value("/v1/in_")).asString();
	string nameSourceSuffix = rf.check("nameSplitterSuffix", Value("/error:o")).asString();
	//string nameSourcePrefix = rf.check("nameSplitterPrefix", Value("/retina/")).asString();
	//string nameSourceSuffix = rf.check("nameSplitterSuffix", Value("/retina/error:o")).asString();

	//Open the gaze controller
	gazePortName = "/";
	gazePortName += getName() + "/gaze";
	Property option;
	option.put("device", "gazecontrollerclient");
	option.put("remote", "/iKinGazeCtrl");
	option.put("local", gazePortName.c_str());

	igaze = NULL;
	if (clientGazeCtrl.open(option)) {
		clientGazeCtrl.view(igaze);
	}
	else
	{
		cout << "Invalid gaze polydriver" << endl;
		return false;
	}

	igaze->storeContext(&store_context_id);

	double neckTrajTime = rf.check("neckTrajTime",
		Value(0.75)).asDouble();
	igaze->setNeckTrajTime(neckTrajTime);
	igaze->bindNeckPitch(-10.0, 10.0);
	igaze->bindNeckRoll(-10.0, 10.0);
	igaze->bindNeckYaw(-10.0, 10.0);

	//Open the retina input ports
	retinaInput.resize(retinaW);
	errorMap.resize(retinaW);
	for (int x = 0; x < retinaW; x++)
	{
		retinaInput[x].resize(retinaH);
		errorMap[x].resize(retinaH);
		for (int y = 0; y < retinaH; y++)
		{
			stringstream ss;
			ss << "/" << moduleName << "/error/" << x << "_" << y << ":i";
			retinaInput[x][y] = new BufferedPort<Bottle>();
			retinaInput[x][y]->open(ss.str().c_str());
		}
	}
	//Wait for the connection
	connectErrorInput(nameSourcePrefix, nameSourceSuffix);

	return true;
}

/************************************************************************/
void FrontalEyeField::connectErrorInput(std::string splitterPrefix, std::string splitterSuffix)
{
	for (int x = 0; x < retinaW; x++)
	{
		for (int y = 0; y < retinaH; y++)
		{
			stringstream ssSource;
			ssSource << splitterPrefix << x << "_" << y << splitterSuffix;
			int attempt = 0;
			while (attempt < 5 && !Network::connect(ssSource.str().c_str(), retinaInput[x][y]->getName().c_str()))
			{
				std::cout << "Attempt to automated connection failed for : " << std::endl
					<< "\t" << ssSource.str().c_str() << std::endl
					<< "\t" << retinaInput[x][y]->getName().c_str() << std::endl;
				Time::delay(0.5);
				//attempt++;
			}

		}
	}
}
/************************************************************************/
bool FrontalEyeField::interruptModule() {
	for (int x = 0; x < retinaW; x++)
	{
		retinaInput[x].resize(retinaH);
		for (int y = 0; y < retinaH; y++)
		{
			retinaInput[x][y]->interrupt();
		}
	}
    return true;
}

/************************************************************************/
bool FrontalEyeField::close() {

    igaze->restoreContext(store_context_id);
    if (clientGazeCtrl.isValid())
        clientGazeCtrl.close();

	for (int x = 0; x < retinaW; x++)
	{
		retinaInput[x].resize(retinaH);
		for (int y = 0; y < retinaH; y++)
		{
			retinaInput[x][y]->close();
		}
	}
    return true;
}


/***************************************************************************/
bool FrontalEyeField::updateModule() 
{
	//Update the error map activity
	int xMax = 0;
	int yMax = 0;
	//std::cout << "Receiving error:" << endl;
	for (int x = 0; x < retinaW; x++)
	{
		for (int y = 0; y < retinaH; y++)
		{
			Bottle* bError = retinaInput[x][y]->read(true);
			if (bError)
			{
				errorMap[x][y].update(bError->get(0).asDouble());
				//std::cout << bError->get(0).asDouble() << '\t';
				if (errorMap[x][y].x>errorMap[xMax][yMax].x)
				{
					xMax = x;
					yMax = y;
				}
			}
		}
		//std::cout << endl;
	}
	//std::cout << endl;
	
	//Print it
	/*
	std::cout << "Activity :" << endl;
	for (int x = 0; x < retinaW; x++)
	{
		for (int y = 0; y < retinaH; y++)
		{
			cout<<errorMap[x][y].x<<'\t';
		}
		cout << endl;
	}
	cout << endl << endl;*/

	//Send a gaze command to the location of this high error
	double SACCADE_TRESHOLD = 0.;
	cout << errorMap[xMax][yMax].x << endl;
	if (errorMap[xMax][yMax].x > SACCADE_TRESHOLD)
	{
		//Reset the whole error map
		resetErrorMap();

		//Send the saccade to x/y
		int caseW = 320.0 / (double)retinaW;
		int caseH = 240.0 / (double)retinaH;
		Vector px(2);
		px[0] = xMax * caseW + caseW/2.0;
		px[1] = yMax * caseH + caseH/2.0;
		std::cout << "Saccade to : " << px.toString() << endl;
		igaze->lookAtMonoPixel(0, px);
		//Time::delay(2.0);
	}
    return true;
}


/************************************************************************/
void FrontalEyeField::resetErrorMap()
{
	for (int x = 0; x < retinaW; x++)
	{
		for (int y = 0; y < retinaH; y++)
		{
			errorMap[x][y].x = 0.0;
		}
	}
}

/************************************************************************/
double FrontalEyeField::getPeriod() {
	return tau;
}


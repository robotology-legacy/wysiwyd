/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project
 * Authors: Stephane Lallee
 * email:   stephane.lallee@gmail.com
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
#include <iostream>
#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/Drivers.h>
#include <vector>
#include <map>
#include <wrdac/clients/icubClient.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;
using namespace wysiwyd::wrdac;

struct RobotPart
{
	PolyDriver *pd;
	IEncoders *encs;
	IPositionControl *ctrlPos;
	bool isValid;
	string moduleName, robotName, partName;
	RobotPart(string _moduleName, string _robot, string _part)
	{
		moduleName = _moduleName;
		robotName = _robot;
		partName = _part;
	}

	bool open()
	{
		//Open motor interfaces
		string remoteName = "/";
		remoteName += robotName;
		remoteName += "/";
		remoteName += partName;

		string localName = "/";
		localName += moduleName;
		localName += "/";
		localName += partName;

		Property options;
		options.put("robot", "icub");
		options.put("device", "remote_controlboard");
		options.put("remote", remoteName.c_str());
		options.put("local", localName.c_str());

		pd = new PolyDriver(options);
		if (!pd->isValid()) {
			cout << "Device not available." << endl;
			Network::fini();
			return false;
		}

		pd->view(ctrlPos);

		if (ctrlPos == NULL)
		{
			cout << "Problem while acquiring the position ctrl view" << endl;
			Network::fini();
			return false;
		}

		pd->view(encs);

		if (encs == NULL)
		{
			cout << "Problem while acquiring the encoders view" << endl;
			Network::fini();
			return false;
		}

		cout << "Part " <<partName<< "opened succesfully"<< endl;
		return true;
	}

	bool close()
	{
		bool error = pd->close();
		delete pd;
		return error;
	}
};

class RobotInstance
{
private:
	map<string, RobotPart*> parts;
public:

	RobotInstance(string moduleName, string robotName, Bottle partList)
	{
		for (int i = 0; i < partList.size(); i++)
		{
			string currentPart = partList.get(i).asString();
			parts[currentPart] = new RobotPart(moduleName, robotName, currentPart);
		}
	}

	bool open()
	{
		bool error = true;
		for (map<string, RobotPart*>::iterator it = parts.begin(); it != parts.end(); it++)
		{
			error &= it->second->open();
		}
		return error;
	}

	bool close()
	{
		bool error = true;
		for (map<string, RobotPart*>::iterator it = parts.begin(); it != parts.end(); it++)
		{
			if (it->second->pd != NULL)
				error &= it->second->close();
		}
		return error;
	}

	BodyPosture getCurrentPosture()
	{
		BodyPosture p;
		p.head.resize(6);
		p.torso.resize(3);
		p.right_arm.resize(16);
		p.left_arm.resize(16);
		if (parts.find("head")!=parts.end()) while(!parts["head"]->encs->getEncoders(p.head.data()));
		if (parts.find("torso") != parts.end()) while (!parts["torso"]->encs->getEncoders(p.torso.data()));
		if (parts.find("right_arm") != parts.end()) while (!parts["right_arm"]->encs->getEncoders(p.right_arm.data()));
		if (parts.find("left_arm") != parts.end()) while (!parts["left_arm"]->encs->getEncoders(p.left_arm.data()));
		return p;
	}
};


//------------------------------------------------------------
class PostureRecorder : public RFModule
{
	RobotInstance *myRobot;
	ICubClient * icub;
	BufferedPort<Bottle> portJoystick;
	int pstrCount;
	string postureRootName;

public:

	bool configure(ResourceFinder &rf)
	{
		string moduleName = rf.check("name", Value("postureRecorder")).asString().c_str();
		string robotName = rf.check("robot", Value("icub")).asString().c_str();
		postureRootName = "generic_";
		pstrCount = 0;
		//if (!rf.check("parts"))
		//{
		//	cout << "A part specification is mandatory (e.g --parts (left_arm right_arm) )" << endl;
		//	return false;
		//}

		//myRobot = new RobotInstance(moduleName, robotName, (*rf.find("parts").asList()));
		myRobot = new RobotInstance(moduleName, robotName, Bottle("head right_arm left_arm torso"));
		if (!myRobot->open())
		{
			cout << "Problem while opening the robot trying to close properly." << endl;
			myRobot->close();
			delete myRobot;
			cout << "Closed." << endl;
			return 0;
		}

		icub = new ICubClient(moduleName, "postureRecorder");

		portJoystick.open("/"+moduleName+"/joystick:i");
		while (!Network::connect("/joystickCtrl/raw_buttons:o", portJoystick.getName()))
		{
			Time::delay(1.0);
			cout << "The joystick is not ready..." << endl;
		}

		cout << "*****************POSTURE RECORDER********************" << endl;
		return true;
	}

	double getPeriod()
	{
		return 0.01;
	}

	bool updateModule()
	{
		int buttonPressed = -1;
		cout << "Ready, waiting for joystick signal to record posture... (blue=save current posture, yellow= save postures to file)" << endl;
		cout << "To quit do ctrl+c and press a button" << endl;

		while (buttonPressed == -1)
		{
			Bottle * bJoystick = portJoystick.read(false);
			if (bJoystick)
			{
				for (int i = 2; i < bJoystick->size(); i++)
				{
					if (bJoystick->get(i).asDouble() == 1)
						buttonPressed = i;
				}
			}
		}

		bool anyButtonPressed = true;
		while (anyButtonPressed)
		{
			Bottle * bJoystick = portJoystick.read(false);
			if (bJoystick)
			{
				anyButtonPressed = false;
				for (int i = 2; i < bJoystick->size(); i++)
				{
					if (bJoystick->get(i).asDouble() == 1)
						anyButtonPressed = true;
				}
			}
		}
		cout << "Button " << buttonPressed << " pressed.";

		switch (buttonPressed)
		{

		case 2:
		{
				  stringstream pName;
				  pName << postureRootName << pstrCount;
				  cout << "Recording current posture as " << pName.str() << endl;
				  icub->addPosture(pName.str(), myRobot->getCurrentPosture());
				  pstrCount++;
				  break;
		}

		case  3:
		{

				   cout << "Recording current posture as a file" << endl;
				   icub->savePostures();
				   break;
		}
		default: cout << "Unknown button." << endl; break;
		}
		return true;
	}

	bool close()
	{
		icub->close();
		myRobot->close();
		return true;
	}
};



int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
        return false;

    ResourceFinder rf;
    rf.configure(argc,argv);
	
	PostureRecorder mod;
	mod.configure(rf);
	return mod.runModule();
}



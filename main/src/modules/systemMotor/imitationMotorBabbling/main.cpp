/*
* Copyright(C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
* Authors: Stephane Lallee
* email : stephane.lallee@gmail.com
* Permission is granted to copy, distribute, and / or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* wysiwyd / license / gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU General
* Public License for more details
*/

#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

int main()
{
	Network::init();

	//Create an iCub Client and check that all dependencies are here before starting
	ICubClient iCub("imitationMotorBabbling","imitationMotorBabbling","client.ini",true);


	char rep = 'n';
	while (rep != 'y'&&!iCub.connect())
	{
		cout << "Some dependencies are not running..." << endl;
		cout << "Continue? y,n" << endl;

		cin >> rep;
		Time::delay(1.0);
	}

	cout << "Connected, starting.." << endl;

	//Moving to safe posture
	iCub.moveToPosture("safe", 5.0);
	std::map<std::string, BodyPosture> allPostures = iCub.getPosturesKnown();
	int pCnt = 0;
	for (std::map<std::string, BodyPosture>::iterator it = allPostures.begin(); it != allPostures.end(); it++)
	{
		if (it->first.find("babbling_") != std::string::npos)
			pCnt++;
	}
	cout << "Found " << pCnt << " babbling postures" << endl;

	int ITERATION_COUNT = 25;

	for (int i = 0; i < ITERATION_COUNT; i++)
	{
		stringstream leftArmPosture, rightArmPosture;
		leftArmPosture << "babbling_" << yarp::os::Random::uniform(0, pCnt);
		rightArmPosture << "babbling_" << yarp::os::Random::uniform(0, pCnt);
		cout << "Sample : " << i << ". Chosen posture : " << leftArmPosture.str() << " for left and " << rightArmPosture.str() << " for right." << endl;
		iCub.moveBodyPartToPosture(leftArmPosture.str(), 2.0, "left_arm");
		iCub.moveBodyPartToPosture(rightArmPosture.str(), 2.0, "right_arm");
		Time::delay(6.0);
	}

    iCub.say("See you next time.");
    iCub.close();
    return 0;
}



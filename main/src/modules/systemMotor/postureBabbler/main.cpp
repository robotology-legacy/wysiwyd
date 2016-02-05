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

class Babbler : public RFModule
{
    ICubClient * icub;
    string postureRootName;
    string inBetweenPostureName;

    int posturesCount;
    list<string> partsControlled;
    double postureReachTime;

    public:

        bool configure(ResourceFinder &rf)
        {
            string moduleName = rf.check("name", Value("postureBabbler")).asString().c_str();
            postureRootName = rf.check("postureRootName", Value("babbling_")).asString().c_str();
            postureReachTime = rf.check("postureReachingTime", Value(3.0)).asDouble();
            inBetweenPostureName = rf.check("inBetweenPosture", Value("safe")).asString();

            Bottle *bParts = rf.find("parts").asList();
            if (bParts == NULL)
                bParts = new Bottle("head left_arm right_arm torso");

            cout << "Controlling parts :" << endl;
            for (int i = 0; i < bParts->size(); i++)
            {
                string part = bParts->get(i).asString();
                partsControlled.push_back(part);
                cout << '\t' << part << endl;
            }

            icub = new ICubClient(moduleName, rf.getContext(), "client.ini", false, true, true);

            char rep = 'n';
            while (rep != 'y'&&!icub->connect())
            {
                cout << "Some dependencies are not running..." << endl;
                cout << "Continue? y,n" << endl;

                cin >> rep;
                Time::delay(1.0);
            }

            //Find all the babbling postures based on the stem name
            std::map<std::string, BodyPosture> allPostures = icub->getPosturesKnown();
            posturesCount = 0;
            for (std::map<std::string, BodyPosture>::iterator it = allPostures.begin(); it != allPostures.end(); it++)
            {
                if (it->first.find(postureRootName) != std::string::npos)
                    posturesCount++;
            }
            cout << "Found " << posturesCount << " babbling postures" << endl;

            icub->moveToPosture(inBetweenPostureName, 5.0);

            cout << "*****************POSTURE BABBLER********************" << endl;
            return true;
        }

        double getPeriod()
        {
            return 1.0;
        }

        bool updateModule()
        {
            stringstream rndPosture;
            rndPosture << postureRootName << yarp::os::Random::uniform(0, posturesCount);
            cout << "Chosen posture : " << rndPosture.str() << endl;

            for (list<string>::iterator itPart = partsControlled.begin(); itPart != partsControlled.end(); itPart++)
            {
                icub->moveBodyPartToPosture(rndPosture.str(), postureReachTime, *itPart);
            }

            Time::delay(postureReachTime);
            cout << "Reaching safe posture " << endl;
            icub->moveToPosture(inBetweenPostureName, postureReachTime);
            Time::delay(postureReachTime);

            return true;
        }

        bool close()
        {
            icub->close();
            delete icub;
            return true;
        }

};

int main(int argc, char *argv[])
{

    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
        return false;

    ResourceFinder rf;
    rf.setDefaultContext("postureBabbler");
    rf.setDefaultConfigFile("default.ini");
    rf.configure(argc, argv);

    Babbler mod;
    mod.configure(rf);
    return mod.runModule();
    return 0;
}



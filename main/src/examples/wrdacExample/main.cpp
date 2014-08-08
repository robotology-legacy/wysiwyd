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
    ICubClient iCub("iCubClientExample");

    string command = "nothing";
            
    char rep = 'n';
    while (rep!='y'&&!iCub.connect())
    {
        cout<<"Some dependencies are not running..."<<endl;
        cout<<"Continue? y,n"<<endl;

        cin>>rep;
        Time::delay(1.0);
    }
            
    cout<<"Connected, starting.."<<endl;

    //This is how you trigger postures, assuming they are defines in conf file
    iCub.moveToPosture("home",5.0);
    Time::delay(5.0);
    iCub.moveToPosture("morningYoga", 3.0);
    Time::delay(3.0);
    iCub.moveToPosture("home", 5.0);

    //This is how you trigger a choregraphy made of postures
    iCub.playChoregraphy("streching",1.0,true);

    //This is how you speak
    iCub.say("Hello, welcome in this example.",true,false);

    //You get access to the facial emotion client (check facialEmotionsExample for more infos)
    iCub.say("You can control my facial expressions.");
    iCub.say("I look happy");
    iCub.getExpressionClient()->express("joy",1.0);
    Time::delay(3.0);
    iCub.say("I look sad");
    iCub.getExpressionClient()->express("sad",1.0);
    Time::delay(3.0);

    //Note that at the moment this is still independend from the real emotions stored in the OPC
    double joyLevel = iCub.icubAgent->m_emotions_intrinsic["joy"];
    ostringstream oss;
    oss<<"However my real joy level is "<<joyLevel;
    iCub.say(oss.str());

    //Drives can be accessed the same way
    //We force the fun drive to be low
    iCub.icubAgent->m_drives["fun"].value = 0.1;
    iCub.commitAgent(); //Commit our modifications to the OPC

    //And we check them
    iCub.say("Checking my drives...");
    iCub.updateAgent(); //Just to refresh the agent from the OPC
    for(map<string,Drive>::iterator d=iCub.icubAgent->m_drives.begin(); d!=iCub.icubAgent->m_drives.end(); d++)
    {
        if (d->second.value < d->second.homeoStasisMin || d->second.value > d->second.homeoStasisMax)
        {
            oss.clear();
            oss<<"I should do something regarding my "<<d->second.name;
            iCub.say(oss.str());
        }
    }

    //We check the objects that the robot can see and look at them
    iCub.say("I can see ");
    list<Object*> inSight = iCub.getObjectsInSight();
    for(list<Object*>::iterator o=inSight.begin();o!=inSight.end();o++)
    {
        string objectName = (*o)->name();
        iCub.look(objectName);
        iCub.say("a " + objectName);
    }

    if (inSight.size() == 0)
        iCub.say("nothing.");

    //We check the objects that the robot can grasp and pick the first one
    list<Object*> inRange = iCub.getObjectsInRange();
    if (inRange.size() > 0)
    {
        string chosenObject = inRange.front()->name();
        iCub.say("I can grasp the " + chosenObject);
        iCub.grasp(chosenObject);
        iCub.release(chosenObject);
    }
    else
        iCub.say("But all of them are out of my range.");

    iCub.say("And that is about everything I can do.");
    iCub.say("See you next time.");

    // close the client or let the destructor do it for you
    iCub.close();

    return 0;
}



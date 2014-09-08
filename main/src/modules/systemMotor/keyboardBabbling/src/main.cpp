/* 
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Grégoire Pointeau
* email:   greg.pointeau@gmail.com
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


* keyboardBabbling

*/

#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <time.h>
#include <iCub/iKin/iKinFwd.h>
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::iKin;
using namespace iCub::ctrl;

YARP_DECLARE_DEVICES(icubmod)


    int main(int argc, char *argv[])
{
    Network::init();
    srand(time(NULL));

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("keyboardBabbling/conf");
    rf.setDefaultConfigFile("default.ini");

    rf.configure(argc, argv);

    Property options;
    options.put("robot", "icubSim"); // typically from the command line.
    options.put("device", "remote_controlboard");

    Value& robotname = options.find("robot");
    string s("/");
    s += robotname.asString();
    s += "/right_arm/keyBabbling";
    options.put("local", s.c_str());
    s.clear();
    s += "/";
    s += robotname.asString();
    s += "/right_arm";
    options.put("remote", s.c_str());



    Property optionCart("(device cartesiancontrollerclient)");
    optionCart.put("remote","/icubSim/cartesianController/right_arm");
    optionCart.put("local","/cartesian_client/right_arm");
    PolyDriver driverCart;
    if (!driverCart.open(optionCart))
        return false;





    PolyDriver dd(options);
    if (!dd.isValid()) {
        cout << "Device not available.  Here are the known devices:\n"<< endl;
        cout << Drivers::factory().toString().c_str() << endl;;
        Network::fini();
        return 0;
    }

    IPositionControl *pos;
    IVelocityControl *vel;
    IPidControl *pid;
    IAmplifierControl *amp;
    IEncoders *armEncUsed;
    IControlMode *armCtrlModeUsed;
    IImpedanceControl *armImpUsed;
    ICartesianControl *armCartUsed;
    IControlLimits *ilimRight;

    bool ok;
    ok = dd.view(pos);
    ok &= dd.view(vel);
    ok &= dd.view(pid);
    ok &= dd.view(amp);
    ok &= dd.view(armCtrlModeUsed);
    ok &= dd.view(armImpUsed);
    ok &= driverCart.view(armCartUsed);
    ok &= dd.view(ilimRight);


    iCubFinger *fingerUsed;
    iCubFinger* fingerRight = new iCubFinger("right_index");
    deque<IControlLimits*> limRight;
    limRight.push_back(ilimRight);
    fingerRight->alignJointsBounds(limRight);



    if (!ok) {
        cout << "Device not able to acquire views" << endl;
        Network::fini();
        dd.close();
        return 0;
    }


    int jnts = 0;
    pos->getAxes(&jnts);
    printf("Working with %d axes\n", jnts);


    // closing the hand:
    pos->positionMove(8, 10.0);
    pos->positionMove(9, 17.0);
    pos->positionMove(10, 150.0);
    pos->positionMove(11, 0.0);
    pos->positionMove(12, 30.0);
    pos->positionMove(13, 75.0);
    pos->positionMove(13, 50.0);
    pos->positionMove(14, 50.0);
    pos->positionMove(15, 160.0);


    bool motionDone = false;
    while (!motionDone)
    {
        motionDone = true;
        for (int i = 8; i < jnts; i++)
        {
            bool jntMotionDone = false;
            pos->checkMotionDone(i, &jntMotionDone);
            motionDone &= jntMotionDone;
        }
    }


    // wait for the hand to be in initial position

    string key = "";
    cout << "waiting to be in intial position" << endl;
    cin >> key;


    Vector initPos(3);

    if (rf.check("rightHandInitial"))
    {
        Bottle *botPos = rf.find("rightHandInitial").asList();
        initPos[0] = botPos->get(0).asDouble();
        initPos[1] = botPos->get(1).asDouble();
        initPos[2] = botPos->get(2).asDouble();
        cout<<"Reaching initial position with right hand"<<initPos.toString(3,3)<<endl;
        armCartUsed->goToPositionSync(initPos);
    }
    else
    {
        cout << "Cannot find the inital position" << endl << "closing module" << endl;
        return 0;
    }

    // get the x, y, z of the init point:



    double minY = -.10;
    double maxY = 0.10;
    double gap = 0.05;

    Vector tempPos(3);

    while (true)
    {
        tempPos[0] = initPos[0];
        tempPos[1] = initPos[1];
        tempPos[2] = initPos[2] + gap; //raise the arm

        armCartUsed->goToPositionSync(tempPos);
        bool raiseHand = false;
        while (!raiseHand)
        {
            armCartUsed->checkMotionDone(&raiseHand);        		
        }

        //hand is up!!
        double tempV = yarp::os::Random::uniform(0,100);
        tempPos[1] = (tempV*0.01* (maxY-minY) + minY);

        armCartUsed->goToPositionSync(tempPos);
        raiseHand = false;
        while (!raiseHand)
        {
            armCartUsed->checkMotionDone(&raiseHand);        		
        }

        //hand is ready
        tempPos[2] = initPos[2]; //raise the arm
        armCartUsed->goToPositionSync(tempPos);
        bool motionFinished = false;
        while (!motionFinished)
        {
            armCartUsed->checkMotionDone(&motionFinished);        		
        }

        // hand is lowered

        Time::delay(5.0);
        cout << "ok" << endl;
    }

    return 0;
}


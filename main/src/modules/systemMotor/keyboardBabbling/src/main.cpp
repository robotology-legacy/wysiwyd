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
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

YARP_DECLARE_DEVICES(icubmod);



class keyboardBabbling: public yarp::os::RFModule {
private:
    IPositionControl *pos;
    IVelocityControl *vel;
    IPidControl *pid;
    IAmplifierControl *amp;
    IEncoders *armEncUsed;
    IControlMode *armCtrlModeUsed;
    IImpedanceControl *armImpUsed;
    ICartesianControl *armCart;
    IControlLimits *ilimRight;

    yarp::os::Port portStreamer;
    PolyDriver driverCart, dd;
    iCubFinger *fingerUsed;

    double gap;
    double period;
    double minY;
    double maxY;
    double delay;
    double thrMove;

    Vector initPos;
    Vector tempPos;
    Vector orientation;

    bool isGoingUp;
    bool isGoingSide;
    bool isGoingDown;
    bool isWaiting;
    bool isMoving;
    double timeSinceLastAction;


public:

    bool interruptModule()
    {return true;}

    bool close(){return true;}

    double getPeriod()
    {
        return period;
    }


    bool configure(yarp::os::ResourceFinder &rf)
    {
        isGoingUp = false;
        isGoingDown = false;
        isGoingSide = true;
        isMoving = false;
        isWaiting = false;
        period = 0.01;

        yarp::os::Property options;
        options.put("robot", "icubSim"); // typically from the command line.
        options.put("device", "remote_controlboard");

        yarp::os::Value& robotname = options.find("robot");
        string s("/");
        s += robotname.asString();
        s += "/right_arm/keyBabbling";
        options.put("local", s.c_str());
        s.clear();
        s += "/";
        s += robotname.asString();
        s += "/right_arm";
        options.put("remote", s.c_str());

        if (!dd.open(options)) {
            cout << "Device not available.  Here are the known devices:\n"<< endl;
            cout << Drivers::factory().toString().c_str() << endl;;
            yarp::os::Network::fini();
            return false;
        }


        portStreamer.open("/keyboardBabbling:stream");

        yarp::os::Property optionCart("(device cartesiancontrollerclient)");
        optionCart.put("remote","/icubSim/cartesianController/right_arm");
        optionCart.put("local","/cartesian_client/right_arm");
        if (!driverCart.open(optionCart))
            return false;

        bool ok;
        ok = dd.view(pos);
        ok &= dd.view(vel);
        ok &= dd.view(pid);
        ok &= dd.view(amp);
        ok &= dd.view(armCtrlModeUsed);
        ok &= dd.view(armImpUsed);
        ok &= driverCart.view(armCart);
        ok &= dd.view(ilimRight);


        iCubFinger* fingerRight = new iCubFinger("right_index");
        deque<IControlLimits*> limRight;
        limRight.push_back(ilimRight);
        fingerRight->alignJointsBounds(limRight);

        if (!ok) {
            cout << "Device not able to acquire views" << endl;
            yarp::os::Network::fini();
            dd.close();
            return false;
        }


        int jnts = 0;
        pos->getAxes(&jnts);
        printf("Working with %d axes\n", jnts);

        vector<bool> mask;
        mask.resize(jnts);
        mask[0]	= false;
        mask[1]	= false;
        mask[2]	= false;
        mask[3]	= false;
        mask[4] = true;
        mask[5] = false;
        mask[6] = true;
        mask[7] = false;
        mask[8] = true;
        mask[9] = true;
        mask[10] = true;
        mask[11] = true;
        mask[12] = true;
        mask[13] = true;
        mask[14] = true;
        mask[15] = true;


        // closing the hand:
        pos->positionMove(4, 30.0);
        pos->positionMove(6, 10.0);
        pos->positionMove(8, 10.0);
        pos->positionMove(9, 17.0);
        pos->positionMove(10, 170.0);
        pos->positionMove(11, 55.0);
        pos->positionMove(12, 10.0);
        pos->positionMove(13, 85.0);
        pos->positionMove(14, 170.0);
        pos->positionMove(15, 170.0);


        bool motionDone = false;
        while (!motionDone)
        {
            motionDone = true;
            for (int i = 0; i < jnts; i++)
            {
                if (mask[i])
                {
                    bool jntMotionDone = false;
                    pos->checkMotionDone(i, &jntMotionDone);
                    motionDone &= jntMotionDone;
                }
            }
        }


        // wait for the hand to be in initial position

        cout << "waiting to be in intial position" << endl;

        initPos = Vector(3);
        tempPos = Vector(3);
        orientation = Vector(3);

        double arm_roll=rf.check("arm_roll",Value(0.0)).asDouble();
        double arm_pitch=rf.check("arm_yaw",Value(0.0)).asDouble();        
        double arm_yaw=rf.check("arm_pitch",Value(0.0)).asDouble();

        Matrix R=zeros(4,4);
        R(0,0)=-1.0; R(2,1)=-1.0; R(1,2)=1.0; R(3,3)=1.0;
        //R(0,0)=-1.0; R(1,1)=1.0; R(2,2)=-1.0; R(3,3)=1.0;

        Vector pitch(4,0.0);
        pitch[2]=1.0;
        pitch[3]=arm_pitch*CTRL_DEG2RAD;

        Vector roll(4,0.0);
        roll[0]=1.0;
        roll[3]=arm_roll*CTRL_DEG2RAD;

        Vector yaw(4,0.0);
        yaw[1]=1.0;
        yaw[3]=arm_yaw*CTRL_DEG2RAD;

        orientation=dcm2axis(axis2dcm(pitch)*axis2dcm(roll)*axis2dcm(yaw)*R);


        //        armCart->getPose(initPos,orientation);

        if (rf.check("rightHandInitial"))
        {
            yarp::os::Bottle *botPos = rf.find("rightHandInitial").asList();
            initPos[0] = botPos->get(0).asDouble();
            initPos[1] = botPos->get(1).asDouble();
            initPos[2] = botPos->get(2).asDouble();
            cout<<"Reaching initial position with right hand"<<initPos.toString(3,3)<<endl;
            armCart->setTrajTime(1.5);
            armCart->goToPose(initPos,orientation);
        }
        else
        {
            cout << "Cannot find the inital position" << endl << "closing module" << endl;
            return 0;
        }

        armCart->checkMotionDone(&isMoving);
        while (!isMoving)
        {
            armCart->checkMotionDone(&isMoving);
        }

        //armCart->getPose(Vector(3),orientation);

        // get the x, y, z of the init point:



        minY = rf.find("min").asDouble();
        maxY = rf.find("max").asDouble();
        gap    = rf.find("gap").asDouble();
        delay  = rf.find("delay").asDouble();
        thrMove  = rf.find("threshold_move").asDouble();

        tempPos[0] = initPos[0];
        tempPos[1] = initPos[1];
        tempPos[2] = initPos[2];

        return true;
    }


    bool updateModule()
    {
        armCart->setTrajTime(1.5);
        if (isWaiting)
        {
            //if I've wait long enough
            if (Time::now() - timeSinceLastAction > delay)
            {
                tempPos[2] += gap; //raise the arm
                armCart->goToPose(tempPos,orientation);
                //                armCart->goToPositionSync(tempPos);
                isWaiting = false;
                isGoingUp = true;
                timeSinceLastAction = Time::now();
                cout << "isGoingUp" << endl;
            }
        }
        else if (isGoingUp)
        {

            armCart->checkMotionDone(&isMoving);

            if (isMoving || (Time::now()-timeSinceLastAction>thrMove))  // is goingup is finished
            {
                //hand is up!!
                double tempV = yarp::os::Random::uniform(0,100);
                tempPos[1] = (tempV*0.01* (maxY-minY) + minY);
                //                armCart->goToPositionSync(tempPos);
                armCart->goToPose(tempPos,orientation);
                // now, robot is going side:
                isGoingUp = false;
                isGoingSide = true;
                cout << "isGoingSide" << endl;
                timeSinceLastAction = Time::now();
            }
        }
        else if (isGoingSide)
        {

            armCart->checkMotionDone(&isMoving);
            // if goingside is finished, goind down
            if (isMoving || (Time::now()-timeSinceLastAction>thrMove))  // is goingup is finished
            {
                tempPos[2] -= gap; //lower the arm
                //                armCart->goToPositionSync(tempPos);
                armCart->goToPose(tempPos,orientation);
                isGoingSide = false;
                isGoingDown = true;
                cout << "isGoingDown" << endl;

                timeSinceLastAction = Time::now();
            }
        }
        else if (isGoingDown)
        {

            armCart->checkMotionDone(&isMoving);        		
            // if the going down if finished, we wait !
            if (isMoving || (Time::now()-timeSinceLastAction>thrMove))  // is goingup is finished
            {
                //hand is ready
                timeSinceLastAction  =   Time::now();
                isGoingDown = false;
                isWaiting = true;
                cout << "isWaiting" << endl;
            }

        }
        isMoving = true;
        yarp::os::Bottle bStream;

        bStream.addDouble(tempPos[0]);
        bStream.addDouble(tempPos[1]);
        bStream.addDouble(tempPos[2]);

        portStreamer.write(bStream);

        return true;
    }

};


int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("yarp network is not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    srand(time(NULL));

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("keyboardBabbling");
    rf.setDefaultConfigFile("default.ini");
    rf.configure(argc, argv);

    keyboardBabbling mod;
    return mod.runModule(rf);
}


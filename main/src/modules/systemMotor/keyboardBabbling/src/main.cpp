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
#include <yarp/math/Rand.h>
#include <iCub/iKin/iKinFwd.h>
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl;

class keyboardBabbling: public RFModule
{
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
    
    Port portStreamer;
    Port portFromCart;
    Port portRpc;

    PolyDriver driverCart, dd;
    iCubFinger *fingerUsed;

    double gap;
    double minY;
    double maxY;
    double delay;
    double thrMove;
    double timeBeginIdle;
    int    yDivisions;
    vector<int> sequenceToPlay;
    int         indexInSequence;
    bool   forward;

    Vector tempPos;
    Vector orientation;

    enum { idle, up, side, down } state;

public:
    bool close()
    {
        return true;
    }

    double getPeriod()
    {
        return 0.1;
    }

    bool configure(ResourceFinder &rf)
    {
        std::string robotname = rf.check("robot",yarp::os::Value("icubSim")).asString();
        Property options;
        options.put("robot", robotname); // typically from the command line.
        options.put("device", "remote_controlboard");

        string s("/");
        s += robotname;
        s += "/right_arm/keyBabbling";
        options.put("local", s.c_str());
        s.clear();
        s += "/";
        s += robotname;
        s += "/right_arm";
        options.put("remote", s.c_str());

        if (!dd.open(options)) {
            cout << "Device not available.  Here are the known devices:\n"<< endl;
            cout << Drivers::factory().toString().c_str() << endl;;
            return false;
        }
        
        portStreamer.open("/keyboardBabbling/stream:o");
        portFromCart.open("/keyboardBabbling/stream:i");
        portRpc.open("/keyboarBabbling/rpc");
        attach(portRpc);

        Property optionCart("(device cartesiancontrollerclient)");
        optionCart.put("remote", "/" + robotname +"/cartesianController/right_arm");
        optionCart.put("local","/keyboardBabbling/cartesian_client/right_arm");
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
            dd.close();
            return false;
        }

        int jnts = 0;
        pos->getAxes(&jnts);
        printf("Working with %d axes\n", jnts);

        vector<bool> mask;
        mask.resize(jnts);
        mask[0] = false;
        mask[1] = false;
        mask[2] = false;
        mask[3] = false;
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

        cout << "Closing the hand" << endl;
        // closing the hand:
        pos->positionMove(4, 30.0);
        pos->positionMove(6, 10.0);
        pos->positionMove(8, 10.0);
        pos->positionMove(9, 17.0);
        pos->positionMove(10, 170.0);
        pos->positionMove(11, 0.0);
        pos->positionMove(12, 0.0);
        pos->positionMove(13, 90.0);
        pos->positionMove(14, 180.0);
        pos->positionMove(15, 180.0);

        cout << "Waiting to be in initial position...";
        bool motionDone=false;
        while (!motionDone)
        {
            motionDone=true;
            for (int i=0; i<jnts; i++)
            {
                if (mask[i])
                {
                    bool jntMotionDone = false;
                    pos->checkMotionDone(i, &jntMotionDone);
                    motionDone &= jntMotionDone;
                }
            }

            Time::yield();  // to avoid killing cpu
        }
        cout << "ok" << endl;
        // wait for the hand to be in initial position

        Matrix R=zeros(3,3);
        R(0,0)=-1.0; R(1,1)=1; R(2,2)=-1.0;
        orientation=dcm2axis(R);
        
        // enable torso movements as well
        // in order to enlarge the workspace
        Vector dof;
        armCart->getDOF(dof);
        dof=1.0; dof[1]=0.0;    // every dof but the torso roll
        armCart->setDOF(dof,dof);
        armCart->setTrajTime(1.0);

        Vector initPos(3,0.0);
        if (rf.check("rightHandInitial"))
        {
            Bottle *botPos = rf.find("rightHandInitial").asList();
            initPos[0] = botPos->get(0).asDouble();
            initPos[1] = botPos->get(1).asDouble();
            initPos[2] = botPos->get(2).asDouble();
            cout<<"Reaching initial position with right hand"<<initPos.toString(3,3).c_str()<<endl;            
            armCart->goToPoseSync(initPos,orientation);
            armCart->waitMotionDone(0.1,3.0);
        }
        else
        {
            cout << "Cannot find the inital position" << endl << "closing module" << endl;
            return false;
        }        

        minY    = rf.find("min").asDouble();
        maxY    = rf.find("max").asDouble();
        gap     = rf.find("gap").asDouble();
        delay   = rf.find("delay").asDouble();
        thrMove = rf.find("threshold_move").asDouble();
        yDivisions = rf.check("yDivisions",Value(10)).asInt();
        Bottle* bsequenceToPlay = rf.find("sequence").asList();
        if (bsequenceToPlay)
        {
            cout << "Using sequence, not random." << endl;
            for (int i = 0; i < bsequenceToPlay->size(); i++)
                sequenceToPlay.push_back(bsequenceToPlay->get(i).asInt());
            indexInSequence = 0;
        }

        tempPos=initPos;
        state=up;
        forward = false;

        timeBeginIdle = Time::now();

        Rand::init();
        return true;
    }

    bool updateModule()
    {
        //Always send the current position of the end effector
        Vector toSend, toSend2;
        armCart->getPose(toSend, toSend2);
        portStreamer.write(toSend);

        if (!forward)
        {
        bool done;
        armCart->checkMotionDone(&done);
/*
        if (!done)
            return true;*/

        if (state==idle)
        {
            if (Time::now() - timeBeginIdle > delay)
            {
                tempPos[2] += gap;
                state = up;
                timeBeginIdle = Time::now();
            }
            else
                return true;
        }
        else if (state == up)
        {

            if (done || Time::now() - timeBeginIdle > thrMove)
            {
                if (sequenceToPlay.size() > 0)
                {
                    int nextIndex = sequenceToPlay[indexInSequence];
                    if (nextIndex >= yDivisions)
                    {
                        cout << "The index in the sequence is larger than the number of division. Clamping." << endl;
                        nextIndex = yDivisions - 1;
                    }

                    tempPos[1] = minY + ((maxY - minY) / (double)yDivisions)*sequenceToPlay[indexInSequence];
                    indexInSequence++;
                    if (indexInSequence >= (int)sequenceToPlay.size())
                        indexInSequence = 0;
                }
                else
                {
                    tempPos[1] = minY + ((maxY - minY) / (double)yDivisions)*Random::uniform(0, yDivisions);
                }
                state = side;
                timeBeginIdle = Time::now();
            }
        }
        else if (state == side)
        {
            if (done || Time::now() - timeBeginIdle > thrMove)
            {

                tempPos[2] -= gap;
                state = down;
                timeBeginIdle = Time::now();
            }
        }
        else if (state == down)
        {
            if (done || Time::now() - timeBeginIdle > thrMove)
            {

                state = idle;
                timeBeginIdle = Time::now();
            }
        }

        armCart->goToPoseSync(tempPos,orientation);
        printf("Going to (%s)\n",tempPos.toString(3,3).c_str());

        }
        else
        {
            Vector cartPos;
            portFromCart.read(cartPos);
            armCart->goToPoseSync(cartPos, orientation);
        }

        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) {

        reply.clear(); 

        if (command.get(0).asString()=="quit") {
            reply.addString("quitting");
            return false;     
        }
        else if (command.get(0).asString()=="forward") {
            cout << "forward on" << endl;
            reply.addString("forward on");
            forward = true;
        }
        else if (command.get(0).asString()=="stop")
        {
            cout << "forward off" << endl;
            reply.addString("forward off");
            forward = false;
        }


        return true;
    }

};


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("yarp network is not available!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("keyboardBabbling");
    rf.setDefaultConfigFile("default.ini");
    rf.configure(argc,argv);

    keyboardBabbling mod;
    return mod.runModule(rf);
}


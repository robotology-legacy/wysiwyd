/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Ugo Pattacini
 * email:   ugo.pattacini@iit.it
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
*/

#include <stdio.h>
#include <string>
#include <fstream>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/perception/models.h>
#include <iCub/action/actionPrimitives.h>

#include "slidingController_IDLServer.h"

#define EXPLORATION_TOL     5e-3

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::perception;
using namespace iCub::action;


/***************************************************************/
class ControlModule: public RFModule, public slidingController_IDLServer
{
protected:
    ActionPrimitivesLayer1  action;
    PolyDriver              driverCart, driverJoint;
    ICartesianControl      *iarm;

    RpcServer portRpc;
    BufferedPort<Bottle> portIn;
    string graspModelFileToWrite;
    deque<string> handKeys;

    Mutex mainMutex;
    bool impedance,oldImpedance;
    double max_dist;
    int context_startup;
    int context;
    string arm;
    Vector xd,od,oldOd,odAdjusted,x0;
    double t0,vel,oldVel,T;
    double exploration_height;
    double exploration_max_force;
    double elbow_height,elbow_weight;

    double expT0;
    deque<Vector> expPos;
    Mutex expMutex;
    Event expEndEvent;
    
    typedef enum { idle, clipped, start, approach, run } States;
    States state, expState;

    /***************************************************************/
    bool calibrateGraspModel(const bool forceCalibration)
    {
        Model *model; action.getGraspModel(model);
        if (model!=NULL)
        {
            if (forceCalibration || !model->isCalibrated())
            {
                Property prop("(finger all_parallel)");
                model->calibrate(prop);

                ofstream fout;
                fout.open(graspModelFileToWrite.c_str());
                model->toStream(fout);
                fout.close();
            }

            return true;
        }

        return false;
    }

    /***************************************************************/
    Bottle changeElbowHeight(const double height, const double weight)
    {
        Bottle tweakOptions;
        Bottle &optTask2=tweakOptions.addList();
        optTask2.addString("task_2");
        Bottle &plTask2=optTask2.addList();
        plTask2.addInt(6);
        Bottle &posPart=plTask2.addList();
        posPart.addDouble(0.0);
        posPart.addDouble(0.0);
        posPart.addDouble(height);
        Bottle &weightsPart=plTask2.addList();
        weightsPart.addDouble(0.0);
        weightsPart.addDouble(0.0);
        weightsPart.addDouble(weight);
        return tweakOptions;
    }

    /***************************************************************/
    void setImpedance(const bool sw, const bool forceSet=false)
    {
        if (!forceSet && (sw==impedance))
            return;

        IControlMode *imode;
        driverJoint.view(imode);

        if (sw)
        {
            IImpedanceControl *iimp;
            driverJoint.view(iimp);

            imode->setImpedanceVelocityMode(0); iimp->setImpedance(0,0.4,0.03);
            imode->setImpedanceVelocityMode(1); iimp->setImpedance(1,0.4,0.03);
            imode->setImpedanceVelocityMode(2); iimp->setImpedance(2,0.4,0.03);
            imode->setImpedanceVelocityMode(3); iimp->setImpedance(3,0.2,0.01);
            imode->setImpedanceVelocityMode(4); iimp->setImpedance(4,0.2,0.0);
        }
        else
        {
            for (int j=0; j<5; j++)
                imode->setVelocityMode(j);
        }

        impedance=sw;
    }

public:
    /***************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("slidingController")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        arm=rf.check("arm",Value("right")).asString().c_str();
        vel=rf.check("vel",Value(0.2)).asDouble();
        elbow_height=rf.check("elbow_height",Value(0.4)).asDouble();
        elbow_weight=rf.check("elbow_weight",Value(30.0)).asDouble();
        double arm_roll=rf.check("arm_roll",Value(5.0)).asDouble();
        double arm_pitch=rf.check("arm_yaw",Value(10.0)).asDouble();        
        double arm_yaw=rf.check("arm_pitch",Value(0.0)).asDouble();
        max_dist=rf.check("max_dist",Value(0.02)).asDouble();
        impedance=rf.check("impedance",Value("off")).asString()=="on";
        exploration_height=rf.check("exploration_height",Value(0.0)).asDouble();
        exploration_max_force=rf.check("exploration_max_force",Value(1000.0)).asDouble();

        Property optionCart("(device cartesiancontrollerclient)");
        optionCart.put("remote",("/"+robot+"/cartesianController/"+arm+"_arm").c_str());
        optionCart.put("local",("/"+name+"/"+arm+"_arm/cartesian").c_str());
        if (!driverCart.open(optionCart))
            return false;

        Property optionJoint("(device remote_controlboard)");
        optionJoint.put("remote",("/"+robot+"/"+arm+"_arm").c_str());
        optionJoint.put("local",("/"+name+"/"+arm+"_arm/joint").c_str());
        if (!driverJoint.open(optionJoint))
        {
            driverCart.close();
            return false;
        }

        driverCart.view(iarm);
        iarm->storeContext(&context_startup);

        Property optionAction;
        string grasp_model_file=(arm=="left"?"grasp_model_file_left":"grasp_model_file_right");
        optionAction.put("robot",robot.c_str());
        optionAction.put("local",(name+"/action").c_str());
        optionAction.put("part",(arm+"_arm").c_str());
        optionAction.put("torso_pitch","on");
        optionAction.put("torso_roll","off");
        optionAction.put("torso_yaw","on");
        optionAction.put("grasp_model_type","springy");
        optionAction.put("grasp_model_file",rf.findFile(grasp_model_file.c_str()).c_str());
        optionAction.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());
        graspModelFileToWrite=rf.getHomeContextPath();
        graspModelFileToWrite+="/";
        graspModelFileToWrite+=rf.find(grasp_model_file.c_str()).asString().c_str();

        if (!action.open(optionAction))
        {
            driverCart.close();
            driverJoint.close();
            return false;
        }

        handKeys=action.getHandSeqList();
        printf("***** List of available hand sequence keys:\n");
        for (size_t i=0; i<handKeys.size(); i++)
            printf("%s\n",handKeys[i].c_str());
        calibrateGraspModel(false);

        iarm->restoreContext(context_startup);

        Vector dof;
        iarm->getDOF(dof);
        dof=1.0; dof[1]=0.0;
        iarm->setDOF(dof,dof);
        iarm->setLimits(0,0.0,10.0);
        iarm->setTrajTime(0.65);
        iarm->setInTargetTol(0.001);

        iarm->tweakSet(changeElbowHeight(elbow_height,elbow_weight));
        iarm->storeContext(&context);

        setImpedance(impedance,true);

        Matrix R=zeros(4,4);
        R(0,0)=-1.0; R(2,1)=-1.0; R(1,2)=-1.0; R(3,3)=1.0;

        Vector roll(4,0.0);
        roll[0]=1.0;
        roll[3]=(arm=="right"?1.0:-1.0)*fabs(arm_roll)*CTRL_DEG2RAD;

        Vector pitch(4,0.0);
        pitch[1]=1.0;
        pitch[3]=arm_pitch*CTRL_DEG2RAD;

        Vector yaw(4,0.0);
        yaw[2]=1.0;
        yaw[3]=arm_yaw*CTRL_DEG2RAD;

        od=dcm2axis(axis2dcm(yaw)*axis2dcm(roll)*axis2dcm(pitch)*R);
        xd.resize(3);

        portIn.open(("/"+name+":i").c_str());
        portRpc.open(("/"+name+":rpc").c_str());
        attach(portRpc);

        state=idle;
        expState=idle;
        return true;
    }

    /***************************************************************/
    void exploreNextPoint(const States nextState)
    {
        xd=expPos.front();
        expPos.pop_front();
        expT0=Time::now();
        expState=nextState;
        printf("explore point: (%s)\n",xd.toString(3,3).c_str());
    }

    /***************************************************************/
    bool handleExploration()
    {
        expMutex.lock();
        if (expState==start)
        {
            oldImpedance=impedance;
            setImpedance(false);
            action.pushAction("point");

            // set up exploring orientation
            Matrix R(4,4);
            R(0,0)=-1.0;
            R(1,1)=(arm=="right"?1.0:-1.0);
            R(2,2)=(arm=="right"?-1.0:1.0);

            oldOd=od;
            od=dcm2axis(R);

            iarm->setLimits(0,0.0,30.0);
            iarm->setTrajTime(1.0);
            iarm->tweakSet(changeElbowHeight(0.1,elbow_weight));

            Vector startPoint(3,0.0);
            startPoint[0]=-0.35; startPoint[2]=0.15;
            iarm->goToPoseSync(startPoint,od);

            // set up exploration points
            Vector expPoint(3); expPoint[2]=exploration_height;

            // #1
            expPoint[0]=-0.45; expPoint[1]=0.0*(arm=="right"?1.0:-1.0);
            expPos.push_back(expPoint);

            // #2
            expPoint[0]=-0.4; expPoint[1]=0.13*(arm=="right"?1.0:-1.0);
            expPos.push_back(expPoint);

            // #3
            expPoint[0]=-0.4; expPoint[1]=0.0*(arm=="right"?1.0:-1.0);
            expPos.push_back(expPoint);

            // #4
            expPoint[0]=-0.45; expPoint[1]=0.0*(arm=="right"?1.0:-1.0);
            expPos.push_back(expPoint);

            // set up cartesian controller
            iarm->waitMotionDone();
            bool done; action.checkActionsDone(done,true);

            IEncoders *iencs;
            driverJoint.view(iencs);
            int nEncs; iencs->getAxes(&nEncs);
            Vector encs(nEncs); iencs->getEncoders(encs.data());
            Vector poss=encs.subVector(7,nEncs-1);

            Vector joints;
            iCubFinger finger(arm+"_index");
            finger.getChainJoints(poss,joints);
            Vector xf=finger.getH(CTRL_DEG2RAD*joints).getCol(3);

            iarm->setTrajTime(1.2);
            iarm->attachTipFrame(xf,Vector(4,0.0));

            oldVel=vel;
            vel=0.04;

            exploreNextPoint(approach);
            expMutex.unlock();
            return true;
        }
        else if (expState==approach)
        {
            Model *model; action.getGraspModel(model);
            if (model!=NULL)
            {
                Value out;
                model->getOutput(out);
                double contact_force=out.asList()->get(1).asDouble();
                if (contact_force>exploration_max_force)
                {
                    printf("contact detected: (%g>%g)\n",contact_force,exploration_max_force);
                    
                    // re-tune the z-coordinates                    
                    iarm->stopControl();                    
                    Vector x,o; iarm->getPose(x,o);
                    for (size_t i=0; i<expPos.size(); i++)
                        expPos[i][2]=x[2];

                    exploreNextPoint(run);
                    expMutex.unlock();
                    return true;
                }
            }

            Vector x,o;
            iarm->getPose(x,o);
            if (norm(xd-x)>EXPLORATION_TOL)
            {
                exploreNextPoint(run);
                expMutex.unlock();
                return true;
            }
        }
        else if (expState==run)
        {
            Vector x,o;
            iarm->getPose(x,o);
            if (norm(xd-x)>EXPLORATION_TOL)
            {
                if (expPos.empty())
                {
                    iarm->stopControl();
                    expEndEvent.signal();
                    printf("...exploration accomplished\n");

                    iarm->setTrajTime(0.8);
                    iarm->setInTargetTol(0.01);

                    // lift up
                    Vector x(3,0.0);
                    x[0]=-0.4; x[2]=0.1;
                    iarm->goToPoseSync(x,od);
                    iarm->waitMotionDone();
                    iarm->restoreContext(context);

                    if (oldImpedance)
                        setImpedance(true);

                    action.pushAction("open");

                    vel=oldVel;
                    od=oldOd;
                    expState=idle;
                }
                else
                {
                    exploreNextPoint(run);
                    expMutex.unlock();
                    return true;
                }
            }
        }

        expMutex.unlock();
        return false;
    }

    /***************************************************************/
    bool computeWayPoint(Vector &wp)
    {
        double t=(Time::now()-t0)/T;
        wp=x0+(xd-x0)*t;
        double d=norm(xd-wp);
        return ((d<max_dist) || (t>=1.0));
    }

    /***************************************************************/
    bool updateModule()
    {
        mainMutex.lock();

        bool netRequest=false;
        bool noInterp=false;
        if (Bottle *b=portIn.read(false))
        {
            if (b->size()>=3)
            {
                xd[0]=b->get(0).asDouble();
                xd[1]=b->get(1).asDouble();
                xd[2]=b->get(2).asDouble();

                if (b->size()>=4)
                    noInterp=(b->get(3).asInt()!=0);

                netRequest=true;
                printf("target received = (%s) (%s)\n",xd.toString(3,3).c_str(),
                       (noInterp?"straight":"interpolating"));
            }
            else
                printf("wrong command received: %s\n",b->toString().c_str());
        }

        bool expRequest=handleExploration();
        if (netRequest || expRequest)
        {
            Vector x,o;
            iarm->getPose(x,o);

            odAdjusted=od;
            if (netRequest)
            {
                double ang=CTRL_DEG2RAD*15.0;
                Vector roll(4,0.0);
                roll[0]=1.0;

                if ((arm=="right") && (x[1]<xd[1]))
                    roll[3]=ang;
                else if ((arm=="left") && (xd[1]<x[1]))
                    roll[3]=-ang;

                Matrix R=axis2dcm(od);
                odAdjusted=dcm2axis(axis2dcm(roll)*R);
            }

            double d=norm(xd-x);
            if ((d<max_dist) || noInterp)
            {
                iarm->goToPose(xd,odAdjusted);
                printf("affordable target = (%s)\n",xd.toString(3,3).c_str());
                state=idle;
            }
            else
            {
                t0=Time::now(); x0=x; T=d/vel;
                Vector wp; computeWayPoint(wp);
                iarm->goToPose(wp,odAdjusted);
                printf("waypoint = (%s)\n",wp.toString(3,3).c_str());
                state=clipped;
            }
        }
        else if (state==clipped)
        {
            Vector wp; 
            if (computeWayPoint(wp))
            {
                iarm->goToPose(xd,odAdjusted);
                printf("final target = (%s)\n",xd.toString(3,3).c_str());
                state=idle;
            }
            else
            {
                iarm->goToPose(wp,odAdjusted);
                printf("waypoint = (%s)\n",wp.toString(3,3).c_str());
            }
        }

        mainMutex.unlock();
        return true;
    }

    /***************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /***************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /***************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString().c_str();
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        // asynchronous stop
        if (cmd=="stop")
        {
            mainMutex.lock();
            iarm->stopControl();
            state=idle;
            printf("asynchronous stop received\n");
            mainMutex.unlock();
            reply.addVocab(ack);
            return true;
        }

        // fingers calibration
        if (cmd=="calibrate")
        {
            calibrateGraspModel(true);
            reply.addVocab(ack);
            return true;
        }

        // impedance setting
        if (cmd=="impedance")
        {
            if (command.size()>1)
            {
                bool sw=command.get(1).asString()=="on";
                setImpedance(sw);
                reply.addVocab(ack);
            }
            else
                reply.addVocab(nack);

            return true;
        }

        // exploration
        if (cmd=="explore")
        {
            expMutex.lock();
            expState=start;
            expEndEvent.reset();
            expMutex.unlock();

            printf("starting exploration...\n");
            expEndEvent.wait();

            reply.addVocab(ack);
            return true;
        }

        // fingers closure
        for (size_t i=0; i<handKeys.size(); i++)
        {
            if (cmd==handKeys[i])
            {
                action.pushAction(cmd);

                bool done;
                if (command.size()>1)
                    if (command.get(1).asString()=="wait")
                        action.checkActionsDone(done,true);

                reply.addVocab(ack);
                return true;
            }
        }

        // default behavior
        return RFModule::respond(command,reply); 
    }

    /***************************************************************/
    bool close()
    {
        action.close();

        iarm->restoreContext(context_startup);
        setImpedance(false,true);

        driverCart.close();
        driverJoint.close();
        portRpc.close();
        portIn.close();

        return true;
    }
};


/***************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("yarp network is not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setDefaultContext("slidingController/conf");
    rf.setDefault("grasp_model_file_left","grasp_model_left.ini");
    rf.setDefault("grasp_model_file_right","grasp_model_right.ini");
    rf.setDefault("hand_sequences_file","hand_sequences.ini");
    rf.configure(argc,argv);

    ControlModule mod;
    return mod.runModule(rf);
}



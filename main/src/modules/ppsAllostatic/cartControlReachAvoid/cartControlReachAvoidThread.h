/*
* Copyright (C) 2015 WYSIWYD
* Authors: Matej Hoffmann  and Ugo Pattacini
* email:   matej.hoffmann@iit.it, ugo.pattacini@itt.it
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/
#ifndef __CARTCONTROLREACHAVOIDTHREAD_H__
#define __CARTCONTROLREACHAVOIDTHREAD_H__

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <stdarg.h>
#include <string> 
#include <iostream>
#include <fstream>
#include <sstream>

#include <gsl/gsl_math.h>

#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <iCub/ctrl/neuralNetworks.h>
#include <iCub/ctrl/minJerkCtrl.h>

#define DEFAULT_THR_PER     10

#define NOARM               0
#define LEFTARM             1
#define RIGHTARM            2
#define USEDARM             3

#define STATE_IDLE              0
#define STATE_REACH             1
#define STATE_CHECKMOTIONDONE   2
#define STATE_RELEASE           3
#define STATE_WAIT              4
#define STATE_WAIT_FOR_GRASP    5
#define STATE_GO_HOME           6

#define REF_ACC 1000000

using namespace std;

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;

class cartControlReachAvoidThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string name;
    // Name of the robot (to address the module toward icub or icubSim):
    string robot;
    // Resource finder used to find for files and configurations:
    ResourceFinder rf;
    //the period used by the thread. 
    int threadPeriod; 
        
         
    /***************************************************************************/
    // INTERNAL VARIABLES
    
    // Stamp for the setEnvelope for the ports
    yarp::os::Stamp ts;
       
    bool useLeftArm;
    bool useRightArm;
    int  armSel;
       
    PolyDriver *drvTorso, *drvLeftArm, *drvRightArm;
    PolyDriver *drvCartLeftArm, *drvCartRightArm;
    
    minJerkVelCtrlForIdealPlant *minJerkVelCtrl;
    
    IEncoders         *encTorso;
    IPositionControl  *posTorso;
    IVelocityControl  *velTorso;
    IEncoders         *encArm;
    IPositionControl  *posArm;
    IVelocityControl  *velArm;
    ICartesianControl *cartArm;
    
    BufferedPort<Bottle> inportTargetCoordinates;
    Bottle                 *targetCoordinates;
    BufferedPort<Bottle> inportAvoidanceVectors;
    BufferedPort<Bottle> inportReachingGain;
    BufferedPort<Bottle> inportAvoidanceGain;
    
    Vector leftArmReachOffs;
    Vector leftArmHandOrien;
    Vector leftArmJointsStiffness;
    Vector leftArmJointsDamping;

    Vector rightArmReachOffs;
    Vector rightArmHandOrien;
    Vector rightArmJointsStiffness;
    Vector rightArmJointsDamping;
    
    Vector *armReachOffs;
    Vector *armHandOrien;
    
    Vector homePoss, homeVels;
    
    bool wentHome;
    bool leftArmImpVelMode;
    bool rightArmImpVelMode;
    
    bool newTargetFromRPC;
    bool newTargetFromPort;

    double trajTime;
    double reachTol;
    double reachTimer, reachTmo;
       
    struct {
        double minX, maxX;
        double minY, maxY;
        double minZ, maxZ;
    } reachableSpace;
  
    Vector openHandPoss;
    Vector handVels;
    
    int armAxes;    //16 joints of the arm+hand (no torso)
    Vector arm; //encoder values
    int torsoAxes;
    Vector torso; //encoder values
    
    Vector targetPosFromPort;
    Vector targetPosFromRPC;
    Vector targetPos;
    
    Matrix R,Rx,Ry,Rz;
    
    int state;
    int startup_context_id_left;
    int startup_context_id_right;
    
    void getTorsoOptions(Bottle &b, const char *type, const int i, Vector &sw, Matrix &lim);
    void getArmOptions(Bottle &b, Vector &reachOffs, Vector &orien, bool &impVelMode,
                       Vector &impStiff, Vector &impDamp);
    void getHomeOptions(Bottle &b, Vector &poss, Vector &vels);
    void initCartesianCtrl(Vector &sw, Matrix &lim, const int sel=USEDARM);
    void getSensorData();
    bool checkPosFromPortInput(Vector &target_pos);
    void selectArm();
    void doReach(); 
    void doIdle();
    void steerTorsoToHome();
    void checkTorsoHome(const double timeout=10.0);
    void stopArmJoints(const int sel=USEDARM);
    void steerArmToHome(const int sel=USEDARM);
    void checkArmHome(const int sel=USEDARM, const double timeout=10.0);
    void stopControl();
    
    void limitRange(Vector &x);
    Matrix &rotx(const double theta);
    Matrix &roty(const double theta);
    Matrix &rotz(const double theta);
    bool reachableTarget(const Vector target_pos);
    bool targetCloseEnough();
    
    
public:
    // CONSTRUCTOR
    cartControlReachAvoidThread(int _rate, const string &_name, const string &_robot,
                      int _v, ResourceFinder &_rf);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    int printMessage(const int l, const char *f, ...);
    // RELEASE
    virtual void threadRelease();
    
    bool setTargetFromRPC(const Vector target_pos);
    bool setHomeFromRPC();
    
};


#endif
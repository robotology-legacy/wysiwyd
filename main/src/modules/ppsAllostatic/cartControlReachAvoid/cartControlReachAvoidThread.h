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

#include <vector>

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
#include <yarp/math/SVD.h>
#include <yarp/os/Log.h>
#include <iCub/ctrl/neuralNetworks.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/common.h>

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
using namespace iCub::skinDynLib;

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
       
    IEncoders         *encArm;
    IPositionControl  *posArm;
    IVelocityControl2 *velArm;
    ICartesianControl *cartArm;
    IControlMode2     *modArm;
    IEncoders         *encTorso;
    IPositionControl  *posTorso;
    IVelocityControl  *velTorso;
    IControlMode2     *modTorso;
   
    minJerkVelCtrlForIdealPlant *minJerkVelCtrl;
    
    BufferedPort<Bottle> inportTargetCoordinates;
    Bottle               *targetCoordinates;
    BufferedPort<Bottle> inportAvoidanceVectors;
    BufferedPort<Bottle> inportReachingGain;
    BufferedPort<Bottle> inportAvoidanceGain;
    
    VectorOf<int> armIdx;
    
    Vector homePoss, homeVels;
          
    int armAxes;    //16 joints of the arm+hand (no torso)
    Vector arm; //encoder values
    int torsoAxes;
    Vector torso; //encoder values
    int cartNrDOF;
    Vector cartDOFconfig;
    
    bool wentHome;
  
    double trajTime;
    double reachTol;
    double reachTimer, reachTmo;
    
    double reachingGain;
    double avoidanceGain;
       
    struct {
        double minX, maxX;
        double minY, maxY;
        double minZ, maxZ;
    } reachableSpace;
  
    struct avoidanceStruct_t{
        SkinPart skin_part;
        Vector x;
        Vector n;
    };
    vector<avoidanceStruct_t> avoidanceVectors;
    
    Vector openHandPoss;
    Vector handVels;
      
    bool newTargetFromRPC;
    bool newTargetFromPort;
    Vector targetPosFromPort;
    Vector targetPosFromRPC;
    Vector targetPos;
    double targetRadius;
    
    Matrix R,Rx,Ry,Rz;
    
    int state;
    int startup_context_id_left;
    int startup_context_id_right;
    
    void getTorsoOptions(Bottle &b, const char *type, const int i, Vector &sw, Matrix &lim);
    void getHomeOptions(Bottle &b, Vector &poss, Vector &vels);
    void initCartesianCtrl(Vector &sw, Matrix &lim, const int sel=USEDARM);
    bool checkTargetFromPortInput(Vector &target_pos, double &target_radius);
    bool getReachingGainFromPort();
    bool getAvoidanceGainFromPort();
    bool getAvoidanceVectorsFromPort();
    void selectArm();
    void doReach(); 
    void doIdle();
    void steerTorsoToHome();
    void checkTorsoHome(const double timeout=10.0);
    void stopArmJoints(const int sel=USEDARM);
    void steerArmToHome(const int sel=USEDARM);
    void checkArmHome(const int sel=USEDARM, const double timeout=10.0);
    void setControlModeArmsAndTorso(const int mode);
    
    void limitRange(Vector &x);
    Matrix &rotx(const double theta);
    Matrix &roty(const double theta);
    Matrix &rotz(const double theta);
    bool reachableTarget(const Vector target_pos);
    bool targetCloseEnough();

    bool computeFoR(const Vector &pos, const Vector &norm, Matrix &FoR);
       
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

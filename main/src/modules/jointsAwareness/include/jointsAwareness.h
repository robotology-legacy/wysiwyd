#include <iostream>
#include <sstream>
#include <map>
#include <fstream>
#include <string>
#include <map>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include "wrdac/clients/icubClient.h"



using namespace wysiwyd::wrdac;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;



class jointsAwareness : public RFModule {
private:

    //conf options
    //wysiwyd::wrdac::ICubClient  *iCub;
    double      period;

    string arm, robot, moduleName;

    yarp::os::Port   rpcPort;

    yarp::os::BufferedPort<yarp::os::Bottle>   armLeftPort;
    yarp::os::BufferedPort<yarp::os::Bottle>   armRightPort;
    yarp::os::BufferedPort<yarp::os::Bottle>   torsoPort;

    yarp::os::BufferedPort<yarp::os::Bottle>   armLeft_2DProj_Port;
    yarp::os::BufferedPort<yarp::os::Bottle>   armRight_2DProj_Port;
    yarp::os::BufferedPort<yarp::os::Bottle>   torso_2DProj_Port;

    yarp::os::BufferedPort<yarp::os::Bottle>   read_ObjLoc_Port;
    yarp::os::BufferedPort<yarp::os::Bottle>   write_Obj2DProj_Port;

    yarp::dev::PolyDriver leftArmClientCartCtrl;
    yarp::dev::PolyDriver rightArmClientCartCtrl;
    yarp::dev::ICartesianControl *iLeftArm = NULL;
    yarp::dev::ICartesianControl *iRightArm =  NULL;

    yarp::dev::PolyDriver gazeClientCtrl;
    yarp::dev::IGazeControl *iGaze;

    std::map<string, yarp::os::BufferedPort<yarp::os::Bottle>*> locPorts_map;
    std::map<string, yarp::os::BufferedPort<yarp::os::Bottle>*> projPorts_map;
    std::map<string, yarp::dev::PolyDriver*> polydriver_map;
    std::map<string, yarp::dev::ICartesianControl*> cartesian_map;

    std::map<string, yarp::os::Bottle> test_map;



    unsigned int torsoJointsNb = 3;
    unsigned int armJointsNb   = 7;
    bool isTorsoDone = false;

    bool configMaps();
    bool configCartesian(string part);
    bool streamCartesian(string part, string cartesianPart = "default");

    bool streamObjects();

public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

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

class jointsAwareness : public yarp::os::RFModule {
private:

    //conf options
    double      period;

    std::string arm, camera, robot, moduleName;

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

    std::map<std::string, yarp::os::BufferedPort<yarp::os::Bottle>*> locPorts_map;
    std::map<std::string, yarp::os::BufferedPort<yarp::os::Bottle>*> projPorts_map;
    std::map<std::string, yarp::dev::PolyDriver*> polydriver_map;
    std::map<std::string, yarp::dev::ICartesianControl*> cartesian_map;

    std::map<std::string, yarp::os::Bottle> test_map;



    unsigned int torsoJointsNb = 3;
    unsigned int armJointsNb   = 7;
    bool isTorsoDone = false;
    bool isObjectStreamed = true;

    bool configMaps();
    bool configCartesian(std::string part, std::string cameraSuffix);
    bool streamCartesian(std::string part, std::string cartesianPart = "default");

    void streamObjects();

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
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
};

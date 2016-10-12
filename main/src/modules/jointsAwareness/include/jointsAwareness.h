#include <map>
#include <fstream>
#include <string>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
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

    yarp::dev::PolyDriver leftArmClientCartCtrl;
    yarp::dev::PolyDriver rightArmClientCartCtrl;
    yarp::dev::ICartesianControl *iLeftArm = NULL;
    yarp::dev::ICartesianControl *iRightArm =  NULL;


    bool configCartesian(yarp::dev::PolyDriver& driver, yarp::dev::ICartesianControl* icart, yarp::os::BufferedPort<Bottle>& port, string part);

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

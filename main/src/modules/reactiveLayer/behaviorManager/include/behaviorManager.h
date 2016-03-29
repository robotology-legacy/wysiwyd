#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/SVD.h>
#include <yarp/math/Rand.h>
#include <map>

#include "dummy.h"
#include "tagging.h"
#include "touchingOrder.h"
#include "pointingOrder.h"
#include "pointing.h"
#include "reactions.h"
#include "narrate.h"
#include "followingOrder.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


class BehaviorManager: public RFModule
{
private:


    vector<Behavior*> behaviors;

    string moduleName;

    double period;

    Port rpc_in_port;

    ICubClient *iCub;

    int behavior_to_trigger;

    Mutex mut;
    Bottle behaviorList;
    string behavior_name;
public:
   bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule()
    {
        return true;
    }

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);

};


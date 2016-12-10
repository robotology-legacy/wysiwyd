#include <iostream>
#include <yarp/os/all.h>

#include "dummy.h"
#include "tagging.h"
#include "pointing.h"
#include "reactions.h"
#include "narrate.h"
#include "followingOrder.h"
#include "recognitionOrder.h"
#include "speech.h"
#include "greeting.h"
#include "ask.h"

class BehaviorManager: public yarp::os::RFModule
{
private:


    std::vector<Behavior*> behaviors;

    std::string moduleName;

    double period;

    yarp::os::Port rpc_in_port;

    wysiwyd::wrdac::ICubClient *iCub;

    // int behavior_to_trigger;

    yarp::os::Mutex mut;
    yarp::os::Bottle behaviorList;
    std::string behavior_name;

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


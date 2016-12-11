#ifndef BEHAVIORMANAGER_H
#define BEHAVIORMANAGER_H

#include <iostream>
#include <yarp/os/all.h>
#include "behavior.h"

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

#endif

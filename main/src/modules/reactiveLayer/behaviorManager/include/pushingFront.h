#ifndef PUSHINGFRONT
#define PUSHINGFRONT

#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>


#include "behavior.h"

class PushingFront: public Behavior
{
private:
    void run(yarp::os::Bottle args=yarp::os::Bottle());

    double target;
    yarp::sig::Vector object;
    std::string obj_name, obj_type;

public:
    PushingFront(Mutex* mut, ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }

    void configure();
    void close_extra_ports() {
        ;
    }
};

#endif // PUSHINGFRONT


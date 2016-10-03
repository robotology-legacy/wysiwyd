#ifndef MOVEOBJECT
#define MOVEOBJECT

#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>


#include "behavior.h"

class MoveObject: public Behavior
{
private:
    void run(const yarp::os::Bottle &args);

    double target_pullback;
    double target_pushfront;
    double target_pushleft;
    double target_pushright;
    yarp::sig::Vector object;
    std::string obj_name, obj_type;

public:
    MoveObject(yarp::os::Mutex* mut, yarp::os::ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }

    void configure();
    void close_extra_ports() {
        ;
    }
};

#endif // MOVEOBJECT

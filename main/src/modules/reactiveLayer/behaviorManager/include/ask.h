#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>


#include "behavior.h"

class Ask: public Behavior
{
public:
    Ask(yarp::os::Mutex* mut, yarp::os::ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }    
    void configure();
    void run(const yarp::os::Bottle &args);
    void close_extra_ports() {
        ;
    }    

    yarp::os::Bottle toSay;
};


#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "wrdac/clients/icubClient.h"
#include <map>

#include "behavior.h"

class Greeting: public Behavior
{
public:
    void configure();
    void run(const yarp::os::Bottle &args);
    Greeting(yarp::os::Mutex* mut, yarp::os::ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }
    void close_extra_ports() {
        ;
    }
};


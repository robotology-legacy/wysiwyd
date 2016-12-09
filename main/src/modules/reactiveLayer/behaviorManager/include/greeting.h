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
    void run(const Bottle &args);
    Greeting(Mutex* mut, ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }
    void close_extra_ports() {
        ;
    }
};


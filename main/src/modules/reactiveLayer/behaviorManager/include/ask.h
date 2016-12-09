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
    Ask(Mutex* mut, ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }    
    void configure();
    void run(Bottle args=Bottle());
    void close_extra_ports() {
        ;
    }    

    Bottle toSay;
};


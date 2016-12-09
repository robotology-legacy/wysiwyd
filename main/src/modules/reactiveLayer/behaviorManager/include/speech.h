#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>


#include "behavior.h"

class Speech: public Behavior
{
public:
    Speech(Mutex* mut, ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }    
    void configure();
    void run(const Bottle &args);
    void close_extra_ports() {
        ;
    }    
};


#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>


#include "behavior.h"


class Narrate: public Behavior
{
private:
    void run(const yarp::os::Bottle &args);
    
public:
    Narrate(yarp::os::Mutex* mut, yarp::os::ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }
        
    void configure();
    void close_extra_ports() {
        ;
    }
};


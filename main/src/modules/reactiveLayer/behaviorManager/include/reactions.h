#include <iostream>
#include <yarp/os/all.h>

#include "behavior.h"


class Reactions: public Behavior
{
private:
    void run(const yarp::os::Bottle &args);
    
public:
    Reactions(yarp::os::Mutex* mut, yarp::os::ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }
       
    void configure();

    void close_extra_ports() {
        ;
    }
};

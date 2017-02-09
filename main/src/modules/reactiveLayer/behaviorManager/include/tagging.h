#ifndef TAGGING_H
#define TAGGING_H

#include <iostream>
#include <yarp/os/all.h>

#include "behavior.h"
#include "followingOrder.h"

class Tagging: public Behavior
{
private:
    void run(const yarp::os::Bottle &args);
    yarp::os::Bottle bKS1, bKS2;
    
public:
    Tagging(yarp::os::Mutex* mut, yarp::os::ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }
       
    void configure();

    void close_extra_ports() {
        ;
    }
};

#endif

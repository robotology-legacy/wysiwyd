#ifndef SPEECH_H
#define SPEECH_H

#include <iostream>
#include <yarp/os/all.h>

#include "behavior.h"


class Speech: public Behavior
{
private:
    void run(const yarp::os::Bottle &args);

public:
    Speech(yarp::os::Mutex* mut, yarp::os::ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }

    void configure();

    void close_extra_ports() {
        ;
    }
};

#endif

#ifndef RECOGNITIONORDER_H
#define RECOGNITIONORDER_H

#include <iostream>
#include <yarp/os/all.h>

#include "behavior.h"


class RecognitionOrder: public Behavior
{
private:
    void run(const yarp::os::Bottle &args);
    yarp::os::Port port_to_homeo;
    std::string port_to_homeo_name;
    std::string homeoPort;

public:
    RecognitionOrder(yarp::os::Mutex* mut, yarp::os::ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }
       
    void configure();

    void close_extra_ports() {
        port_to_homeo.interrupt();
        port_to_homeo.close();
    }
    bool manual;
};

#endif

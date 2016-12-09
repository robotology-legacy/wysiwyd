#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>


#include "behavior.h"

class recognitionOrder: public Behavior
{
private:
    void run(const Bottle &args);
    yarp::os::Port port_to_homeo;
    string port_to_homeo_name;
    string homeoPort;

public:
    recognitionOrder(Mutex* mut, ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }
       
    void configure();

    void close_extra_ports() {
        ;
    }
    bool manual;
};


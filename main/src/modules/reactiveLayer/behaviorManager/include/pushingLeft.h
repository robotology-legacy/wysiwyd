#ifndef PUSHINGLEFT
#define PUSHINGLEFT

#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>


#include "behavior.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;



class PushingLeft: public Behavior
{
private:
    void run(Bottle args=Bottle());
    double target;
public:
    PushingLeft(Mutex* mut, ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }

    void configure();
    void close_extra_ports() {
        ;
    }
};

#endif // PUSHINGLEFT


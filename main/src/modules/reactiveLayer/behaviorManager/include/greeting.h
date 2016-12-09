#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <yarp/math/SVD.h>
#include <yarp/math/Rand.h>
#include "wrdac/clients/icubClient.h"
#include <map>

#include "behavior.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wysiwyd::wrdac;

class Greeting: public Behavior
{
public:
    void configure();
    void run(Bottle args=Bottle());
    Greeting(Mutex* mut, ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }
    void close_extra_ports() {
        ;
    }
};


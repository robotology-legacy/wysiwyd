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

class Dummy: public Behavior
{

public:
    void configure() {
        name = "dummy";
        external_port_name = "None";
        from_sensation_port_name = "None";        
    }

    void run(Bottle args=Bottle()) {
        yDebug() << "Dummmy::run";
    }

    void close_extra_ports() {
        ;
    }

};


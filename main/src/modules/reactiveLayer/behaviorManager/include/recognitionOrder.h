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



class recognitionOrder: public Behavior
{
private:
    void run(Bottle args=Bottle());
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


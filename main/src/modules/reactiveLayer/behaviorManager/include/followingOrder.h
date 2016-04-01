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



class FollowingOrder: public Behavior
{
private:
    yarp::os::Port port_to_narrate;
    string port_to_narrate_name;
    void run(Bottle args=Bottle());

public:
    FollowingOrder(Mutex* mut): Behavior(mut) {
        ;
    }
      
    void configure();

    bool handleAction(string type, string target, string action);
    bool handleSearch(string type, string target);
    bool handleNarrate();

    void close_extra_ports() {
        ;
    }
};

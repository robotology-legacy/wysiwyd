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

    //followingOrder option
    Bottle listKS1;
    Bottle listKS2;

public:
    FollowingOrder(Mutex* mut, ResourceFinder &rf): Behavior(mut, rf) {
        ;
    }
      
    void configure();

    bool handleAction(string type, string target, string action);
    bool handleActionBP(string type, string target, string action);
    bool handleActionKS(string type, string action);
    bool handleSearch(string type, string action);
    bool handleNarrate();

    void close_extra_ports() {
        ;
    }
};

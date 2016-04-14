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
    Bottle bKS1;
    Bottle bKS2;
    string babblingArm;

public:
    FollowingOrder(Mutex* mut, ResourceFinder &rf): Behavior(mut, rf) {
        ;
    }
      
    void configure();

    bool handleAction(string type, string target, string action);
    bool handleActionBP(string type, string target, string action);
    bool handleActionKS(string action, string type);
    bool handleSearch(string type, string action);
    bool handleNarrate();

    int randKS(Bottle bKS);

    void close_extra_ports() {
        ;
    }
};

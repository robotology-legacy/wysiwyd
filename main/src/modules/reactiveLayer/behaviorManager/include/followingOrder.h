#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>


#include "behavior.h"

class FollowingOrder: public Behavior
{
private:
    yarp::os::Port port_to_narrate;
    yarp::os::Port port_to_homeo;
    yarp::os::Port port_to_avoidance;
    string port_to_narrate_name;
    string port_to_avoidance_name;
    string port_to_homeo_name;

    void run(const Bottle &args);
    string homeoPort;

    //followingOrder option
    Bottle bKS1;
    Bottle bKS2;
    string babblingArm;

public:
    FollowingOrder(Mutex* mut, ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        ;
    }
      
    void configure();
    bool manual;

    bool handleAction(string type, string target, string action);
    bool handleActionBP(string type, string target, string action);
    bool handleActionKS(string action, string type);
    bool handleSearch(string type, string action, bool verboseSearch);
    bool handleNarrate();
    bool handleGame(string type);
    bool handleEnd();

    int randKS(Bottle bKS);

    void close_extra_ports() {
        ;
    }
};

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
    bool finding;
    bool pointing;
    void run(Bottle args=Bottle());

public:
    FollowingOrder(Mutex* mut): Behavior(mut) {
        ;
    }
      
    void configure();

    bool handlePoint(string type, string target);
    bool handlePush(string type, string target);
    bool handleLook(string type, string target);
    bool handleSearch(string type, string target);

    void close_extra_ports() {
        ;
    }
};


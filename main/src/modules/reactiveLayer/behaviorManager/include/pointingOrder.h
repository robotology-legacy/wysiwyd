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



class PointingOrder: public Behavior
{
private:
    bool finding;
    bool pointing;

public:
    void configure();
    void run(Bottle args=Bottle());

    bool handlePoint(string target);
    bool handleSearch(string target);

    void close_extra_ports() {
        ;
    }
};


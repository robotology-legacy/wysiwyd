#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "wrdac/clients/icubClient.h"
#include <map>

#include "sensation.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

class Test: public Sensation
{
private:
    bool on;
    string moduleName, unknown_obj_port_name, confusion_port_name;
    BufferedPort<Bottle> in;
    yarp::os::BufferedPort<Bottle> out;


public:
    void configure();
    void publish();
    void close_ports() {
        ;
    }

};

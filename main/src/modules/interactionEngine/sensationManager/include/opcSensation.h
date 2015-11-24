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
using namespace yarp::math;
using namespace wysiwyd::wrdac;

class OpcSensation: public Sensation
{
private:
    ICubClient *iCub;
    bool confusion;
    string moduleName, unknown_obj_port_name, confusion_port_name;
    yarp::os::BufferedPort<Bottle> unknown_obj_port;
    yarp::os::BufferedPort<Bottle> confusion_port;

    Bottle handleTagging();

public:
    void configure();
    void publish();

};
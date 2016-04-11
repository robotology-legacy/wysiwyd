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

class OpcSensation: public Sensation
{
private:
    ICubClient *iCub;
    bool confusion;
    string moduleName, is_touched_port_name, unknown_entities_port_name, known_entities_port_name, opc_has_unknown_port_name, opc_has_known_port_name, opc_has_agent_name; //, show_port_name, known_obj_port_name, friendly_port_name, greeting_port_name;
    yarp::os::BufferedPort<Bottle> unknown_entities_port;
    yarp::os::BufferedPort<Bottle> opc_has_unknown_port;
    yarp::os::BufferedPort<Bottle> known_entities_port;
    yarp::os::BufferedPort<Bottle> opc_has_known_port;
    yarp::os::BufferedPort<Bottle> opc_has_agent_port;
    yarp::os::BufferedPort<Bottle> is_touched_port;
    // yarp::os::BufferedPort<Bottle> touch_location_port;
    // yarp::os::BufferedPort<Bottle> friendly_port, greeting_port;

    // yarp::os::BufferedPort<Bottle> known_obj_port;
    // yarp::os::BufferedPort<Bottle> show_port;

    Bottle handleEntities();
    void handleTouch();
    // Bottle handlePointing();

public:

    void configure();
    void publish();

    void close_ports() {
        unknown_entities_port.interrupt();
        unknown_entities_port.close();
        opc_has_unknown_port.interrupt();
        opc_has_unknown_port.close();       
        known_entities_port.interrupt();
        known_entities_port.close();
        opc_has_known_port.interrupt();
        opc_has_known_port.close();   
        opc_has_agent_port.interrupt();
        opc_has_agent_port.close();
        is_touched_port.interrupt();
        is_touched_port.close();
    }
};

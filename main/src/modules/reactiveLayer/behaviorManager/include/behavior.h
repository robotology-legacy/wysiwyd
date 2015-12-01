#ifndef BEHAVIOR
#define BEHAVIOR

#include <string>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

class Behavior
{
public:

    void openPorts(string port_name_prefix="") {
        if (from_sensation_port_name != "None") {
            sensation_port_in.open("/" + port_name_prefix +"/" + name + "/sensation:i");
        }    
        if (external_port_name != "None") {
            rpc_out_port.open("/" + port_name_prefix +"/" + name + "/to_external_module");
        }
    }

    ICubClient *iCub;

    string name, from_sensation_port_name, external_port_name;
    BufferedPort<Bottle> sensation_port_in;
    // Port rpc_in_port;
    Port rpc_out_port;
    virtual void configure() = 0;
    virtual void run(Bottle args=Bottle()) = 0;
    
};

#endif
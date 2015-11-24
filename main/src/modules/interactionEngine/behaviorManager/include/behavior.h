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
        // rpc_in_port.open("/" + port_name_prefix +"/" + name + "/trigger:i");
        // attach(rpc_in_port);
        // rpc_in_port.setReader(*this);
        // this->useCallback();
        if (from_sensation_port_name != "None") {
            sensation_port_in.open("/" + port_name_prefix +"/" + name + "/sensation:i");
        }    
        if (external_port_name != "None") {
            rpc_out_port.open("/" + port_name_prefix +"/" + name + "/to_external_module");
        }
        // rpc_out_port.addOutput(external_port_name);        
    }
    // virtual bool read(ConnectionReader& connection) {
    // Bottle b;
    // bool ok = b.read(connection);
    // if (!ok) return false;
    //     // process data in b
    //     yInfo() << "Behavior " + name + " read";
    //     run();
    // return true;        
    // }

    // bool respond(const Bottle& cmd, Bottle& reply) {
    //     yInfo() << "Behavior " + name + " read";
    //     run();  
    //     return true;     
    // }

    ICubClient *iCub;

    string name, from_sensation_port_name, external_port_name;
    BufferedPort<Bottle> sensation_port_in;
    // Port rpc_in_port;
    Port rpc_out_port;
    virtual void configure() = 0;
    virtual void run() = 0;
    
};

#endif
#ifndef BEHAVIOR
#define BEHAVIOR

#include <string>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"
#include <wrdac/clients/clients.h>
#include "wrdac/subsystems/subSystem_ABM.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

class Behavior
{
private:
    Mutex* mut;

    virtual void run(Bottle args=Bottle()) = 0;
public:

    Behavior(Mutex* _mut) {
        mut = _mut;
    }

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
    Port rpc_out_port;

    void trigger(Bottle args=Bottle()) {
        yDebug() << "Behavior::trigger starts"; 
        if (mut->tryLock()) {
            yDebug() << "Behavior::trigger mutex closed"; 
            run(args);
            // Time::delay(0.0);
            mut->unlock();
        }
        yDebug() << "Behavior::trigger ends";
    }

    virtual void configure() = 0;
    virtual void close_extra_ports() = 0;

    void close_ports() {
        close_extra_ports();
        sensation_port_in.interrupt();
        sensation_port_in.close();
        rpc_out_port.interrupt();
        rpc_out_port.close();
    }
    
};

#endif

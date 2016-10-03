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

    Behavior(Mutex* _mut, ResourceFinder &_rf, std::string _behaviorName) : mut(_mut), behaviorName(_behaviorName), rf(_rf){
        from_sensation_port_name = "None";
        external_port_name = "None";
    }

    void openPorts(string port_name_prefix) {
        if (from_sensation_port_name != "None") {
            sensation_port_in.open("/" + port_name_prefix +"/" + behaviorName + "/sensation:i");
        }
        if (external_port_name != "None") {
            rpc_out_port.open("/" + port_name_prefix +"/" + behaviorName + "/to_external_module");
        }
        behavior_start_stop_port.open("/" + port_name_prefix +"/" + behaviorName + "/start_stop:o");
    }

    ICubClient *iCub;
    string from_sensation_port_name, external_port_name;
    BufferedPort<Bottle> sensation_port_in, behavior_start_stop_port;

    Port rpc_out_port;
    std::string behaviorName;
    ResourceFinder& rf;

    bool trigger(Bottle args=Bottle()) {
        yDebug() << "Behavior::trigger starts"; 
        if (mut->tryLock()) {
            yDebug() << "Behavior::trigger mutex closed"; 
            yarp::os::Bottle & msg = behavior_start_stop_port.prepare();
            msg.clear();
            msg.addString("start");
            behavior_start_stop_port.write();

            run(args);
            
            msg = behavior_start_stop_port.prepare();
            msg.clear();
            msg.addString("stop");
            behavior_start_stop_port.write();
            mut->unlock();
            return true;
        }
        yDebug() << "Behavior::trigger ends";
        return false;
    }

    virtual void configure() = 0;
    virtual void close_extra_ports() = 0;

    void interrupt_ports() {
        sensation_port_in.interrupt();
        rpc_out_port.interrupt();
        behavior_start_stop_port.interrupt();
    }

    void close_ports() {
        close_extra_ports();
        sensation_port_in.interrupt();
        sensation_port_in.close();
        rpc_out_port.interrupt();
        rpc_out_port.close();
        behavior_start_stop_port.interrupt();
        behavior_start_stop_port.close();
    }
    
};

#endif

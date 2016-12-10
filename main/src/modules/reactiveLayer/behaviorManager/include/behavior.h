#ifndef BEHAVIOR
#define BEHAVIOR

#include <string>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"

class Behavior
{
private:
    yarp::os::Mutex* mut;

    virtual void run(const yarp::os::Bottle &args) = 0;
public:

    Behavior(yarp::os::Mutex* _mut, yarp::os::ResourceFinder &_rf, std::string _behaviorName) : mut(_mut), behaviorName(_behaviorName), rf(_rf){
        from_sensation_port_name = "None";
        external_port_name = "None";
    }

    void openPorts(std::string port_name_prefix) {
        if (from_sensation_port_name != "None") {
            sensation_port_in.open("/" + port_name_prefix +"/" + behaviorName + "/sensation:i");
        }
        if (external_port_name != "None") {
            rpc_out_port.open("/" + port_name_prefix +"/" + behaviorName + "/to_external_module");
        }
        behavior_start_stop_port.open("/" + port_name_prefix +"/" + behaviorName + "/start_stop:o");
    }

    wysiwyd::wrdac::ICubClient *iCub;
    std::string from_sensation_port_name, external_port_name;
    yarp::os::BufferedPort<yarp::os::Bottle> sensation_port_in, behavior_start_stop_port;

    yarp::os::Port rpc_out_port;
    std::string behaviorName;
    yarp::os::ResourceFinder& rf;

    bool trigger(const yarp::os::Bottle& args) {
        yDebug() << behaviorName << "::trigger starts"; 
        if (mut->tryLock()) {
            yDebug() << behaviorName << "::trigger mutex closed"; 
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
            yDebug() << behaviorName << "::trigger mutex open"; 
            return true;
        }
        yDebug() << behaviorName << "::trigger ends";
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

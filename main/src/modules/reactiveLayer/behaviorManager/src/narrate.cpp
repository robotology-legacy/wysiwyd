#include "narrate.h"

using namespace std;
using namespace yarp::os;

void Narrate::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    external_port_name = "/narrativeGraph/rpc";
    yInfo() << "external_port_name: " << external_port_name;

    from_sensation_port_name = "None";
}

void Narrate::run(const Bottle &args) {
    yInfo() << "Narrate::run";
    Bottle cmd;
    Bottle rply;
    cmd.clear();
    cmd.addString("HRI");
    yInfo() << "Proactively narrating...";
    
    rpc_out_port.write(cmd, rply);
}

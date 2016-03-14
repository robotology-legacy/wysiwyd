#include "narrate.h"

void Narrate::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    name = "narrate";
    external_port_name = "/narrationHandler/rpc";
    //from_sensation_port_name = "/opcSensation/known_entities:o";
}

void Narrate::run(Bottle args/*=Bottle()*/) {
    yInfo() << "Narrate::run";
    Bottle cmd;
    Bottle rply;
    cmd.clear();
    cmd.addString("start");
    cmd.addString("narration");
    yInfo() << "Proactively narrating...";
    rpc_out_port.write(cmd, rply);

}

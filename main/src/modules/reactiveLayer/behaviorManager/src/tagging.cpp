#include "tagging.h"

void Tagging::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    name = "tagging";
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/opcSensation/unknown_entities:o";


};

void Tagging::run(Bottle args/*=Bottle()*/) {
    yInfo() << "Tagging::run";
    // Bottle *objects = sensation.get(1).asList();
    //iCub->say("I dont know all of these objects");
    int id = 0; //Should be a random
    yDebug() << "send rpc to proactiveTagging";//<<endl;
    Bottle *sensation = sensation_port_in.read();
    //If there is an unknown object (to see with agents and rtobjects), add it to the rpc_command bottle, and return true
    Bottle cmd;
    Bottle rply;
    cmd.clear();
    cmd.addString("exploreUnknownEntity");
    cmd.addString(sensation->get(id).asList()->get(0).asString());
    cmd.addString(sensation->get(id).asList()->get(1).asString());
    yInfo() << "Proactively tagging...";
    rpc_out_port.write(cmd, rply);

}

#include "pointing.h"

void Pointing::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    name = "pointing";
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/opcSensation/known_obj:o";


};

void Pointing::run(Bottle args/*=Bottle()*/) {
    yInfo() << "Pointing::run";
    Bottle *sensation = sensation_port_in.read();
    int id = 0;
    // Bottle *objects = sensation.get(1).asList();
    iCub->say("Do you know this is a " + sensation->get(id).asList()->get(1).asString());
    Bottle bHand("right"); //Hardcoded!!!
    iCub->point(sensation->get(id).asList()->get(1).asString(), bHand);

}
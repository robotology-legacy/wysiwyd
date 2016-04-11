#include "pointing.h"

void Pointing::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    name = "pointing";
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/opcSensation/known_entities:o";
}

void Pointing::run(Bottle args/*=Bottle()*/) {
    yInfo() << "Pointing::run";
    Bottle *sensation = sensation_port_in.read();
    
    int id = yarp::os::Random::uniform(0, sensation->size()-1);
    yDebug()<<"Randomly selected: "<< id; // should be random here 
    string obj_name = sensation->get(id).asList()->get(1).asString();
    
    iCub->opc->checkout();
    yDebug() << "[pointing]: opc checkout";
    iCub->lookAtPartner();
    Time::delay(0.5);
    
    iCub->say("I could point to the " + obj_name);
    iCub->home();
    Time::delay(1.5);

    bool succeeded = iCub->point(obj_name);
    Time::delay(0.2);

    if (succeeded) {
        iCub->lookAtPartner();
        iCub->say("Do you know that this is a " + obj_name, false);
    } else {
        iCub->say(" I couldn't find the " + obj_name);
    }

    iCub->home();

}

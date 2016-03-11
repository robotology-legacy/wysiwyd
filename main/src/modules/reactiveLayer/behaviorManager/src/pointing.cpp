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
    int id = 0;  // should be random here
    string obj_name = sensation->get(id).asList()->get(1).asString();
    
    iCub->say("I could point to the " + obj_name);
    Time::delay(2.0);
    // yInfo() << "About to point the " + obj_name;
    // return;

    iCub->say("Do you know that this is a " + obj_name);
    Object* obj = iCub->opc->addOrRetrieveEntity<Object>(obj_name);
    string sHand = "right";
    if (obj->m_ego_position[1]<0)
        sHand = "left";
    Bottle bHand(sHand);
    iCub->point(obj_name, bHand);
    Time::delay(0.5);
    iCub->home();

}

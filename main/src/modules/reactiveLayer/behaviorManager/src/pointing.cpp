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

    int id = yarp::os::Random::uniform(0, sensation->size() - 1);
    yDebug() << "Randomly selected: " << id; // should be random here 
    string obj_name = sensation->get(id).asList()->get(1).asString();

    iCub->opc->checkout();
    yInfo() << "[pointing] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();
    string aName = "";
    for (auto& entity : lEntities)
    {
        if (entity->entity_type() == "agent")
        {
            aName = entity->name();
        }
    }
    if (aName == "")
    {
        yInfo() << "No agents detected, looking action aborted.";
    }
    else
    {
        iCub->look(aName);
        Time::delay(0.5);
    }

    iCub->say("I could point to the " + obj_name);
    Time::delay(2.0);

    Object* obj = iCub->opc->addOrRetrieveEntity<Object>(obj_name);
    //string sHand = "right";
    //if (obj->m_ego_position[1] < 0)
    //    sHand = "left";

    //Bottle bHand(sHand);

    //bool succeeded = iCub->point(obj_name, bHand);
    bool succeeded = iCub->point(obj_name);
    Time::delay(0.2);

    if (succeeded)
    {
        if (aName != "")
        {
            iCub->look(aName);
            Time::delay(0.5);
        }

        iCub->say("Do you know that this is a " + obj_name, false);
    }


    else {
        iCub->say(" I couldn't find the " + obj_name);
    }

    iCub->home();

}

#include "pushingLeft.h"

using namespace std;

void PushingLeft::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    external_port_name = "/karmaMotor/rpc";
    from_planner_port_name = "/planner/context:o";

    if (rf.check("leftPos"))
    {
        target = rf.find("leftPos").asDouble();
    }

}

void PushingLeft::run(Bottle args/*=Bottle()*/) {
    yInfo() << "PushingLeft::run";

    string obj_name;
    string obj_type;

    Bottle *planner = planner_port_in.read();

    if (planner!=NULL)
    {
        obj_type = planner->get(0).asString();
        obj_name = planner->get(1).asString();
    }

//    iCub->say("I will push the " + obj_name + "to the left");
//    iCub->opc->checkout();
//    yDebug() << "[pushingLeft]: opc checkout";
//    Time::delay(0.5);

    Vector object(3,0.0);
    double radius = abs(object[1]-target);

    yInfo() << "received context from planner:" << obj_type << "and" << obj_name;

    // Obtain object coordinate from opc port

    bool succeeded = iCub->pushKarma(object,180,radius);
    Time::delay(0.2);

    if (succeeded) {
        iCub->lookAtPartner();
    } else {
        iCub->say(" I missed");
    }

    iCub->home();

}

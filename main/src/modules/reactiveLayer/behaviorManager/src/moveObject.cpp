#include "moveObject.h"

using namespace std;
using namespace yarp::os;

void MoveObject::configure() {
    Bottle targetGroup = (rf.findGroup("targetPos"));
    target_pullback = targetGroup.check("backPos",Value(-0.25)).asDouble();
    target_pushfront = targetGroup.check("frontPos",Value(-0.5)).asDouble();
    target_pushleft = targetGroup.check("leftPos",Value(-0.2)).asDouble();
    target_pushright = targetGroup.check("rightPos",Value(0.2)).asDouble();
}

void MoveObject::run(const Bottle &args) {
    yInfo() << "MoveObject::run";

    bool succeeded;
    string obj_name, move_type;

    if (args.size()!=0) {
        obj_name = args.get(0).asList()->get(0).asString();
        move_type = args.get(0).asList()->get(1).asString();
    } else {
        yError() << "Wrong number of parameters, abort";
        return;
    }
    yInfo() << "received context from planner:" << move_type << "and" << obj_name;

    iCub->lookAtPartner();

    if(move_type == "front") {
        iCub->say("I will push the " + obj_name + " to the front");
    } else if(move_type == "back") {
        iCub->say("I will pull the " + obj_name + " to the back");
    } else if(move_type == "left") {
        iCub->say("I will push the " + obj_name + " to the left");
    } else if(move_type == "right") {
        iCub->say("I will push the " + obj_name + " to the right");
    } else {
        yError() << "[moveObject] Wrong direction";
        iCub->say("I don't know this direction");
        return;
    }

    iCub->home();
    yarp::os::Time(1.0);

    iCub->opc->checkout();

    Bottle options;
    options.addString("fixate");
    options.addString("wait");
    iCub->look(obj_name,options); // to have a better estimate of where to move the object to

    iCub->opc->checkout();

    if(move_type == "front") {
        succeeded = iCub->pushKarmaFront(obj_name, target_pushfront);
    } else if(move_type == "back") {
        succeeded = iCub->pullKarmaBack(obj_name, target_pullback);
    } else if(move_type == "left") {
        succeeded = iCub->pushKarmaLeft(obj_name, target_pushleft);
    } else if(move_type == "right") {
        succeeded = iCub->pushKarmaRight(obj_name, target_pushright);
    } else {
        yError() << "[moveObject] Wrong direction";
        iCub->say("I don't know this direction");
        return;
    }

    if (!succeeded) {
        iCub->lookAtPartner();
        iCub->say("I could not move the object");
        yError() << "Karma did not succeed moving the object";
    }

    iCub->home();
}

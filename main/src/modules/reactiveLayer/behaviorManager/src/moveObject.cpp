#include "moveObject.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void MoveObject::configure() {
    Bottle targetGroup = (rf.findGroup("targetPos"));
    target_pullback = targetGroup.check("backPos",Value(-0.1)).asDouble();
    target_pushfront = targetGroup.check("frontPos",Value(0.1)).asDouble();
    target_pushleft = targetGroup.check("leftPos",Value(-0.1)).asDouble();
    target_pushright = targetGroup.check("rightPos",Value(0.1)).asDouble();
}

void MoveObject::run(const Bottle &args) {
    yInfo() << "MoveObject::run";
    iCub->home(); //To make sure that it can see the objects

    bool succeeded;
    string obj_type, obj_name, move_type;

    if (args.size()==3) {
        obj_type = args.get(0).asList()->get(0).asString();
        obj_name = args.get(0).asList()->get(1).asString();
        move_type = args.get(0).asList()->get(2).asString();
    } else {
        yError() << "Wrong number of parameters, abort";
        return;
    }
    yInfo() << "received context from planner:" << obj_type.c_str() << "and" << obj_name.c_str();

    iCub->look(obj_name); // to have a better estimate of where to move the object to
    if(move_type == "front") {
        iCub->say("I will push the " + obj_name + "to the front");
        succeeded = iCub->pushKarmaFront(obj_name, target_pushfront);
    } else if(move_type == "back") {
        iCub->say("I will pull the " + obj_name + "to the back");
        succeeded = iCub->pullKarmaBack(obj_name, target_pullback);
    } else if(move_type == "left") {
        iCub->say("I will push the " + obj_name + "to the left");
        succeeded = iCub->pushKarmaLeft(obj_name, target_pushleft);
    } else if(move_type == "right") {
        iCub->say("I will push the " + obj_name + "to the right");
        succeeded = iCub->pushKarmaRight(obj_name, target_pushright);
    } else {
        yError() << "[moveObject] Wrong direction";
        iCub->say("I don't know this direction");
        return;
    }

    Time::delay(0.2);

    if (succeeded) {
        iCub->lookAtPartner();
    } else {
        iCub->say(" I could not move the object");
        yError() << "Karma did not succeed moving the object";
    }

    iCub->home();
}

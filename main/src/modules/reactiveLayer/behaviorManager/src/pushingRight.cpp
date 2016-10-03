#include "pushingRight.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void PushingRight::configure() {

    external_port_name = "/karmaMotor/rpc";

    Bottle targetGroup = (rf.findGroup("targetPos"));
    if (targetGroup.check("rightPos"))
    {
        target = targetGroup.find("rightPos").asDouble();
        yInfo(" [pushingRight]: rightPos is %f",target);
    }
    else
    {
        target = 0.1;
        yWarning(" [pushingRight]: no rightPos. Set to default: %f", target);
    }
}

void PushingRight::run(Bottle args/*=Bottle()*/) {
    yInfo() << "PushingRight::run";
    yDebug() << args.toString();

    //if (!args.isNull())
    if (args.size()!=0)
    {
        yDebug()<<args.get(0).asString();
        yDebug()<<args.get(0).asList()->toString();
        yDebug()<<args.get(0).asList()->get(0).asString();
        obj_type = args.get(0).asList()->get(0).asString();
        obj_name = args.get(0).asList()->get(1).asString();
        yDebug() << "type and name extracted";
    }

    yDebug()<< "icub about to say stuff";
    iCub->say("I will push the " + obj_name + "to the Right");

    yInfo() << "received context from planner:" << obj_type.c_str() << "and" << obj_name.c_str();
    iCub->home();

    bool succeeded = iCub->pushKarmaRight(obj_name,target);
    cout<<(succeeded?"success":"failed")<<endl;
    Time::delay(0.2);

    if (succeeded) {
        iCub->lookAtPartner();
    } else {
        iCub->say(" I missed");
    }

    iCub->home();
}

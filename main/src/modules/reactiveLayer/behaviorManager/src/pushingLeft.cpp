#include "pushingLeft.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void PushingLeft::configure() {

    external_port_name = "/karmaMotor/rpc";

    Bottle targetGroup = (rf.findGroup("targetPos"));
    if (targetGroup.check("leftPos"))
    {
        target = targetGroup.find("leftPos").asDouble();
        yInfo(" [pushingLeft]: leftPos is %f",target);
    }
    else
    {
        target = -0.1;
        yWarning(" [pushingLeft]: no leftPos. Set to default: %f", target);
    }
}

void PushingLeft::run(Bottle args/*=Bottle()*/) {
    yInfo() << "PushingLeft::run";

    if (!args.isNull())
    {
        obj_type = args.get(0).asList()->toString();
        obj_name = args.get(1).asList()->toString();
    }

//    iCub->say("I will push the " + obj_name + "to the left");

    yInfo() << "received context from planner:" << obj_type.c_str() << "and" << obj_name.c_str();
    iCub->home();

    bool succeeded = iCub->pushKarmaLeft(obj_name,target);
    cout<<(succeeded?"success":"failed")<<endl;
    Time::delay(0.2);

//    if (succeeded) {
//        iCub->lookAtPartner();
//    } else {
//        iCub->say(" I missed");
//    }

    iCub->home();
}




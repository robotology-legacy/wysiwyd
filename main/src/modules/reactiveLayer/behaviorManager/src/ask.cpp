#include "ask.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void Ask::configure() {
    ;
}

void Ask::run(const Bottle &args) {
    // expects a bottle (objectName actionType)
    yInfo() << "Ask::run";

    string action = args.get(0).asList()->get(1).asString();
    string object = args.get(0).asList()->get(0).asString();
    string direction;
    if (action == "pull")
    {
        direction = "away from";
    }
    else if (action == "push")
    {
        direction = "closer to";
    }

    yDebug() << args.toString();

    iCub->lookAtPartner();

    string aux = "Can you please " + action + " the " + object + " " + direction + " me?";
    iCub->say(aux);
    yInfo() << "iCub has requested a " + action + " from the human.";

    iCub->home();
    Time::delay(3.0);
}


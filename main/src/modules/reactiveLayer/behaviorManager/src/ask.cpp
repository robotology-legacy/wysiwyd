#include "ask.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void Ask::configure() {
    ;
}

void Ask::run(Bottle args) {
    // expects a bottle (objectName actionType)
    yInfo() << "Ask::run";

    string aux;
    string action = args.get(0).asList()->get(1).asString();
    string object = args.get(0).asList()->get(0).asString();
    string direction = "";
    if (action == "pull")
    {
        direction = "away from";
    }
    else if (action == "push")
    {
        direction = "closer to";
    }

    yDebug() << args.toString();
    aux = "Can you please " + action + " the " + object + " " + direction + " me?";

    iCub->lookAtPartner();

    iCub->say(aux);
    yInfo() << "iCub has requested a " + action + " from the human.";

    Time::delay(2.0);
    iCub->home();
    Time::delay(3.5);

}


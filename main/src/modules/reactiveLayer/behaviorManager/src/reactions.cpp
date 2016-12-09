#include "reactions.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void Reactions::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    external_port_name = "None";
    from_sensation_port_name = "None";
}

void Reactions::run(const Bottle &args) {
    yInfo() << "Reactions::run";

    Object* touchLocation = dynamic_cast<Object*>(iCub->opc->getEntity("touchLocation"));
    iCub->opc->update(touchLocation);
    touchLocation->m_present = 1.0;
    iCub->opc->commit(touchLocation);

    iCub->say("Wow");
    iCub->look("touchLocation");
    iCub->say("Haaaahaha ... hahaaa");
    Time::delay(1.0);    

    yInfo() << "Reactions::run ends";

}

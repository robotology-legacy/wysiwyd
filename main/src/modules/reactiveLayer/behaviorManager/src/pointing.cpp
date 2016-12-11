#include "pointing.h"

using namespace std;
using namespace yarp::os;


void Pointing::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/opcSensation/known_entities:o";
}

void Pointing::run(const Bottle &args) {
    yInfo() << "Pointing::run";
    Bottle *sensation = sensation_port_in.read();

    string obj_name, sentence;
    bool no_objects = true;
    if (args.size()!=0){
        yDebug()<<args.toString() << args.size();
        obj_name = args.get(0).asList()->get(0).asString();
        yDebug() << "Object selected: " << obj_name;
        sentence = "Okay, this is the ";
        no_objects=false;
    }else{
        if(sensation->size()==0) {
            iCub->say("There are no objects I can point at.");
            return;
        }
        int id = yarp::os::Random::uniform(0, sensation->size() - 1);
        obj_name = sensation->get(id).asList()->get(1).asString();
        yDebug() << "Randomly selected: " << id << " " << obj_name;
        sentence = "I could point to the ";
    }
    
    iCub->opc->checkout();
    yDebug() << "[pointing]: opc checkout";
    iCub->lookAtPartner();
    Time::delay(0.25);
    
    iCub->say(sentence + obj_name);

    bool succeeded = iCub->point(obj_name);
    Time::delay(0.2);

    if (no_objects){
        if (succeeded) {
            iCub->lookAtPartner();
            iCub->say("Do you know that this is a " + obj_name, false);
        } else {
            iCub->say(" I couldn't find the " + obj_name);
        }
    }
    iCub->home();

}

#include "followingOrder.h"

void FollowingOrder::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    name = "followingOrder";
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/ears/target:o";

    port_to_narrate_name = "/behaviorManager/narrate:o";
    port_to_narrate.open(port_to_narrate_name);
}

void FollowingOrder::run(Bottle args/*=Bottle()*/) {
    yInfo() << "FollowingOrder::run";

    Bottle* sens = sensation_port_in.read();
    string action = sens->get(0).asString();
    string type;
    string target;
    if (sens->size()>0)
        type = sens->get(1).asString();
    if (sens->size()>1)
        target = sens->get(2).asString();
    yDebug() << action;
    yDebug() << type;
    yInfo() << target;

    if (target != "none"){
        yInfo() << "there are elements to search!!!";
        handleSearch(type, target);
    }

    if (action == "point" || action == "look at" || action == "push"){
        // Be careful: both handlePoint (point in response of a human order) and handlePointing (point what you know)
        if (sens->size()<2){
            yInfo()<<"I can't" << action << "if you don't tell me the object";
            iCub->say("I can't " + action + "if you don't tell me the object");
        } else{
            handleAction(type, target, action);
        }
    } else if (action == "narrate"){
        handleNarrate();
        yInfo() << "narrating!!!";
    }
}

bool FollowingOrder::handleNarrate(){
    string port_narrate = "/narrativeHandler/rpc";
    
    if (!yarp::os::Network::isConnected(port_to_narrate_name, port_narrate))
        yarp::os::Network::connect(port_to_narrate_name, port_narrate);

    yInfo() << "Narrate::run";
    Bottle cmd;
    Bottle rply;
    cmd.clear();
    cmd.addString("narrate");
    yInfo() << "Proactively narrating...";


    
    port_to_narrate.write(cmd, rply);
    return true;
}

bool FollowingOrder::handleAction(string type, string target, string action) {
    iCub->home();
    yarp::os::Time::delay(1.0);

    yInfo() << "[handleAction] type: " << type << "target:" << target << "action:" << action;
    iCub->opc->checkout();
    yInfo() << " [handleAction]: opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    for (auto& entity : lEntities)
    {
        if (entity->name() == target) {
            if (entity->entity_type() == "object")
            {
                Object* o = dynamic_cast<Object*>(entity);
                if(o && (o->m_present==1.0)) {
                    yInfo() << "I'd like to" << action << "the" << target;

                    if(action == "point") {
                        iCub->point(target);
                        iCub->say("oh! this is a " + target);
                    } else if(action == "look at") {
                        iCub->look(target);
                        iCub->say("oh! look at the " + target);
                    } else if(action == "push") {
                        iCub->push(target);
                        iCub->say("oh! look how I pushed the " + target);
                    }
                    yarp::os::Time::delay(1.0);
                    iCub->home();

                    return true;
                }
            }
        }
    }

    yWarning() << "Cannot" << action << "the" << target;
    iCub->lookAtAgent();
    iCub->say("I cannot " + action + " the " + target);
    iCub->home();
    return false;
}

bool FollowingOrder::handleSearch(string type, string target)
{
    // look if the object (from human order) exist and if not, trigger proactivetagging

    iCub->opc->checkout();
    yInfo() << " [handleSearch] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    for (auto& entity: lEntities)
    {
        string sName = entity->name();
        if (sName == target) {
            yDebug() << "Entity found: "<<target;
            if (entity->entity_type() == "object")
            {
                Object* o = dynamic_cast<Object*>(iCub->opc->getEntity(sName));
                yInfo() << "I found the entity in the opc: " << sName;
                if(o && o->m_present==1.0) {
                    return true;
                }
            }
        }
    }

    yInfo() << "I need to explore by name!";

    // ask for the object
    yInfo() << "send rpc to proactiveTagging";

    //If there is an unknown object (to see with agents and rtobjects), add it to the rpc_command bottle, and return true
    Bottle cmd;
    Bottle rply;

    cmd.addString("searchingEntity");
    cmd.addString(type);
    cmd.addString(target);
    rpc_out_port.write(cmd,rply);
    yDebug() << rply.toString();

    return true;

}

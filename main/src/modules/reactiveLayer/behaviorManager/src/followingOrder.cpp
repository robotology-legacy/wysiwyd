#include "followingOrder.h"

void FollowingOrder::configure() {

    Bottle bFollowingOrder = rf.findGroup("followingOrder");
    bKS1.clear();
    bKS2.clear();
    bKS1 = *bFollowingOrder.find("ks1").asList();
    bKS2 = *bFollowingOrder.find("ks2").asList();

    portToHomeo_name = "/"+name+"/toHomeo:o";
    homeoPort = "/homeostasis/rpc";

    babblingArm = bFollowingOrder.find("babblingArm").asString();

    // Todo: set the value beow from a config file (but we are not in a module here)
    name = "followingOrder";
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/ears/target:o";

    port_to_narrate_name = "/behaviorManager/narrate:o";
    port_to_narrate.open(port_to_narrate_name);
    port_to_homeo.open(portToHomeo_name);
}

void FollowingOrder::run(Bottle args/*=Bottle()*/) {

    yInfo() << "FollowingOrder::run";

    if (!Network::isConnected(portToHomeo_name,homeoPort)){
        if (!Network::connect(portToHomeo_name,homeoPort)){
            yWarning()<<"Port to Homeostasis not available. Could not freeze the drives...";
        }
    }
    if (Network::isConnected(portToHomeo_name,homeoPort)){
        yInfo()<<"freezing drives";
        Bottle cmd;
        Bottle rply;
        cmd.addString("freeze");
        cmd.addString("all");

        port_to_homeo.write(cmd, rply);
    }

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

    //TODOOOOOOOOO: no search for KS

    if ( target != "none" && type != "bodypart" && type != "kinematic structure" && type != "kinematic structure correspondence"){           //we dont have searchEntity for bodypart
        yInfo() << "there are objects to search!!!";
        handleSearch(type, target);
    }

    //FollowingOrder implying objects
    if ( (action == "point" || action == "look at" || action == "push") && type == "object"){
        // Be careful: both handlePoint (point in response of a human order) and handlePointing (point what you know)
        if (sens->size()<2){
            yInfo()<<"I can't" << action << "if you don't tell me the object";
            iCub->say("I can't " + action + "if you don't tell me the object");
        } else{
            handleAction(type, target, action);
        }
    } else if (action == "move" && type == "bodypart") { //FollowingOrder implying bodypart
        if (sens->size()<2){
            yInfo()<<"I can't" << action << "if you don't tell me the bodypart";
            iCub->say("I can't " + action + "if you don't tell me the bodypart");
        } else{
            handleActionBP(type, target, action);
        }
    } else if (action == "narrate"){
        handleNarrate();
        yInfo() << "narrating!!!";
    }  else if (action == "show" && (type == "kinematic structure" || type == "kinematic structure correspondence")){
        handleActionKS(action, type);
    }

    if (Network::isConnected(portToHomeo_name,homeoPort)){
        yInfo()<<"unfreezing drives";
        Bottle cmd;
        Bottle rply;
        cmd.addString("unfreeze");
        cmd.addString("all");
        port_to_homeo.write(cmd, rply);
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
    rply.clear();
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
    iCub->lookAtPartner();
    iCub->say("I don't think the " + target + " is here. I cannot " + action + " it");
    iCub->home();
    return false;
}

//Randomly go through a bottle with instance number (int>0) to pick one
int FollowingOrder::randKS(Bottle bKS) {
    int ks = -1;

    if(bKS.size() != 0){
        //randomly pick one of the KS1 + int protection
        int ksIndex = Random::uniform(0,(bKS.size()-1));
        if(bKS.get(ksIndex).isInt()){
            ks = bKS.get(ksIndex).asInt() ;
            return ks;
        } else {
            yError() << "[handleActionKS] wrong input for" << bKS.toString() << ": it should be an int, and not " << bKS.get(ksIndex).toString();
            return -1;
        }
    } else {
        yError() << "[handleActionKS]: no instance for " << bKS.toString() << "found!" ;
        return -1;
    }
}

bool FollowingOrder::handleActionKS(string action, string type) {
    int ks = -1;

    if(type == "kinematic structure") {
        ks = randKS(bKS1);         //extract list of KS1
    } else if(type=="kinematic structure correspondence") {
        ks = randKS(bKS2);
    } else {
        iCub->say("I do not know this type of kinematic structure");
        return false;
    }

    //if ks1 < 0 in case provide a negative instance...
    if(ks < 0){
        yError() << "[handleActionKS] wrong input for KS: it should be an instance > 0 and not " << ks;
        return false;
    }

    yInfo() << "[handleActionKS] type: " << type  << "action:" << action << "with instance ks = " << ks;
    iCub->lookAtPartner();
    iCub->say("I estimate my arm looks like this");
    iCub->getABMClient()->triggerStreaming(ks);
    iCub->home();

    return true;
}

bool FollowingOrder::handleActionBP(string type, string target, string action) {
    yInfo() << "[handleActionBP] type: " << type << "target:" << target << "action:" << action;
    iCub->opc->checkout();
    yInfo() << " [handleActionBP]: opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    for (auto& entity : lEntities)
    {
        if (entity->name() == target) {
            if (entity->entity_type() == type) //type has been checked to be "bodypart" before, in run()
            {
                Bodypart* BPentity = dynamic_cast<Bodypart*>(entity);
                iCub->say("Nice, I will do some exercise with my " + babblingArm + " arm");
                yInfo() << "I'd like to" << action << "the" << target;

                if(action == "move") { //only action available right now for bodypart but keep the check for future development
                    int joint = BPentity->m_joint_number;
                    //send rpc command to bodySchema to move the corresponding part
                    yInfo() << "Start bodySchema";
                    double babbling_duration = 4.0;
                    iCub->say("I am moving my " + target, false);
                    iCub->babbling(joint, babblingArm, babbling_duration);
                }

                yarp::os::Time::delay(1.0);
                iCub->home();

                return true;
            }
        }
    }

    yWarning() << "Cannot" << action << "my" << target;
    iCub->lookAtPartner();
    iCub->say("I cannot " + action + " my " + target + ", I do not know this word");
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

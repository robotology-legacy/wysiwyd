#include "followingOrder.h"

void FollowingOrder::configure() {
    Bottle bFollowingOrder = rf.findGroup("followingOrder");
    bKS1.clear();
    bKS2.clear();
    bKS1 = *bFollowingOrder.find("ks1").asList();
    bKS2 = *bFollowingOrder.find("ks2").asList();

    homeoPort = "/homeostasis/rpc";

    babblingArm = bFollowingOrder.find("babblingArm").asString();

    // Todo: set the value beow from a config file (but we are not in a module here)
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/ears/target:o";

    port_to_narrate_name = "/"+behaviorName+"/narrate:o";
    port_to_narrate.open(port_to_narrate_name);

    port_to_homeo_name = "/"+behaviorName+"/toHomeo:o";
    port_to_homeo.open(port_to_homeo_name);

    port_to_avoidance_name = "/"+behaviorName+"/avoidance:o";
    port_to_avoidance.open(port_to_avoidance_name);

    manual = false;
}

void FollowingOrder::run(Bottle args/*=Bottle()*/) {

    yInfo() << "FollowingOrder::run";

    if (!Network::isConnected(port_to_homeo_name,homeoPort)){
        if (!Network::connect(port_to_homeo_name,homeoPort)){
            yWarning()<<"Port to Homeostasis not available. Could not freeze the drives...";
        }
    }
    if (Network::isConnected(port_to_homeo_name,homeoPort)){
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

    if ( target != "none" && type != "bodypart" && type != "kinematic structure" && type != "kinematic structure correspondence"){           //we dont have searchEntity for bodypart
        bool verboseSearch=true;
        if( action == "this is" ) {
            verboseSearch=false;
        }
        yInfo() << "there are objects to search!!!";
        handleSearch(type, target, verboseSearch);
    }

    //FollowingOrder implying objects
    if ( (action == "point" || action == "look at" || action == "push" || action == "this is") && type == "object"){
        // Be careful: both handlePoint (point in response of a human order) and handlePointing (point what you know)
        if (sens->size()<2){
            iCub->say("I can't " + action + "if you don't tell me the object");
        } else{
            handleAction(type, target, action);
        }
    } else if (action == "move" && type == "bodypart") { //FollowingOrder implying bodypart
        if (sens->size()<2) {
            iCub->say("I can't " + action + "if you don't tell me the bodypart");
        } else {
            handleActionBP(type, target, action);
        }
    } else if (action == "narrate") {
        handleNarrate();
    }  else if (action == "show" && (type == "kinematic structure" || type == "kinematic structure correspondence")){
        handleActionKS(action, type);
    } else if (action == "end") {
        handleEnd();
    } else if (action == "game") {
        handleGame(type);
    } else {
        iCub->say("I don't know what you mean.");
    }

    if (!manual && Network::isConnected(port_to_homeo_name, homeoPort)){
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
    Bottle cmd, rply;
    cmd.addString("narrate");
    yInfo() << "Proactively narrating...";

    port_to_narrate.write(cmd, rply);
    return true;
}

bool FollowingOrder::handleAction(string type, string target, string action) {
    yInfo() << "[handleAction] type: " << type << "target:" << target << "action:" << action;
    if(action == "this is") {
        yDebug() << "[handleAction] For action \"" + action + "\" there is nothing to do here. Return.";
        return true;
    }

    iCub->opc->checkout();
    yInfo() << " [handleAction]: opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    for (auto& entity : lEntities)
    {
        if (entity->name() == target) {
            if (entity->entity_type() == "object")
            {
                Object* o = dynamic_cast<Object*>(entity);
                if(o && o->m_present==1.0) {
                    yInfo() << "I'd like to" << action << "the" << target;

                    if(action == "point") {
                        iCub->say("oh! this is a " + target + ".", false);
                        iCub->point(target);
                    } else if(action == "look at") {
                        iCub->say("oh! look at the " + target + ".", false);
                        iCub->look(target);
                        yarp::os::Time::delay(2.0);
                    } else if(action == "push") {
                        iCub->say("oh! look how I pushed the " + target + ".", false);
                        iCub->push(target);
                    } else {
                        yError() << "This action is not supported!";
                    }
                    yarp::os::Time::delay(0.1);
                    iCub->home();

                    return true;
                } else if (o && o->m_present==0.0) {
                    iCub->lookAtPartner();
                    iCub->say("I know the " + target + " but it is not here.");
                    iCub->home();
                    return true;
                }
            }
        }
    }

    yWarning() << "Cannot" << action << "the" << target;
    iCub->lookAtPartner();
    iCub->say("I don't know the " + target + " but I know it is not here. I will not " + action + " it.");
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
    iCub->opc->checkout();
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
    if(type == "kinematic structure") {
        iCub->say("Please look at the screen behind me. See how my arms look the same.");
    } else if(type == "kinematic structure correspondence") {
        iCub->say("I think our body parts are similar. I show you what I mean on the screen.");
    }
    iCub->getABMClient()->triggerStreaming(ks, true, true, 1.0, "icubSim", true);
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
                iCub->lookAtPartner();
                iCub->say("Nice, I will do some exercise with my " + babblingArm + " arm");

                if(action == "move") { //only action available right now for bodypart but keep the check for future development
                    int joint = BPentity->m_joint_number;
                    //send rpc command to bodySchema to move the corresponding part
                    yInfo() << "Start bodySchema";
                    double babbling_duration = 4.0;
                    iCub->say("I will move my " + target, false);
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

bool FollowingOrder::handleSearch(string type, string target, bool verboseSearch)
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
    cmd.addInt(verboseSearch);
    rpc_out_port.write(cmd,rply);
    yDebug() << rply.toString();

    return true;
}

bool FollowingOrder::handleEnd() {
    iCub->opc->checkout();
    yInfo() << "[handleEnd] time to sleep";
    iCub->lookAtPartner();
    iCub->say("Nice to play with you! See you soon.");
    iCub->home();

    yInfo()<<"[handleEnd] freezing drives";
    manual = true;
    Bottle cmd, rply;
    cmd.addString("freeze");
    cmd.addString("all");
    port_to_homeo.write(cmd, rply);

    return true;
}

bool FollowingOrder::handleGame(string type) {
    string port_avoidance = "/reactController/rpc:i";

    if (!yarp::os::Network::isConnected(port_to_avoidance_name, port_avoidance))
        yarp::os::Network::connect(port_to_avoidance_name, port_avoidance);

    if (!yarp::os::Network::isConnected(port_to_avoidance_name, port_avoidance)) {
        iCub->say("I cannot play this game right now.");
        return false;
    }

    iCub->opc->checkout();
    yInfo() << "[handleGame] type:" << type;
    iCub->lookAtPartner();

    Bottle avoidance_cmd, avoidance_reply;
    if(type == "start") {
        iCub->say("Nice. Let me draw a circle. I will avoid obstacles.");
        iCub->say("Please put the table away.");
        yarp::os::Time::delay(10);
        iCub->home();
        iCub->say("Okay I am ready");

        /*Bottle sub;
        sub.addDouble(-0.28);
        sub.addDouble(0.2);
        sub.addDouble(0.05);
        avoidance_cmd.addString("set_xd");
        avoidance_cmd.addList() = sub;
        yDebug() << "To avoidance:" << avoidance_cmd.toString();
        port_to_avoidance.write(avoidance_cmd, avoidance_reply);
        yDebug() << "Reply avoidance: " << avoidance_reply.toString();

        yarp::os::Time::delay(2.0);

        avoidance_cmd.addString("set_relative_circular_xd");
        avoidance_cmd.addDouble(0.1);
        avoidance_cmd.addDouble(0.15);
        yDebug() << "To avoidance:" << avoidance_cmd.toString();
        port_to_avoidance.write(avoidance_cmd, avoidance_reply);
        yDebug() << "Reply avoidance: " << avoidance_reply.toString();*/
    } else if (type == "end") {
        iCub->say("This was fun! Thanks for playing with me.");
        avoidance_cmd.addString("stop");
        yDebug() << "To avoidance:" << avoidance_cmd.toString();
        port_to_avoidance.write(avoidance_cmd, avoidance_reply);
        yDebug() << "Reply avoidance: " << avoidance_reply.toString();
    }

    yInfo()<<"[handleGame] freezing drives";
    manual = true;
    Bottle homeo_cmd, homeo_reply;
    if(type == "start") {
        homeo_cmd.addString("freeze");
    } else if(type == "end") {
        homeo_cmd.addString("unfreeze");
    }
    homeo_cmd.addString("all");
    port_to_homeo.write(homeo_cmd, homeo_reply);

    return true;
}

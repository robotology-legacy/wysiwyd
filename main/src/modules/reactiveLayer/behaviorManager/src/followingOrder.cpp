#include "followingOrder.h"

void FollowingOrder::configure() {

    Bottle bFollowingOrder = rf.findGroup("followingOrder");
    listKS1.clear();
    listKS2.clear();
    listKS1 = *bFollowingOrder.find("ks1").asList();
    listKS2 = *bFollowingOrder.find("ks2").asList();


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
    string type = "";
    string target = "";
    if (sens->size()>0)
        type = sens->get(1).asString();
    if (sens->size()>1)
        target = sens->get(2).asString();

    yDebug() << action;
    yDebug() << type;
    yInfo() << target;

    if (target != "none" && type != "bodypart"){           //we dont have searchEntity for bodypart
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
    }  else if (action == "show" && (type == "kinematic_structure" || type == "kinematic_structure_correspondence")){
        handleActionKS(type, action);
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
    iCub->lookAtPartner();
    iCub->say("I cannot " + action + " the " + target);
    iCub->home();
    return false;
}

bool FollowingOrder::handleActionKS(string type, string action) {

    iCub->home();
    yarp::os::Time::delay(1.0);

    //extract list of KS1
    if(listKS1.size() != 0){

        //int ks1Index = rand(0,listKS1.size()-1);


    } else {
        yError() << "[handleActionKS]: no instance for KS1 found!" ;
        return false;
    }

    //if not null: ok

    //if kinematic_structure_correspondence: extract list of KS2

        //if not null: ok

    //send rpc command to KS


    yInfo() << "[handleActionKS] type: " << type  << "action:" << action;
    iCub->opc->checkout();
    yInfo() << " [handleActionBP]: opc checkout";


    return false;
}

bool FollowingOrder::handleActionBP(string type, string target, string action) {

    iCub->home();
    yarp::os::Time::delay(1.0);

    //*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! This is a HACK! To be changed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    string babblingArm = "left" ;
    //Need to update the "part" in the OPC object to extract left/right arm.
    //*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! This is a HACK! To be changed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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

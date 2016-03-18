#include "followingOrder.h"

void FollowingOrder::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    name = "followingOrder";
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/ears/target:o";

    port_to_narrate_name = "/behaviorManager/narrate:o";
    port_to_narrate.open(port_to_narrate_name);

    finding=false;
    pointing=false;
}

void FollowingOrder::run(Bottle args/*=Bottle()*/) {
    yInfo() << "FollowingOrder::run";

    Bottle* sens = sensation_port_in.read();
    string action = sens->get(0).asString();
    string type;
    string target;
    if (sens->size()>0)
        type = sens->get(1).asString();
    else if (sens->size()>1)
        target = sens->get(2).asString();
    yDebug() << action;
    yDebug() << type;
    yInfo() << target;

    if (target != "none"){
        yInfo() << "there are elements to search!!!";//<<endl;
        finding = handleSearch(type, target);
    }

    if (action == "point"){
        // Be carfull: both handlePoint (point in response of a human order) and handlePointing (point what you know)
        if (sens->size()<2){
            yInfo()<<"I can't point if  you don't tell me the objects";
            iCub->say("I can't point if  you don't tell me the objects");
        }else{
            pointing = handlePoint(type, target);
            yInfo() << "pointing elements to point!!!";//<<endl;
            yDebug() << finding;// << endl;
        }
    }else if (action == "look at"){
        if (sens->size()<2){
            yInfo()<<"I can't look if  you don't tell me the objects";
            iCub->say("I can't look if  you don't tell me the objects");
        }else{
        handleLook(type, target);
        yInfo() << "looking elements to look at!!!";//<<endl;
        yDebug() << finding;// << endl;
    }
    }else if (action == "push"){
        if (sens->size()<2){
            yInfo()<<"I can't push if  you don't tell me the objects";
            iCub->say("I can't push if  you don't tell me the objects");
        }else{
        //handlePush(type, target); //Feature under development
        yInfo() << "pushing elements to look at!!!";//<<endl;
        yDebug() << finding;// << endl;
    }
    }else if (action == "narrate"){
        handleNarrate();
        yInfo() << "narrating!!!";//<<endl;
        yDebug() << finding;// << endl;
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

bool FollowingOrder::handlePoint(string type, string target)
{
    // Point an object (from human order). Independent of proactivetagging
    iCub->opc->checkout();
    yInfo() << " [handlePoint] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();
    string e_name = target;
    // point RPC useless
    //bool pointRPC = false;

    for (auto& entity : lEntities)
    {
        string sName = entity->name();

        yDebug() << "Checking entity: " << e_name << " to " << sName;//<<endl;
        if (sName == e_name) {
            if (entity->entity_type() == "object")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                yInfo() << "I already knew that the object was in the opc: " << sName;
                Object* o = dynamic_cast<Object*>(entity);
                if(o && o->m_present) {
                    //pointRPC=true;
                    yInfo() << "I'd like to point " << e_name;// <<endl;
                    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>(e_name);
                    string sHand = "right";
                    if (obj1->m_ego_position[1]<0) sHand = "left";
                    Bottle bHand(sHand);
                    //iCub->say("I'm going to point the " + target);
                    iCub->point(e_name, bHand);
                    iCub->say("oh! this is a " + e_name);
                    yarp::os::Time::delay(2.0);
                    iCub->home();
                    target = "none";//pointList.pop_back();
                    return true;
                }

            }
        }
    }
    return false;  
}

bool FollowingOrder::handleLook(string type, string target)
{
    // Point an object (from human order). Independent of proactivetagging
    iCub->opc->checkout();
    yInfo() << " [handleLook] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();
    string e_name = target;
    // point RPC useless
    //bool pointRPC = false;

    for (auto& entity : lEntities)
    {
        string sName = entity->name();

        yDebug() << "Checking entity: " << e_name << " to " << sName;//<<endl;
        if (sName == e_name) {
            if (entity->entity_type() == "object")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                yInfo() << "I already knew that the object was in the opc: " << sName;
                Object* o = dynamic_cast<Object*>(entity);
                if(o && o->m_present) {
                    //pointRPC=true;
                    yInfo() << "I'd like to look " << e_name;// <<endl;
                    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>(e_name);
                    //iCub->say("I'm going to look the " + target);
                    iCub->look(obj1->name());
                    iCub->say("oh! look at this!");
                    yarp::os::Time::delay(1.0);
                    //iCub->home();
                    target = "none";//pointList.pop_back();
                    return true;
                }

            }
        }
    }
    return false;  
}

/* //Feature to be added in a near future
bool FollowingOrder::handlePush(string type, string target)
{
    // Point an object (from human order). Independent of proactivetagging
    iCub->opc->checkout();
    yInfo() << " [handlePush] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();
    string e_name = target;
    // point RPC useless
    //bool pointRPC = false;

    for (auto& entity : lEntities)
    {
        string sName = entity->name();

        yDebug() << "Checking entity: " << e_name << " to " << sName;//<<endl;
        if (sName == e_name) {
            if (entity->entity_type() == "object")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                yInfo() << "I already knew that the object was in the opc: " << sName;
                Object* o = dynamic_cast<Object*>(entity);
                if(o && o->m_present) {
                    //pointRPC=true;
                    yInfo() << "I'd like to push " << e_name;// <<endl;
                    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>(e_name);
                    string sHand = "right";
                    if (obj1->m_ego_position[1]<0) sHand = "left";
                    Bottle bHand(sHand);
                    //iCub->say("I'm going to push the " + target);
                    iCub->push(e_name, bHand);
                    iCub->say("oh! look how I push the " + e_name);
                    yarp::os::Time::delay(2.0);
                    iCub->home();
                    target = "none";//pointList.pop_back();
                    return true;
                }

            }
        }
    }
    return false;  
}
*/
bool FollowingOrder::handleSearch(string type, string target)
{
    // look if the object (from human order) exist and if not, trigger proactivetagging

    iCub->opc->checkout();
    yInfo() << " [handleSearch] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();
    bool tagRPC = false;

    string e_name = target;

    for (auto& entity: lEntities)
    {
        string sName = entity->name();
        if (sName == e_name) {
            yDebug() << "Entity found: "<<e_name;//<<endl;
            if (entity->entity_type() == "object")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                Object* o = dynamic_cast<Object*>(iCub->opc->getEntity(sName));
                yInfo() << "I found the entity in the opc: " << sName;
                if(o && o->m_present==1.0) {
                    //searchList.pop_back();
                    // return, so "if(tagRPC)" ... is never executed
                    return true;
                }
            } else {
                tagRPC = true;
            }
        }
    }

    yInfo() << "I need to explore by name!";// << endl;

    // ask for the object
    yInfo() << "send rpc to proactiveTagging";//<<endl;

    //If there is an unknown object (to see with agents and rtobjects), add it to the rpc_command bottle, and return true
    Bottle cmd;
    Bottle rply;

    cmd.addString("searchingEntity");
    cmd.addString(type);
    cmd.addString(e_name);
    rpc_out_port.write(cmd,rply);
    yDebug() << rply.toString(); //<< endl;

    //searchList.pop_back();
    return true;
      
    //if no unknown object was found, return false
    //return false;
}

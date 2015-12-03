#include "pointingOrder.h"

void PointingOrder::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    name = "pointingOrder";
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/ears/target:o";
    
    finding=false;
    pointing=false;    


}

void PointingOrder::run(Bottle args/*=Bottle()*/) {
    yInfo() << "PointingOrder::run";


    string target = sensation_port_in.read()->get(0).asString();
    yInfo() << target ;
    if (target != "none"){
        yInfo() << "there are elements to search!!!";//<<endl;
        finding = handleSearch(target);
        // Be carfull: both handlePoint (point in response of a human order) and handlePointing (point what you know)
        pointing = handlePoint(target);
        yInfo() << "there are elements to point!!!";//<<endl;
        yDebug() << finding;// << endl;
    }
}

bool PointingOrder::handlePoint(string target)
{
    iCub->say("I will point the " + target);
    // Point an object (from human order). Independent of proactivetagging
    iCub->opc->checkout();
    yInfo() << " [handlePoint] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();
    string e_name = target;
    // point RPC useless
    //bool pointRPC = false;


    for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
    {
        string sName = (*itEnt)->name();
        

        yDebug() << "Checking entity: " << e_name << " to " << sName;//<<endl;
        if (sName == e_name) {
            if ((*itEnt)->entity_type() == "object")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                yInfo() << "I already knew that the object was in the opc: " << sName;
                Object* o = dynamic_cast<Object*>(*itEnt);
                if(o && o->m_present) {
                    //pointRPC=true;
                    yInfo() << "I'd like to point " << e_name;// <<endl;
                    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>(e_name);
                    string sHand = "right";
                    if (obj1->m_ego_position[1]<0) sHand = "left";
                    Bottle bHand(sHand);
                    iCub->say("I'm going to point the " + target);
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


bool PointingOrder::handleSearch(string target)
{
    // look if the object (from human order) exist and if not, trigger proactivetagging

    iCub->opc->checkout();
    yInfo() << " [handleSearch] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();
    bool tagRPC = false;

    string e_name = target;

    for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
    {
        string sName = (*itEnt)->name();
        

        if (sName == e_name) {
            yDebug() << "Entity found: "<<e_name;//<<endl;
            if ((*itEnt)->entity_type() == "object")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                yInfo() << "I found the entity in the opc: " << sName;
                Object* o = dynamic_cast<Object*>(*itEnt);
                yInfo() << "I found the entity in the opc: " << sName;
                if(o && o->m_present) {
                    //searchList.pop_back();
                    // return, so "if(tagRPC)" ... is never executed
                    return true;
                }
            }else{
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
    rply.clear();

    cmd.clear();
    cmd.addString("searchingEntity");
    cmd.addString(e_name);
    rpc_out_port.write(cmd,rply);
    yDebug() << rply.toString(); //<< endl;

    //searchList.pop_back();
    return true;
      
    //if no unknown object was found, return false
    //return false;
}
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

    Bottle* sens = sensation_port_in.read();
    string type = sens->get(0).asString();
    string target = sens->get(1).asString();

    yDebug() << type;
    yInfo() << target;

    if (target != "none"){
        yInfo() << "there are elements to search!!!";
        finding = handleSearch(type, target);
        // Be carfull: both handlePoint (point in response of a human order) and handlePointing (point what you know)
        pointing = handlePoint(type, target);
        yInfo() << "there are elements to point!!!";
        yDebug() << finding;
    }
}

bool PointingOrder::handlePoint(string type, string target)
{
    // Point an object (from human order). Independent of proactivetagging
    iCub->opc->checkout();
    yInfo() << " [handlePoint] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();
    
    for (auto& entity : lEntities)
    {
        string sName = entity->name();

        yDebug() << "Checking entity: " << target << " to " << sName;//<<endl;
        if (sName == target) {
            if (entity->entity_type() == "object")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                yInfo() << "I already knew that the object was in the opc: " << sName;
                Object* o = dynamic_cast<Object*>(entity);
                if(o && (o->m_present==1.0)) {
                    yInfo() << "I'd like to point " << target;// <<endl;

                    iCub->point(target);
                    iCub->say("oh! this is a " + target);
                    yarp::os::Time::delay(2.0);
                    iCub->home();

                    return true;
                }
            }
        }
    }

    iCub->lookAtAgent();
    iCub->say("I cannot point to the " + target);
    iCub->home();
    return false;
}


bool PointingOrder::handleSearch(string type, string target)
{
    // look if the object (from human order) exist and if not, trigger proactivetagging

    iCub->opc->checkout();
    yInfo() << " [handleSearch] : opc checkout";
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    string e_name = target;

    for (auto& entity: lEntities)
    {
        string sName = entity->name();
        if (sName == e_name) {
            yDebug() << "Entity found: "<<e_name;
            if (entity->entity_type() == "object")
            {
                Object* o = dynamic_cast<Object*>(entity);
                yInfo() << "I found the entity in the opc: " << sName;
                if(o && (o->m_present==1.0)) {
                    return true;
                }
            }
        }
    }

    yInfo() << "I need to explore by name!";

    // ask for the object
    yInfo() << "send rpc to proactiveTagging";//<<endl;

    //If there is an unknown object (to see with agents and rtobjects), add it to the rpc_command bottle, and return true
    Bottle cmd;
    Bottle rply;

    cmd.addString("searchingEntity");
    cmd.addString(type);
    cmd.addString(e_name);
    rpc_out_port.write(cmd,rply);
    yDebug() << rply.toString(); 

    return true;
}

#include "wrdac/subsystems/subSystem_ABM.h"
#include "tagging.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;


void Tagging::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/opcSensation/unknown_entities:o";

    Bottle bFollowingOrder = rf.findGroup("followingOrder");
    bKS1.clear();
    bKS2.clear();
    bKS1 = *bFollowingOrder.find("ks1").asList();
    bKS2 = *bFollowingOrder.find("ks2").asList();
}

void Tagging::run(const Bottle &args) {
    yInfo() << "Tagging::run";
    yDebug() << "send rpc to proactiveTagging";
    string type, target;
    //bool no_objects = true;
    Bottle cmd;
    Bottle rply;
    if (args.size()!=0){
        if(args.size()<=2){
            yDebug()<<args.toString() << args.size();
            target = args.get(0).asList()->get(0).asString();
            type = args.get(0).asList()->get(1).asString();
            yDebug() << "Object selected: " << target << "Type: "<<type;
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
                            return;
                        }
                    }
                }
            }

            yInfo() << "I need to explore by name!";

            // ask for the object
            yInfo() << "send rpc to proactiveTagging";

            //If there is an unknown object (to see with agents and rtobjects), add it to the rpc_command bottle, and return true


            cmd.addString("searchingEntity");
            cmd.addString(type);
            cmd.addString(target);
            rpc_out_port.write(cmd,rply);
            yDebug() << rply.toString();
        } else {
            yDebug()<<"ERRORRRR!!!!! Input not valid!!";
        }
    } else {
        Bottle *sensation = sensation_port_in.read();
        int id = yarp::os::Random::uniform(0, sensation->size()-1);

        // If there are unknown agents, prioritise tagging it.
        for (int i = 0; i < sensation->size(); i++) {
            string type = sensation->get(i).asList()->get(0).asString();
            string target = sensation->get(i).asList()->get(1).asString();

            if ((type == "agent") && (target == "partner"))
            {
                id = i;
                break;
            }
        }
        type=sensation->get(id).asList()->get(0).asString();
        target=sensation->get(id).asList()->get(1).asString();
        yDebug() << "Object selected: " << target << "Type: "<<type;
        //If there is an unknown object (to see with agents and rtobjects), add it to the rpc_command bottle, and return true
        
        cmd.clear();
        cmd.addString("exploreUnknownEntity");
        cmd.addString(type);
        cmd.addString(target);
        yInfo() << "Proactively tagging:" << cmd.toString();
    }
    
    rpc_out_port.write(cmd, rply);

    if(type=="bodypart" && target.find("unknown_self") == 0) {
        int ks = FollowingOrder::randKS(bKS2);
        if(ks<0) {
            yError() << "No kinematic structure instances defined in config file";
        } else {
            iCub->lookAtPartner();
            iCub->say("Actually, I found similar parts of your body. Let me show you on the screen.");
            iCub->getABMClient()->triggerStreaming(ks, true, true, 0.5, "icubSim", true);
        }
        iCub->opc->checkout();
        yarp::sig::Vector lHandVec = iCub->getPartnerBodypartLoc(EFAA_OPC_BODY_PART_TYPE_HAND_L);
        if(lHandVec.size()==0) {
            iCub->say("Although I know our hands look the same I cannot point at your hand because I cannot see it right now.");
        } else {
            iCub->say("Look, because our hands look the same I know this is your hand.");
            iCub->pointfar(lHandVec);
        }
    }

    yInfo() << "Proactive tagging ends";

}

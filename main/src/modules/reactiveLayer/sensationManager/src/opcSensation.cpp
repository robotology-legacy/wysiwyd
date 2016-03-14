#include <algorithm>    // std::random_shuffle
#include "opcSensation.h"

void OpcSensation::configure()
{
    moduleName = "opcSensation";
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName,"sensation","client.ini",isRFVerbose);
    iCub->opc->isVerbose = false;
    char rep = 'n';
    while (rep!='y'&&!iCub->connect())
    {
        cout<<"iCubClient : Some dependencies are not running..."<<endl;
        //cout<<"Continue? y,n"<<endl;
        //cin>>rep;
        break; //to debug
        Time::delay(1.0);
    }

    opc_has_unknown_port_name = "/" + moduleName + "/opc_has_unknown:o"; //This goes to homeostasis
    opc_has_unknown_port.open(opc_has_unknown_port_name);

    unknown_entities_port_name = "/" + moduleName + "/unknown_entities:o"; //this goes to behaviors
    unknown_entities_port.open(unknown_entities_port_name);
 
    opc_has_known_port_name = "/" + moduleName + "/opc_has_known:o"; //This goes to homeostasis
    opc_has_known_port.open(opc_has_known_port_name);

    opc_has_agent_name = "/" + moduleName + "/hasAgent:o"; //This goes to homeostasis
    opc_has_agent_port.open(opc_has_agent_name);

    known_entities_port_name = "/" + moduleName + "/known_entities:o"; //this goes to behaviors
    known_entities_port.open(known_entities_port_name);

    is_touched_port_name = "/" + moduleName + "/is_touched:o";
    is_touched_port.open(is_touched_port_name);

    // touch_location_port_name = "/" + moduleName + "/touch_location:o";
    // touch_location_port.open(touch_location_port_name);

    // friendly_port_name = "/" + moduleName + "/friendly:o"; //for homeostatic drive
    // friendly_port.open(friendly_port_name);

    // greeting_port_name = "/" + moduleName + "/greeting:o"; //for behaviour
    // greeting_port.open(greeting_port_name);

    // show_port_name = "/" + moduleName + "/show:o"; //This goes to homeostasis
    // show_port.open(show_port_name);

    // known_obj_port_name = "/" + moduleName + "/known_obj:o"; //this goes to behaviors
    // known_obj_port.open(known_obj_port_name);

    // Rand::init();

    cout<<"Configuration done."<<endl;

}

void OpcSensation::publish()
{
    // should change handleTagging to handleUnknownEntities?
    Bottle res = handleEntities();
    
    yarp::os::Bottle &has_unkn = opc_has_unknown_port.prepare();
    has_unkn.clear();
    has_unkn.addInt(int(res.get(0).asInt()));
    opc_has_unknown_port.write();
    
    yarp::os::Bottle &unkn = unknown_entities_port.prepare();
    unkn.clear();
    unkn.append(*res.get(1).asList());
    unknown_entities_port.write();
  
    yarp::os::Bottle &has_kn = opc_has_known_port.prepare();
    has_kn.clear();
    has_kn.addInt(int(res.get(2).asInt()));
    opc_has_known_port.write();
    
    yarp::os::Bottle &kn = known_entities_port.prepare();
    kn.clear();
    kn.append(*res.get(3).asList());
    known_entities_port.write();

    yarp::os::Bottle &has_ag = opc_has_agent_port.prepare();
    has_ag.clear();
    has_ag.addInt(int(res.get(4).asInt()));
    opc_has_agent_port.write();

    handleTouch();

    // yarp::os::Bottle &frie = friendly_port.prepare();
    // frie.clear();
    // frie.addInt(int(res.get(0).asInt()));
    // friendly_port.write();

    // yarp::os::Bottle &greet = greeting_port.prepare();
    // greet.clear();
    // greet.append(*res.get(1).asList());
    // greeting_port.write();
    
    // Actually, this hanfle pointing should be included into the handleTagging 
    // for iterating over the OPC only onece and extracting whatever is needed


    // Bottle res2 = handlePointing();

    // yarp::os::Bottle &show = show_port.prepare();
    // show.clear();
    // show.addInt(int(res2.get(0).asInt()));
    // show_port.write();
    
    // yarp::os::Bottle &kn = known_obj_port.prepare();
    // kn.clear();
    // kn.append(*res2.get(1).asList());
    // known_obj_port.write();
    
}

void OpcSensation::handleTouch()
{
    list<Relation> tactileRelations = iCub->opc->getRelationsMatching("icub","is","any","touchLocation");
    Bottle& out = is_touched_port.prepare();
    out.clear();
    out.addInt(int(tactileRelations.size() > 0));  // is touched
    is_touched_port.write();

    // if (tactileRelations.size() > 0) {
    //     out = touch_location_port.prepare()
    //     out.clear();
    //     out.
    // }

}

Bottle OpcSensation::handleEntities()
{
    iCub->opc->checkout();
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    bool unknown_obj = false;
    bool known_obj = false;
    bool agentPresent = false;
    Bottle u_entities, k_entities;
    // Bottle u_partner;
    Bottle ob;
    Bottle partners;
    Bottle body_parts;
    Bottle known_entity;

    for (auto& entity : lEntities)
    {
        string sName = entity->name();
        string sNameCut = sName;
        string original_name = sName;
        string delimiter = "_";
        size_t pos = 0;
        string token;
        if ((pos = sName.find(delimiter)) != string::npos) {
            token = sName.substr(0, pos);
            sName.erase(0, pos + delimiter.length());
            sNameCut = token;
        }
        // check if label is known

        if (sNameCut == "unknown") {
            if (entity->entity_type() == "object")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                // yInfo() << "I found an unknown entity: " << sName;
                Object* o = dynamic_cast<Object*>(entity);
                if(o && o->m_present) {
                    unknown_obj = true;
                    //Output is a list of objects entity + objects name [the arguments for tagging]
                    // o->m_saliency = unknownObjectSaliency;
                    ob.clear();
                    ob.addString(entity->entity_type());
                    ob.addString(original_name);
                    u_entities.addList()=ob;
                    //Could also send saliency
                }
            } 
            else if(entity->entity_type() == "bodypart") {
                    unknown_obj = true;
                    //Output is a list of objects entity + objects name [the arguments for tagging]
                    // o->m_saliency = unknownObjectSaliency;
                    body_parts.clear();
                    body_parts.addString(entity->entity_type());
                    body_parts.addString(original_name);
                    u_entities.addList()=body_parts;
            }
        } 

        else if (sNameCut == "partner" && entity->entity_type() == "agent") {
            yInfo() << "I found an unknown partner: " << sName;
            Agent* a = dynamic_cast<Agent*>(entity);
            if(a && a->m_present) {
                unknown_obj = true;
                //Output is a list of objects entity + objects name [the arguments for tagging]
                // o->m_saliency = unknownObjectSaliency;
                partners.clear();
                partners.addString(entity->entity_type());
                partners.addString(original_name);
                u_entities.addList()=partners;
                //Could also send saliency
            }
        } 
        else {
            if (entity->entity_type() == "bodypart" && (dynamic_cast<Bodypart*>(entity)->m_tactile_number == -1 || dynamic_cast<Bodypart*>(entity)->m_kinStruct_instance == -1))
            {
                unknown_obj = true;
            //Output is a list of objects entity + objects name [the arguments for tagging]
            // o->m_saliency = unknownObjectSaliency;
                body_parts.clear();
                body_parts.addString(entity->entity_type());
                body_parts.addString(original_name);
                u_entities.addList()=body_parts;
            }
            else if (entity->entity_type() == "object") {  // Known entities
                known_obj = true;
                known_entity.clear();
                known_entity.addString(entity->entity_type());
                known_entity.addString(original_name);
                k_entities.addList()=known_entity;           
            }
        }
        if (entity->entity_type() == "agent") {  // Known entities
            agentPresent = true;
                    
        }
    }
    //if no unknown object was found, return false

    yDebug() << "B = " + u_entities.toString();

    Bottle out;
    out.addInt(int(unknown_obj));
    out.addList()=u_entities;
    out.addInt(int(known_obj));
    out.addList()=k_entities;
    out.addInt(int(agentPresent));
    // out.addList()=u_partner;
    // cout << out.toString()<<endl;
    return out;
}


// Bottle OpcSensation::handlePointing()
// {
//     // Actually, this hanfle pointing should be included into the handleTagging 
//     // for iterating over the OPC only onece and extracting whatever is needed
//     iCub->opc->checkout();
//     list<Entity*> lEntities = iCub->opc->EntitiesCache();
//     int known_object = 0;
//     //int counter = 0;
//     Bottle k_objects;
//     Bottle ob;
//     for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
//     {
//         string sName = (*itEnt)->name();
//         string sNameCut = sName;
//         string original_name = sName;
//         string delimiter = "_";
//         size_t pos = 0;
//         string token;
//         if ((pos = sName.find(delimiter)) != string::npos) {
//             token = sName.substr(0, pos);
//             sName.erase(0, pos + delimiter.length());
//             sNameCut = token;
//         }
//         // check is label is known

//         if (sNameCut != "unknown") {

//             if ((*itEnt)->entity_type() == "object" )//|| (*itEnt)->entity_type() == "bodypart")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
//             {
//                 known_object = true;
//                 ob.clear();
//                 ob.addString((*itEnt)->entity_type());
//                 ob.addString(original_name);
//                 k_objects.addList()=ob;
                
//             }
//         }
//     }
//     //if no unknown object was found, return false
//     Bottle out;
//     out.addInt(int(known_object));
//     out.addList()=k_objects;
//     return out;
// }
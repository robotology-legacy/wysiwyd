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

    confusion_port_name = "/" + moduleName + "/confusion:o"; //This goes to homeostasis
    confusion_port.open(confusion_port_name);

    unknown_entities_port_name = "/" + moduleName + "/unknown_entities:o"; //this goes to behaviors
    unknown_entities_port.open(unknown_entities_port_name);

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
    Bottle res = handleUnknownEntities();
    
    yarp::os::Bottle &confus = confusion_port.prepare();
    confus.clear();
    confus.addInt(int(res.get(0).asInt()));
    confusion_port.write();
    
    yarp::os::Bottle &unkn = unknown_entities_port.prepare();
    unkn.clear();
    unkn.append(*res.get(1).asList());
    unknown_entities_port.write();

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

Bottle OpcSensation::handleUnknownEntities()
{
    iCub->opc->checkout();
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    bool unknown_obj = false;
    Bottle u_entities;
    // Bottle u_partner;
    Bottle ob;
    Bottle partners;
    Bottle body_parts;

    for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
    {
        string sName = (*itEnt)->name();
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
            if ((*itEnt)->entity_type() == "object")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                // yInfo() << "I found an unknown entity: " << sName;
                Object* o = dynamic_cast<Object*>(*itEnt);
                if(o && o->m_present) {
                    unknown_obj = true;
                    //Output is a list of objects entity + objects name [the arguments for tagging]
                    // o->m_saliency = unknownObjectSaliency;
                    ob.clear();
                    ob.addString((*itEnt)->entity_type());
                    ob.addString(original_name);
                    u_entities.addList()=ob;
                    //Could also send saliency
                }
            } 
            else if((*itEnt)->entity_type() == "bodypart") {
                    unknown_obj = true;
                    //Output is a list of objects entity + objects name [the arguments for tagging]
                    // o->m_saliency = unknownObjectSaliency;
                    body_parts.clear();
                    body_parts.addString((*itEnt)->entity_type());
                    body_parts.addString(original_name);
                    u_entities.addList()=body_parts;
            }
        } 

        else if (sNameCut == "partner" && (*itEnt)->entity_type() == "agent") {
            yInfo() << "I found an unknown partner: " << sName;
            Agent* a = dynamic_cast<Agent*>(*itEnt);
            if(a && a->m_present) {
                unknown_obj = true;
                //Output is a list of objects entity + objects name [the arguments for tagging]
                // o->m_saliency = unknownObjectSaliency;
                partners.clear();
                partners.addString((*itEnt)->entity_type());
                partners.addString(original_name);
                u_entities.addList()=partners;
                //Could also send saliency
            }
        } 
        else {
            if ((*itEnt)->entity_type() == "bodypart" && (dynamic_cast<Bodypart*>(*itEnt)->m_tactile_number == -1 || dynamic_cast<Bodypart*>(*itEnt)->m_kinStruct_instance == -1))
                {
                    unknown_obj = true;
                    //Output is a list of objects entity + objects name [the arguments for tagging]
                    // o->m_saliency = unknownObjectSaliency;
                    body_parts.clear();
                    body_parts.addString((*itEnt)->entity_type());
                    body_parts.addString(original_name);
                    u_entities.addList()=body_parts;                    
                }//unknown_obj = true;
        }
    }
    //if no unknown object was found, return false

    yDebug() << "B = " + u_entities.toString();

    Bottle out;
    out.addInt(int(unknown_obj));
    out.addList()=u_entities;
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



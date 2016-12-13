#include <algorithm>    // std::random_shuffle
#include "opcSensation.h"

void OpcSensation::configure()
{
    moduleName = "opcSensation";
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName,"sensation","client.ini",isRFVerbose);
    iCub->opc->isVerbose = false;
    while (!iCub->connect())
    {
        cout<<"iCubClient : Some dependencies are not running..."<<endl;
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

    u_entities.clear();
    k_entities.clear();
    pf3dTrackerPort.open("/"+moduleName+"/pf3dTracker:i");

    outputPPSPort.open("/"+moduleName+"/objects:o");

    yInfo() << "Configuration done.";

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
}

void OpcSensation::handleTouch()
{
    list<Relation> tactileRelations = iCub->opc->getRelationsMatching("icub","is","any","touchLocation");
    Bottle& out = is_touched_port.prepare();
    out.clear();
    out.addInt(int(tactileRelations.size() > 0));  // is touched
    is_touched_port.write();
}

void OpcSensation::addToEntityList(Bottle& list, string type, string name) {
    Bottle item;
    item.addString(type);
    item.addString(name);
    list.addList() = item;
}

Bottle OpcSensation::handleEntities()
{
    iCub->opc->checkout();
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    bool agentPresent = false;
    Bottle temp_u_entities, temp_k_entities, temp_up_entities, temp_kp_entities, temp_p_entities, temp_o_positions;
    Bottle objects;

    for (auto& entity : lEntities)
    {
        if(entity->entity_type() == "object") {
            Object* o = dynamic_cast<Object*>(entity);
                if(o) {
                    addToEntityList(temp_o_positions, o->objectAreaAsString(), entity->name());
                }
        }

        if (entity->name().find("unknown") == 0) {
            if (entity->entity_type() == "object")
            {
                Object* o = dynamic_cast<Object*>(entity);
                
                if(o && (o->m_present==1.0)) {
                    addToEntityList(temp_up_entities, entity->entity_type(), entity->name());
                }
                addToEntityList(temp_u_entities, entity->entity_type(), entity->name());
            }
            else if(entity->entity_type() == "bodypart") {
                addToEntityList(temp_u_entities, entity->entity_type(), entity->name());
                addToEntityList(temp_up_entities, entity->entity_type(), entity->name());
            }
        }
        else if (entity->name() == "partner" && entity->entity_type() == "agent") {
            Agent* a = dynamic_cast<Agent*>(entity);
            if(a && (a->m_present==1.0)) {
                addToEntityList(temp_up_entities, entity->entity_type(), entity->name());
            }
            addToEntityList(temp_u_entities, entity->entity_type(), entity->name());
        }
        else {
            if (entity->entity_type() == "bodypart" && (dynamic_cast<Bodypart*>(entity)->m_tactile_number == -1))
            {
                addToEntityList(temp_u_entities, entity->entity_type(), entity->name());
                addToEntityList(temp_up_entities, entity->entity_type(), entity->name());
            }
            else if (entity->entity_type() == "object"){
                Object* obj1 = dynamic_cast<Object*>(entity);

                //Handle red balls
                if (obj1 && entity->name() == "red_ball"){
                    // Set color:
                    obj1->m_color[0] = 250;
                    obj1->m_color[1] = 0;
                    obj1->m_color[2] = 0;
                    if (!Network::isConnected("/pf3dTracker/data:o",("/"+moduleName+"/pf3dTracker:i").c_str())){
                        yDebug("RedBall Port not connected...");
                        obj1->m_present = 0.0;
                    }else{
                        //update position
                        Bottle *bot = pf3dTrackerPort.read();
                        obj1->m_ego_position[0] = bot->get(0).asDouble();
                        obj1->m_ego_position[1] = bot->get(1).asDouble();
                        obj1->m_ego_position[2] = bot->get(2).asDouble();
                        obj1->m_dimensions[0] = bot->get(3).asDouble();
                        obj1->m_dimensions[1] = bot->get(4).asDouble();
                        obj1->m_dimensions[2] = bot->get(5).asDouble();
                        obj1->m_present = bot->get(6).asDouble();
                        
                    }
                }
                if (obj1 && obj1->m_present == 1.0){
                    //send data to PPS
                    Bottle objec;
                    objec.clear();
                    objec.addDouble(obj1->m_ego_position[0]);          //X
                    objec.addDouble(obj1->m_ego_position[1]);          //Y
                    objec.addDouble(obj1->m_ego_position[2]);          //Z
                    double dimensions = 0.07;//sqrt(pow(obj1->m_dimensions[0],2) + pow(obj1->m_dimensions[1],2) + pow(obj1->m_dimensions[2],2));
                    objec.addDouble(dimensions);                       //RADIUS
                    objec.addDouble(min(obj1->m_value,0.0)*(-1.0));    //Threat: Only negative part of value!
                    objects.addList()=objec;
                    addToEntityList(temp_kp_entities, entity->entity_type(), entity->name());
                }
                addToEntityList(temp_k_entities, entity->entity_type(), entity->name());
                iCub->opc->commit(obj1);
            }
            if (entity->entity_type() == "agent") {  // Known entities
                if (dynamic_cast<Agent*>(entity)->m_present == 1.0)
                {
                    agentPresent = true;
                }
            }
        }
        if (dynamic_cast<Object*>(entity) && dynamic_cast<Object*>(entity)->m_present == 1.0)
        {
            addToEntityList(temp_p_entities, entity->entity_type(), entity->name());
        }
        //Add agent right hand to interfere with robot left hand
        if (entity->name() == iCub->getPartnerName(false) && entity->entity_type() == "agent") {
            Agent* a = dynamic_cast<Agent*>(entity);
            if(a && (a->m_present==1.0)) {
                Bottle right_hand;
                right_hand.addDouble(a->m_body.m_parts["handRight"][0]);          //X
                right_hand.addDouble(a->m_body.m_parts["handRight"][1]);          //Y
                right_hand.addDouble(a->m_body.m_parts["handRight"][2]);          //Z
                double dimensions = 0.07;//sqrt(pow(obj1->m_dimensions[0],2) + pow(obj1->m_dimensions[1],2) + pow(obj1->m_dimensions[2],2));
                right_hand.addDouble(dimensions);                       //RADIUS
                right_hand.addDouble(-0.5);    //Currently hardcoded threat. Make adaptive
                objects.addList()=right_hand;
            }
        }
    }

    u_entities.copy( temp_u_entities);
    k_entities.copy( temp_k_entities);
    p_entities.copy( temp_p_entities);
    up_entities.copy( temp_up_entities);
    kp_entities.copy( temp_kp_entities);
    o_positions.copy( temp_o_positions);

    Bottle& output=outputPPSPort.prepare();
    output.clear();
    output.addList()=objects;
    outputPPSPort.write();
    Bottle out;
    out.addInt(int(up_entities.size()!=0));
    out.addList()=up_entities;
    out.addInt(int(kp_entities.size()!=0));
    out.addList()=kp_entities;
    out.addInt(int(agentPresent));

    return out;
}

int OpcSensation::get_property(string name,string property)
{
    Bottle b;
    bool check_position=false;

    if (property == "known")
    {
        b = k_entities;
    }
    else if (property == "unknown")
    {
        b = u_entities;
    }
    else if (property == "present")
    {
        b = p_entities;
    }
    else 
    {
        b = o_positions;
        check_position=true;
    }
    if (check_position)
    {
        yDebug()<<"Checking object position"<<name<<property;
        yDebug()<<"b:"<<b.toString();
        for (int i=0;i<b.size();i++)
        {
            if (name == "any"){
                if (b.get(i).asList()->get(0).asString()==property)
                {
                    yDebug() << "check_position, name==any, return 1";
                    return 1;
                }
            }else{
                if (b.get(i).asList()->get(1).asString()==name && b.get(i).asList()->get(0).asString()==property)
                {
                    yDebug() << "check_position, name!=any, return 1 " << b.toString();
                    return 1;
                }
            }
        }
        yDebug() << "check_position, return 0";
        return 0;
    }else{
        if (name == "any"){
            if (b.size()!=0){
                yDebug() << "not check_position, name==any, return 1";
                return 1;
            }else{
                yDebug() << "not check_position, name==any, return 0";
                return 0;
            }
        }else{
            for (int i=0;i<b.size();i++)
            {
                if (b.get(i).asList()->get(1).asString()==name)
                {
                    yDebug() << "not check_position, name!=any, return 1";
                    return 1;
                }
            }
            yDebug() << "not check_position, name!=any, return 0";
            return 0;
        }
    }
}


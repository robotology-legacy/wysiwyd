#include "opcPopulater.h"



bool opcPopulater::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("opcPopulater")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "opcPopulater", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= false;
    if (!iCub->connect())
    {
        yInfo() << " iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }
    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    //double seed = (double)time(NULL);
    //int count = 0;
    //while (seed == time(NULL)){ ++count; }
    //seed = (int)seed % 100;

    //yInfo() << " seed is " << seed;
    //Random::seed((int)seed);
    //yInfo() << " random is " << Random::uniform();

    return true;
}


bool opcPopulater::close() {
    iCub->close();
    delete iCub;

    return true;
}


bool opcPopulater::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n" +
        "populateSpecific1 entity_type entity_name \n" +
        "populateSpecific2 \n" +
        "addUnknownEntity entity_type\n" +
        "populateABM \n" +
        "setSaliencyEntity entity_name saliency_name\n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "populateSpecific1") {
        yInfo() << " populateSpecific1";
        (populateSpecific1(command)) ? reply.addString("populateSpecific done !") : reply.addString("populateSpecific failed !");
    }
    else if (command.get(0).asString() == "populateSpecific2") {
        yInfo() << " populateSpecific2";
        (populateSpecific2()) ? reply.addString("populateSpecific done !") : reply.addString("populateSpecific failed !");
    }
    else if (command.get(0).asString() == "populateABM") {
        yInfo() << " populateABM";
        (populateABM(command)) ? reply.addString("populateABM done !") : reply.addString("populateABM failed !");
    }
    else if (command.get(0).asString() == "addUnknownEntity") {
        yInfo() << " addUnknownEntity";
        (addUnknownEntity(command)) ? reply.addString("addUnknownEntity done !") : reply.addString("addUnknownEntity failed !");
    }
    else if (command.get(0).asString() == "setSaliencyEntity") {
        yInfo() << " setSaliencyEntity";
        (setSaliencyEntity(command)) ? reply.addString("setSaliencyEntity done !") : reply.addString("setSaliencyEntity failed !");
    }
    else {
        yInfo() << helpMessage;
        reply.addString("wrong command");
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool opcPopulater::updateModule() {
    return true;
}


bool opcPopulater::populateSpecific1(Bottle bInput){

    if (bInput.size() != 3)
    {
        yWarning() << " in opcPopulater::populateSpecific | wrong number of input";
        return false;
    }
    string sName = bInput.get(2).toString();

    if (bInput.get(1).toString() == "agent")
    {
        Agent* agent = iCub->opc->addOrRetrieveEntity<Agent>(sName);
        agent->m_ego_position[0] = (-1.5) * (Random::uniform()) - 0.5;
        agent->m_ego_position[1] = (2) * (Random::uniform()) - 1;
        agent->m_ego_position[2] = 0.60;
        agent->m_present = 1;
        agent->m_color[0] = Random::uniform(0, 80);
        agent->m_color[1] = Random::uniform(180, 250);
        agent->m_color[2] = Random::uniform(80, 180);
        iCub->opc->commit(agent);

        agent = NULL;
    }

    else if (bInput.get(1).toString() == "object")
    {
        Object* obj = iCub->opc->addOrRetrieveEntity<Object>(sName);
        obj->m_ego_position[0] = (-1.5) * (Random::uniform()) - 0.2;
        obj->m_ego_position[1] = (2) * (Random::uniform()) - 1;
        obj->m_ego_position[2] = 0.20;
        obj->m_present = 1;
        obj->m_color[0] = Random::uniform(100, 180);
        obj->m_color[1] = Random::uniform(0, 80);
        obj->m_color[2] = Random::uniform(180, 250);
        iCub->opc->commit(obj);

        obj = NULL;
    }

    else if (bInput.get(1).toString() == "rtobject")
    {
        RTObject* obj = iCub->opc->addOrRetrieveEntity<RTObject>(sName);
        obj->m_ego_position[0] = (-1.5) * (Random::uniform()) - 0.2;
        obj->m_ego_position[1] = (2) * (Random::uniform()) - 1;
        obj->m_present = 1;
        obj->m_color[0] = Random::uniform(180, 250);
        obj->m_color[1] = Random::uniform(100, 180);
        obj->m_color[2] = Random::uniform(0, 80);
        iCub->opc->commit(obj);

        obj = NULL;
    }

    return true;
}


bool opcPopulater::addUnknownEntity(Bottle bInput){

    if (bInput.size() != 2)
    {
        //        yWarning() << " in opcPopulater::addUnknownEntity | wrong number of inputCACA";
        return false;
    }

    iCub->opc->checkout();
    string sName = "unknown";
    yInfo() << " to be added: " << bInput.get(1).toString() << " called " << sName;

    if (bInput.get(1).toString() == "agent")
    {
        Agent* agent = iCub->opc->addEntity<Agent>(sName);
        agent->m_ego_position[0] = (-1.5) * (Random::uniform()) - 0.5;
        agent->m_ego_position[1] = (2) * (Random::uniform()) - 1;
        agent->m_ego_position[2] = 0.60;
        agent->m_present = 1;
        agent->m_color[0] = Random::uniform(0, 80);
        agent->m_color[1] = Random::uniform(180, 250);
        agent->m_color[2] = Random::uniform(80, 180);
        iCub->opc->commit(agent);

        agent = NULL;
    }

    else if (bInput.get(1).toString() == "object")
    {
        Object* obj = iCub->opc->addEntity<Object>(sName);
        obj->m_ego_position[0] = (-1.5) * (Random::uniform()) - 0.2;
        obj->m_ego_position[1] = (2) * (Random::uniform()) - 1;
        obj->m_ego_position[2] = 0.20;
        obj->m_present = 1;
        obj->m_color[0] = Random::uniform(100, 180);
        obj->m_color[1] = Random::uniform(0, 80);
        obj->m_color[2] = Random::uniform(180, 250);
        iCub->opc->commit(obj);

        obj = NULL;
    }

    else if (bInput.get(1).toString() == "rtobject")
    {
        RTObject* obj = iCub->opc->addEntity<RTObject>(sName);
        obj->m_ego_position[0] = (-1.5) * (Random::uniform()) - 0.2;
        obj->m_ego_position[1] = (2) * (Random::uniform()) - 1;
        obj->m_present = 1;
        obj->m_color[0] = Random::uniform(180, 250);
        obj->m_color[1] = Random::uniform(100, 180);
        obj->m_color[2] = Random::uniform(0, 80);
        iCub->opc->commit(obj);

        obj = NULL;
    }
    else
    {
        yInfo() << " " << bInput.get(1).toString() << " unknown kind of entity";
        return false;
    }
    return true;
}


/*
*  change the saliency of an entity in the OPC
*  input: Bottle ("setSaliencyEntity" entity_name saliency_level )
*/

bool opcPopulater::setSaliencyEntity(Bottle bInput){

    if (bInput.size() != 3)
    {
        yWarning() << " in opcPopulater::setSaliencyEntity| wrong number of input";
        return false;
    }


    string sName = bInput.get(1).toString();
    double targetSaliency = bInput.get(2).asDouble();

    iCub->opc->checkout();
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
    {
        if ((*itEnt)->name() == sName)
        {
            if ((*itEnt)->entity_type() == "agent")
            {
                Agent* temp = dynamic_cast<Agent*>(*itEnt);
                temp->m_saliency = targetSaliency;
            }
            if ((*itEnt)->entity_type() == "object")
            {
                Object* temp = dynamic_cast<Object*>(*itEnt);
                temp->m_saliency = targetSaliency;
            }
            if ((*itEnt)->entity_type() == "rtobject")
            {
                RTObject* temp = dynamic_cast<RTObject*>(*itEnt);
                temp->m_saliency = targetSaliency;
            }
        }
    }

    iCub->opc->commit();

    return true;
}



bool opcPopulater::populateABM(Bottle bInput)
{
    // first the Giraffe is close to larry (from left to right)
    iCub->opc->checkout();
    Agent* Larry = iCub->opc->addOrRetrieveEntity<Agent>("Larry");
    Agent* Robert = iCub->opc->addOrRetrieveEntity<Agent>("Robert");
    Object* Giraffe = iCub->opc->addOrRetrieveEntity<Object>("giraffe");
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Has = iCub->opc->addOrRetrieveEntity<Action>("has");
    iCub->opc->commit();

    Relation LarryHasGiraffe(Larry, Has, Giraffe);
    Relation LarryWantsGiraffe(Larry, Want, Giraffe);
    Relation RobertHasGiraffe(Robert, Has, Giraffe);
    Relation RobertWantsGiraffe(Robert, Want, Giraffe);

    int iRepetition = 5;
    double dDelay = 2.5;
    double dThresholdDelay = 1.5;

    for (int i = 0; i < iRepetition; i++)
    {

        Bottle bTempLarry;
        bTempLarry.addString("populateSpecific1");
        bTempLarry.addString("agent");
        bTempLarry.addString("Larry");
        populateSpecific1(bTempLarry);

        Bottle bTempRobert;
        bTempRobert.addString("populateSpecific1");
        bTempRobert.addString("agent");
        bTempRobert.addString("Robert");
        populateSpecific1(bTempRobert);

        Bottle bTempGiraffe;
        bTempGiraffe.addString("populateSpecific1");
        bTempGiraffe.addString("object");
        bTempGiraffe.addString("giraffe");
        populateSpecific1(bTempGiraffe);

        iCub->opc->checkout();

        Giraffe->m_ego_position[0] = Larry->m_ego_position[0] + 0.15;
        Giraffe->m_ego_position[1] = Larry->m_ego_position[1] + 0.15;
        Giraffe->m_ego_position[2] = Larry->m_ego_position[2];

        iCub->opc->removeRelation(LarryHasGiraffe);
        iCub->opc->removeRelation(LarryWantsGiraffe);
        iCub->opc->removeRelation(RobertHasGiraffe);
        iCub->opc->removeRelation(RobertWantsGiraffe);
        iCub->opc->commit();
        iCub->opc->checkout();

        iCub->opc->addRelation(LarryHasGiraffe);
        iCub->opc->addRelation(RobertWantsGiraffe);
        iCub->opc->commit();

        Time::delay(dThresholdDelay + dDelay*Random::uniform());

        if (iCub->getABMClient()->Connect())
        {
            std::list<std::pair<std::string, std::string> > lArgument;
            lArgument.push_back(std::pair<std::string, std::string>("Give me the giraffe", "sentence"));
            lArgument.push_back(std::pair<std::string, std::string>("(predicate give) (subject larry) (giraffe object)", "semantic"));
            lArgument.push_back(std::pair<std::string, std::string>("qRM", "provider"));
            lArgument.push_back(std::pair<std::string, std::string>("Robert", "speaker"));
            iCub->getABMClient()->sendActivity("action",
                "sentence",
                "recog",
                lArgument,
                true);
        }

        Time::delay(dThresholdDelay + dDelay*Random::uniform());

        iCub->opc->checkout();

        // SENTENCE
        Giraffe->m_ego_position[0] = Robert->m_ego_position[0] + 0.15;
        Giraffe->m_ego_position[1] = Robert->m_ego_position[1] + 0.15;
        Giraffe->m_ego_position[2] = Robert->m_ego_position[2];

        iCub->opc->removeRelation(RobertWantsGiraffe);
        iCub->opc->removeRelation(LarryHasGiraffe);
        iCub->opc->addRelation(RobertHasGiraffe);

        iCub->opc->commit();
        iCub->opc->checkout();

        Time::delay(dThresholdDelay + dDelay*Random::uniform());

    }


    return true;
}


bool opcPopulater::populateSpecific2(){


    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>("obj_left");
    obj1->m_ego_position[0] = (-0.35);
    obj1->m_ego_position[1] = (-0.30);
    obj1->m_ego_position[2] = 0.0;
    obj1->m_present = 1;
    obj1->m_color[0] = Random::uniform(0, 80);
    obj1->m_color[1] = Random::uniform(80, 180);
    obj1->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj1);

    Agent* obj2 = iCub->opc->addOrRetrieveEntity<Agent>("agent_left");
    obj2->m_ego_position[0] = (-0.30);
    obj2->m_ego_position[1] = (-0.40);
    obj2->m_ego_position[2] = 0.30;
    obj2->m_present = 1;
    obj2->m_color[0] = Random::uniform(0, 180);
    obj2->m_color[1] = Random::uniform(0, 80);
    obj2->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj2);

    Agent* obj3 = iCub->opc->addOrRetrieveEntity<Agent>("agent_right");
    obj3->m_ego_position[0] = (-0.30);
    obj3->m_ego_position[1] = (0.40);
    obj3->m_ego_position[2] = 0.30;
    obj3->m_present = 1;
    obj3->m_color[0] = Random::uniform(100, 180);
    obj3->m_color[1] = Random::uniform(80, 180);
    obj3->m_color[2] = Random::uniform(0, 80);
    iCub->opc->commit(obj3);


    Object* obj4 = iCub->opc->addOrRetrieveEntity<Object>("obj_right");
    obj4->m_ego_position[0] = (-0.35);
    obj4->m_ego_position[1] = (0.30);
    obj4->m_ego_position[2] = 0.0;
    obj4->m_present = 1;
    obj4->m_color[0] = Random::uniform(100, 180);
    obj4->m_color[1] = Random::uniform(0, 80);
    obj4->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj4);


    obj1 = NULL;
    obj2 = NULL;
    obj3 = NULL;

    return true;
}


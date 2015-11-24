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

    if (!iCub->getABMClient())
    {
        yWarning() << " WARNING ABM NOT CONNECTED";
    }

    iCub->lookStop();
    iCub->home();

    iCub->say("O P C populater is ready", false);
    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";


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
        "populateABMiCubStory \n" +
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
    else if (command.get(0).asString() == "populateABMiCubStory") {
        yInfo() << " populateABMiCubStory";
        (populateABMiCubStory(command)) ? reply.addString("populateABMiCubStory done !") : reply.addString("populateABMiCubStory failed !");
    }
    else if (command.get(0).asString() == "addUnknownEntity") {
        yInfo() << " addUnknownEntity";
        (addUnknownEntity(command)) ? reply.addString("addUnknownEntity done !") : reply.addString("addUnknownEntity failed !");
    }
    else if (command.get(0).asString() == "setSaliencyEntity") {
        yInfo() << " setSaliencyEntity";
        (setSaliencyEntity(command)) ? reply.addString("setSaliencyEntity done !") : reply.addString("setSaliencyEntity failed !");
    }
    else if (command.get(0).asString() == "clear") {
        yInfo() << " clearing OPC";
        iCub->opc->clear();
        iCub->opc->update();
        reply.addString("clearing OPC");
    }
    else {
        yInfo() << helpMessage;
        reply.addString("wrong command");
        reply.addString(helpMessage);
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


/*
* Populate the ABM with precific action comming from a external file
*/
bool opcPopulater::populateABMiCubStory(Bottle bInput)
{
    // first the Giraffe is close to larry (from left to right)
    iCub->opc->clear();
    iCub->opc->checkout();
    Agent* Nathan = iCub->opc->addOrRetrieveEntity<Agent>("Nathan");
    Agent* icub = iCub->opc->addOrRetrieveEntity<Agent>("iCub");
    Object* Giraffe = iCub->opc->addOrRetrieveEntity<Object>("giraffe");
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Has = iCub->opc->addOrRetrieveEntity<Action>("has");
    iCub->opc->commit();

    Relation NathanHasGiraffe(Nathan, Has, Giraffe);
    Relation iCubWantsGiraffe(icub, Want, Giraffe);
    Relation iCubHasGiraffe(icub, Has, Giraffe);


    double XNathan = -1.5,
        YNathan = 0,
        ZNathan = 0.6,
        distanceNat_Gir = 0.2;

    int iRepetition = 5;
    double dDelay = 2.5;
    double dThresholdDelay = 1.5;

    yInfo() << " initialisation of the OPC";

    Nathan->m_ego_position[0] = XNathan;
    Nathan->m_ego_position[1] = YNathan;
    Nathan->m_ego_position[2] = ZNathan;
    Nathan->m_present = 1;
    Nathan->m_color[0] = Random::uniform(0, 80);
    Nathan->m_color[1] = Random::uniform(180, 250);
    Nathan->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Nathan);

    Giraffe->m_ego_position[0] = XNathan + distanceNat_Gir*(Random::uniform() - 0.5);
    Giraffe->m_ego_position[1] = YNathan + distanceNat_Gir*(Random::uniform() - 0.5);
    Giraffe->m_ego_position[2] = 0;
    Giraffe->m_present = 1;
    Giraffe->m_color[0] = Random::uniform(0, 250);
    Giraffe->m_color[1] = Random::uniform(0, 250);
    Giraffe->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Giraffe);

    icub->m_ego_position[0] = 0.0;
    icub->m_ego_position[1] = 0.0;
    icub->m_ego_position[2] = 0.0;
    icub->m_present = 1;
    icub->m_color[0] = Random::uniform(0, 80);
    icub->m_color[1] = Random::uniform(180, 250);
    icub->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(icub);

    iCub->opc->addRelation(NathanHasGiraffe);
    iCub->opc->addRelation(iCubWantsGiraffe);
    yInfo() << " delay...";

    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " start recording in ABM";

    if (iCub->getABMClient()->Connect())
    {
        yInfo(" abm connected");
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>("take", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>("giraffe", "object"));
        lArgument.push_back(std::pair<std::string, std::string>("qRM", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "take",
            "action",
            lArgument,
            true);
    }
    yInfo() << " start grasping";
    bool finished = iCub->getARE()->take(Giraffe->m_ego_position);

    if (iCub->getABMClient()->Connect())
    {
        std::list<std::pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("iCub", "agent"));
        lArgument.push_back(pair<string, string>("take", "predicate"));
        lArgument.push_back(pair<string, string>("giraffe", "object"));
        lArgument.push_back(pair<string, string>("failed", "status"));
        lArgument.push_back(pair<string, string>("outofreach", "reason"));
        lArgument.push_back(pair<string, string>("qRM", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "take",
            "action",
            lArgument,
            finished);
    }
    yInfo() << " end of grasping... delay";
    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " searching for an action";
    if (iCub->getABMClient()->Connect())
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>("reason", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>("(predicate have) (agent icub) (object giraffe)", "goal"));
        lArgument.push_back(std::pair<std::string, std::string>("abmReasoning", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "reason",
            "reasoning",
            lArgument,
            true);
    }

    yInfo(" return: sentence 'give'...delay");
    Time::delay(dDelay*Random::uniform());
    yInfo() << " searching for an action";
    if (iCub->getABMClient()->Connect())
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>("reason", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>("(predicate have) (agent icub) (object giraffe)", "goal"));
        lArgument.push_back(std::pair<std::string, std::string>("(predicate sentence) (speaker icub) (object giraffe)", "result"));
        lArgument.push_back(std::pair<std::string, std::string>("addressee#have#object", "needs"));
        lArgument.push_back(std::pair<std::string, std::string>("abmReasoning", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "reason",
            "reasoning",
            lArgument,
            false);
    }

    Time::delay(dDelay*Random::uniform());
    yInfo() << " whatIs give ?";
    if (iCub->getABMClient()->Connect())
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>("reason", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>("give", "whatIs"));
        lArgument.push_back(std::pair<std::string, std::string>("abmReasoning", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "reason",
            "reasoning",
            lArgument,
            true);
    }

    Time::delay(dDelay*Random::uniform());
    yInfo() << " return: whatIs give ?";
    if (iCub->getABMClient()->Connect())
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>("reason", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>("give", "whatIs"));
        lArgument.push_back(std::pair<std::string, std::string>("addressee#have#object", "sentence_before"));
        lArgument.push_back(std::pair<std::string, std::string>("speaker#have#object", "sentence_after"));
        lArgument.push_back(std::pair<std::string, std::string>("agent#have#object", "action_before"));
        lArgument.push_back(std::pair<std::string, std::string>("recipient#have#object", "action_after"));
        lArgument.push_back(std::pair<std::string, std::string>("abmReasoning", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "reason",
            "reasoning",
            lArgument,
            false);
    }


    Time::delay(dDelay*Random::uniform());
    yInfo(" iCub ask the giraffe");

    list<pair<string, string> > lArgument;
    string sentence;
    sentence = "Give me the giraffe please";
    lArgument.push_back(pair<string, string>(sentence, "sentence"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>("Nathan", "agent"));
    lArgument.push_back(pair<string, string>("giraffe", "object"));
    lArgument.push_back(pair<string, string>("iCub", "adj1"));
    lArgument.push_back(pair<string, string>("iCub", "speaker"));
    lArgument.push_back(pair<string, string>("none", "subject"));
    lArgument.push_back(pair<string, string>("Nathan", "addressee"));
    iCub->getABMClient()->sendActivity("action",
        "sentence",
        "recog",
        lArgument,
        true);


    Time::delay(dDelay*Random::uniform());
    yInfo(" Nathan gives the giraffe");

    lArgument.clear();

    lArgument.push_back(pair<string, string>("Nathan", "agent"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>("giraffe", "object"));
    lArgument.push_back(pair<string, string>("iCub", "recipient"));

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        true);

    yInfo() << " in delay of action";
    Time::delay(4 + 4 * Random::uniform());

    Giraffe->m_ego_position[0] = -0.15 - 0.1 * Random::uniform();
    Giraffe->m_ego_position[1] = 0.15 - 0.3 * Random::uniform();
    iCub->opc->commit(Giraffe);

    iCub->opc->removeRelation(NathanHasGiraffe);
    iCub->opc->removeRelation(iCubWantsGiraffe);
    iCub->opc->addRelation(iCubHasGiraffe);

    Time::delay(3);

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        false);


    return true;
}




bool opcPopulater::populateSpecific2(){

    double errorMargin = 0.1;

    double distanceXObject = -0.3;
    double distanceYObject = 0.4;
    double distanceZObject = 0.0;

    double distanceXAgent = -0.3;
    double distanceYAgent = 0.4;
    double distanceZAgent = 0.3;

    iCub->opc->clear();

    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>("obj_left");
    obj1->m_ego_position[0] = distanceXObject + errorMargin * (Random::uniform() - 0.5);
    obj1->m_ego_position[1] = -1.* distanceYObject + errorMargin * (Random::uniform() - 0.5);
    obj1->m_ego_position[2] = distanceZObject + errorMargin * (Random::uniform() - 0.5);
    obj1->m_present = 1;
    obj1->m_color[0] = Random::uniform(0, 80);
    obj1->m_color[1] = Random::uniform(80, 180);
    obj1->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj1);

    Agent* obj2 = iCub->opc->addOrRetrieveEntity<Agent>("agent_left");
    obj2->m_ego_position[0] = distanceXAgent + errorMargin * (Random::uniform() - 0.5);
    obj2->m_ego_position[1] = -1.* distanceYAgent + errorMargin * (Random::uniform() - 0.5);
    obj2->m_ego_position[2] = distanceZAgent + errorMargin * (Random::uniform() - 0.5);
    obj2->m_present = 1;
    obj2->m_color[0] = Random::uniform(0, 180);
    obj2->m_color[1] = Random::uniform(0, 80);
    obj2->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj2);

    Agent* obj3 = iCub->opc->addOrRetrieveEntity<Agent>("agent_right");
    obj3->m_ego_position[0] = distanceXAgent + errorMargin * (Random::uniform() - 0.5);
    obj3->m_ego_position[1] = distanceYAgent + errorMargin * (Random::uniform() - 0.5);
    obj3->m_ego_position[2] = distanceZAgent + errorMargin * (Random::uniform() - 0.5);
    obj3->m_present = 1;
    obj3->m_color[0] = Random::uniform(100, 180);
    obj3->m_color[1] = Random::uniform(80, 180);
    obj3->m_color[2] = Random::uniform(0, 80);
    iCub->opc->commit(obj3);


    Object* obj4 = iCub->opc->addOrRetrieveEntity<Object>("obj_right");
    obj4->m_ego_position[0] = distanceXObject + errorMargin * (Random::uniform() - 0.5);
    obj4->m_ego_position[1] = distanceYObject + errorMargin * (Random::uniform() - 0.5);
    obj4->m_ego_position[2] = distanceZObject + errorMargin * (Random::uniform() - 0.5);
    obj4->m_present = 1;
    obj4->m_color[0] = Random::uniform(100, 180);
    obj4->m_color[1] = Random::uniform(0, 80);
    obj4->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj4);


    return true;
}


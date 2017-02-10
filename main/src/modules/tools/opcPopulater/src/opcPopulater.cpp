#include "opcPopulater.h"
#include "wrdac/subsystems/subSystem_LRH.h"
#include "wrdac/subsystems/subSystem_ARE.h"

bool opcPopulater::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("opcPopulater")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "opcPopulater", "client.ini", isRFVerbose);
    iCub->opc->isVerbose = false;
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

    if (!iCub->getLRH())
    {
        yWarning() << " WARNING LRH NOT CONNECTED";
    }

    Bottle &bRFInfo = rf.findGroup("populateSpecific2");

    X_obj = bRFInfo.check("X_obj", Value(-0.4)).asDouble();
    Y_obj = bRFInfo.check("Y_obj", Value(0.4)).asDouble();
    Z_obj = bRFInfo.check("Z_obj", Value(0.0)).asDouble();
    X_ag = bRFInfo.check("X_ag", Value(-0.4)).asDouble();
    Y_ag = bRFInfo.check("Y_ag", Value(0.4)).asDouble();
    Z_ag = bRFInfo.check("Z_ag", Value(0.3)).asDouble();
    noise = bRFInfo.check("noise", Value(0.0)).asDouble();

    cout << "X_obj " << X_obj << endl;
    cout << "Y_obj " << Y_obj << endl;
    cout << "Z_obj " << Z_obj << endl;
    cout << "X_ag " << X_ag << endl;
    cout << "Y_ag " << Y_ag << endl;
    cout << "Z_ag " << Z_ag << endl;
    cout << "noise " << noise << endl;

    move = false;
    spd1.push_back(0.);
    spd1.push_back(0.);
    spd1.push_back(0.);
    spd2.push_back(0.);
    spd2.push_back(0.);
    spd2.push_back(0.);
    iter = 0;

    iCub->lookStop();
    iCub->home();

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
        "populateSpecific3 \n"
        "addUnknownEntity entity_type\n" +
        "populateABM \n" +
        "populateABMiCubStory \n" +
        "storyFromPOV POV\n" +
        "populateScenario + N\n" +
        "setSaliencyEntity entity_name saliency_name\n";

    reply.clear();


    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "populateMoving") {
        yInfo() << " populateMoving";
        (populateMoving()) ? reply.addString("populateMoving done !") : reply.addString("populateMoving failed !");
    }
    else if (command.get(0).asString() == "populateRedBall") {
        yInfo() << " populateRedBall";
        (populateRedBall()) ? reply.addString("populateRedBall done !") : reply.addString("populateRedBall failed !");
    }
    else if (command.get(0).asString() == "populateSpecific1") {
        yInfo() << " populateSpecific1";
        (populateEntityRandom(command)) ? reply.addString("populateSpecific done !") : reply.addString("populateSpecific failed !");
    }
    else if (command.get(0).asString() == "populateSpecific2") {
        yInfo() << " populateSpecific2";
        (populateSpecific()) ? reply.addString("populateSpecific done !") : reply.addString("populateSpecific failed !");
    }
    else if (command.get(0).asString() == "populateSpecific3") {
        yInfo() << " populateSpecific3";
        (populateSpecific3()) ? reply.addString("populated 2 objects !") : reply.addString("populateSpecific failed !");
    }
    else if (command.get(0).asString() == "populateABM") {
        yInfo() << " populateABM";
        (populateABM(command)) ? reply.addString("populateABM done !") : reply.addString("populateABM failed !");
    }
    else if (command.get(0).asString() == "populateABMiCubStory") {
        yInfo() << " populateABMiCubStory";
        (populateABMiCubStory(command)) ? reply.addString("populateABMiCubStory done !") : reply.addString("populateABMiCubStory failed !");
    }
    else if (command.get(0).asString() == "storyFromPOV") {
        yInfo() << " storyFromPOV";
        (storyFromPOV(command)) ? reply.addString("storyFromPOV done !") : reply.addString("storyFromPOV failed !");
    }
    else if (command.get(0).asString() == "addUnknownEntity") {
        yInfo() << " addUnknownEntity";
        (addUnknownEntity(command)) ? reply.addString("addUnknownEntity done !") : reply.addString("addUnknownEntity failed !");
    }
    else if (command.get(0).asString() == "setSaliencyEntity") {
        yInfo() << " setSaliencyEntity";
        (setSaliencyEntity(command)) ? reply.addString("setSaliencyEntity done !") : reply.addString("setSaliencyEntity failed !");
    }
    else if (command.get(0).asString() == "setValueEntity") {
        yInfo() << " setValueEntity";
        (setValueEntity(command)) ? reply.addString("setValueEntity done !") : reply.addString("setValueEntity failed !");
    }
    else if (command.get(0).asString() == "clear") {
        yInfo() << " clearing OPC";
        iCub->opc->clear();
        iCub->opc->update();
        reply.addString("clearing OPC");
    }
    else if (command.get(0) == "populateScenario"){
        int scenarioNB = 1;
        if (command.size() == 2){
            scenarioNB = command.get(1).asInt();
            if (scenarioNB < 1) scenarioNB = 1;
            if (scenarioNB > 6) scenarioNB = 6;
        }
        yInfo() << " populating with scenario: " << scenarioNB;
        switch (scenarioNB)
        {
        case 1: populateScenario1() ? reply.addString("populating scenario 1 done !") : reply.addString("populating scenario failed !"); break;
        case 2: populateScenario2() ? reply.addString("populating scenario 2 done !") : reply.addString("populating scenario failed !"); break;
        case 3: populateScenario3() ? reply.addString("populating scenario 3 done !") : reply.addString("populating scenario failed !"); break;
        case 4: populateScenario4() ? reply.addString("populating scenario 4 done !") : reply.addString("populating scenario failed !"); break;
        case 5: populateScenario5() ? reply.addString("populating scenario 5 done !") : reply.addString("populating scenario failed !"); break;
        case 6: populateScenario6() ? reply.addString("populating scenario 6 done !") : reply.addString("populating scenario failed !"); break;
        }
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
    //Just for testing: 
    if (move == true)
    {
        //double speed = 1./10.;
        iCub->opc->checkout();

        Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>("moving_1");
        Object* obj2 = iCub->opc->addOrRetrieveEntity<Object>("moving_2");

        if (iter % 30 == 0)
        {
            spd1[0] = (rand() % 100)*pow(-1., (rand() % 2) + 1);
            spd1[1] = (rand() % 100)*pow(-1., (rand() % 2) + 1);
            spd1[2] = (rand() % 100)*pow(-1., (rand() % 2) + 1);
            spd2[0] = (rand() % 100)*pow(-1., (rand() % 2) + 1);
            spd2[1] = (rand() % 100)*pow(-1., (rand() % 2) + 1);
            spd2[2] = (rand() % 100)*pow(-1., (rand() % 2) + 1);
            double av1 = sqrt(pow(spd1[0], 2) + pow(spd1[1], 2) + pow(spd1[2], 2));
            double av2 = sqrt(pow(spd2[0], 2) + pow(spd2[1], 2) + pow(spd2[2], 2));
            spd1[0] /= av1;
            spd1[1] /= av1;
            spd1[2] /= av1;
            spd2[0] /= av2;
            spd2[1] /= av2;
            spd2[2] /= av2;
        }
        /* obj1->m_ego_position[0] = min(max(obj1->m_ego_position[0]+spd1[0]*period*speed,-0.2),-0.5);
         obj1->m_ego_position[1] = min(max(obj1->m_ego_position[1]+spd1[1]*period*speed,0.0),0.5);
         obj1->m_ego_position[2] = min(max(obj1->m_ego_position[2]+spd1[2]*period*speed,0.0),0.5);
         obj2->m_ego_position[0] = min(max(obj2->m_ego_position[0]+spd2[0]*period*speed,-0.2),-0.5);
         obj2->m_ego_position[1] = min(max(obj2->m_ego_position[1]+spd2[1]*period*speed,0.0),0.5);
         obj2->m_ego_position[2] = min(max(obj2->m_ego_position[2]+spd2[2]*period*speed,0.0),0.5);*/
        obj1->m_ego_position[0] = -0.18;
        obj1->m_ego_position[1] = 0.10;
        obj1->m_ego_position[2] = 0.1;
        obj2->m_ego_position[0] = -0.18;
        obj2->m_ego_position[1] = -0.10;
        obj2->m_ego_position[2] = 0.1;

        iCub->opc->commit(obj1);
        iCub->opc->commit(obj2);

        iter += 1;
    }
    return true;
}

bool opcPopulater::populateEntityRandom(Bottle bInput){

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
        agent->m_present = 1.0;
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
        obj->m_present = 1.0;
        obj->m_color[0] = Random::uniform(100, 180);
        obj->m_color[1] = Random::uniform(0, 80);
        obj->m_color[2] = Random::uniform(180, 250);
        obj->m_value = 1.5;
        yDebug() << "value: " << obj->m_value;
        iCub->opc->commit(obj);

        obj = NULL;
    }

    else if (bInput.get(1).toString() == "rtobject")
    {
        RTObject* obj = iCub->opc->addOrRetrieveEntity<RTObject>(sName);
        obj->m_ego_position[0] = (-1.5) * (Random::uniform()) - 0.2;
        obj->m_ego_position[1] = (2) * (Random::uniform()) - 1;
        obj->m_present = 1.0;
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
        yWarning() << " in opcPopulater::addUnknownEntity | wrong number of input";
        return false;
    }

    iCub->opc->checkout();
    string sName = "unknown";
    yInfo() << " to be added: " << bInput.get(1).toString() << " called " << sName;

    if (bInput.get(1).toString() == "agent")
    {
        sName = "partner";
        Agent* agent = iCub->opc->addEntity<Agent>(sName);
        agent->m_ego_position[0] = (-1.5) * (Random::uniform()) - 0.5;
        agent->m_ego_position[1] = (2) * (Random::uniform()) - 1;
        agent->m_ego_position[2] = 0.60;
        agent->m_present = 1.0;
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
        obj->m_present = 1.0;
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
        obj->m_present = 1.0;
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

bool opcPopulater::setValueEntity(Bottle bInput){

    if (bInput.size() != 3)
    {
        yWarning() << " in opcPopulater::setValueEntity| wrong number of input";
        return false;
    }


    string sName = bInput.get(1).toString();
    double targetValue = bInput.get(2).asDouble();

    iCub->opc->checkout();

    Entity *e = iCub->opc->getEntity(sName);
    if (e && (e->entity_type() == "agent" || e->entity_type() == "object" || e->entity_type() == "rtobject")) {
        Object* temp = dynamic_cast<Object*>(e);
        temp->m_value = targetValue;
        iCub->opc->commit();
    }
    else{
        yWarning() << "Trying to change value of the non-object entity: " << sName << ". Please check!";
    }
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
    Action* Have = iCub->opc->addOrRetrieveEntity<Action>("have");
    iCub->opc->commit();

    Relation LarryHasGiraffe(Larry, Have, Giraffe);
    Relation LarryWantsGiraffe(Larry, Want, Giraffe);
    Relation RobertHasGiraffe(Robert, Have, Giraffe);
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
        populateEntityRandom(bTempLarry);

        Bottle bTempRobert;
        bTempRobert.addString("populateSpecific1");
        bTempRobert.addString("agent");
        bTempRobert.addString("Robert");
        populateEntityRandom(bTempRobert);

        Bottle bTempGiraffe;
        bTempGiraffe.addString("populateSpecific1");
        bTempGiraffe.addString("object");
        bTempGiraffe.addString("giraffe");
        populateEntityRandom(bTempGiraffe);

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
            list<pair<string, string> > lArgument;
            lArgument.push_back(pair<string, string>("Give me the giraffe", "sentence"));
            lArgument.push_back(pair<string, string>("(predicate give) (subject larry) (giraffe object)", "semantic"));
            lArgument.push_back(pair<string, string>("qRM", "provider"));
            lArgument.push_back(pair<string, string>("Robert", "speaker"));
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


    string sOject = bInput.check("object", Value("croco")).asString();
    string sAgent = bInput.check("agent", Value("Interlocutor")).asString();


    Time::delay(4.);
    // first the ObjStory is close to larry (from left to right)
    iCub->opc->clear();
    iCub->opc->checkout();
    Agent* Interlocutor = iCub->opc->addOrRetrieveEntity<Agent>(sAgent);
    Agent* icub = iCub->opc->addOrRetrieveEntity<Agent>("icub");
    Object* ObjStory = iCub->opc->addOrRetrieveEntity<Object>(sOject);
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Have = iCub->opc->addOrRetrieveEntity<Action>("have");
    iCub->opc->commit();

    Relation InterlocutorHasObjStory(Interlocutor, Have, ObjStory);
    Relation iCubWantsObjStory(icub, Want, ObjStory);
    Relation iCubHasObjStory(icub, Have, ObjStory);


    double XInterlocutor = -1.5,
        YInterlocutor = 0,
        ZInterlocutor = 0.6,
        distanceNat_Gir = 0.2;

    //int iRepetition = 5;
    double dDelay = 2.5;
    double dThresholdDelay = 1.5;

    yInfo() << " initialisation of the OPC";

    Interlocutor->m_ego_position[0] = XInterlocutor;
    Interlocutor->m_ego_position[1] = YInterlocutor;
    Interlocutor->m_ego_position[2] = ZInterlocutor;
    Interlocutor->m_present = 1.0;
    Interlocutor->m_color[0] = Random::uniform(0, 80);
    Interlocutor->m_color[1] = Random::uniform(180, 250);
    Interlocutor->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Interlocutor);

    ObjStory->m_ego_position[0] = XInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    ObjStory->m_ego_position[1] = YInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    ObjStory->m_ego_position[2] = 0;
    ObjStory->m_present = 1.0;
    ObjStory->m_color[0] = Random::uniform(0, 250);
    ObjStory->m_color[1] = Random::uniform(0, 250);
    ObjStory->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(ObjStory);

    icub->m_ego_position[0] = 0.0;
    icub->m_ego_position[1] = 0.0;
    icub->m_ego_position[2] = 0.0;
    icub->m_present = 1.0;
    icub->m_color[0] = Random::uniform(0, 80);
    icub->m_color[1] = Random::uniform(180, 250);
    icub->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(icub);

    iCub->opc->addRelation(InterlocutorHasObjStory);
    iCub->opc->addRelation(iCubWantsObjStory);
    yInfo() << " delay...";

    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " start recording in ABM";

    //if (iCub->getABMClient()->Connect())
    //{
    //    yInfo(" abm connected");
    //    list<pair<string, string> > lArgument;
    //    lArgument.push_back(pair<string, string>("icub", "agent"));
    //    lArgument.push_back(pair<string, string>("take", "predicate"));
    //    lArgument.push_back(pair<string, string>(sOject, "object"));
    //    lArgument.push_back(pair<string, string>("qRM", "provider"));
    //    iCub->getABMClient()->sendActivity("action",
    //        "take",
    //        "action",
    //        lArgument,
    //        true);
    //}
    yInfo() << " start grasping";
    iCub->take(ObjStory->m_ego_position);

    //if (iCub->getABMClient()->Connect())
    //{
    //    list<pair<string, string> > lArgument;
    //    lArgument.push_back(pair<string, string>("icub", "agent"));
    //    lArgument.push_back(pair<string, string>("take", "predicate"));
    //    lArgument.push_back(pair<string, string>(sOject, "object"));
    //    lArgument.push_back(pair<string, string>("failed", "status"));
    //    lArgument.push_back(pair<string, string>("outofreach", "reason"));
    //    lArgument.push_back(pair<string, string>("qRM", "provider"));
    //    iCub->getABMClient()->sendActivity("action",
    //        "take",
    //        "action",
    //        lArgument,
    //        finished);
    //}
    yInfo() << " end of grasping... delay";
    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " searching for an action";
    if (iCub->getABMClient()->Connect())
    {
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        ostringstream osTmp;
        osTmp << "(predicate have) (agent icub) (object " << sOject << ")";
        string tmp = osTmp.str();
        lArgument.push_back(pair<string, string>(tmp, "goal"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        ostringstream osTmp;
        osTmp << "(predicate have) (agent icub) (object " << sOject << ")";
        string tmp = osTmp.str();
        lArgument.push_back(pair<string, string>(tmp, "goal"));
        ostringstream osTmp2;
        osTmp2 << "(predicate sentence) (speaker icub) (object " << sOject << ")";
        string tmp2 = osTmp2.str();
        lArgument.push_back(pair<string, string>(tmp2, "result"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "needs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "sentence_before"));
        lArgument.push_back(pair<string, string>("speaker#have#object", "sentence_after"));
        lArgument.push_back(pair<string, string>("agent#have#object", "action_before"));
        lArgument.push_back(pair<string, string>("recipient#have#object", "action_after"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "reason",
            "reasoning",
            lArgument,
            false);
    }


    Time::delay(dDelay*Random::uniform());
    yInfo(" iCub ask the ObjStory");

    list<pair<string, string> > lArgument;
    string sentence;
    sentence = "Give me the " + sOject;
    sentence += " please";
    lArgument.push_back(pair<string, string>(sentence, "sentence"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>(sAgent, "agent"));
    lArgument.push_back(pair<string, string>(sOject, "object"));
    lArgument.push_back(pair<string, string>("icub", "adj1"));
    lArgument.push_back(pair<string, string>("icub", "speaker"));
    lArgument.push_back(pair<string, string>("none", "subject"));
    lArgument.push_back(pair<string, string>(sAgent, "addressee"));
    iCub->getABMClient()->sendActivity("action",
        "sentence",
        "recog",
        lArgument,
        true);


    Time::delay(dDelay*Random::uniform());
    yInfo(" Interlocutor gives the ObjStory");

    lArgument.clear();

    lArgument.push_back(pair<string, string>(sAgent, "agent"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>(sOject, "object"));
    lArgument.push_back(pair<string, string>("icub", "recipient"));

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        true);

    yInfo() << " in delay of action";
    Time::delay(4 + 4 * Random::uniform());

    ObjStory->m_ego_position[0] = -0.15 - 0.1 * Random::uniform();
    ObjStory->m_ego_position[1] = 0.15 - 0.3 * Random::uniform();
    iCub->opc->commit(ObjStory);

    iCub->opc->removeRelation(InterlocutorHasObjStory);
    iCub->opc->removeRelation(iCubWantsObjStory);
    iCub->opc->addRelation(iCubHasObjStory);

    Time::delay(3);

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        false);


    return true;
}


bool opcPopulater::populateSpecific(){

    double errorMargin = noise;
    iCub->opc->clear();

    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>("bottom_left");
    obj1->m_ego_position[0] = X_obj + errorMargin * (Random::uniform() - 0.5);
    obj1->m_ego_position[1] = -1.* Y_obj + errorMargin * (Random::uniform() - 0.5);
    obj1->m_ego_position[2] = Z_obj + errorMargin * (Random::uniform() - 0.5);
    obj1->m_present = 1.0;
    obj1->m_color[0] = Random::uniform(0, 80);
    obj1->m_color[1] = Random::uniform(80, 180);
    obj1->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj1);

    Object* obj2 = iCub->opc->addOrRetrieveEntity<Object>("top_left");
    obj2->m_ego_position[0] = X_ag + errorMargin * (Random::uniform() - 0.5);
    obj2->m_ego_position[1] = -1.* Y_ag + errorMargin * (Random::uniform() - 0.5);
    obj2->m_ego_position[2] = Z_ag + errorMargin * (Random::uniform() - 0.5);
    obj2->m_present = 1.0;
    obj2->m_color[0] = Random::uniform(0, 180);
    obj2->m_color[1] = Random::uniform(0, 80);
    obj2->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj2);

    Object* obj3 = iCub->opc->addOrRetrieveEntity<Object>("top_right");
    obj3->m_ego_position[0] = X_ag + errorMargin * (Random::uniform() - 0.5);
    obj3->m_ego_position[1] = Y_ag + errorMargin * (Random::uniform() - 0.5);
    obj3->m_ego_position[2] = Z_ag + errorMargin * (Random::uniform() - 0.5);
    obj3->m_present = 1.0;
    obj3->m_color[0] = Random::uniform(100, 180);
    obj3->m_color[1] = Random::uniform(80, 180);
    obj3->m_color[2] = Random::uniform(0, 80);
    iCub->opc->commit(obj3);


    Object* obj4 = iCub->opc->addOrRetrieveEntity<Object>("bottom_right");
    obj4->m_ego_position[0] = X_obj + errorMargin * (Random::uniform() - 0.5);
    obj4->m_ego_position[1] = Y_obj + errorMargin * (Random::uniform() - 0.5);
    obj4->m_ego_position[2] = Z_obj + errorMargin * (Random::uniform() - 0.5);
    obj4->m_present = 1.0;
    obj4->m_color[0] = Random::uniform(100, 180);
    obj4->m_color[1] = Random::uniform(0, 80);
    obj4->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj4);


    return true;
}


bool opcPopulater::populateSpecific3(){

    iCub->opc->clear();

    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>("unknown_1");
    obj1->m_ego_position[0] = -0.4;
    obj1->m_ego_position[1] = 0.25;
    obj1->m_ego_position[2] = 0;
    obj1->m_present = 1.0;
    obj1->m_color[0] = Random::uniform(0, 80);
    obj1->m_color[1] = Random::uniform(80, 180);
    obj1->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj1);

    Object* obj2 = iCub->opc->addOrRetrieveEntity<Object>("unknown_2");
    obj2->m_ego_position[0] = -0.4;
    obj2->m_ego_position[1] = -0.25;
    obj2->m_ego_position[2] = 0;
    obj2->m_present = 1.0;
    obj2->m_color[0] = Random::uniform(100, 180);
    obj2->m_color[1] = Random::uniform(0, 80);
    obj2->m_color[2] = Random::uniform(180, 250);
    iCub->opc->commit(obj2);


    return true;
}


bool opcPopulater::populateMoving(){

    iCub->opc->clear();

    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>("moving_1");
    obj1->m_ego_position[0] = -0.18;
    obj1->m_ego_position[1] = 0.10;
    obj1->m_ego_position[2] = 0.1;

    obj1->m_present = 1.0;
    obj1->m_color[0] = 20;
    obj1->m_color[1] = 20;
    obj1->m_color[2] = 20;
    obj1->m_value = 0.0;
    iCub->opc->commit(obj1);

    Object* obj2 = iCub->opc->addOrRetrieveEntity<Object>("moving_2");
    obj2->m_ego_position[0] = -0.18;
    obj2->m_ego_position[1] = -0.10;
    obj2->m_ego_position[2] = 0.1;

    obj2->m_present = 1.0;
    obj2->m_color[0] = 200;
    obj2->m_color[1] = 20;
    obj2->m_color[2] = 20;
    obj2->m_value = -1.0;
    iCub->opc->commit(obj2);

    //move=true;


    return true;
}

bool opcPopulater::populateRedBall(){

    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>("red_ball");
    obj1->m_ego_position[0] = -0.4;
    obj1->m_ego_position[1] = 0.25;
    obj1->m_ego_position[2] = 0;
    obj1->m_present = 0.0;
    obj1->m_color[0] = 250;
    obj1->m_color[1] = 0;
    obj1->m_color[2] = 0;
    obj1->m_value = -1.0;
    iCub->opc->commit(obj1);
    return true;
}


/*
* Populate the ABM with precific action comming from a external file
*/
bool opcPopulater::storyFromPOV(Bottle bInput)
{
    bool fromMeaning = true;
    vector<string>  listSentencePOViCub;
    vector<string>  listSentencePOVNathan;
    vector<string>  listMeaningPOViCub;
    vector<string>  listMeaningPOVNathan;

    listSentencePOViCub.push_back("I wanted to get the giraffe");
    listSentencePOViCub.push_back("but I failed to grasp it");
    listSentencePOViCub.push_back("because it laid outofreach");
    listSentencePOViCub.push_back("so I found a different action");
    listSentencePOViCub.push_back("if I could ask you to give it to me");
    listSentencePOViCub.push_back("then you would give it to me");
    listSentencePOViCub.push_back("so I asked you to give it to me");
    listSentencePOViCub.push_back("and you gave it to me");
    listSentencePOViCub.push_back("now I have the giraffe");

    listMeaningPOViCub.push_back(", wanted I , get I giraffe <o> [_-_-_-_-_-_-_-_][A-P-_-_-_-_-_-_][A-_-P-O-_-_-_-_] <o>");
    listMeaningPOViCub.push_back("but , failed I , grasp I it <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-O-_-_-_] <o>");
    listMeaningPOViCub.push_back("because , laid it outofreach <o> [P-_-_-_-_-_-_-_][_-A-P-R-_-_-_-_] <o>");
    listMeaningPOViCub.push_back("so , found I action different <o> [P-_-_-_-_-_-_-_][_-A-P-R-O-_-_-_] <o>");
    listMeaningPOViCub.push_back("if , could I , ask I you , give you it me <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-R-_-_-_][_-A-_-_-_-P-O-R] <o>");
    listMeaningPOViCub.push_back("then , would you , give you it me <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-O-R-_-_] <o>");
    listMeaningPOViCub.push_back("so , asked I you , give you it me <o> [P-_-_-_-_-_-_-_][_-A-P-R-_-_-_-_][_-_-_-A-P-O-R-_] <o>");
    listMeaningPOViCub.push_back("and , gave you it me <o> [P-_-_-_-_-_-_-_][_-A-P-O-R-_-_-_] <o>");
    listMeaningPOViCub.push_back("now , have I giraffe <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>");

    listSentencePOVNathan.push_back("you wanted to get the giraffe");
    listSentencePOVNathan.push_back("but you failed to grasp it");
    listSentencePOVNathan.push_back("because it laid outofreach");
    listSentencePOVNathan.push_back("so you found a different action");
    listSentencePOVNathan.push_back("if you could ask me to give it to you");
    listSentencePOVNathan.push_back("then I would give it to you");
    listSentencePOVNathan.push_back("so you asked me to give it to you");
    listSentencePOVNathan.push_back("and I gave it to you");
    listSentencePOVNathan.push_back("now you have the giraffe");

    listMeaningPOVNathan.push_back(", wanted you , get you giraffe <o> [_-_-_-_-_-_-_-_][A-P-_-_-_-_-_-_][A-_-P-O-_-_-_-_] <o>");
    listMeaningPOVNathan.push_back("but , failed you , grasp youI it <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-O-_-_-_] <o>");
    listMeaningPOVNathan.push_back("because , laid it outofreach <o> [P-_-_-_-_-_-_-_][_-A-P-R-_-_-_-_] <o>");
    listMeaningPOVNathan.push_back("so , found you action different <o> [P-_-_-_-_-_-_-_][_-A-P-R-O-_-_-_] <o>");
    listMeaningPOVNathan.push_back("if , could you , ask I me , give me it you<o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-R-_-_-_][_-A-_-_-_-P-O-R] <o>");
    listMeaningPOVNathan.push_back("then , would I , give I it you <o> [P-_-_-_-_-_-_-_][_-A-P-_-_-_-_-_][_-A-_-P-O-R-_-_] <o>");
    listMeaningPOVNathan.push_back("so , asked you me, give me it you <o> [P-_-_-_-_-_-_-_][_-A-P-R-_-_-_-_][_-_-_-A-P-O-R-_] <o>");
    listMeaningPOVNathan.push_back("and , gave I it you <o> [P-_-_-_-_-_-_-_][_-A-P-O-R-_-_-_] <o>");
    listMeaningPOVNathan.push_back("now , have you giraffe <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>");

    unsigned int isentence = 0;
    vector<string>  currentPOV;

    fromMeaning ? currentPOV = listMeaningPOViCub : currentPOV = listSentencePOViCub;

    if (bInput.size() == 2){
        if (bInput.get(1).toString() == "Nathan") {
            fromMeaning ? currentPOV = listMeaningPOVNathan : currentPOV = listSentencePOVNathan;
        }
    }

    yInfo(" OPC populater starting story from POV");

    fromMeaning ? iCub->getLRH()->narrator = "Narrator" :
        iCub->getLRH()->interlocutor = "Narrator";

    Time::delay(12.);
    // first the Giraffe is close to larry (from left to right)
    iCub->opc->clear();
    iCub->opc->checkout();
    Agent* Nathan = iCub->opc->addOrRetrieveEntity<Agent>("Nathan");
    Agent* icub = iCub->opc->addOrRetrieveEntity<Agent>("icub");
    Object* Giraffe = iCub->opc->addOrRetrieveEntity<Object>("giraffe");
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Have = iCub->opc->addOrRetrieveEntity<Action>("have");
    iCub->opc->commit();

    Relation NathanHasGiraffe(Nathan, Have, Giraffe);
    Relation iCubWantsGiraffe(icub, Want, Giraffe);
    Relation iCubHasGiraffe(icub, Have, Giraffe);


    double XNathan = -1.5,
        YNathan = 0,
        ZNathan = 0.6,
        distanceNat_Gir = 0.2;

    //int iRepetition = 5;
    double dDelay = 2.5;
    double dThresholdDelay = 1.5;

    yInfo() << " initialisation of the OPC";

    Nathan->m_ego_position[0] = XNathan;
    Nathan->m_ego_position[1] = YNathan;
    Nathan->m_ego_position[2] = ZNathan;
    Nathan->m_present = 1.0;
    Nathan->m_color[0] = Random::uniform(0, 80);
    Nathan->m_color[1] = Random::uniform(180, 250);
    Nathan->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Nathan);

    Giraffe->m_ego_position[0] = XNathan + distanceNat_Gir*(Random::uniform() - 0.5);
    Giraffe->m_ego_position[1] = YNathan + distanceNat_Gir*(Random::uniform() - 0.5);
    Giraffe->m_ego_position[2] = 0;
    Giraffe->m_present = 1.0;
    Giraffe->m_color[0] = Random::uniform(0, 250);
    Giraffe->m_color[1] = Random::uniform(0, 250);
    Giraffe->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Giraffe);

    icub->m_ego_position[0] = 0.0;
    icub->m_ego_position[1] = 0.0;
    icub->m_ego_position[2] = 0.0;
    icub->m_present = 1.0;
    icub->m_color[0] = Random::uniform(0, 80);
    icub->m_color[1] = Random::uniform(180, 250);
    icub->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(icub);

    iCub->opc->addRelation(NathanHasGiraffe);
    iCub->opc->addRelation(iCubWantsGiraffe);
    yInfo() << " delay...";

    Time::delay(dThresholdDelay + dDelay*Random::uniform());


    // APPEARANCE OF AGENT

    // GOAL CREATION

    // REASONING 1

    // RESULT REASONING

    // TAKE TRY

    // TAKE FAILED OUT OF REACH

    // REASONING ?

    // SENTENCE GIVE

    // ACTION DETECTOR GIVE APPEARS

    // FINAL SITUATION/

    iCub->getLRH()->meaningToSentence(listMeaningPOViCub[isentence]);

    (fromMeaning) ? iCub->getLRH()->meaningToSentence(currentPOV[isentence]) :
        iCub->getLRH()->SentenceToMeaning(currentPOV[isentence]);
    isentence++;

    Time::delay(1.0);

    yInfo() << " start recording in ABM";

    if (iCub->getABMClient()->Connect())
    {
        yInfo(" abm connected");
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("take", "predicate"));
        lArgument.push_back(pair<string, string>("giraffe", "object"));
        lArgument.push_back(pair<string, string>("qRM", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "take",
            "action",
            lArgument,
            true);
    }


    yInfo() << " start grasping";
    bool finished = iCub->take(Giraffe->m_ego_position);


    if (iCub->getABMClient()->Connect())
    {
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
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

    (fromMeaning) ? iCub->getLRH()->meaningToSentence(currentPOV[isentence]) :
        iCub->getLRH()->SentenceToMeaning(currentPOV[isentence]);
    isentence++;

    (fromMeaning) ? iCub->getLRH()->meaningToSentence(currentPOV[isentence]) :
        iCub->getLRH()->SentenceToMeaning(currentPOV[isentence]);
    isentence++;

    yInfo() << " searching for an action";
    if (iCub->getABMClient()->Connect())
    {
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("(predicate have) (agent icub) (object giraffe)", "goal"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("(predicate have) (agent icub) (object giraffe)", "goal"));
        lArgument.push_back(pair<string, string>("(predicate sentence) (speaker icub) (object giraffe)", "result"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "needs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "sentence_before"));
        lArgument.push_back(pair<string, string>("speaker#have#object", "sentence_after"));
        lArgument.push_back(pair<string, string>("agent#have#object", "action_before"));
        lArgument.push_back(pair<string, string>("recipient#have#object", "action_after"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "reason",
            "reasoning",
            lArgument,
            false);
    }

    (fromMeaning) ? iCub->getLRH()->meaningToSentence(currentPOV[isentence]) :
        iCub->getLRH()->SentenceToMeaning(currentPOV[isentence]);
    isentence++;

    (fromMeaning) ? iCub->getLRH()->meaningToSentence(currentPOV[isentence]) :
        iCub->getLRH()->SentenceToMeaning(currentPOV[isentence]);
    isentence++;

    (fromMeaning) ? iCub->getLRH()->meaningToSentence(currentPOV[isentence]) :
        iCub->getLRH()->SentenceToMeaning(currentPOV[isentence]);
    isentence++;

    Time::delay(dDelay*Random::uniform());
    yInfo(" iCub ask the giraffe");

    list<pair<string, string> > lArgument;
    string sentence = "Give me the giraffe please";
    lArgument.push_back(pair<string, string>(sentence, "sentence"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>("Nathan", "agent"));
    lArgument.push_back(pair<string, string>("giraffe", "object"));
    lArgument.push_back(pair<string, string>("icub", "adj1"));
    lArgument.push_back(pair<string, string>("icub", "speaker"));
    lArgument.push_back(pair<string, string>("none", "subject"));
    lArgument.push_back(pair<string, string>("Nathan", "addressee"));
    iCub->getABMClient()->sendActivity("action",
        "sentence",
        "say",
        lArgument,
        true);


    (fromMeaning) ? iCub->getLRH()->meaningToSentence(currentPOV[isentence]) :
        iCub->getLRH()->SentenceToMeaning(currentPOV[isentence]);
    isentence++;


    Time::delay(dDelay*Random::uniform());
    yInfo(" Nathan gives the giraffe");

    lArgument.clear();

    lArgument.push_back(pair<string, string>("Nathan", "agent"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>("giraffe", "object"));
    lArgument.push_back(pair<string, string>("icub", "recipient"));

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

    (fromMeaning) ? iCub->getLRH()->meaningToSentence(currentPOV[isentence]) :
        iCub->getLRH()->SentenceToMeaning(currentPOV[isentence]);
    isentence++;

    (fromMeaning) ? iCub->getLRH()->meaningToSentence(currentPOV[isentence]) :
        iCub->getLRH()->SentenceToMeaning(currentPOV[isentence]);
    isentence++;


    return true;
}


/*
* 1 agent, that gives the wrong object first.
*/
bool opcPopulater::populateScenario1()
{
    string sAgent = "Tobi";
    string sObject = "blue cube";
    string sObjError = "octopus";

    Time::delay(4.);
    // first the ObjStory is close to larry (from left to right)
    iCub->opc->clear();
    iCub->opc->checkout();
    Agent* Interlocutor = iCub->opc->addOrRetrieveEntity<Agent>(sAgent);
    Agent* icub = iCub->opc->addOrRetrieveEntity<Agent>("icub");
    Object* Croco = iCub->opc->addOrRetrieveEntity<Object>(sObject);
    Object* Mouse = iCub->opc->addOrRetrieveEntity<Object>(sObjError);
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Have = iCub->opc->addOrRetrieveEntity<Action>("have");
    iCub->opc->commit();

    Relation InterlocutorHasCroco(Interlocutor, Have, Croco);
    Relation InterlocutorHasMouse(Interlocutor, Have, Mouse);
    Relation iCubWantsCroco(icub, Want, Croco);
    Relation iCubHasCroco(icub, Have, Croco);
    Relation iCubHasMouse(icub, Have, Mouse);


    double XInterlocutor = -1.5,
        YInterlocutor = 0,
        ZInterlocutor = 0.6,
        distanceNat_Gir = 0.2;

    //int iRepetition = 5;
    double dDelay = 1.5;
    double dThresholdDelay = 1.5;

    yInfo() << " initialisation of the OPC";

    Interlocutor->m_ego_position[0] = XInterlocutor;
    Interlocutor->m_ego_position[1] = YInterlocutor;
    Interlocutor->m_ego_position[2] = ZInterlocutor;
    Interlocutor->m_present = 1.0;
    Interlocutor->m_color[0] = Random::uniform(0, 80);
    Interlocutor->m_color[1] = Random::uniform(180, 250);
    Interlocutor->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Interlocutor);

    Croco->m_ego_position[0] = XInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    Croco->m_ego_position[1] = YInterlocutor + distanceNat_Gir*(Random::uniform() + 0.5);
    Croco->m_ego_position[2] = -0.1;
    Croco->m_present = 1.0;
    Croco->m_color[0] = Random::uniform(0, 250);
    Croco->m_color[1] = Random::uniform(0, 250);
    Croco->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Croco);

    Mouse->m_ego_position[0] = XInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    Mouse->m_ego_position[1] = YInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    Mouse->m_ego_position[2] = -0.1;
    Mouse->m_present = 1.0;
    Mouse->m_color[0] = Random::uniform(0, 250);
    Mouse->m_color[1] = Random::uniform(0, 250);
    Mouse->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Mouse);

    icub->m_ego_position[0] = 0.0;
    icub->m_ego_position[1] = 0.0;
    icub->m_ego_position[2] = 0.0;
    icub->m_present = 1.0;
    icub->m_color[0] = Random::uniform(0, 80);
    icub->m_color[1] = Random::uniform(180, 250);
    icub->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(icub);

    iCub->opc->addRelation(InterlocutorHasCroco);
    iCub->opc->addRelation(InterlocutorHasMouse);
    iCub->opc->addRelation(iCubWantsCroco);
    yInfo() << " delay...";

    Time::delay(dThresholdDelay + dDelay*Random::uniform());
    Bottle bOption;

    yInfo() << " start recording in ABM";

    yInfo() << " start grasping";
    iCub->take(Croco->m_ego_position, bOption, sObject);

    yInfo() << " end of grasping... delay";
    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " searching for an action";
    if (iCub->getABMClient()->Connect())
    {
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        ostringstream osTmp;
        osTmp << "(predicate have) (agent icub) (object " << sObject << ")";
        string tmp = osTmp.str();
        lArgument.push_back(pair<string, string>(tmp, "goal"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        ostringstream osTmp;
        osTmp << "(predicate have) (agent icub) (object " << sObject << ")";
        string tmp = osTmp.str();
        lArgument.push_back(pair<string, string>(tmp, "goal"));
        ostringstream osTmp2;
        osTmp2 << "(predicate sentence) (speaker icub) (object " << sObject << ")";
        string tmp2 = osTmp2.str();
        lArgument.push_back(pair<string, string>(tmp2, "result"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "needs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "sentence_before"));
        lArgument.push_back(pair<string, string>("speaker#have#object", "sentence_after"));
        lArgument.push_back(pair<string, string>("agent#have#object", "action_before"));
        lArgument.push_back(pair<string, string>("recipient#have#object", "action_after"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "reason",
            "reasoning",
            lArgument,
            false);
    }


    Time::delay(dDelay*Random::uniform());
    yInfo(" iCub ask the ObjStory");

    list<pair<string, string> > lArgument;
    string sentence;
    sentence = "Give me the " + sObject;
    sentence += " please";

    iCub->say(sentence);

    Time::delay(dDelay*Random::uniform());
    yInfo(" Interlocutor gives the ObjError");

    lArgument.clear();

    lArgument.push_back(pair<string, string>(sAgent, "agent"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>(sObjError, "object"));
    lArgument.push_back(pair<string, string>("icub", "recipient"));

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        true);

    yInfo() << " in delay of action";
    Time::delay(2 + 4 * Random::uniform());

    Mouse->m_ego_position[0] = -0.20 - 0.1 * Random::uniform();
    Mouse->m_ego_position[1] = -0.25 + 0.1 * Random::uniform();
    iCub->opc->commit(Mouse);

    iCub->opc->removeRelation(InterlocutorHasMouse);
    iCub->opc->addRelation(iCubHasMouse);

    Time::delay(2.);

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        false);

    // Realisation of the iCub

    Time::delay(2.);
    iCub->say("This is not the " + sObject, false);
    iCub->say("Can you give me the " + sObject + " please", false);


    Time::delay(dDelay*Random::uniform());
    yInfo(" Interlocutor gives the good object");

    lArgument.clear();

    lArgument.push_back(pair<string, string>(sAgent, "agent"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>(sObject, "object"));
    lArgument.push_back(pair<string, string>("icub", "recipient"));

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        true);

    yInfo() << " in delay of action";
    Time::delay(2 + 4 * Random::uniform());

    Croco->m_ego_position[0] = -0.25;
    Croco->m_ego_position[1] = 0.15;
    iCub->opc->commit(Croco);

    iCub->opc->removeRelation(InterlocutorHasCroco);
    iCub->opc->addRelation(iCubHasCroco);
    iCub->opc->removeRelation(iCubWantsCroco);

    Time::delay(2);

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        false);

    iCub->opc->clear();

    return true;
}


/*
*  2 Agents + iCub + 1 step
*/
bool opcPopulater::populateScenario2(){
    string sLarry = "Daniel";
    string sRobert = "Jordi";
    string sObject = "duck";
    string sBox = "box";

    Time::delay(4.);
    // first the ObjStory is close to larry (from left to right)
    iCub->opc->clear();
    iCub->opc->checkout();
    Agent* Robert = iCub->opc->addOrRetrieveEntity<Agent>(sRobert);
    Agent* Larry = iCub->opc->addOrRetrieveEntity<Agent>(sLarry);
    Agent* icub = iCub->opc->addOrRetrieveEntity<Agent>("icub");
    Object* Croco = iCub->opc->addOrRetrieveEntity<Object>(sObject);
    Object* Box = iCub->opc->addOrRetrieveEntity<Object>(sBox);
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Have = iCub->opc->addOrRetrieveEntity<Action>("have");
    Action* Cover = iCub->opc->addOrRetrieveEntity<Action>("cover");
    iCub->opc->commit();

    Relation iCubWantsCroco(icub, Want, Croco);
    Relation iCubHasCroco(icub, Have, Croco);
    Relation BoxCoverCroco(Box, Cover, Croco);
    Relation LarryHaveCroco(Larry, Have, Croco);


    double XInterlocutor = -1.5,
        YInterlocutor = 0.4,
        ZInterlocutor = 0.05,
        distanceNat_Gir = 0.2;

    //int iRepetition = 5;
    double dDelay = 2.;
    double dThresholdDelay = 1.;

    yInfo() << " initialisation of the OPC";

    Robert->m_ego_position[0] = XInterlocutor;
    Robert->m_ego_position[1] = YInterlocutor;
    Robert->m_ego_position[2] = ZInterlocutor;
    Robert->m_present = 1.0;
    Robert->m_color[0] = Random::uniform(0, 80);
    Robert->m_color[1] = Random::uniform(180, 250);
    Robert->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Robert);

    Larry->m_ego_position[0] = XInterlocutor;
    Larry->m_ego_position[1] = -YInterlocutor;
    Larry->m_ego_position[2] = ZInterlocutor;
    Larry->m_present = 1.0;
    Larry->m_color[0] = Random::uniform(0, 80);
    Larry->m_color[1] = Random::uniform(180, 250);
    Larry->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Larry);

    double xPos = XInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    double yPos = distanceNat_Gir*(Random::uniform() - 0.5);

    Croco->m_ego_position[0] = xPos;
    Croco->m_ego_position[1] = yPos;
    Croco->m_ego_position[2] = -0.1;
    Croco->m_present = 1.0;
    Croco->m_color[0] = Random::uniform(0, 250);
    Croco->m_color[1] = Random::uniform(0, 250);
    Croco->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Croco);

    Box->m_ego_position[0] = xPos;
    Box->m_ego_position[1] = yPos;
    Box->m_ego_position[2] = 0.15;
    Box->m_dimensions[0] = 0.25;
    Box->m_dimensions[1] = 0.25;
    Box->m_dimensions[2] = 0.03;
    Box->m_present = 1.0;
    Box->m_color[0] = Random::uniform(0, 250);
    Box->m_color[1] = Random::uniform(0, 250);
    Box->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Box);

    icub->m_ego_position[0] = 0.0;
    icub->m_ego_position[1] = 0.0;
    icub->m_ego_position[2] = 0.0;
    icub->m_present = 1.0;
    icub->m_color[0] = Random::uniform(0, 80);
    icub->m_color[1] = Random::uniform(180, 250);
    icub->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(icub);

    iCub->opc->addRelation(BoxCoverCroco);
    iCub->opc->addRelation(iCubWantsCroco);
    yInfo() << " delay...";
    Bottle bOption;

    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " start recording in ABM";

    yInfo() << " start grasping";
    iCub->take(Croco->m_ego_position, bOption, sObject);

    yInfo() << " end of grasping... delay";
    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    Time::delay(dDelay*Random::uniform());
    yInfo(" iCub ask the ObjStory");

    list<pair<string, string> > lArgument;
    string sentence;
    sentence = sRobert + " can you remove the box please?";

    Time::delay(1.);
    iCub->say(sentence, true, false, "default", true, sRobert);

    lArgument.clear();
    sentence = "Then " + sLarry + " can you give me the " + sObject + " please";

    Time::delay(1.);
    iCub->say(sentence, true, false, "default", true, sLarry);

    Time::delay(dDelay*Random::uniform());
    yInfo(" Interlocutor gives the ObjError");

    lArgument.clear();

    lArgument.push_back(pair<string, string>(sRobert, "agent"));
    lArgument.push_back(pair<string, string>("remove", "predicate"));
    lArgument.push_back(pair<string, string>(sBox, "object"));
    iCub->getABMClient()->sendActivity("action",
        "remove",
        "action",
        lArgument,
        true);
    yInfo() << " in delay of action";
    Time::delay(2.5);


    Box->m_ego_position[1] += 0.75;
    iCub->opc->commit(Box);

    iCub->opc->removeRelation(BoxCoverCroco);
    iCub->opc->addRelation(LarryHaveCroco);


    iCub->getABMClient()->sendActivity("action",
        "remove",
        "action",
        lArgument,
        false);

    Time::delay(dDelay*Random::uniform());
    yInfo(" Interlocutor gives the ObjError");

    lArgument.clear();

    lArgument.push_back(pair<string, string>(sLarry, "agent"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>(sObject, "object"));
    lArgument.push_back(pair<string, string>("icub", "recipient"));

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        true);

    Time::delay(2.5);

    Croco->m_ego_position[0] = -0.35;
    Croco->m_ego_position[1] = 0.1;
    Croco->m_ego_position[2] = 0.;
    iCub->opc->commit(Croco);

    iCub->opc->removeRelation(LarryHaveCroco);
    iCub->opc->addRelation(iCubHasCroco);
    iCub->opc->removeRelation(iCubWantsCroco);

    Time::delay(3);

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        false);

    Time::delay(2.);

    iCub->opc->clear();

    return true;
}


/*
*  iCub + Agent + Box
*  iCub fails, ask to remove, grasp.
*/
bool opcPopulater::populateScenario3(){

    string sAgent = "Daniel";
    string sObject = "blue cube";
    string sBox = "box";

    Time::delay(4.);
    // first the ObjStory is close to larry (from left to right)
    iCub->opc->clear();
    iCub->opc->checkout();
    Agent* Interlocutor = iCub->opc->addOrRetrieveEntity<Agent>(sAgent);
    Agent* icub = iCub->opc->addOrRetrieveEntity<Agent>("icub");
    Object* Croco = iCub->opc->addOrRetrieveEntity<Object>(sObject);
    Object* Box = iCub->opc->addOrRetrieveEntity<Object>(sBox);
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Have = iCub->opc->addOrRetrieveEntity<Action>("have");
    Action* Cover = iCub->opc->addOrRetrieveEntity<Action>("cover");
    iCub->opc->commit();

    Relation iCubWantsCroco(icub, Want, Croco);
    Relation iCubHasCroco(icub, Have, Croco);
    Relation BoxCoverCroco(Box, Cover, Croco);


    double XInterlocutor = -1.5,
        YInterlocutor = 0,
        ZInterlocutor = 0.6,
        distanceNat_Gir = 0.2;

    //int iRepetition = 5;
    double dDelay = 2.5;
    double dThresholdDelay = 1.5;

    yInfo() << " initialisation of the OPC";

    Interlocutor->m_ego_position[0] = XInterlocutor;
    Interlocutor->m_ego_position[1] = YInterlocutor;
    Interlocutor->m_ego_position[2] = ZInterlocutor;
    Interlocutor->m_present = 1.0;
    Interlocutor->m_color[0] = Random::uniform(0, 80);
    Interlocutor->m_color[1] = Random::uniform(180, 250);
    Interlocutor->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Interlocutor);


    double xPos = XInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    double yPos = YInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);

    Croco->m_ego_position[0] = xPos;
    Croco->m_ego_position[1] = yPos;
    Croco->m_ego_position[2] = -0.1;
    Croco->m_present = 1.0;
    Croco->m_color[0] = Random::uniform(0, 250);
    Croco->m_color[1] = Random::uniform(0, 250);
    Croco->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Croco);

    Box->m_ego_position[0] = xPos;
    Box->m_ego_position[1] = yPos;
    Box->m_ego_position[2] = 0.15;
    Box->m_dimensions[0] = 0.25;
    Box->m_dimensions[1] = 0.25;
    Box->m_dimensions[2] = 0.03;
    Box->m_present = 1.0;
    Box->m_color[0] = Random::uniform(0, 250);
    Box->m_color[1] = Random::uniform(0, 250);
    Box->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Box);

    icub->m_ego_position[0] = 0.0;
    icub->m_ego_position[1] = 0.0;
    icub->m_ego_position[2] = 0.0;
    icub->m_present = 1.0;
    icub->m_color[0] = Random::uniform(0, 80);
    icub->m_color[1] = Random::uniform(180, 250);
    icub->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(icub);

    iCub->opc->addRelation(BoxCoverCroco);
    iCub->opc->addRelation(iCubWantsCroco);
    yInfo() << " delay...";
    Bottle bOption;

    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " start recording in ABM";

    yInfo() << " start grasping";
    iCub->take(Croco->m_ego_position, bOption, sObject);

    yInfo() << " end of grasping... delay";
    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    Time::delay(dDelay*Random::uniform());
    yInfo(" iCub ask the ObjStory");

    list<pair<string, string> > lArgument;
    string sentence;
    sentence = "Can you remove the box please?";
    iCub->say(sentence);

    Time::delay(dDelay*Random::uniform());
    yInfo(" Interlocutor gives the ObjError");

    lArgument.clear();

    lArgument.push_back(pair<string, string>(sAgent, "agent"));
    lArgument.push_back(pair<string, string>("remove", "predicate"));
    lArgument.push_back(pair<string, string>(sBox, "object"));

    iCub->getABMClient()->sendActivity("action",
        "remove",
        "action",
        lArgument,
        true);

    yInfo() << " in delay of action";
    Time::delay(2 + 4 * Random::uniform());

    Box->m_ego_position[1] += 0.75;
    iCub->opc->commit(Box);

    iCub->opc->removeRelation(BoxCoverCroco);

    Time::delay(3);

    iCub->getABMClient()->sendActivity("action",
        "remove",
        "action",
        lArgument,
        false);

    // ICUB TAKE THE CROCO

    Time::delay(2.);

    yInfo() << " start grasping";

    if (iCub->getABMClient()->Connect())
    {
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("take", "predicate"));
        lArgument.push_back(pair<string, string>(sObject, "object"));
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("opcPopulater", "provider"));
        lArgument.push_back(pair<string, string>("ARE", "subsystem"));
        iCub->getABMClient()->sendActivity("action", "take", "action", lArgument, true);
    }

    Croco->m_ego_position[0] = -0.15 - 0.1 * Random::uniform();
    Croco->m_ego_position[1] = 0.15 - 0.3 * Random::uniform();
    iCub->opc->commit(Croco);

    iCub->opc->addRelation(iCubHasCroco);
    iCub->opc->removeRelation(iCubWantsCroco);

    Time::delay(4 * Random::uniform());

    if (iCub->getABMClient()->Connect())
    {
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("take", "predicate"));
        lArgument.push_back(pair<string, string>(sObject, "object"));
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("opcPopulater", "provider"));
        lArgument.push_back(pair<string, string>("ARE", "subsystem"));
        lArgument.push_back(pair<string, string>("success", "status"));
        iCub->getABMClient()->sendActivity("action", "take", "action", lArgument, false);
    }

    iCub->opc->clear();


    return true;

}


/*
* Nao just grasp the croco
*/
bool opcPopulater::populateScenario4(){
    string sObject = "duck";

    Time::delay(4.);


    /*
    * iCub wants the croco
    */

    iCub->opc->clear();
    iCub->opc->checkout();
    Agent* icub = iCub->opc->addOrRetrieveEntity<Agent>("icub");
    Object* Croco = iCub->opc->addOrRetrieveEntity<Object>(sObject);
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Have = iCub->opc->addOrRetrieveEntity<Action>("have");
    iCub->opc->commit();

    Relation iCubWantsCroco(icub, Want, Croco);
    Relation iCubHasCroco(icub, Have, Croco);

    double XInterlocutor = -1.5,
        YInterlocutor = 0,
        distanceNat_Gir = 0.2;

    //int iRepetition = 5;
    double dDelay = 2.;
    double dThresholdDelay = 1.5;

    yInfo() << " initialisation of the OPC";

    Croco->m_ego_position[0] = XInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    Croco->m_ego_position[1] = YInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    Croco->m_ego_position[2] = -0.1;
    Croco->m_present = 1.0;
    Croco->m_color[0] = Random::uniform(0, 250);
    Croco->m_color[1] = Random::uniform(0, 250);
    Croco->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Croco);

    icub->m_ego_position[0] = 0.0;
    icub->m_ego_position[1] = 0.0;
    icub->m_ego_position[2] = 0.0;
    icub->m_present = 1.0;
    icub->m_color[0] = Random::uniform(0, 80);
    icub->m_color[1] = Random::uniform(180, 250);
    icub->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(icub);

    iCub->opc->addRelation(iCubWantsCroco);
    yInfo() << " delay...";

    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " start recording in ABM";

    yInfo() << " start grasping";

    if (iCub->getABMClient()->Connect())
    {
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("take", "predicate"));
        lArgument.push_back(pair<string, string>(sObject, "object"));
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("opcPopulater", "provider"));
        lArgument.push_back(pair<string, string>("ARE", "subsystem"));
        iCub->getABMClient()->sendActivity("action", "take", "action", lArgument, true);
    }

    Croco->m_ego_position[0] = -0.15 - 0.1 * Random::uniform();
    Croco->m_ego_position[1] = 0.15 - 0.3 * Random::uniform();
    iCub->opc->commit(Croco);

    iCub->opc->addRelation(iCubHasCroco);
    iCub->opc->removeRelation(iCubWantsCroco);

    Time::delay(4 * Random::uniform());

    if (iCub->getABMClient()->Connect())
    {
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("take", "predicate"));
        lArgument.push_back(pair<string, string>(sObject, "object"));
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("opcPopulater", "provider"));
        lArgument.push_back(pair<string, string>("ARE", "subsystem"));
        lArgument.push_back(pair<string, string>("success", "status"));
        iCub->getABMClient()->sendActivity("action", "take", "action", lArgument, false);
    }

    iCub->opc->clear();


    return true;
}


/*
* iCub + Agent: iCub wants croco, fails, ask, get
*/
bool opcPopulater::populateScenario5(){
    string sAgent = "Tobi";
    string sObject = "octopus";

    Time::delay(4.);
    // first the ObjStory is close to larry (from left to right)
    iCub->opc->clear();
    iCub->opc->checkout();
    Agent* Interlocutor = iCub->opc->addOrRetrieveEntity<Agent>(sAgent);
    Agent* icub = iCub->opc->addOrRetrieveEntity<Agent>("icub");
    Object* Croco = iCub->opc->addOrRetrieveEntity<Object>(sObject);
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Have = iCub->opc->addOrRetrieveEntity<Action>("have");
    iCub->opc->commit();

    Relation InterlocutorHasCroco(Interlocutor, Have, Croco);
    Relation iCubWantsCroco(icub, Want, Croco);
    Relation iCubHasCroco(icub, Have, Croco);


    double XInterlocutor = -1.5,
        YInterlocutor = 0,
        ZInterlocutor = 0.05,
        distanceNat_Gir = 0.2;

    //int iRepetition = 5;
    double dDelay = 2.5;
    double dThresholdDelay = 1.5;

    yInfo() << " initialisation of the OPC";

    Interlocutor->m_ego_position[0] = XInterlocutor;
    Interlocutor->m_ego_position[1] = YInterlocutor;
    Interlocutor->m_ego_position[2] = ZInterlocutor;
    Interlocutor->m_present = 1.0;
    Interlocutor->m_color[0] = Random::uniform(0, 80);
    Interlocutor->m_color[1] = Random::uniform(180, 250);
    Interlocutor->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Interlocutor);

    Croco->m_ego_position[0] = XInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    Croco->m_ego_position[1] = YInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    Croco->m_ego_position[2] = -0.1;
    Croco->m_present = 1.0;
    Croco->m_color[0] = Random::uniform(0, 250);
    Croco->m_color[1] = Random::uniform(0, 250);
    Croco->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Croco);

    icub->m_ego_position[0] = 0.0;
    icub->m_ego_position[1] = 0.0;
    icub->m_ego_position[2] = 0.0;
    icub->m_present = 1.0;
    icub->m_color[0] = Random::uniform(0, 80);
    icub->m_color[1] = Random::uniform(180, 250);
    icub->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(icub);

    iCub->opc->addRelation(InterlocutorHasCroco);
    iCub->opc->addRelation(iCubWantsCroco);
    yInfo() << " delay...";
    Bottle bOption;

    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " start recording in ABM";

    yInfo() << " start grasping";
    iCub->take(Croco->m_ego_position, bOption, sObject);

    yInfo() << " end of grasping... delay";
    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " searching for an action";
    if (iCub->getABMClient()->Connect())
    {
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        ostringstream osTmp;
        osTmp << "(predicate have) (agent icub) (object " << sObject << ")";
        string tmp = osTmp.str();
        lArgument.push_back(pair<string, string>(tmp, "goal"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        ostringstream osTmp;
        osTmp << "(predicate have) (agent icub) (object " << sObject << ")";
        string tmp = osTmp.str();
        lArgument.push_back(pair<string, string>(tmp, "goal"));
        ostringstream osTmp2;
        osTmp2 << "(predicate sentence) (speaker icub) (object " << sObject << ")";
        string tmp2 = osTmp2.str();
        lArgument.push_back(pair<string, string>(tmp2, "result"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "needs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        list<pair<string, string> > lArgument;
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "sentence_before"));
        lArgument.push_back(pair<string, string>("speaker#have#object", "sentence_after"));
        lArgument.push_back(pair<string, string>("agent#have#object", "action_before"));
        lArgument.push_back(pair<string, string>("recipient#have#object", "action_after"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "reason",
            "reasoning",
            lArgument,
            false);
    }


    Time::delay(dDelay*Random::uniform());
    yInfo(" iCub ask the ObjStory");

    list<pair<string, string> > lArgument;
    string sentence;
    sentence = "Give me the " + sObject + " please";
    iCub->say(sentence, false);

    Time::delay(dDelay*Random::uniform());
    yInfo(" Interlocutor gives the ObjError");

    lArgument.clear();

    lArgument.push_back(pair<string, string>(sAgent, "agent"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>(sObject, "object"));
    lArgument.push_back(pair<string, string>("icub", "recipient"));

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        true);

    yInfo() << " in delay of action";
    Time::delay(2 + 2 * Random::uniform());

    Croco->m_ego_position[0] = -0.30;
    Croco->m_ego_position[1] = 0.1;
    iCub->opc->commit(Croco);

    iCub->opc->removeRelation(InterlocutorHasCroco);
    iCub->opc->addRelation(iCubHasCroco);
    iCub->opc->removeRelation(iCubWantsCroco);

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        false);

    // Realisation of the iCub


    Time::delay(2.);

    iCub->opc->clear();

    return true;
}


/*
* 2 agents + iCub + box + 2 steps
*/
bool opcPopulater::populateScenario6(){
    string sLarry = "Daniel";
    string sRobert = "Jordi";
    string sObject = "blue cube";
    string sBox = "box";

    Time::delay(4.);
    // first the ObjStory is close to larry (from left to right)
    iCub->opc->clear();
    iCub->opc->checkout();
    Agent* Robert = iCub->opc->addOrRetrieveEntity<Agent>(sRobert);
    Agent* Larry = iCub->opc->addOrRetrieveEntity<Agent>(sLarry);
    Agent* icub = iCub->opc->addOrRetrieveEntity<Agent>("icub");
    Object* Croco = iCub->opc->addOrRetrieveEntity<Object>(sObject);
    Object* Box = iCub->opc->addOrRetrieveEntity<Object>(sBox);
    Action* Want = iCub->opc->addOrRetrieveEntity<Action>("want");
    Action* Have = iCub->opc->addOrRetrieveEntity<Action>("have");
    Action* Cover = iCub->opc->addOrRetrieveEntity<Action>("cover");
    iCub->opc->commit();

    Relation iCubWantsCroco(icub, Want, Croco);
    Relation iCubHasCroco(icub, Have, Croco);
    Relation BoxCoverCroco(Box, Cover, Croco);
    Relation LarryHaveCroco(Larry, Have, Croco);


    double XInterlocutor = -1.5,
        YInterlocutor = 0.5,
        ZInterlocutor = 0.6,
        distanceNat_Gir = 0.2;

    //int iRepetition = 5;
    double dDelay = 2.5;
    double dThresholdDelay = 1.5;

    yInfo() << " initialisation of the OPC";

    Robert->m_ego_position[0] = XInterlocutor;
    Robert->m_ego_position[1] = YInterlocutor;
    Robert->m_ego_position[2] = ZInterlocutor;
    Robert->m_present = 1.0;
    Robert->m_color[0] = Random::uniform(0, 80);
    Robert->m_color[1] = Random::uniform(180, 250);
    Robert->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Robert);

    Larry->m_ego_position[0] = XInterlocutor;
    Larry->m_ego_position[1] = -YInterlocutor;
    Larry->m_ego_position[2] = ZInterlocutor;
    Larry->m_present = 1.0;
    Larry->m_color[0] = Random::uniform(0, 80);
    Larry->m_color[1] = Random::uniform(180, 250);
    Larry->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(Larry);

    double xPos = XInterlocutor + distanceNat_Gir*(Random::uniform() - 0.5);
    double yPos = distanceNat_Gir*(Random::uniform() - 0.5);

    Croco->m_ego_position[0] = xPos;
    Croco->m_ego_position[1] = yPos;
    Croco->m_ego_position[2] = -0.1;
    Croco->m_present = 1.0;
    Croco->m_color[0] = Random::uniform(0, 250);
    Croco->m_color[1] = Random::uniform(0, 250);
    Croco->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Croco);

    Box->m_ego_position[0] = xPos;
    Box->m_ego_position[1] = yPos;
    Box->m_ego_position[2] = 0.15;
    Box->m_dimensions[0] = 0.25;
    Box->m_dimensions[1] = 0.25;
    Box->m_dimensions[2] = 0.03;
    Box->m_present = 1.0;
    Box->m_color[0] = Random::uniform(0, 250);
    Box->m_color[1] = Random::uniform(0, 250);
    Box->m_color[2] = Random::uniform(0, 250);
    iCub->opc->commit(Box);

    icub->m_ego_position[0] = 0.0;
    icub->m_ego_position[1] = 0.0;
    icub->m_ego_position[2] = 0.0;
    icub->m_present = 1.0;
    icub->m_color[0] = Random::uniform(0, 80);
    icub->m_color[1] = Random::uniform(180, 250);
    icub->m_color[2] = Random::uniform(80, 180);
    iCub->opc->commit(icub);

    iCub->opc->addRelation(BoxCoverCroco);
    iCub->opc->addRelation(iCubWantsCroco);
    yInfo() << " delay...";
    Bottle bOption;


    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " start grasping";
    iCub->take(Croco->m_ego_position, bOption, sObject);

    yInfo() << " end of grasping... delay";
    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    Time::delay(dDelay*Random::uniform());
    yInfo(" iCub ask the ObjStory");

    list<pair<string, string> > lArgument;
    string sentence;
    sentence = "Can you remove the box please " + sRobert + " ?";
    iCub->say(sentence, true, false, "default", true, sRobert);

    Time::delay(dDelay*Random::uniform());
    yInfo(" Interlocutor gives the ObjError");

    lArgument.clear();

    lArgument.push_back(pair<string, string>(sRobert, "agent"));
    lArgument.push_back(pair<string, string>("remove", "predicate"));
    lArgument.push_back(pair<string, string>(sBox, "object"));

    iCub->getABMClient()->sendActivity("action",
        "remove",
        "action",
        lArgument,
        true);

    yInfo() << " in delay of action";
    Time::delay(2 + 4 * Random::uniform());

    Box->m_ego_position[1] += 0.75;
    iCub->opc->commit(Box);

    iCub->opc->removeRelation(BoxCoverCroco);
    iCub->opc->addRelation(LarryHaveCroco);
    Time::delay(2);

    iCub->getABMClient()->sendActivity("action",
        "remove",
        "action",
        lArgument,
        false);


    // TRY TO GRASP

    yInfo() << " start grasping";
    iCub->take(Croco->m_ego_position, bOption, sObject);

    yInfo() << " end of grasping... delay";
    Time::delay(dThresholdDelay + dDelay*Random::uniform());

    yInfo() << " searching for an action";
    if (iCub->getABMClient()->Connect())
    {
        lArgument.clear();
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        ostringstream osTmp;
        osTmp << "(predicate have) (agent icub) (object " << sObject << ")";
        string tmp = osTmp.str();
        lArgument.push_back(pair<string, string>(tmp, "goal"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        lArgument.clear();
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        ostringstream osTmp;
        osTmp << "(predicate have) (agent icub) (object " << sObject << ")";
        string tmp = osTmp.str();
        lArgument.push_back(pair<string, string>(tmp, "goal"));
        ostringstream osTmp2;
        osTmp2 << "(predicate sentence) (speaker icub) (object " << sObject << ")";
        string tmp2 = osTmp2.str();
        lArgument.push_back(pair<string, string>(tmp2, "result"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "needs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        lArgument.clear();
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
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
        lArgument.clear();
        lArgument.push_back(pair<string, string>("icub", "agent"));
        lArgument.push_back(pair<string, string>("reason", "predicate"));
        lArgument.push_back(pair<string, string>("give", "whatIs"));
        lArgument.push_back(pair<string, string>("addressee#have#object", "sentence_before"));
        lArgument.push_back(pair<string, string>("speaker#have#object", "sentence_after"));
        lArgument.push_back(pair<string, string>("agent#have#object", "action_before"));
        lArgument.push_back(pair<string, string>("recipient#have#object", "action_after"));
        lArgument.push_back(pair<string, string>("abmReasoning", "provider"));
        iCub->getABMClient()->sendActivity("action",
            "reason",
            "reasoning",
            lArgument,
            false);
    }


    Time::delay(dDelay*Random::uniform());
    yInfo(" iCub ask the ObjStory");

    lArgument.clear();
    sentence = "Give me the " + sObject + " please " + sLarry;
    iCub->say(sentence, true, false, "default", true, sLarry);

    Time::delay(dDelay*Random::uniform());
    yInfo(" Interlocutor gives the ObjError");

    lArgument.clear();

    lArgument.push_back(pair<string, string>(sLarry, "agent"));
    lArgument.push_back(pair<string, string>("give", "predicate"));
    lArgument.push_back(pair<string, string>(sObject, "object"));
    lArgument.push_back(pair<string, string>("icub", "recipient"));

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        true);

    yInfo() << " in delay of action";
    Time::delay(4 + 4 * Random::uniform());

    Croco->m_ego_position[0] = -0.15 - 0.1 * Random::uniform();
    Croco->m_ego_position[1] = 0.15 - 0.3 * Random::uniform();
    iCub->opc->commit(Croco);

    iCub->opc->removeRelation(LarryHaveCroco);
    iCub->opc->addRelation(iCubHasCroco);
    iCub->opc->removeRelation(iCubWantsCroco);

    Time::delay(3);

    iCub->getABMClient()->sendActivity("action",
        "give",
        "action",
        lArgument,
        false);

    // Realisation of the iCub

    Time::delay(2.);

    iCub->opc->clear();

    return true;
}

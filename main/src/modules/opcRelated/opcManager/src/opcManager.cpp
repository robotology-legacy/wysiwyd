#include <opcManager.h>


/* configure the module, default connection to the OPC called : "OPC" */
bool opcManager::configure(yarp::os::ResourceFinder &rf)
{
    moduleName      =   rf.check("name", 
        Value("opcManager"), 
        "module name (string)").asString().c_str();

    s_realOPC = rf.check("realOpc", Value("OPC")).asString().c_str();
    s_mentalOPC = rf.check("mentalOpc", Value("mentalOPC")).asString().c_str();

    setName(moduleName.c_str());
    string namePortReal = moduleName+ "/" + s_realOPC,
        namePortMental = moduleName + "/" + s_mentalOPC;

    realOPC = new OPCClient(namePortReal.c_str());
    while(!realOPC->isConnected())
    {  
        cout<<"Connecting to " << s_realOPC << "..." <<realOPC->connect(s_realOPC)<<endl;
        Time::delay(0.5);
    }

    mentalOPC = new OPCClient(namePortMental.c_str());
    while(!mentalOPC->isConnected())
    {  
        cout<<"Connecting to " << s_mentalOPC << "..." <<mentalOPC->connect(s_mentalOPC)<<endl;
        Time::delay(0.5);
    }

    realOPC->update();
    mentalOPC->update();

    //Set the verbosity of the OPC communication to false
    realOPC->isVerbose = false;
    mentalOPC->isVerbose = false;

    string handlerPortName = getName() + "/rpc";

    string nameToAbmReasoning = getName() + "/toABMR";

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    portToAbmReasoning.open(nameToAbmReasoning.c_str());
    Network::connect(portToAbmReasoning.getName(), "/efaa/abmReasoning/rpc");

    attach(portToAbmReasoning);
    attach(handlerPort);                  // attach to port

    cout << endl << endl << "----------------------------------------------" << endl << endl << "opcManager ready !" << endl << endl;

    return true ;
}

/* Interrupt the module*/
bool opcManager::interruptModule()
{
    cout << "Interrupting your module, for port cleanup" << endl;

    realOPC->interrupt();
    realOPC->close();

    mentalOPC->interrupt();
    mentalOPC->close();

    handlerPort.interrupt();
    handlerPort.close();

    portToAbmReasoning.interrupt();
    portToAbmReasoning.close();


    return true;
}

/* Close the module*/
bool opcManager::close()
{
    delete realOPC;
    delete mentalOPC;

    return true;
}

/* Respond function */
bool opcManager::respond(const yarp::os::Bottle& bCommand, yarp::os::Bottle& bReply)
{  
    string helpMessage =  string(getName().c_str()) + 
        " commands are: \n" +  
        "help \n" + 
        "connect + name\n" +
        "quit \n" ;

    bReply.clear();
    string keyWord = bCommand.get(0).asString().c_str();

    if (keyWord == "quit") {
        bReply.addString("quitting");
        return false;     
    }
    else if (keyWord=="help") {
        cout << helpMessage;
        bReply.addString("ok");
    }
    else if (keyWord == "connect") {
        bReply = connect(bCommand);
    }
    else if (keyWord == "updateBeliefs") {
        bReply.addString("nack");
        if (bCommand.size() == 2)
        {
            if (bCommand.get(1).isString())
            {
                bReply.addList() = updateBelief(bCommand.get(1).toString().c_str());
            }
        }
    }

    else if (keyWord == "synchronise")
    {
        bReply.addString("ack");
        bReply.addList() = synchoniseOPCs();
    }

    else if (keyWord == "executeActivity")
    {
        bReply.addString("ack");
        bReply.addList() = simulateActivity(bCommand);
    }

    else if (keyWord == "diffOPC")
    {
        bReply.addString("ack");
        bReply.addList() = diffOPC();
    }

    return true;
}


double opcManager::getPeriod()
{
    return 1.0;
}

/* Update loop, check for the behavior allowed */
bool opcManager::updateModule()
{

    return true;
}

/* artificialy populate the OPC */
bool opcManager::populate()
{
    if (!realOPC->isConnected())
    {
        cout << "Error, OPC not connected" <<endl;;
        return false;
    }
    //Clear
    realOPC->clear();

    //Performs a checkout of the existing OPC
    realOPC->checkout();

    //Initialise our entities (retrieved from opc or created and added to the opc)
    Agent* icub = realOPC->addAgent("iCub");

    Drive icub_energy_drive("energy", 1.0, 0.2, 1.0);
    Drive icub_fun_drive("fun", 0.5, 0.7, 1.0);
    Drive icub_social_drive("social", 0.5, 0.7, 1.0);
    Drive icub_knowledge_drive("knowledge", 0.5, 0.7, 1.0);
    icub->m_drives.clear();

    icub->m_drives[icub_energy_drive.name] = icub_energy_drive;
    icub->m_drives[icub_fun_drive.name] = icub_fun_drive;
    icub->m_drives[icub_social_drive.name] = icub_social_drive;
    icub->m_drives[icub_knowledge_drive.name] = icub_knowledge_drive;

    //Modify some properties & commit
    /* starting emotions */

    icub->m_emotions_intrinsic.insert( make_pair( "anger", 0.2 ) ); 
    icub->m_emotions_intrinsic.insert( make_pair( "disgust", 0.2 ) ); 
    icub->m_emotions_intrinsic.insert( make_pair( "fear", 0.2 ) );
    icub->m_emotions_intrinsic.insert( make_pair( "joy", 0.2 ) );
    icub->m_emotions_intrinsic.insert( make_pair( "sadness", 0.2 ) );
    icub->m_emotions_intrinsic.insert( make_pair( "surprise", 0.2 ) );

    realOPC->commit(icub);

    return true;
}

/* Connect the opc client to an OPC */
Bottle opcManager::connect(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() !=2) {
        bOutput.addString("Error in connect, wrong number of input");   }

    if (!bInput.get(1).isString()){
        bOutput.addString("Error in connect, wrong format of input");   }

    realOPC = new OPCClient(moduleName.c_str());
    int iTry = 0;
    while(!realOPC->isConnected())
    {  
        cout<<"Connecting to realOPC..."<<realOPC->connect("OPC")<<endl;
        Time::delay(0.5);
        iTry++;
        if (iTry > 20)
        {
            bOutput.addString("Connection failed, please check your port");
            return bOutput;
        }
    }
    realOPC->update();
    bOutput.addString("Connection done");
    return bOutput;
}


Bottle opcManager::updateBelief(string sOPCname)
{
    cout << "Updating the beliefs in OPC ... " ;

    Bottle bOutput;
    // Create the beliefs
    bool bReal = sOPCname == s_realOPC;

    if (bReal)
    {
        realOPC->checkout();
        realOPC->update();
    }
    else
    {
        mentalOPC->checkout();
        mentalOPC->update();
    }

    Bottle bMessenger;
    bMessenger.addString("updateObjectLocation");
    bMessenger.addString(sOPCname.c_str());
    portToAbmReasoning.write(bMessenger, bMessenger);

    // Set the Bottles for queryOPC to the 

    Bottle isRtobject, conditionAgent, conditionRTO;
    isRtobject.addString(EFAA_OPC_ENTITY_TAG);
    isRtobject.addString("==");
    isRtobject.addString(EFAA_OPC_ENTITY_RTOBJECT);

    Bottle isAgent;
    isAgent.addString(EFAA_OPC_ENTITY_TAG);
    isAgent.addString("==");
    isAgent.addString(EFAA_OPC_ENTITY_AGENT);

    Bottle isPresent;
    isPresent.addString(EFAA_OPC_OBJECT_PRESENT_TAG);
    isPresent.addString("==");
    isPresent.addInt(1);

    conditionAgent.addList() = isAgent;
    conditionAgent.addString("&&");
    conditionAgent.addList() = isPresent;

    conditionRTO.addList() = isRtobject;
    conditionRTO.addString("&&");
    conditionRTO.addList() = isPresent;


    list<Entity*> PresentEntities;
    list<Relation> listRelations;


    // Create the Relations
    Adjective* present;
    Action* is;
    Action* isDoing;

    if (bReal)
    {
        PresentEntities = realOPC->Entities(conditionAgent);
        list<Entity*> tmp = realOPC->Entities(conditionRTO);
        PresentEntities.splice(PresentEntities.end(), tmp);
        listRelations = realOPC->getRelations();
        present = realOPC->addAdjective("isPresent");
        present->m_quality = "presence";
        is = realOPC->addAction("is");
        isDoing = realOPC->addAction("isDoing");
    }
    else
    {
        PresentEntities = mentalOPC->Entities(conditionAgent);
        list<Entity*> tmp = mentalOPC->Entities(conditionRTO);
        PresentEntities.splice(PresentEntities.end(), tmp);
        listRelations = mentalOPC->getRelations();
        present = mentalOPC->addAdjective("isPresent");
        present->m_quality = "presence";
        is = mentalOPC->addAction("is");
        isDoing = mentalOPC->addAction("isDoing");
    }

    for (list<Entity*>::iterator it_E = PresentEntities.begin() ; it_E != PresentEntities.end() ; it_E++)
    {
        if (bReal)
            realOPC->addRelation(*it_E, is, present,time_relation);
        else
            mentalOPC->addRelation(*it_E, is, present,time_relation);
        listRelations.push_back(Relation(*it_E, is, present));
    }

    // get the Agents present

    vector<Relation> vRelToAdd,
        vRelToRemove;

    list<Entity*> PresentAgents;
    if (bReal)
        PresentAgents = (realOPC->Entities(conditionAgent));
    else
        PresentAgents = (mentalOPC->Entities(conditionAgent));

    for (list<Entity*>::iterator it_E = PresentAgents.begin() ; it_E != PresentAgents.end() ; it_E++)
    {
        Agent* TempAgent;

        if (bReal)
            TempAgent = realOPC->addAgent((*it_E)->name());
        else
            TempAgent = mentalOPC->addAgent((*it_E)->name());

        list<Relation> AgentBeliefs = TempAgent->beliefs();
        vRelToAdd.clear();
        vRelToRemove.clear();
        bool bRelPresent = false;

        // Searching the relations to Remove
        // for each previous beliefs of an agent
        for(list<Relation>::iterator it_RAg = AgentBeliefs.begin() ; it_RAg != AgentBeliefs.end() ; it_RAg++)
        {
            // search is the relation is present in the world
            bRelPresent = false;

            //for each relation in the world
            for (list<Relation>::iterator it_RWorl = listRelations.begin() ; it_RWorl != listRelations.end() ; it_RWorl++)
            {   
                if (!bRelPresent)
                {

                    // is the new relation is already known
                    if (it_RAg->toString() == it_RWorl->toString() )
                    {
                        bRelPresent = true;
                    }
                }
            }
            // if the previous relation is no more present
            if (!bRelPresent)
                vRelToRemove.push_back(*it_RAg);
        }


        // Searching the relations to Add
        //for each relation in the world
        for (list<Relation>::iterator it_RWorl = listRelations.begin() ; it_RWorl != listRelations.end() ; it_RWorl++)
        {   
            bRelPresent = false;

            // for each previous beliefs of an agent
            for(list<Relation>::iterator it_RAg = AgentBeliefs.begin() ; it_RAg != AgentBeliefs.end() ; it_RAg++)
            {
                // search is the relation has to be added               
                if (!bRelPresent)
                {
                    // is the new relation is already known
                    if (it_RAg->toString() == it_RWorl->toString() )
                        bRelPresent = true;
                }
            }
            // if the previous relation is no more present
            if (!bRelPresent)
                vRelToAdd.push_back(*it_RWorl);
        }


        // Removing the old relations :
        for (vector<Relation>::iterator it_R = vRelToRemove.begin() ; it_R != vRelToRemove.end() ; it_R++)
        {
            TempAgent->removeBelief(*it_R);
        }


        // Adding the new relations :
        for (vector<Relation>::iterator it_R = vRelToAdd.begin() ; it_R != vRelToAdd.end() ; it_R++)
        {
            TempAgent->addBelief(*it_R);
        }
        if (bReal)
            realOPC->commit(*it_E);
        else
            mentalOPC->commit(*it_E);
    }

    if (bReal)
        realOPC->commit();
    else
        mentalOPC->commit();

    cout << "done" << endl << endl;
    return bOutput; 
}

/* 
Synchonise the content of the OPC in the mentalOPC 
*/
Bottle opcManager::synchoniseOPCs()
{
    cout << endl << "Begining of the synchronisation of the OPCs" << endl;
    Bottle bOutput;

    if (!(realOPC->isConnected() && mentalOPC->isConnected() ))
    {
        cout << "Error in opcManager::synchroniseOPCs : OPC not connected" << endl;
        bOutput.addString("Error in opcManager::synchroniseOPCs : OPC not connected");
        return bOutput;
    }

    realOPC->checkout();
    mentalOPC->checkout();

    list<Entity*> lEntities     = realOPC->EntitiesCacheCopy();
    list<Relation> lRelations   =   realOPC->getRelations();

    //clean GUI :
    list<Entity*> lMental = mentalOPC->EntitiesCacheCopy();
    for (list<Entity*>::iterator it_E = lMental.begin() ; it_E != lMental.end() ; it_E++)
    {
        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_OBJECT)   {
            Object *Ob = mentalOPC->addObject((*it_E)->name());
            Ob->m_present = 0;  }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_AGENT)    {
            Agent *Ag = mentalOPC->addAgent((*it_E)->name());
            Ag->m_present = 0;  }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_RTOBJECT) {
            RTObject *Rt = mentalOPC->addRTObject((*it_E)->name());
            Rt->m_present = 0;  }
    }

    mentalOPC->commit();
    Time::delay(time_action);
    mentalOPC->clear();
    mentalOPC->checkout();

    for (list<Entity*>::iterator it_E = lEntities.begin() ; it_E != lEntities.end() ; it_E++)
    {
        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_OBJECT)   {
            Object *Ob = mentalOPC->addObject((*it_E)->name());
            Ob->fromBottle((*it_E)->asBottle());    }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_AGENT)    {
            Agent *Ag = mentalOPC->addAgent((*it_E)->name());
            Ag->fromBottle((*it_E)->asBottle());    }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_RTOBJECT) {
            RTObject *Rt = mentalOPC->addRTObject((*it_E)->name());
            Rt->fromBottle((*it_E)->asBottle());    }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_ADJECTIVE)    {
            Adjective *Ad = mentalOPC->addAdjective((*it_E)->name());
            Ad->fromBottle((*it_E)->asBottle());    }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_ACTION)   {
            Action *Ac = mentalOPC->addAction((*it_E)->name());
            Ac->fromBottle((*it_E)->asBottle());    }   
    }
    mentalOPC->commit();

    for (list<Relation>::iterator it_R = lRelations.begin() ; it_R != lRelations.end() ; it_R++)
    {
        mentalOPC->addRelation(*it_R);
    }

    mentalOPC->update();

    cout << "Synchronisation done" << endl; 
    bOutput.addString("synchronisation done.");

    return bOutput;
}

/*
Simulate an action in the mentalOPC

1- ask the action to abmReasoning
2- execute the action the mentalOPC

* @param bInput : move relative (coordX coordY) (action action_name (agrument 1 to n ) (role 1 to n) )

(example : move relative (0.08066 -0.22343) (action push (east Toy agent1) (spatial1 object1 agent1) ) )
*/
Bottle opcManager::simulateAction(Bottle bAction)
{
    Bottle bArgument, bRole, bActivity, bOutput;

    bool fAbsolut = (bAction.get(1).toString() == "absolut");
    pair<double, double> pMove;
    Bottle bMove = *(bAction.get(2).asList());

    pMove.first  = bMove.get(0).asDouble();
    pMove.second = bMove.get(1).asDouble();

    bActivity = *(bAction.get(3).asList());

    bArgument = *(bActivity.get(2).asList());
    bRole = *(bActivity.get(3).asList());

    string sObject;
    bool fObject = false;

    for (int i = 0 ; i < bRole.size() ; i++)
    {
        if (bRole.get(i).toString() == "object1")   {
            sObject = bArgument.get(i).toString().c_str();
            fObject = true; }
    }

    if (!fObject)   {
        bOutput.addString("Error in opcManager::simulateAction | object not found");
        return bOutput; }

    mentalOPC->update();

    RTObject *OBJECT =  mentalOPC->addRTObject(sObject);

    if (fAbsolut)   {
        OBJECT->m_ego_position[0] = pMove.first;
        OBJECT->m_ego_position[1] = pMove.second;   }
    else    {
        OBJECT->m_ego_position[0] = OBJECT->m_ego_position[0] + pMove.first;
        OBJECT->m_ego_position[1] = OBJECT->m_ego_position[1] + pMove.second;   }

    mentalOPC->commit();

    return bAction;
}

/*
Simulate an activity in the mentalOPC

1- ask the action to abmReasoning
2- execute the action the mentalOPC

* @param bInput : Bottle ("executeAction"       actionName      argument        object      agent)
*/
Bottle opcManager::simulateActivity(Bottle bInput)
{
    Bottle bAction, bMessenger, bArgument, bRole, bActivity, bOutput;
    portToAbmReasoning.write(bInput, bMessenger);

    bActivity = *(bMessenger.get(1).asList());

    for (int i = 0 ; i < bActivity.size(); i++)
    {
        bAction = *bActivity.get(i).asList();
        simulateAction(bAction);
        Time::delay(time_action);
    }

    updateBelief(s_mentalOPC);

    return bMessenger;
}

/*
Return the differences between the 2 opcs
*/
Bottle opcManager::diffOPC()
{
    cout << endl << "**************************************** " << endl << "Searching for differencies between mental and real OPC " << endl << endl;

    // Snapshot of the 2 opc
    Bottle bOutput, bReal, bMental, bDiff;

    realOPC->checkout();
    mentalOPC->checkout();

    updateBelief(s_mentalOPC);
    updateBelief(s_realOPC);
    // Set the Bottles for queryOPC to the 

    Bottle isRtobject, condition;
    isRtobject.addString(EFAA_OPC_ENTITY_TAG);
    isRtobject.addString("==");
    isRtobject.addString(EFAA_OPC_ENTITY_RTOBJECT);

    Bottle isAgent;
    isAgent.addString(EFAA_OPC_ENTITY_TAG);
    isAgent.addString("==");
    isAgent.addString(EFAA_OPC_ENTITY_AGENT);

    Bottle isPresent;
    isPresent.addString(EFAA_OPC_OBJECT_PRESENT_TAG);
    isPresent.addString("==");
    isPresent.addInt(1);

    condition.addList() = isPresent;

    list<Entity*>       RealEntities    = (realOPC->EntitiesCacheCopy());
    list<Entity*>       MentalEntities  = (mentalOPC->EntitiesCacheCopy());

    list<Relation>      RealRelations = realOPC->getRelations();
    list<Relation>      MentalRelations = mentalOPC->getRelations();

    vector<Relation> vRelRealnotMental,
        vRelMentalnotReal;

    Bottle bEnRnM,      // Bottle with the entities real and not mental
        bEnMnR;         // Bottle with the entities mental and not real

    bEnRnM.addString("Entities Real and not Mental");
    bEnMnR.addString("Entities Mental and not Real");

    /// For each entity present in the real OPC
    for (list<Entity*>::iterator it_Real = RealEntities.begin() ; it_Real != RealEntities.end() ; it_Real ++)
    {
        bool fEntity = false;
        /// search for it in the mental OPC
        for (list<Entity*>::iterator it_Mental = MentalEntities.begin() ; it_Mental != MentalEntities.end() ; it_Mental ++)
        {
            if (!fEntity && (*it_Real)->name() == (*it_Mental)->name())
            {
                fEntity = true;
                Bottle bDiffTemp = OPCEARS.getDifferencies(*it_Mental, *it_Real);
                if (bDiffTemp.toString() != "none" && bDiffTemp.toString() != "" )
                {
                    cout << "this object changed : " << (*it_Real)->name() << " (" << (*it_Real)->entity_type() << ")" << endl << bDiffTemp.toString() << endl << endl;
                    bDiff.addList() = bDiffTemp;
                }
            }
        }

        if (!fEntity && (*it_Real)->entity_type() != EFAA_OPC_ENTITY_ADJECTIVE)
        {
            Bottle bEntityTemp;
            bEntityTemp.addString((*it_Real)->name().c_str());
            bEntityTemp.addString((*it_Real)->entity_type().c_str());
            bEnRnM.addList() = bEntityTemp;
        }
    }


    /// For each entity present in the mental OPC
    for (list<Entity*>::iterator it_Mental = MentalEntities.begin() ; it_Mental != MentalEntities.end() ; it_Mental ++)
    {
        bool fEntity = false;
        /// search for it in the real OPC
        for (list<Entity*>::iterator it_Real = RealEntities.begin() ; it_Real != RealEntities.end() ; it_Real ++)
        {
            if (!fEntity && (*it_Real)->name() == (*it_Mental)->name())
            {
                fEntity = true;
            }
        }

        if (!fEntity && (*it_Mental)->entity_type() != EFAA_OPC_ENTITY_ADJECTIVE )
        {
            Bottle bEntityTemp;
            bEntityTemp.addString((*it_Mental)->name().c_str());
            bEntityTemp.addString((*it_Mental)->entity_type().c_str());
            bEnMnR.addList() = bEntityTemp;
        }
    }


    bOutput.addList() = bDiff;
    if (bEnMnR.size() !=1)
    {
        cout << bEnMnR.toString() << endl << endl;
        bOutput.addList() = bEnMnR;
    }
    if (bEnRnM.size() !=1)
    {
        cout << bEnRnM.toString() << endl << endl;
        bOutput.addList() = bEnRnM;
    }

    return bOutput;
}

/*
*   Return the list of beliefs of an agent in an OPC given
*   bInput format : getBeliefs real/mental  agent
*/
Bottle opcManager::getBeliefs(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.size() !=3)
    {
        cout << "Error in opcManager::getBeliefs | wrong size of input" << endl;
        bOutput.addString("Error in opcManager::getBeliefs | wrong size of input");
        return bOutput;
    }

    if (bInput.get(1).toString() != "real" && bInput.get(1).toString() != "mental" )
    {
        cout << "Error in opcManager::getBeliefs | unknown OPC (real/mental)" << endl;
        bOutput.addString("Error in opcManager::getBeliefs | unknown OPC (real/mental)");
        return bOutput;
    }

    Agent *agent;
    if (bInput.get(1).toString() == "real")
    {
        agent = realOPC->addAgent(bInput.get(2).toString().c_str());
    }
    else
    {
        agent = mentalOPC->addAgent(bInput.get(2).toString().c_str());
    }

    list<Relation> lRelation = agent->beliefs();

    cout << endl << agent->name() << " has the following beliefs in the " << bInput.get(1).toString() << " OPC (" << lRelation.size() << ") : " << endl;

    for (list<Relation>::iterator it_R = lRelation.begin() ; it_R != lRelation.end() ; it_R++)
    {
        Bottle bTemp = it_R->asLightBottle();
        cout << bTemp.toString() << endl;
        bOutput.addList() = bTemp;
    }

    cout << endl ;

    return bOutput;
}

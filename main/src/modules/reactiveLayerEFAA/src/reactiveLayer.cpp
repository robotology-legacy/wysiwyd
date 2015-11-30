#include "reactiveLayer.h"

bool ReactiveLayer::close()
{
    //iCub->getReactableClient()->SendOSC(yarp::os::Bottle("/event reactable pong stop"));
    iCub->close();
    delete iCub;
    return true;
}

bool ReactiveLayer::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("ReactiveLayer")).asString().c_str();
    setName(moduleName.c_str());

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName,"reactiveLayer","client.ini",isRFVerbose);
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

    //Set the voice
    std::string ttsOptions = rf.check("ttsOptions", yarp::os::Value("iCubina 85.0")).asString();
    if (iCub->getSpeechClient())
        iCub->getSpeechClient()->SetOptions(ttsOptions);

    //Configure the various components
    configureOPC(rf);
    configureAllostatic(rf);
    configureTactile(rf);
    configureSalutation(rf);

    cout<<"Configuration done."<<endl;

    rpc.open ( ("/"+moduleName+"/rpc").c_str());
    attach(rpc);

    //Initialise timers
    lastFaceUpdate = Time::now();
    physicalInteraction = false;
    someonePresent = false;

    //iCub->getReactableClient()->SendOSC(yarp::os::Bottle("/event reactable pong start"));

    return true;
}

void ReactiveLayer::configureTactile(yarp::os::ResourceFinder &rf)
{
    //Initialise the tactile response
    Bottle grpTactile = rf.findGroup("TACTILE");
    Bottle *tactileStimulus = grpTactile.find("stimuli").asList();

    if (tactileStimulus)
    {
        for (int d = 0; d<tactileStimulus->size(); d++)
        {
            string tactileStimulusName = tactileStimulus->get(d).asString().c_str();
            StimulusEmotionalResponse response;
            Bottle * bSentences = grpTactile.find((tactileStimulusName + "-sentence").c_str()).asList();
            for (int s = 0; s<bSentences->size(); s++)
            {
                response.m_sentences.push_back(bSentences->get(s).asString().c_str());
            }
            //choregraphies
            Bottle *bChore = grpTactile.find((tactileStimulusName + "-chore").c_str()).asList();
            for (int sC = 0; bChore && sC<bChore->size(); sC++)
            {
                response.m_choregraphies.push_back(bChore->get(sC).asString().c_str());
            }
            std::string sGroupTemp = tactileStimulusName;
            sGroupTemp += "-effect";
            Bottle *bEffects = grpTactile.find(sGroupTemp.c_str()).asList();
            for (int i = 0; bEffects && i<bEffects->size(); i += 2)
            {
                response.m_emotionalEffect[bEffects->get(i).asString().c_str()] = bEffects->get(i + 1).asDouble();
            }
            tactileEffects[tactileStimulusName] = response;
        }
    }
}

void ReactiveLayer::configureSalutation(yarp::os::ResourceFinder &rf)
{
    ;
    //Initialise the gestures response
    Bottle grpSocial = rf.findGroup("SOCIAL");
    salutationLifetime = grpSocial.check("salutationLifetime", Value(15.0)).asDouble();
    Bottle *socialStimulus = grpSocial.find("stimuli").asList();

    if (socialStimulus)
    {
        for (int d = 0; d<socialStimulus->size(); d++)
        {
            string socialStimulusName = socialStimulus->get(d).asString().c_str();
            StimulusEmotionalResponse response;
            Bottle * bSentences = grpSocial.find((socialStimulusName + "-sentence").c_str()).asList();
            for (int s = 0; s<bSentences->size(); s++)
            {
                response.m_sentences.push_back(bSentences->get(s).asString().c_str());
            }
            std::string sGroupTemp = socialStimulusName;
            sGroupTemp += "-effect";
            Bottle *bEffects = grpSocial.find(sGroupTemp.c_str()).asList();
            for (int i = 0; bEffects && i<bEffects->size(); i += 2)
            {
                response.m_emotionalEffect[bEffects->get(i).asString().c_str()] = bEffects->get(i + 1).asDouble();
            }
            salutationEffects[socialStimulusName] = response;
        }
    }

    //Add the relevant Entities for handling salutation
    iCub->opc->addOrRetrieveAction("is");
    iCub->opc->addOrRetrieveAdjective("saluted");
}

void ReactiveLayer::configureAllostatic(yarp::os::ResourceFinder &rf)
{
    //Initialise the iCub allostatic model
    cout << "Initializing drives";
    Bottle grpAllostatic = rf.findGroup("ALLOSTATIC");
    Bottle *drivesList = grpAllostatic.find("drives").asList();
    iCub->icubAgent->m_drives.clear();
    if (drivesList)
    {
        for (int d = 0; d<drivesList->size(); d++)
        {
            string driveName = drivesList->get(d).asString().c_str();
            Drive drv;
            drv.name = driveName;
            drv.homeoStasisMin = grpAllostatic.check((driveName + "-homeostasisMin").c_str(), Value(0.25)).asDouble();
            drv.homeoStasisMax = grpAllostatic.check((driveName + "-homeostasisMax").c_str(), Value(0.75)).asDouble();
            drv.decay = grpAllostatic.check((driveName + "-decay").c_str(), Value(0.05)).asDouble();
            drv.value = (drv.homeoStasisMax + drv.homeoStasisMin) / 2.0;
            iCub->icubAgent->m_drives[driveName] = drv;

            //Under effects
            StimulusEmotionalResponse responseUnder;
            Bottle * bSentences = grpAllostatic.find((driveName + "-under-sentences").c_str()).asList();
            for (int s = 0; bSentences && s<bSentences->size(); s++)
            {
                responseUnder.m_sentences.push_back(bSentences->get(s).asString().c_str());
            }
            Bottle *bChore = grpAllostatic.find((driveName + "-under-chore").c_str()).asList();
            for (int sC = 0; bChore && sC<bChore->size(); sC++)
            {
                responseUnder.m_choregraphies.push_back(bChore->get(sC).asString().c_str());
            }
            homeostaticUnderEffects[driveName] = responseUnder;

            //Over effects
            StimulusEmotionalResponse responseOver;
            bSentences = grpAllostatic.find((driveName + "-over-sentences").c_str()).asList();
            for (int s = 0; bSentences&& s<bSentences->size(); s++)
            {
                responseOver.m_sentences.push_back(bSentences->get(s).asString().c_str());
            }
            bChore = grpAllostatic.find((driveName + "-over-chore").c_str()).asList();
            for (int sC = 0; bChore && sC<bChore->size(); sC++)
            {
                responseOver.m_choregraphies.push_back(bChore->get(sC).asString().c_str());
            }
            homeostaticOverEffects[driveName] = responseOver;
        }
    }
    cout << "done" << endl;

    //Initialise the iCub emotional model
    cout << "Initializing emotions...";
    Bottle grpEmotions = rf.findGroup("EMOTIONS");
    Bottle *emotionsList = grpEmotions.find("emotions").asList();
    double emotionalDecay = grpEmotions.check("emotionalDecay", Value(0.1)).asDouble();

    iCub->icubAgent->m_emotions_intrinsic.clear();
    if (emotionsList)
    {
        for (int d = 0; d<emotionsList->size(); d++)
        {
            string emoName = emotionsList->get(d).asString().c_str();
            iCub->icubAgent->m_emotions_intrinsic[emoName] = 0.0;
        }
    }
    cout << "done" << endl;

    faceUpdatePeriod = grpEmotions.check("expressionUpdatePeriod", Value(0.5)).asDouble();

    cout << "Commiting iCubAgent...";
    iCub->commitAgent();
    cout << "done." << endl;

    InternalVariablesDecay* decayThread;
    decayThread = new InternalVariablesDecay(500, emotionalDecay);
    decayThread->start();
}

bool ReactiveLayer::updateModule()
{
    cout<<".";

    handleSalutation(someonePresent);
    physicalInteraction = handleTactile();
    updateAllostatic();
    updateEmotions();

    return true;
}

bool ReactiveLayer::handleTactile()
{
    bool gotSignal = false;
    list<Relation> tactileRelations = iCub->opc->getRelationsMatching("icub","is","any","touchLocation");

    if (tactileRelations.size() > 0)
    {
        cout<<"I am touched"<<endl;
        Relation r = *tactileRelations.begin();
        //Look at the place where it has been touched
        Object* touchLocation = dynamic_cast<Object*>(iCub->opc->getEntity("touchLocation"));
        iCub->opc->update(touchLocation);
        touchLocation->m_present = true;
        iCub->opc->commit(touchLocation);

        iCub->look("touchLocation");//r.complement_place().c_str());
        Time::delay(1.0);

        //If we have a stored reaction pattern to this tactile stimulus
        if (tactileEffects.find(r.object()) != tactileEffects.end() )
        {
            iCub->say(tactileEffects[r.object()].getRandomSentence(), false);
            //Apply each emotional effect
            for(map<string, double>::iterator itEffects = tactileEffects[r.object()].m_emotionalEffect.begin() ; itEffects != tactileEffects[r.object()].m_emotionalEffect.end(); itEffects++)
            {
                iCub->icubAgent->m_emotions_intrinsic[itEffects->first] += itEffects->second;
                iCub->icubAgent->m_emotions_intrinsic[itEffects->first] = min(1.0, max(0.0,itEffects->second));
            }
            //play choreography
            if(yarp::os::Random::uniform()>0.33)
            {
                string randomChoregraphy = tactileEffects[r.object()].getRandomChoregraphy();
                iCub->playBodyPartChoregraphy(randomChoregraphy.c_str(), "left_arm",1.0,false);
                iCub->playBodyPartChoregraphy(randomChoregraphy.c_str(), "right_arm",1.0,true);
                cout << "random Choreography Chosen: " << randomChoregraphy.c_str() << endl;
            }
        }
        iCub->commitAgent();

        iCub->opc->removeRelation(r);
        touchLocation->m_present = false;
        iCub->opc->commit(touchLocation);
        iCub->look("partner");
        iCub->lookAround();
        gotSignal = true;
    }
    return gotSignal;
}


bool ReactiveLayer::handleSalutation(bool& someoneIsPresent)
{
    someoneIsPresent = false;
    //Handle the salutation of newcomers
    list<Entity*> allAgents = iCub->opc->Entities(EFAA_OPC_ENTITY_TAG, "==", EFAA_OPC_ENTITY_AGENT);
    list<Relation> salutedAgents = iCub->opc->getRelations("saluted");
    list<Relation> identity = iCub->opc->getRelationsMatching("partner", "named");
    string identityName = "unknown";
    if (identity.size() > 0)
        identityName = identity.front().object();

    for (list<Entity*>::iterator currentAgentIt = allAgents.begin(); currentAgentIt != allAgents.end(); currentAgentIt++)
    {
        Agent* currentAgent = dynamic_cast<Agent*>(*currentAgentIt);


        if (currentAgent->name() != "icub" && currentAgent->m_present)
        {
            someoneIsPresent = true;
            string currentPartner = currentAgent->name();
            //cout<<"Testing salutation for "<<currentPartner<<" with name "<<identityName<<endl;

            bool saluted = false;
            for (list<Relation>::iterator it = salutedAgents.begin(); it != salutedAgents.end(); it++)
            {
                //cout<< it->toString()<<endl;
                if (it->subject() == identityName)
                {
                    //cout<<"Same agent detected... Reseting salutation lifetime"<<endl;
                    //This guy has already been saluted, we reset the lifetime
                    iCub->opc->setLifeTime(it->ID(), salutationLifetime);
                    saluted = true;
                }
            }

            if (!saluted)
            {
                iCub->look(currentPartner);
                iCub->say(salutationEffects["humanEnter"].getRandomSentence(), false);
                iCub->playChoregraphy("wave");
                if (identityName != "unknown")
                    iCub->say(identityName + "! nice to see you again!", false);
                iCub->opc->addRelation(Relation(identityName, "is", "saluted"), salutationLifetime);
                return true;
            }
        }
    }
    return false;
}



bool ReactiveLayer::updateAllostatic()
{
    iCub->updateAgent();

    //Update some specific drives based on the previous stimuli encountered
    if (physicalInteraction)
        iCub->icubAgent->m_drives["physicalInteraction"].value += 0.1;
    if (someonePresent)
        iCub->icubAgent->m_drives["socialInteraction"].value += iCub->icubAgent->m_drives["socialInteraction"].decay * 2;

    //Trigger drive related sentences
    for (map<string, Drive>::iterator d = iCub->icubAgent->m_drives.begin(); d != iCub->icubAgent->m_drives.end(); d++)
    {
        //Check under homeostasis
        if (d->second.value < d->second.homeoStasisMin)
        {
            iCub->say(homeostaticUnderEffects[d->first].getRandomSentence());
            d->second.value += (d->second.homeoStasisMax - d->second.homeoStasisMin) / 3.0;
        }
        //Check over homeostasis
        if (d->second.value > d->second.homeoStasisMax)
        {
            iCub->say(homeostaticOverEffects[d->first].getRandomSentence());
            d->second.value -= (d->second.homeoStasisMax - d->second.homeoStasisMin) / 3.0;;
        }
    }
    iCub->commitAgent();
    return true;
}

bool ReactiveLayer::updateEmotions()
{
    //Expresses the maximum emotion
    string maxEmotion; double emotionalValue;
    iCub->getHighestEmotion(maxEmotion, emotionalValue);
    if (lastFaceUpdate + faceUpdatePeriod<Time::now())
    {
        iCub->getExpressionClient()->express(maxEmotion, emotionalValue);
        lastFaceUpdate = Time::now();
        //cout<<"Expressing "<<maxEmotion<<" at "<<emotionalValue<<endl;
    }
    return true;
}

void ReactiveLayer::configureOPC(yarp::os::ResourceFinder &rf)
{
    //Populate the OPC if required
    cout<<"Populating OPC";
    Bottle grpOPC = rf.findGroup("OPC");
    bool shouldPopulate = grpOPC.find("populateOPC").asInt() == 1;
    if (shouldPopulate)
    {
        Bottle *agentList = grpOPC.find("agent").asList();
        if (agentList)
        {
            for(int d=0; d<agentList->size(); d++)
            {
                string name = agentList->get(d).asString().c_str();
                Agent* agent = iCub->opc->addOrRetrieveAgent(name);
                agent->m_present = false;
                iCub->opc->commit(agent);
            }
        }

        Bottle *objectList = grpOPC.find("object").asList();
        if (objectList)
        {
            for(int d=0; d<objectList->size(); d++)
            {
                string name = objectList->get(d).asString().c_str();
                Object* o = iCub->opc->addOrRetrieveObject(name);
                o->m_present = false;
                iCub->opc->commit(o);
            }
        }

        Bottle *rtobjectList = grpOPC.find("rtobject").asList();
        if (rtobjectList)
        {
            for(int d=0; d<rtobjectList->size(); d++)
            {
                string name = rtobjectList->get(d).asString().c_str();
                RTObject* o = iCub->opc->addOrRetrieveRTObject(name);
                o->m_present = false;
                iCub->opc->commit(o);
            }
        }

        Bottle *adjectiveList = grpOPC.find("adjective").asList();
        if (adjectiveList)
        {
            for(int d=0; d<adjectiveList->size(); d++)
            {
                string name = adjectiveList->get(d).asString().c_str();
                iCub->opc->addOrRetrieveAdjective(name);
            }
        }

        Bottle *actionList = grpOPC.find("action").asList();
        if (actionList)
        {
            for(int d=0; d<actionList->size(); d++)
            {
                string name = actionList->get(d).asString().c_str();
                iCub->opc->addOrRetrieveAction(name);
            }
        }
    }
    cout<<"done"<<endl;
}


bool ReactiveLayer::respond(const Bottle& cmd, Bottle& reply)
{
    reply.addString("NACK");
    return true;
}

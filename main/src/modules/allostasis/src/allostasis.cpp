#include "allostasis.h"
#include <cmath>


bool AlostaticModule::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("allostasis")).asString().c_str();
    setName(moduleName.c_str());

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName,"allostasis","client.ini",isRFVerbose);
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
    distanceList.clear();
    windowSize = rf.check("windowSize",Value(20)).asInt();
    for (int i =0;i<windowSize;i++)
    {
    	distanceList.addDouble(0.0);
    }
    meanx=windowSize/2;
    meany=0;
    cov=0;
    varx=0;
    for (int n = 0; n<meanx;n++)
    {	
    	varx += pow((2*n +1),2.0);
    }
    varx /=2;
    derivative = cov/varx;

    //open input ports
    std::string targetPortIn = rf.check("targetPort",Value("/" + moduleName + "/reachingTarget:i")).asString().c_str();
    std::string avoidPortIn = rf.check("avoidPort", Value("/" + moduleName + "/avoidanceVectors:i")).asString().c_str();
    std::string eePortIn = rf.check("endEffectorPort", Value("/" + moduleName + "/rightArmState:i")).asString().c_str();

    targetPort.open(targetPortIn);
    avoidPort_i.open(avoidPortIn);
    endEffectorPort.open(eePortIn);


    //Open output ports
    std::string reachPortOut = "/" + moduleName + "reachingGain:o";
    std::string avoidPortOut = "/" + moduleName + "avoidanceGain:o";
    std::string explorePortOut = "/" + moduleName + "explorationGain:o";

    reachPort.open(reachPortOut);
    avoidPort_o.open(avoidPortOut);
    explorePort.open(explorePortOut);

    //Set the voice
    //std::string ttsOptions = rf.check("ttsOptions", yarp::os::Value("iCubina 85.0")).asString();
    //if (iCub->getSpeechClient())
    //    iCub->getSpeechClient()->SetOptions(ttsOptions);

    //Configure the various components
	configureOPC(rf);
	configureAllostatic(rf);
/*
	configureTactile(rf);
	configureSalutation(rf);
*/
    

    rpc.open ( ("/"+moduleName+"/rpc").c_str());
    attach(rpc);

	//Initialise timers

    //lastFaceUpdate = Time::now();
/*	physicalInteraction = false;
	someonePresent = false;
*/
	cout<<"Configuration done."<<endl;
    return true;
}

void AlostaticModule::configureAllostatic(yarp::os::ResourceFinder &rf)
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
		}
	}
	cout << "done" << endl;
	
	//Initialise the iCub emotional model
	//Actually, this is a vestigial piece of code that probably does nothing related to allostasis
	cout << "Initializing emotions...";
	Bottle grpEmotions = rf.findGroup("EMOTIONS");
	//Bottle *emotionsList = grpEmotions.find("emotions").asList();
	double emotionalDecay = grpEmotions.check("emotionalDecay", Value(0.1)).asDouble();

	cout << "Commiting iCubAgent...";
	iCub->commitAgent();
	cout << "done." << endl;

	InternalVariablesDecay* decayThread;
	decayThread = new InternalVariablesDecay(500, emotionalDecay);
	decayThread->start();
}

bool AlostaticModule::updateModule()
{
    cout<<"."<<endl;
    //Read target position
    Bottle target = *targetPort.read();

    //Read Avoidance vectors
    Bottle obstacleList = *avoidPort_i.read();

    //Read End-effector position
    Bottle endEffector = *endEffectorPort.read();
    //Compute distance to target
    double squared_distance = pow(target.get(0).asDouble() - endEffector.get(0).asDouble(),2.0) + pow(target.get(1).asDouble() - endEffector.get(1).asDouble(),2.0) + pow(target.get(2).asDouble() - endEffector.get(2).asDouble(),2.0);
    
    distanceList.addDouble(sqrt(squared_distance));
    if (distanceList.size() >= windowSize)
    	distanceList = distanceList.tail();

    //compute avoidance module
    //for (int i = 0;i)
    int i = 0;
    Bottle *obstacle = obstacleList.get(i).asList();
    obstacle_distance = sqrt(pow(obstacle->get(0).asDouble(),2.0) + pow(obstacle->get(1).asDouble(),2.0) + pow(obstacle->get(2).asDouble(),2.0));
    if (obstacle_distance>0)
    {
    	obstacle_distance = 1/obstacle_distance;
	}else{
		obstacle_distance = 100;
	}
    //Compute success rate -> Linear Regression
    meany = meany*(windowSize-1)/windowSize + (1/windowSize)*squared_distance;


	updateAllostatic();
	
	//write reachGain
	Bottle &gr = reachPort.prepare();
	gr.clear();
	gr.addDouble(iCub->icubAgent->m_drives["reaching"].value);
	
	//write avoidGain
	Bottle &ga = avoidPort_o.prepare();
	ga.clear();
	ga.addDouble(iCub->icubAgent->m_drives["avoiding"].value);
	avoidPort_o.write();
	//write exploreGain
	Bottle &ge = explorePort.prepare();
	ge.clear();
	ge.addDouble(iCub->icubAgent->m_drives["exploring"].value);
	

	reachPort.write();

	explorePort.write();

	//updateEmotions();
    return true;
}


bool AlostaticModule::updateAllostatic()
{
	iCub->updateAgent();
	iCub->icubAgent->m_drives["reaching"].value = distanceList.get(windowSize-1).asDouble();
	iCub->icubAgent->m_drives["avoiding"].value = obstacle_distance;
	//iCub->icubAgent->m_drives["avoiding"].value = 

	//Update some specific drives based on the previous stimuli encountered
	/*if (physicalInteraction)
		iCub->icubAgent->m_drives["physicalInteraction"].value += 0.1;
	if (someonePresent)
		iCub->icubAgent->m_drives["socialInteraction"].value += iCub->icubAgent->m_drives["socialInteraction"].decay * 2;
	*/



	/*
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
	*/
	iCub->commitAgent();
	return true;
}

bool AlostaticModule::updateEmotions()
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

void AlostaticModule::configureOPC(yarp::os::ResourceFinder &rf)
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
                Agent* agent = iCub->opc->addAgent(name);
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
                Object* o = iCub->opc->addObject(name);
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
                RTObject* o = iCub->opc->addRTObject(name);
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
                iCub->opc->addAdjective(name);
            }
        }

        Bottle *actionList = grpOPC.find("action").asList();
        if (actionList)
        {
            for(int d=0; d<actionList->size(); d++)
            {
                string name = actionList->get(d).asString().c_str();
                iCub->opc->addAction(name);
            }
        }
    }
    cout<<"done"<<endl;
}


bool AlostaticModule::respond(const Bottle& cmd, Bottle& reply)
{
	reply.addString("NACK");
    return true;
}

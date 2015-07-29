#include <algorithm>    // std::random_shuffle
#include "reactiveLayer.h"

bool ReactiveLayer::close()
{
	//iCub->getReactableClient()->SendOSC(yarp::os::Bottle("/event reactable pong stop"));
	iCub->close();
    delete iCub;
    return true;
}

int ReactiveLayer::openPorts(string driveName,int d)
{
	rpc_ports.resize(rpc_ports.size()+1);
    outputM_ports.resize(outputM_ports.size()+1);
    outputm_ports.resize(outputm_ports.size()+1);

    //Create Ports
    string portName = "/" + moduleName + "/" + driveName;
    
    rpc_ports[d] = new Port;
    outputm_ports[d] = new BufferedPort<Bottle>;
    outputM_ports[d] = new BufferedPort<Bottle>;
    
    string pn = portName + "/rpc:o";
    string targetPortName = "/" + homeo_name + "/" + driveName + "/min:o";
    /*
    cout << "Configuring port " <<d<< " : "<< pn << " ..." << endl;
    if (!rpc_ports[d]->open((pn).c_str())) 
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }
    

    while(!Network::connect(targetPortName,portName))
    	{
            cout<<"Setting up homeostatic connections... "<< targetPortName << " " << portName <<endl;
            yarp::os::Time::delay(0.5);
        }

    */
    pn = portName + "/min:i";
    cout << "Configuring port " <<d<< " : "<< pn << " ..." << endl;
    if (!outputm_ports[d]->open((pn).c_str())) 
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }
    targetPortName = "/" + homeo_name + "/" + driveName + "/min:o";
    yarp::os::Time::delay(0.1);
    while(!Network::connect(targetPortName,pn))
    {cout<<"Setting up homeostatic connections... "<< targetPortName << " " << pn <<endl;yarp::os::Time::delay(0.5);}
    pn = portName + "/max:i";
    cout << "Configuring port " <<d<< " : "<< pn << " ..." << endl;
    yarp::os::Time::delay(0.1);
    if (!outputM_ports[d]->open((pn).c_str())) 
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }
    yarp::os::Time::delay(0.1);
    targetPortName = "/" + homeo_name + "/" + driveName + "/max:o";
    while(!Network::connect(targetPortName,pn))
    {cout<<"Setting up homeostatic connections... "<< targetPortName << " " << pn <<endl;yarp::os::Time::delay(0.5);}

    return 42;
}

bool ReactiveLayer::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name",Value("ReactiveLayer")).asString().c_str();
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
    //if (iCub->getSpeechClient())
        //iCub->getSpeechClient()->SetOptions(ttsOptions);

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

    Rand::init();

	//iCub->getReactableClient()->SendOSC(yarp::os::Bottle("/event reactable pong start"));

    yInfo("Init done");

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
    iCub->opc->addOrRetrieveEntity<Action>("is");
    iCub->opc->addOrRetrieveEntity<Adjective>("saluted");
}

void ReactiveLayer::configureAllostatic(yarp::os::ResourceFinder &rf)
{
	//The homeostatic module should be running in parallel, independent from this, so the objective of 
	//this config would be to have a proper list  and connect to each port

	homeo_name = "homeostasis";
	string homeo_rpc_name = "/" + homeo_name + "/rpc";
	string to_h_rpc_name="/"+moduleName+"/toHomeoRPC:o";
	to_homeo_rpc.open(to_h_rpc_name);

	while(!Network::connect(to_h_rpc_name,homeo_rpc_name))
	{
		cout<<"Trying to connect to homeostasis..."<<endl;
        cout << "from " << to_h_rpc_name << " to " << homeo_rpc_name << endl;
		yarp::os::Time::delay(0.2);
	}

	/*
	Bottle cmd;
	Bottle rpl;
	rpl.clear();
	cmd.clear();
	cmd.addString("names");
	to_homeo_rpc.write(cmd,rpl);
	if (rpl.get(0).asString()!="nack")
	{
		n_drives = rpl.size();
		drive_names = rpl;
	}
	*/

	//Initialise the iCub allostatic model. Drives for interaction engine will be read from IE default.ini file
	cout << "Initializing drives..."<<endl;
	Bottle grpAllostatic = rf.findGroup("ALLOSTATIC");
	drivesList = *grpAllostatic.find("drives").asList();
	iCub->icubAgent->m_drives.clear();
	Bottle cmd;

    double priority_sum = 0.; 
    double priority;
	for (int d = 0; d<drivesList.size(); d++)
	{
		cmd.clear();
		string driveName = drivesList.get(d).asString().c_str();
		cmd.addString("add");
		cmd.addString("conf");
		Bottle drv;
		drv.clear();
		Bottle aux;
		aux.clear();
		aux.addString("name");
		aux.addString(driveName);
		drv.addList()=aux;
		aux.clear();
		drv.append(grpAllostatic);
        cmd.append(drv);//addList()=drv;
        Bottle rply;
        rply.clear();
        rply.get(0).asString();
        cout << cmd.toString() << endl;
		/*while(rply.get(0).auxsString()!="ack")
            {*/
                //to_homeo_rpc.write(cmd,rply);
                cout << rply.toString()<<endl;
              /*  cout<<"cannot create drive "<< driveName << "..."<<endl;
            }*/


		int answer = openPorts(driveName,d);
		cout << "The answer is " << answer <<endl;


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
        string under_port_name = grpAllostatic.check((driveName + "-under-behavior-port").c_str(), Value("None")).asString();

        cout << under_port_name << endl;

        // set drive priorities. Default to 1.
        priority = grpAllostatic.check((driveName + "-priority"), Value(1.)).asDouble();
        priority_sum += priority;
        drivePriorities.push_back(priority);

        
        if (under_port_name != "None")
            {
            	responseUnder.active = true;
                string out_port_name = "/" + moduleName + "/" + driveName + "/under_action:o";
                responseUnder.output_port = new Port();
                responseUnder.output_port->open(out_port_name);
                cout << "trying to connect to " << under_port_name << endl;
                while(!Network::connect(out_port_name,under_port_name))
                {
                    cout << "." << endl;
                    yarp::os::Time::delay(0.5);
                }
            }else{
            	responseUnder.active = false;
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
        string over_port_name = grpAllostatic.check((driveName + "-over-behavior-port").c_str(), Value("None")).asString();
        if (over_port_name != "None")
        {
        	responseOver.active=true;
            string out_port_name = "/" + moduleName + "/" + driveName + "/over_action:o";
            responseOver.output_port = new Port();
            responseOver.output_port->open(out_port_name);
            cout << "trying to connect to " << over_port_name << endl;
            while(!Network::connect(out_port_name,over_port_name))
            {
                cout << "." << endl;
                yarp::os::Time::delay(0.5);
            }
        }else{
            	responseOver.active = false;
            }

		homeostaticOverEffects[driveName] = responseOver;
	}
	cout << "done" << endl;

    // Normalize drive priorities
    if ( ! Normalize(drivePriorities))
        cout << "Error: Drive priorities sum up to 0." << endl;

    cout << "Drive priorities: " << drivePriorities << endl; 

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

bool ReactiveLayer::Normalize(vector<double>& vec) {
    double sum = 0.;
    for (unsigned int i = 0; i<vec.size(); i++)
        sum += vec[i];
    if (sum == 0.)
        return false;
    for (unsigned int i = 0; i<vec.size(); i++)
        vec[i] /= sum;
    return true;
}

bool ReactiveLayer::updateModule()
{
    cout<<".";

	//handleSalutation(someonePresent);
	//physicalInteraction = handleTactile();
	confusion = handleTagging();
    cout << confusion << endl;
    //learning = handlePointing();
    updateAllostatic();
	//updateEmotions();
    	
    return true;
}
bool ReactiveLayer::handlePointing()
{
    iCub->opc->checkout();
    list<Entity*> lEntities = iCub->opc->EntitiesCache();

    int counter = 0;
    for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
    {
        string sName = (*itEnt)->name();
        string sNameCut = sName;
        string delimiter = "_";
        size_t pos = 0;
        string token;
        if ((pos = sName.find(delimiter)) != string::npos) {
            token = sName.substr(0, pos);
            sName.erase(0, pos + delimiter.length());
            sNameCut = token;
        }
        // check is label is known

        if (sNameCut != "unknown") {

            if ((*itEnt)->entity_type() == "object" )//|| (*itEnt)->entity_type() == "bodypart")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                cout << "I'd like to point " << (*itEnt)->name() <<endl;
                //If there is an unknown object (to see with agents and rtobjects), add it to the rpc_command bottle, and return true
                homeostaticUnderEffects["pointing"].rpc_command.clear();
                homeostaticUnderEffects["pointing"].rpc_command.addString("point");
                //homeostaticUnderEffects["pointing"].rpc_command.addString((*itEnt)->entity_type());
                homeostaticUnderEffects["pointing"].rpc_command.addString((*itEnt)->name());
                return true;
                /*
                Object* temp = dynamic_cast<Object*>(*itEnt);
                if (temp->m_saliency > highestSaliency)
                {
                    if (secondSaliency != 0.0)
                    {
                        secondSaliency = highestSaliency;
                    }
                    highestSaliency = temp->m_saliency;
                    sNameBestEntity = temp->name();
                    sTypeBestEntity = temp->entity_type();
                }
                else
                {
                    if (temp->m_saliency > secondSaliency)
                    {
                        secondSaliency = temp->m_saliency;
                    }
                }
                counter++;
                */
            }
        }
    }
    //if no unknown object was found, return false
    return counter > 0; 
}
bool ReactiveLayer::handleTagging()
{
    iCub->opc->checkout();
    list<Entity*> lEntities = iCub->opc->EntitiesCache();
    //vector<Entity*> vEntities;
    //copy(lEntities.begin(), lEntities.end(), vEntities.begin());
    //std::random_shuffle ( vEntities.begin(), vEntities.end() );
    //list<Entity*> lEntitiesShuffled(vEntities.begin(), vEntities.end());

    for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
    {
        string sName = (*itEnt)->name();
        string sNameCut = sName;
        string delimiter = "_";
        size_t pos = 0;
        string token;
        if ((pos = sName.find(delimiter)) != string::npos) {
            token = sName.substr(0, pos);
            sName.erase(0, pos + delimiter.length());
            sNameCut = token;
        }
        // check is label is known

        bool sendRPC = false;

        if (sNameCut == "unknown") {
            if ((*itEnt)->entity_type() == "object" || (*itEnt)->entity_type() == "bodypart")//|| (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
            {
                cout << "I found unknown entities!!!!"<<endl;
                Object* o = dynamic_cast<Object*>(*itEnt);
                if(o && o->m_present) {
                    sendRPC = true;
                }
            }
        } else {
            if ((*itEnt)->entity_type() == "bodypart" && dynamic_cast<Bodypart*>(*itEnt)->m_tactile_number == -1)
                sendRPC = true;
        }

        if (sendRPC) {
            //If there is an unknown object (to see with agents and rtobjects), add it to the rpc_command bottle, and return true
            homeostaticUnderEffects["tagging"].rpc_command.clear();
            homeostaticUnderEffects["tagging"].rpc_command.addString("exploreUnknownEntity");
            homeostaticUnderEffects["tagging"].rpc_command.addString((*itEnt)->entity_type());
            homeostaticUnderEffects["tagging"].rpc_command.addString((*itEnt)->name());
            return true;
            /*
            Object* temp = dynamic_cast<Object*>(*itEnt);
            if (temp->m_saliency > highestSaliency)
            {
                if (secondSaliency != 0.0)
                {
                    secondSaliency = highestSaliency;
                }
                highestSaliency = temp->m_saliency;
                sNameBestEntity = temp->name();
                sTypeBestEntity = temp->entity_type();
            }
            else
            {
                if (temp->m_saliency > secondSaliency)
                {
                    secondSaliency = temp->m_saliency;
                }
            }
            counter++;
            */            
        }
    }
    //if no unknown object was found, return false
    return false; 
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
        Object* touchLocation = (Object*) iCub->opc->getEntity("touchLocation");
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
		Agent* currentAgent = (Agent*)(*currentAgentIt);


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

// Return the index of a drive to solve according to priorities and homeostatis levels
// Return -1 if no drive to be solved
DriveOutCZ ReactiveLayer::chooseDrive() {

    DriveOutCZ result;
    bool inCZ;
    int numOutCz = 0;
    double random, cum;
    vector<double> outOfCzPriorities(drivePriorities);

    for ( int i =0;i<drivesList.size();i++) {
         inCZ = outputm_ports[i]->read()->get(0).asDouble() <= 0 && outputM_ports[i]->read()->get(0).asDouble() <= 0;
         if (inCZ) {
            outOfCzPriorities[i] = 0.;           
         }
         else {
            numOutCz ++;
         }
        cout << "Drive " << i << ", " << drivesList.get(i).asString() << ". Priority: " << outOfCzPriorities[i] << "." << endl; 
    }
    if (! numOutCz) {
        result.idx = -1;
        return result;
    }
    if ( ! Normalize(outOfCzPriorities)) {
        result.idx = -1;
        return result;
    }
    random = Rand::scalar();
    cum = outOfCzPriorities[0];
    int idx = 0;
    while (cum < random) {
        cum += outOfCzPriorities[idx + 1];
        idx++;
    }
    result.idx = idx;
    if (outputm_ports[idx]->read()->get(0).asDouble() > 0)
        result.level = UNDER;
    if (outputM_ports[idx]->read()->get(0).asDouble() > 0)
        result.level = OVER;    
    return result;
}


bool ReactiveLayer::updateAllostatic()
{
	iCub->updateAgent();

	//Update some specific drives based on the previous stimuli encountered
	if (physicalInteraction)
		{
			Bottle cmd;
			cmd.clear();
			cmd.addString("delta");
			cmd.addString("physicalInteraction");
			cmd.addString("val");
			cmd.addDouble(0.1);

			to_homeo_rpc.write(cmd);
		}

		
		//iCub->icubAgent->m_drives["physicalInteraction"].value += 0.1;
	if (someonePresent)
		{
			Bottle cmd;
			cmd.clear();
			cmd.addString("par");
			cmd.addString("socialInteraction");
			cmd.addString("dec");
			cmd.addDouble(-0.002);

			to_homeo_rpc.write(cmd);
		}
	//iCub->icubAgent->m_drives["socialInteraction"].value += iCub->icubAgent->m_drives["socialInteraction"].decay * 2;

	if (confusion)
	{
		Bottle cmd;
		cmd.clear();
		cmd.addString("par");
		cmd.addString("tagging");
		cmd.addString("dec");
		cmd.addDouble(0.006);
        cout << cmd.toString()<<endl;
        Bottle rply;
        rply.clear();
		to_homeo_rpc.write(cmd,rply);
        cout<<rply.toString()<<endl;

	}else{
		Bottle cmd;
		cmd.clear();
		cmd.addString("par");
		cmd.addString("tagging");
		cmd.addString("dec");
		cmd.addDouble(-0.01);
        cout << cmd.toString()<<endl;

		to_homeo_rpc.write(cmd);
	}
    if (learning)
    {
        Bottle cmd;
        cmd.clear();
        cmd.addString("par");
        cmd.addString("pointing");
        cmd.addString("dec");
        cmd.addDouble(0.021);
        cout << cmd.toString()<<endl;
        Bottle rply;
        rply.clear();
        to_homeo_rpc.write(cmd,rply);
        cout<<rply.toString()<<endl;

    }else{
        Bottle cmd;
        cmd.clear();
        cmd.addString("par");
        cmd.addString("pointing");
        cmd.addString("dec");
        cmd.addDouble(0.0);
        cout << cmd.toString()<<endl;

        to_homeo_rpc.write(cmd);
    }
    //cout <<drivesList.size()<<endl;

    DriveOutCZ activeDrive = chooseDrive();


    int i; // the chosen drive

    if (activeDrive.idx == -1) {
        cout << "No drive out of CZ." << endl;
        return true;
    }
    else
        i = activeDrive.idx;

    //Under homeostasis


    if (activeDrive.level == UNDER)
    {
        cout << "Drive " << activeDrive.idx << " chosen. Under level." << endl;
        iCub->say(homeostaticUnderEffects[drivesList.get(i).asString().c_str()].getRandomSentence());
        if (homeostaticUnderEffects[drivesList.get(i).asString().c_str()].active)
        {
            cout << "Command sent!!!"<< endl;
            cout <<homeostaticUnderEffects[drivesList.get(i).asString().c_str()].active << homeostaticUnderEffects[drivesList.get(i).asString().c_str()].rpc_command.toString() << endl;
            Bottle rply;
            rply.clear();
            homeostaticUnderEffects[drivesList.get(i).asString().c_str()].output_port->write(homeostaticUnderEffects[drivesList.get(i).asString().c_str()].rpc_command,rply);
            yarp::os::Time::delay(0.1);
            cout<<rply.toString()<<endl;

        }
        Bottle cmd;
        cmd.clear();
        cmd.addString("delta");
        cmd.addString(drivesList.get(i).asString().c_str());
        cmd.addString("val");
        cmd.addDouble(0.35);

        rpc_ports[i]->write(cmd);

        //d->second.value += (d->second.homeoStasisMax - d->second.homeoStasisMin) / 3.0;
    }

    //Over homeostasis

    if (activeDrive.level == OVER)
    {
        cout << "Drive " << activeDrive.idx << " chosen. Under level." << endl;
		iCub->say(homeostaticOverEffects[drivesList.get(i).asString().c_str()].getRandomSentence());
		Bottle cmd;
		cmd.clear();
		cmd.addString("delta");
		cmd.addString(drivesList.get(i).asString().c_str());
		cmd.addString("val");
		cmd.addDouble(-0.15);

		rpc_ports[i]->write(cmd);
		//d->second.value -= (d->second.homeoStasisMax - d->second.homeoStasisMin) / 3.0;;
	}

    //cout<<"come on..."<<endl;
	//iCub->commitAgent();
    //cout<<"commited"<<endl;
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
                Agent* agent = iCub->opc->addOrRetrieveEntity<Agent>(name);
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
                Object* o = iCub->opc->addOrRetrieveEntity<Object>(name);
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
                RTObject* o = iCub->opc->addOrRetrieveEntity<RTObject>(name);
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
                iCub->opc->addOrRetrieveEntity<Adjective>(name);
            }
        }

        Bottle *actionList = grpOPC.find("action").asList();
        if (actionList)
        {
            for(int d=0; d<actionList->size(); d++)
            {
                string name = actionList->get(d).asString().c_str();
                iCub->opc->addOrRetrieveEntity<Action>(name);
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

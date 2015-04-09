#include "gameCluedo.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::math;
using namespace wysiwyd::wrdac;

bool GameCluedo::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("GameCluedo")).asString().c_str();
    setName(moduleName.c_str());

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName,"gameCluedo","client.ini",isRFVerbose);
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

    //Configure the various components
    configureOPC(rf);
    configureSpeech(rf);
    cout << "Configuration done."<<endl;

    rpc.open ( ("/"+moduleName+"/rpc").c_str());
    attach(rpc);

	gameStarted = false;
	isMyTurn = false;

    return true;
}

bool GameCluedo::updateModule()
{
	if (gameStarted)
	{
		if (isMyTurn)
		{
			//We ask a question
			Relation* relationToComplete;
			Role missingRole;
			findPartialRelation(relationToComplete, missingRole);

			if (relationToComplete == NULL)
			{
				cout << "I already know everything." << endl;
			}
			else
			{
				//Formulate the question and wait for a statement
				string question = formQuestionFromRelation(relationToComplete, missingRole);
				iCub->say(question, true);
				while (!handleSpeech(true))
				{
					cout << "Waiting for an answer to : " << question << endl;
				}
			}
			isMyTurn = !isMyTurn;
		}
		else
		{
			//We just wait for other's question				
			while (!handleSpeech(false))
			{
				cout << "Waiting for a question from user." <<endl;
			}
			isMyTurn = !isMyTurn;
		}
		return true;
	}
	else
	{
		cout << "Waiting to start..." << endl;
	}
}

void GameCluedo::populateSpeechRecognizerVocabulary()
{
    if (iCub->getSpeechClient()->isRunning())
    {
        list<Entity*> allEntities = iCub->opc->EntitiesCache();
        for(list< Entity* >::iterator itEnt =  allEntities.begin() ; itEnt !=allEntities.end() ; itEnt++)
        {
            iCub->getSpeechClient()->STT_ExpandVocabulory((*itEnt)->entity_type(),(*itEnt)->name());
            cout<<"Augmentation of vocabulory : "<<(*itEnt)->entity_type()<<"-->"<<(*itEnt)->name()<<endl;
        }
    }
    else
    {
        cout<<"Warning: Unable to populate speech recognizer vocabulary. Is speechRecognizer running?"<<endl;
    }
}

bool GameCluedo::handleSpeech(bool expectAffirmation)
{
    bool gotSignal = false;
    Bottle* speechCmd = iCub->getSpeechClient()->STT(false);
    if (speechCmd)
    {
        if (speechCmd->size() != 2)
        {
            std::cout << "in adaptativeLayer::handleSpeech | error: bottle received size !=2" << std::endl;
            return false;
        }
        gotSignal = true;
        std::cout << speechCmd->toString() << std::endl;
        std::cout << speechCmd->toString() << std::endl;
        cout<<"Raw sentence: |"<<speechCmd->get(0).toString()<<"|"<<endl;
        //cout<<"Semantic:"<<speechCmd->get(1).toString();

        Bottle* semanticBottle = speechCmd->get(1).asList();

        string sentenceType = semanticBottle->get(0).asString();

        //if (sentenceType == "SUBNODE")
        //{
        //    Bottle keyBot;
        //    keyBot.addString(semanticBottle->get(1).asList()->check("keyword",Value("none")).asString());
        //    pSpeechRecognizerKeywordOut.write(keyBot);
        //    return true;
        //}

        //if (sentenceType == "miscSentences")
        //{
        //    string rawSentence = speechCmd->get(0).toString();
        //    cout<<"Catched a misc sentence : "<<semanticBottle->toString().c_str()<<endl;
        //    return true;
        //}

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        //Create a relation for information exchange
        Bottle bRelation;
        Relation relationForm = this->getRelationFromSemantic(*semanticBottle->get(1).asList());
        cout<< "Sentence type : "<<sentenceType<<endl;
        cout<< "Relation form : "<<relationForm.toString()<<endl;
        string answerFromRobot = "";

		//Discard the affirmation in place of question & vice versa
		if (expectAffirmation && !(sentenceType == "AFFIRMATIVE"))
		{
			iCub->say("I am sorry, but could you answer my question, please?");
			return false;
		}
		else if (!expectAffirmation && (sentenceType == "AFFIRMATIVE"))
		{
			iCub->say("I am sorry, I did not get it. Can you repeat your question?");
			return false;
		}

		//Finally process
        //if (sentenceType == "IMPERATIVE")
        //{
        //    //Execute the action
        //}
        //else 
		if (sentenceType == "AFFIRMATIVE")
        {
            if (!iCub->opc->containsRelation(relationForm))
            {
                iCub->opc->addRelation(relationForm);
                answerFromRobot = "Ok, I will know that " + relationForm.toString();
            }
            else
            {
                answerFromRobot = "I already knew that.";
            }
            //Update the other model to reflect his knowledge
            Agent* partner = iCub->opc->addAgent("partner");
            partner->addBelief(relationForm);
            iCub->opc->commit(partner);
        }
        else //interrogative
        {
            if (sentenceType == "INTERROGATIVE_WHO")
                relationForm.m_subject = "?";
            if (sentenceType == "INTERROGATIVE_WHAT")
                relationForm.m_object = "?";
            if (sentenceType == "INTERROGATIVE_WHEN")
                relationForm.m_complement_time = "?";
            if (sentenceType == "INTERROGATIVE_WHERE")
                relationForm.m_complement_place = "?";
            if (sentenceType == "INTERROGATIVE_HOW")
                relationForm.m_complement_manner = "?";

            Relation relationReturn(relationForm);

            //Retrieve from OPC
            string matchingSubject;
            string matchingObject;
            string matchingVerb;
            string matchingPlace;
            string matchingTime;
            string matchingManner;
            relationForm.subject() != "?" && relationForm.subject() != "none" ?     matchingSubject = relationForm.subject()  :    matchingSubject = "any";
            relationForm.verb() != "?"  && relationForm.verb() != "none"    ?       matchingVerb = relationForm.verb()      :    matchingVerb = "any";
            relationForm.object() != "?" && relationForm.object() != "none"   ?     matchingObject = relationForm.object()    :    matchingObject = "any";
            relationForm.complement_place()   != "?" && relationForm.complement_place() != "none" ? matchingPlace =  relationForm.complement_place() :   matchingPlace = "any" ;
            relationForm.complement_time()    != "?" && relationForm.complement_time() != "none" ? matchingTime = relationForm.complement_time()   :   matchingTime = "any"   ;
            relationForm.complement_manner()  != "?" && relationForm.complement_manner() != "none" ? matchingManner = relationForm.complement_manner():  matchingManner = "any" ;
            list<Relation> allAnswers = iCub->opc->getRelationsMatching(matchingSubject,matchingVerb,matchingObject,matchingPlace,matchingTime,matchingManner);

            //We say the first thing recovered. Ideally we should see if our model of the other already contains it.
            list<Relation> realAnswers;
            for(list<Relation>::iterator pAns = allAnswers.begin(); pAns!= allAnswers.end(); pAns++)
            {
                //We check if this relation answer to the specific interrogation
                bool isAnswering = false;
                if (relationForm.subject() == "?" && pAns->subject() != "none")
                    isAnswering = true;
                if (relationForm.object() == "?" && pAns->object() != "none")
                    isAnswering = true;
                if (relationForm.complement_place() == "?" && pAns->complement_place() != "none")
                    isAnswering = true;
                if (relationForm.complement_time() == "?" && pAns->complement_time() != "none")
                    isAnswering = true;
                if (relationForm.complement_manner() == "?" && pAns->complement_manner() != "none")
                    isAnswering = true;

                if (isAnswering)
                    realAnswers.push_back(*pAns);
            }

            if (realAnswers.size() >0)
            {
                Agent* partner = iCub->opc->addAgent("partner");
                Relation relationToBeStated = realAnswers.front();
                bool partnerShouldHaveKnown = true;

                //Go through all the possible answers and take the one that is not known by our modl of the other
                for(list<Relation>::iterator pAns = realAnswers.begin(); pAns!= realAnswers.end(); pAns++)
                {
                    //If we know that the partner does't know this relation we say it
                    if (!partner->checkBelief(*pAns))
                    {
                        relationToBeStated = *pAns;
                        partnerShouldHaveKnown = false;
                        break;
                    }
                }

                //Update the other model
                partner->addBelief(relationToBeStated);
                iCub->opc->commit(partner);

                //Create good answer from relation
                answerFromRobot = "I know that ";
                answerFromRobot += relationToBeStated.subject();
                answerFromRobot += " " ;
                answerFromRobot += relationToBeStated.verb();
                answerFromRobot += " " ;
                if (relationToBeStated.object() != "none")
                {
                    answerFromRobot += " the " ;
                    answerFromRobot += relationToBeStated.object();
                    answerFromRobot += " " ;
                }
                if (relationToBeStated.complement_place() != "none")
                {
                    answerFromRobot += "in the " ;
                    answerFromRobot += relationToBeStated.complement_place();
                    answerFromRobot += " " ;
                }
                if (relationToBeStated.complement_time() != "none")
                {
                    answerFromRobot += relationToBeStated.complement_time();
                    answerFromRobot += " " ;
                }
                if (relationToBeStated.complement_manner() != "none")
                {
                    answerFromRobot += relationToBeStated.complement_manner();
                    answerFromRobot += " " ;
                }
                answerFromRobot +=".";

                if( partnerShouldHaveKnown)
                {
                    answerFromRobot += " But I thought you already knew that.";
                }

            }
            else
                answerFromRobot = "I do not know...";
        }

        iCub->say(answerFromRobot);
    }
    return gotSignal;
}

string GameCluedo::getEntityFromWordGroup(Bottle *b)
{
    if (b->check("object"))
        return b->find("object").asString();
    if (b->check("agent"))
        return b->find("agent").asString();
    if (b->check("rtobject"))
        return b->find("rtobject").asString();
    if (b->check("action"))
        return b->find("action").asString();
    if (b->check("adjective"))
        return b->find("adjective").asString();

    return "none";
}

Relation GameCluedo::getRelationFromSemantic(Bottle b)
{
    string s = "none";
    string v = "none";
    string o = "none";
    string p = "none";
    string t = "none";
    string m = "none";

    if (b.check("groupSubject"))
        s = getEntityFromWordGroup(b.find("groupSubject").asList());
    if (b.check("groupVerbal"))
        v = getEntityFromWordGroup(b.find("groupVerbal").asList());
    if (b.check("groupObject"))
        o = getEntityFromWordGroup(b.find("groupObject").asList());
    if (b.check("groupPlace"))
        p = getEntityFromWordGroup(b.find("groupPlace").asList());
    if (b.check("groupTime"))
        t = getEntityFromWordGroup(b.find("groupTime").asList());
    if (b.check("groupManner"))
        m = getEntityFromWordGroup(b.find("groupManner").asList());

    return Relation(s,v,o,p,t,m);
}

void GameCluedo::configureSpeech(yarp::os::ResourceFinder &rf)
{
    //Port for broadcasting recognized keywords to other modules
    //pSpeechRecognizerKeywordOut.open("/"+getName()+"/speechGrammar/keyword:o");

    //Set the tts options
    string ttsOptions = rf.check("ttsOptions",Value("iCub")).asString().c_str();
    iCub->getSpeechClient()->SetOptions(ttsOptions);

    //Populate the speech reco if needed
    bool shouldPopulateGrammar = rf.find("shouldPopulateGrammar").asInt() == 1;
    if (shouldPopulateGrammar)
        populateSpeechRecognizerVocabulary();
}

void GameCluedo::configureOPC(yarp::os::ResourceFinder &rf)
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
				std::string name = agentList->get(d).asString().c_str();
				wysiwyd::wrdac::Agent* agent = iCub->opc->addAgent(name);
                agent->m_present = false;
                iCub->opc->commit(agent);
            }
        }

        Bottle *objectList = grpOPC.find("object").asList();
        if (objectList)
        {
            for(int d=0; d<objectList->size(); d++)
            {
                std::string name = objectList->get(d).asString().c_str();
				wysiwyd::wrdac::Object* o = iCub->opc->addObject(name);
                o->m_present = false;
                iCub->opc->commit(o);
            }
        }

        Bottle *rtobjectList = grpOPC.find("rtobject").asList();
        if (rtobjectList)
        {
            for(int d=0; d<rtobjectList->size(); d++)
            {
				std::string name = rtobjectList->get(d).asString().c_str();
				wysiwyd::wrdac::RTObject* o = iCub->opc->addRTObject(name);
                o->m_present = false;
                iCub->opc->commit(o);
            }
        }

        Bottle *adjectiveList = grpOPC.find("adjective").asList();
        if (adjectiveList)
        {
            for(int d=0; d<adjectiveList->size(); d++)
            {
				std::string name = adjectiveList->get(d).asString().c_str();
                iCub->opc->addAdjective(name);
            }
        }

        Bottle *actionList = grpOPC.find("action").asList();
        if (actionList)
        {
            for(int d=0; d<actionList->size(); d++)
            {
				std::string name = actionList->get(d).asString().c_str();
                iCub->opc->addAction(name);
            }
        }
    }
    cout<<"done"<<endl;
}


bool GameCluedo::respond(const Bottle& cmd, Bottle& reply)
{
	if (cmd.get(0).toString() == "start")
	{
		iCub->say("Hello! Nice to meet you. Let's play a clue game.", true);
		gameStarted = true;
		reply.addString("ACK");
	}
	else
	{
		reply.addString("NACK");
	}
    return true;
}

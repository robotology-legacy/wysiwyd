#include "proactiveTagging.h"


/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string proactiveTagging::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    cout << "path is: " << sPath << endl;

    if (!isGrammar)
    {
        cout << "Error in proactiveTagging::grammarToString. Couldn't open file : " << sPath << "." << endl;
        return "Error in proactiveTagging::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

bool proactiveTagging::configure(yarp::os::ResourceFinder &rf)
{

    string opcName;
    string handlerPortName;

    string moduleName = rf.check("name", Value("proactiveTagging")).asString().c_str();
    setName(moduleName.c_str());

    GrammarYesNo = rf.findFileByName(rf.check("GrammarYesNo", Value("nodeYesNo.xml")).toString());
    GrammarAskNameObject = rf.findFileByName(rf.check("GrammarAskNameObject", Value("GrammarAskNameObject.xml")).toString());
    GrammarAskNameAgent = rf.findFileByName(rf.check("GrammarAskNameAgent", Value("GrammarAskNameAgent.xml")).toString());

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "proactiveTagging", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= false;
    if (!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        Time::delay(1.0);
    }

    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    if (!iCub->getRecogClient())
    {
        cout << "WARNING SPEECH RECOGNIZER NOT CONNECTED" << endl;
    }
    if (!iCub->getABMClient())
    {
        cout << "WARNING ABM NOT CONNECTED" << endl;
    }

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";



    return true;
}


bool proactiveTagging::close() {
    iCub->close();
    delete iCub;

    return true;
}


bool proactiveTagging::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n"
        "exploreUnknowneEntity entity_type entity_name \n" +
        "exploreEntityByName entity_name \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "exploreUnknownEntity") {
        yInfo() << " exploreUnknownEntity";
        reply = exploreUnknownEntity(command);
    }
    else if (command.get(0).asString() == "exploreEntityByName") {
        yInfo() << " exploreEntityByName";
        reply = exploreEntityByName(command);
    }
    else {
        cout << helpMessage;
        reply.addString("ok");
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool proactiveTagging::updateModule() {
    return true;
}






void proactiveTagging::checkRelations()
{
    iCub->opc->update();
    iCub->opc->checkout();

    string none = "none";

    list<Relation> lRelations = iCub->opc->getRelations();

    //FOR EACH RELATION
    for (list<Relation>::iterator itRel = lRelations.begin(); itRel != lRelations.end(); itRel++)
    {
        cout << itRel->toString() << endl;

        if (itRel->complement_manner() == none)
        {
            cout << "should ask the manner complement" << endl;
            string sManner = askManner(itRel->subject(), itRel->verb(), itRel->object());

            iCub->opc->removeRelation(*itRel);

            cout << "relation is now: " << itRel->toString() << endl;
            itRel->m_complement_manner = sManner;
            iCub->opc->addAdjective(sManner);
            iCub->opc->addRelation(*itRel);
            iCub->opc->commit();
        }
        cout << itRel->complement_manner() << endl;
        cout << itRel->complement_place() << endl;
        cout << itRel->complement_time() << endl;
    }

}



string proactiveTagging::askManner(string agent, string verb, string object)
{

    ostringstream osSentenceToSay,
        osError;
    osSentenceToSay << "Do you know how " << agent << " " << verb << " the " << object << " ?";

    iCub->say(osSentenceToSay.str());


    Bottle bOutput;

    //bool fGetaReply = false;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bMessenger; //to be send TO speech recog

    //bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarAskManner), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        cout << bRecognized.get(1).toString() << endl;
        return "none";
    }

    bAnswer = *bRecognized.get(1).asList();
    cout << "bAnswer is : " << bAnswer.toString() << endl;

    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)
    bSemantic = *bAnswer.get(1).asList();


    if (bAnswer.get(0).asString() == "stop")
    {
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return "stop";
    }


    string sManner = bSemantic.check("manner", Value("none")).asString();
    cout << "answer is : " << sManner << endl;

    return sManner;
    //TODO dialogue interaction to ask for complement of manner
}

/*
* Ask the name of an unknown entity
* input: exploreUnknownEntity entityType entityName (eg: exploreUnknownEntity agent unknown_25)
* ask through speech the name of an unknwon entity
*/
Bottle proactiveTagging::exploreUnknownEntity(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 3)
    {
        yInfo() << " proactiveTagging::exploreEntity | Problem in input size.";
        bOutput.addString("Problem in input size");
        return bOutput;
    }

    string currentEntityType = bInput.get(1).toString();
    string sNameTarget = bInput.get(2).toString();

    yInfo() << " EntityType : " << currentEntityType;
    double timeDelay = 1.;

    if (currentEntityType == "agent")
    {
        yInfo() << " Hello, I don't know you. Who are you ?";
        iCub->getSpeechClient()->TTS(" Hello, I don't know you. Who are you ?", false);
        iCub->say(" Hello, I don't know you. Who are you ?");
        //bool fGetaReply = false;
        Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic; // semantic information of the content of the recognition
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameAgent), 20);

        if (bRecognized.get(0).asInt() == 0)
        {
            yWarning() << " error in proactiveTagging::exploreEntity | askNameAgent | Error in speechRecog";
            bOutput.addString("error");
            bOutput.addString("error in speechRecog");
            return bOutput;
        }

        bAnswer = *bRecognized.get(1).asList();
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo() << " in proactiveTagging::exploreEntity | askNameAgent | stop called";
            bOutput.addString("error");
            bOutput.addString("stop called");
            return bOutput;
        }

        bSemantic = *bAnswer.get(1).asList();
        string sName = bSemantic.check("agent", Value("unknown")).asString();

        Agent* agentToChange = dynamic_cast<Agent*>(iCub->opc->getEntity(sNameTarget));
        agentToChange->changeName(sName);
        iCub->opc->commit(agentToChange);

        yInfo() << " Well, Nice to meet you " << sName;
        iCub->say("Well, Nice to meet you " + sName);
        Time::delay(timeDelay);

        iCub->opc->update();

        bOutput.addString("success");
        bOutput.addString("agent");
        return bOutput;
    }

    if (currentEntityType == "object")
    {
        yInfo() << " Hum, what is this object ?";
        iCub->getSpeechClient()->TTS(" Hum, what is this object ?", false);
        //      iCub->say(" Hum, what is this object ?");

        //bool fGetaReply = false;
        Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic; // semantic information of the content of the recognition
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameObject), 20);

        if (bRecognized.get(0).asInt() == 0)
        {
            yWarning() << " error in proactiveTagging::exploreEntity | askNameObject | Error in speechRecog";
            bOutput.addString("error");
            bOutput.addString("error in speechRecog");
            return bOutput;
        }

        bAnswer = *bRecognized.get(1).asList();
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo() << " in proactiveTagging::exploreEntity | askNameObject | stop called";
            bOutput.addString("error");
            bOutput.addString("stop called");
            return bOutput;
        }

        yInfo() << " bAnswer is: " << bAnswer.toString();
        bSemantic = *bAnswer.get(1).asList();
        yInfo() << " bSemantic is: " << bSemantic.toString();

        string sName = bSemantic.check("object", Value("unknown")).asString();

        Object* objectToChange = dynamic_cast<Object*>(iCub->opc->getEntity(sNameTarget));
        objectToChange->changeName(sName);

        iCub->opc->commit(objectToChange);
        yInfo() << " I get it, this is a " << sName;
        iCub->say(" I get it, this is a " + sName);

        bOutput.addString("success");
        bOutput.addString("object");
        return bOutput;
    }

    if (currentEntityType == "rtobject")
    {
        yInfo() << " Hum, what is this object ?";
        //        iCub->say(" Hum, what is this object ?");
        iCub->getSpeechClient()->TTS(" Hum, what is this object ?", false);

        //bool fGetaReply = false;
        Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic; // semantic information of the content of the recognition
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameObject), 20);

        if (bRecognized.get(0).asInt() == 0)
        {
            yWarning() << " error in proactiveTagging::exploreEntity | askNameRTObject | Error in speechRecog";
            bOutput.addString("error");
            bOutput.addString("error in speechRecog");
            return bOutput;
        }

        bAnswer = *bRecognized.get(1).asList();
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo() << " in proactiveTagging::exploreEntity | askNameRTObject | stop called";
            bOutput.addString("error");
            bOutput.addString("stop called");
            return bOutput;
        }

        yInfo() << " bAnswer is: " << bAnswer.toString();
        bSemantic = *bAnswer.get(1).asList();
        yInfo() << " bSemantic is: " << bSemantic.toString();

        string sName = bSemantic.check("object", Value("unknown")).asString();

        RTObject* objectToChange = dynamic_cast<RTObject*>(iCub->opc->getEntity(sNameTarget));
        objectToChange->changeName(sName);
        iCub->opc->commit(objectToChange);

        yInfo() << " So this is a " << sName;
        iCub->say(" So this is a " + sName);

        bOutput.addString("success");
        bOutput.addString("rtobject");
        return bOutput;
    }

    return bOutput;
}

/*
* Search for the entity corresponding to a certain name in all the unknown entities
* return a bottle of 2 elements.
* First element is: error - warning - success
* Second element is: information about the action
*/
Bottle proactiveTagging::exploreEntityByName(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 2)
    {
        yInfo() << " proactiveTagging::exploreEntityByName | Problem in input size.";
        bOutput.addString("Problem in input size");
        return bOutput;
    }

    string sNameTarget = bInput.get(1).toString();
    yInfo() << " Entity to find: " << sNameTarget;

    // check if the entity is already present in the OPC
    if (iCub->opc->isConnected())
    {
        iCub->opc->checkout();
        list<Entity*> lEntities = iCub->opc->EntitiesCacheCopy();

        for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
        {
            if ((*itEnt)->name() == sNameTarget)
            {
                yInfo() << " Entity " << sNameTarget << " is already known.";
                bOutput.addString("warning");
                bOutput.addString("entity already exists");
                return bOutput;
            }
        }
    }
    else
    {
        yWarning() << " in proactiveTagging::exploreEntityByName | OPC not Connected";
        bOutput.addString("error");
        bOutput.addString("OPC not connected");

        return bOutput;
    }


    // if there is several objects unknown (or at least one)
    string sSentence = "I don't known which of these objects is a " + sNameTarget;
    iCub->say(sSentence);
    yInfo() << " " << sSentence;

    sSentence = "Can you show me the " + sNameTarget;
    iCub->say(sSentence);
    yInfo() << " " << sSentence;

    bool bFound = false;

    Time::delay(2.);

    // start detecting unknown objects
    while (!bFound)
    {
        iCub->opc->checkout();
        list<Entity*> lEntities = iCub->opc->EntitiesCacheCopy();

        double highestSaliency = 0.0;
        double secondSaliency = 0.0;
        string sNameBestEntity = "none";
        string sTypeBestEntity = "none";

        for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
        {
            string sName = (*itEnt)->name();
            string sNameCut = sName;
            string delimiter = "_";
            size_t pos = 0;
            string token;
            while ((pos = sName.find(delimiter)) != string::npos) {
                token = sName.substr(0, pos);
                sName.erase(0, pos + delimiter.length());
                sNameCut = token;
            }
            // check is label is known

            if (sNameCut == "unknown")
            {
                if ((*itEnt)->entity_type() == "agent")
                {
                    Agent* temp = dynamic_cast<Agent*>(*itEnt);
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
                }

                if ((*itEnt)->entity_type() == "object")
                {
                    Object* temp = iCub->opc->addOrRetrieveObject((*itEnt)->name());
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
                }

                if ((*itEnt)->entity_type() == "rtobject")
                {
                    RTObject* temp = dynamic_cast<RTObject*>(*itEnt);
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
                }
            }
        }

        bFound = false;
        if (highestSaliency > thresholdSalienceDetection)
        {
            //the object with highest salience is salient enough
            if (secondSaliency != 0.0)
            {
                // there are other salient objects
                if ((highestSaliency / secondSaliency) > thresholdDistinguishObjectsRatio)
                {
                    //but it is enough difference
                    bFound = true;
                }
            }
            else
            {
                //other object are not salient
                bFound = true;
            }
        }
        if (sNameBestEntity == "none")
        {
            bFound = false;
        }

        if (bFound)
        {
            // change name
            if (sTypeBestEntity == "agent")
            {
                Agent* TARGET = dynamic_cast<Agent*>(iCub->opc->getEntity(sNameBestEntity));
                TARGET->changeName(sNameTarget);
            }
            if (sTypeBestEntity == "object")
            {
                Object* TARGET = dynamic_cast<Object*>(iCub->opc->getEntity(sNameBestEntity));
                TARGET->changeName(sNameTarget);
            }
            if (sTypeBestEntity == "rtobject")
            {
                RTObject* TARGET = dynamic_cast<RTObject*>(iCub->opc->getEntity(sNameBestEntity));
                TARGET->changeName(sNameTarget);
            }
            yInfo() << " name changed: " << sNameBestEntity << " is now " << sNameTarget;
            bOutput.addString("name changed");
        }
    }

    iCub->opc->commit();

    return bOutput;
}

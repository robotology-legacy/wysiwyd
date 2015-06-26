#include "qRM.h"


/*
*  Get the context path of a .grxml grammar, and return it as a string
*
*/
string qRM::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        yInfo() << " Error in qRM::grammarToString. Couldn't open file : " << sPath << " .";
        return "Error in qRM::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

bool qRM::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("qRM")).asString().c_str();
    setName(moduleName.c_str());

    nameMainGrammar = rf.findFileByName(rf.check("nameMainGrammar", Value("mainLoopGrammar.xml")).toString());
    nameGrammarYesNo = rf.findFileByName(rf.check("nameGrammarYesNo", Value("nodeYesNo.xml")).toString());
    nameGrammarAskNameObject = rf.findFileByName(rf.check("GrammarAskNameObject", Value("GrammarAskNameObject.xml")).toString());
    nameGrammarAskNameAgent = rf.findFileByName(rf.check("GrammarAskNameAgent", Value("GrammarAskNameAgent.xml")).toString());


    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    bool  bEveryThingisGood = true;

    double seed = (double)time(NULL);
    int count = 0;
    while (seed == time(NULL)){ ++count; }
    seed = (int)seed % 100;
    srand((int)seed);
    yInfo() << " seed is: " << seed;
    yInfo() << " rand is: " << rand();

    // Open port2reasoning
    port2abmReasoningName = "/";
    port2abmReasoningName += getName() + "/toAbmR";

    if (!Port2abmReasoning.open(port2abmReasoningName.c_str())) {
        yInfo() << getName() << " : Unable to open port " << port2abmReasoningName;
        bEveryThingisGood &= false;
    }
    bEveryThingisGood &= Network::connect(port2abmReasoningName.c_str(), "/abmReasoning/rpc");

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "qRM", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= false;
    if (!iCub->connect())
    {
        yInfo() << " iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    //  calibrationThread = new AutomaticCalibrationThread(100,"ical");
    string test;
    //  calibrationThread->start();
    //  calibrationThread->suspend();

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    if (!iCub->getRecogClient())
    {
        yInfo() << " WARNING SPEECH RECOGNIZER NOT CONNECTED";
    }
    if (!iCub->getABMClient())
    {
        yInfo() << " WARNING ABM NOT CONNECTED";
    }

    //  calibrationRT("right");

    //  populateOpc();

    //  nodeTest();

    return true;
}


bool qRM::close() {
    iCub->close();
    delete iCub;

    return true;
}


bool qRM::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "help") {
        yInfo() << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString() == "calib")
    {
        yInfo() << " calibration of the RT";
        if (command.size() == 2)
        {
            yInfo() << " default : left hand";
            reply = calibrationRT("left");
        }
        else
        {
            yInfo() << " using " << command.get(1).asString() << " hand";
            reply = calibrationRT(command.get(1).asString());
        }
    }
    else if (command.get(0).asString() == "exploreEntity") {
        yInfo() << " exploreEntity";
        reply = exploreEntity(command);
    }


    return true;
}

/* Called periodically every getPeriod() seconds */
bool qRM::updateModule() {
    // mainLoop();
    return true;
}


Bottle qRM::calibrationRT(std::string side)
{
    Bottle bOutput;
    string sOutput;
    string test;

    if (iCub->getABMClient())
    {
        list<pair<string, string> > arguments;

        arguments.push_back(pair<string, string>("table", "table"));
        iCub->getABMClient()->sendActivity("activity", "calibration", "navigation", arguments, true);
    }

    //Calibrate
    iCub->say("Now I will self calibrate.");
    // iCub->look("cursor_0");
    //Start the thread that will get the points pairs
    calibrationThread->clear();

    calibrationThread->resume();

    //this is blocking until the calibration is done
    slidingController_IDL* slidingClient = new slidingController_IDL;

    if (side == "right")
    {
        slidingClient = iCub->getSlidingController()->clientIDL_slidingController_right;
    }
    else
    {
        slidingClient = iCub->getSlidingController()->clientIDL_slidingController_left;
    }


    calibrationThread->resume();
    slidingClient->explore();
    calibrationThread->suspend();

    iCub->say("Great! Now I better understand this table.");
    calibrationThread->updateCalibration();

    delete slidingClient;


    if (iCub->getABMClient())
    {
        list<pair<string, string> > arguments;

        arguments.push_back(pair<string, string>("table", "table"));
        iCub->getABMClient()->sendActivity("activity", "calibration", "navigation", arguments, false);
    }

    bOutput.addString("calibration to the reactable endded using the hand " + side);

    return bOutput;
}


void  qRM::mainLoop()
{
    ostringstream osError;     // Error message

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bSpeak, // bottle for tts
        bTemp;

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(nameMainGrammar).c_str());

    Bottle bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameMainGrammar));

    while (!fGetaReply)
    {
        //   Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        yInfo() << " Reply from Speech Recog : " << bSpeechRecognized.toString();

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << " Check " << nameMainGrammar;
            bOutput.addString(osError.str());
            yInfo() << osError.str();
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << " Grammar not recognized";
            bOutput.addString(osError.str());
            yInfo() << osError.str();
        }

        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }

    }
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)


    if (bAnswer.get(0).asString() == "stop")
    {
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        yInfo() << osError.str();
    }


    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    // semantic is the list of the semantic elements of the sentence except the type ef sentence
    bSemantic = *bAnswer.get(1).asList()->get(1).asList();


    if (sQuestionKind == "LOOK")
    {
        string sObject = bSemantic.check("words", Value("none")).asString();
        list<pair<string, string> > lArgument;

        lArgument.push_back(pair<string, string>(sObject, "word"));

        vector<Object*>   presentObjects;
        iCub->opc->checkout();
        list<Entity*> entities = iCub->opc->EntitiesCache();
        presentObjects.clear();
        for (list<Entity*>::iterator it = entities.begin(); it != entities.end(); it++)
        {
            //!!! ONLY RT_OBJECT and AGENTS ARE TRACKED !!!
            if (((*it)->isType(EFAA_OPC_ENTITY_RTOBJECT) || (*it)->isType(EFAA_OPC_ENTITY_AGENT)) && ((Object*)(*it))->m_present)
            {
                presentObjects.push_back((Object*)(*it));
            }
        }


        double maxSalience = 0;
        string nameTrackedObject = "none";
        for (vector<Object*>::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
        {
            if (maxSalience < (*it)->m_saliency)
            {
                maxSalience = (*it)->m_saliency;
                nameTrackedObject = (*it)->name();
            }
        }


        yInfo() << " Most salient is : " << nameTrackedObject << " with saliency=" << maxSalience;
        lArgument.push_back(pair<string, string>(nameTrackedObject, "focus"));

        iCub->getABMClient()->sendActivity("says", "look", "sentence", lArgument, true);

    }
    else if (sQuestionKind == "SHOW")
    {
        string sWord = bSemantic.check("words", Value("none")).asString();
        bSendReasoning.addString("askWordKnowledge");
        bSendReasoning.addString("getObjectFromWord");
        bSendReasoning.addString(sWord);
        Port2abmReasoning.write(bSendReasoning, bMessenger);

        list<pair<string, string> > lArgument;

        lArgument.push_back(pair<string, string>(sWord, "word"));

        yInfo() << " bMessenger is : " << bMessenger.toString();

        iCub->getABMClient()->sendActivity("says", "look", "sentence", lArgument, true);

    }
    else if (sQuestionKind == "WHAT")
    {
        bSendReasoning.addString("askWordKnowledge");
        bSendReasoning.addString("getWordFromObject");

        vector<Object*>   presentObjects;
        iCub->opc->checkout();
        list<Entity*> entities = iCub->opc->EntitiesCache();
        presentObjects.clear();
        for (list<Entity*>::iterator it = entities.begin(); it != entities.end(); it++)
        {
            //!!! ONLY RT_OBJECT and AGENTS ARE TRACKED !!!
            if (((*it)->isType(EFAA_OPC_ENTITY_RTOBJECT) || (*it)->isType(EFAA_OPC_ENTITY_AGENT)) && ((Object*)(*it))->m_present)
            {
                presentObjects.push_back((Object*)(*it));
            }
        }


        double maxSalience = 0;
        string nameTrackedObject = "none";
        for (vector<Object*>::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
        {
            if (maxSalience < (*it)->m_saliency)
            {
                maxSalience = (*it)->m_saliency;
                nameTrackedObject = (*it)->name();
            }
        }

        yInfo() << " Most salient is : " << nameTrackedObject << " with saliency=" << maxSalience;

        list<pair<string, string> > lArgument;

        lArgument.push_back(pair<string, string>(nameTrackedObject, "word"));

        yInfo() << " bMessenger is : " << bMessenger.toString();

        iCub->getABMClient()->sendActivity("says", "look", "sentence", lArgument, true);

        bSendReasoning.addString(nameTrackedObject);
        Port2abmReasoning.write(bSendReasoning, bMessenger);

        yInfo() << " bMessenger is : " << bMessenger.toString();

    }
}


void  qRM::nodeTest()
{
    ostringstream osError;     // Error message

    iCub->say("Yep ?");
    yInfo() << " Yep ? ";

    Bottle bOutput;

    //bool fGetaReply = false;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bMessenger; //to be send TO speech recog

    //   bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(testMax1), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        return;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)


    if (bAnswer.get(0).asString() == "stop")
    {
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        yInfo() << osError.str();
    }

    yInfo() << " bRecognized " << bRecognized.toString();

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarYesNo), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        return;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)


    if (bAnswer.get(0).asString() == "stop")
    {
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        yInfo() << osError.str();
    }

    yInfo() << " bRecognized " << bRecognized.toString();

    nodeTest();
}

void  qRM::nodeSentenceTemporal()
{
    ostringstream osError;     // Error message

    iCub->say("Yep ?");
    yInfo() << " Yep ? ";

    Bottle bOutput;

    //bool fGetaReply = false;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bMessenger; //to be send TO speech recog

    //    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarSentenceTemporal), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        return;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)


    if (bAnswer.get(0).asString() == "stop")
    {
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        yInfo() << osError.str();
    }

    yInfo() << " bRecognized " << bRecognized.toString();

    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    // semantic is the list of the semantic elements of the sentence except the type ef sentence
    bSemantic = *bAnswer.get(1).asList()->get(1).asList();

    string sObject = bSemantic.check("object", Value("none")).asString();
    string sAgent = bSemantic.check("agent", Value("none")).asString();
    string sVerb = bSemantic.check("action", Value("none")).asString();
    string sLocation = bSemantic.check("location", Value("none")).asString();
    string sAdverb = bSemantic.check("adverb", Value("none")).asString();

    list<pair<string, string> > lArgument;

    if (sObject != "none") { lArgument.push_back(pair<string, string>(sObject, "object")); }
    if (sAgent != "none") { lArgument.push_back(pair<string, string>(sAgent, "agent")); }
    if (sVerb != "none") { lArgument.push_back(pair<string, string>(sVerb, "action")); }
    if (sLocation != "none") { lArgument.push_back(pair<string, string>(sLocation, "adv")); }
    if (sAdverb != "none") { lArgument.push_back(pair<string, string>(sAdverb, "adv")); }

    if (sAgent == "none" || sVerb == "none" || sObject == "none")
    {
        iCub->say("I didn't really catch what you said...");
        nodeSentenceTemporal();
        return;
    }

    ostringstream osResponse;
    osResponse << " So, you said that " << (sAgent == "I") ? "you " : ((sAgent == "You") ? "I" : sAgent);
    osResponse << " will " << sVerb << " to the " << sLocation << " " << sAdverb << " ?";

    iCub->say(osResponse.str().c_str());
    yInfo() << " osResponse";
    if (!nodeYesNo())
    {
        nodeSentenceTemporal();
        return;
    }

    iCub->getABMClient()->sendActivity("action", sVerb, "qRM", lArgument, true);

    yInfo() << " Wait for end signal";

    nodeYesNo();    // wait for end of action

    iCub->getABMClient()->sendActivity("action", sVerb, "qRM", lArgument, false);

    iCub->say("Ok ! Another ?");

    yInfo() << " Another ? ";
    if (nodeYesNo())
    {
        nodeSentenceTemporal();
        return;
    }
    else
    {
        return;
    }
}



bool qRM::nodeYesNo()
{
    //bool fGetaReply = false;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bMessenger; //to be send TO speech recog

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarYesNo), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        yInfo() << bRecognized.get(1).toString();
        return false;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

    if (bAnswer.get(0).asString() == "yes")    return true;

    return false;
}



bool qRM::populateOpc(){
    iCub->opc->update();
    iCub->opc->commit();

    Agent* agent = iCub->opc->addAgent("Carol");
    agent->m_ego_position[0] = -1.4;
    agent->m_ego_position[2] = 0.60;
    agent->m_present = 1;
    agent->m_color[0] = 200;
    agent->m_color[1] = 50;
    agent->m_color[2] = 50;
    iCub->opc->commit();

    return true;
}

Bottle qRM::exploreEntity(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 3)
    {
        yInfo() << " qRM::exploreEntity | Problem in input size.";
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
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarAskNameAgent), 20);

        if (bRecognized.get(0).asInt() == 0)
        {
            yWarning() << " error in qRM::exploreEntity | askNameAgent | Error in speechRecog";
            bOutput.addString("error");
            bOutput.addString("error in speechRecog");
            return bOutput;
        }

        bAnswer = *bRecognized.get(1).asList();
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo() << " in qRM::exploreEntity | askNameAgent | stop called";
            bOutput.addString("error");
            bOutput.addString("stop called");
            return bOutput;
        }

        bSemantic = *bAnswer.get(1).asList();
        string sName = bSemantic.check("agent", Value("unknown")).asString();

        Agent* agentToChange = iCub->opc->addAgent(sNameTarget);
        Time::delay(timeDelay);

        agentToChange->m_present = false;
        iCub->opc->commit(agentToChange);
        Time::delay(timeDelay);

        agentToChange->changeName(sName);
        iCub->opc->commit(agentToChange);
        Time::delay(timeDelay);

        agentToChange->m_present = true;
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
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarAskNameObject), 20);

        if (bRecognized.get(0).asInt() == 0)
        {
            yWarning() << " error in qRM::exploreEntity | askNameObject | Error in speechRecog";
            bOutput.addString("error");
            bOutput.addString("error in speechRecog");
            return bOutput;
        }

        bAnswer = *bRecognized.get(1).asList();
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo() << " in qRM::exploreEntity | askNameObject | stop called";
            bOutput.addString("error");
            bOutput.addString("stop called");
            return bOutput;
        }

        yInfo() << " bAnswer is: " << bAnswer.toString();
        bSemantic = *bAnswer.get(1).asList();
        yInfo() << " bSemantic is: " << bSemantic.toString();

        string sName = bSemantic.check("object", Value("unknown")).asString();

        Object* objectToChange = iCub->opc->addObject(sNameTarget);
        objectToChange->m_present = false;
        iCub->opc->commit(objectToChange);

        Time::delay(0.5);

        objectToChange->changeName(sName);
        objectToChange->m_present = true;
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
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarAskNameObject), 20);

        if (bRecognized.get(0).asInt() == 0)
        {
            yWarning() << " error in qRM::exploreEntity | askNameRTObject | Error in speechRecog";
            bOutput.addString("error");
            bOutput.addString("error in speechRecog");
            return bOutput;
        }

        bAnswer = *bRecognized.get(1).asList();
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo() << " in qRM::exploreEntity | askNameRTObject | stop called";
            bOutput.addString("error");
            bOutput.addString("stop called");
            return bOutput;
        }

        yInfo() << " bAnswer is: " << bAnswer.toString();
        bSemantic = *bAnswer.get(1).asList();
        yInfo() << " bSemantic is: " << bSemantic.toString();

        string sName = bSemantic.check("object", Value("unknown")).asString();

        RTObject* objectToChange = iCub->opc->addRTObject(sNameTarget);
        objectToChange->m_present = false;
        iCub->opc->commit(objectToChange);

        Time::delay(0.5);

        objectToChange->changeName(sName);
        objectToChange->m_present = true;
        iCub->opc->commit(objectToChange);
        yInfo() << " So this is a " << sName;
        iCub->say(" So this is a " + sName);

        bOutput.addString("success");
        bOutput.addString("rtobject");
        return bOutput;
    }

    return bOutput;
}
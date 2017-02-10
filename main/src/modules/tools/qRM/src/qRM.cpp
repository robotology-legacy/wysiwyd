#include "qRM.h"
#include "wrdac/subsystems/subSystem_ABM.h"
#include "wrdac/subsystems/subSystem_slidingCtrl.h"
#include "wrdac/subsystems/subSystem_recog.h"
#include "wrdac/subsystems/subSystem_ARE.h"
#include "wrdac/subsystems/subSystem_speech.h"

bool qRM::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("qRM")).asString().c_str();
    setName(moduleName.c_str());

    MainGrammar = rf.findFileByName(rf.check("MainGrammar", Value("LoopGrammar.xml")).toString());
    GrammarYesNo = rf.findFileByName(rf.check("GrammarYesNo", Value("nodeYesNo.xml")).toString());
    GrammarAskNameObject = rf.findFileByName(rf.check("GrammarAskNameObject", Value("GrammarAskNameObject.xml")).toString());
    GrammarAskNameAgent = rf.findFileByName(rf.check("GrammarAskNameAgent", Value("GrammarAskNameAgent.xml")).toString());
    grammarAction = rf.findFileByName(rf.check("grammarAction", Value("grammarAction.xml")).toString());
    thresholdDistinguishObjectsRatio = rf.check("thresholdDistinguishObjectsRatio", Value(3.0)).asDouble();
    thresholdSalienceDetection = rf.check("thresholdSalienceDetection", Value(2.0)).asDouble();

    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    bool  bEveryThingisGood = true;

    double seed = (double)time(NULL);
    int count = 0;
    while (seed == time(NULL)){ ++count; }
    seed = (int)seed % 100;
    srand((int)seed);


    // Open port2reasoning
    port2abmReasoningName = "/";
    port2abmReasoningName += getName() + "/toAbmR";

    if (!Port2abmReasoning.open(port2abmReasoningName.c_str())) {
        yInfo() << getName() << " : Unable to open port " << port2abmReasoningName;
        bEveryThingisGood = false;
    }
    bEveryThingisGood &= Network::connect(port2abmReasoningName.c_str(), "/abmReasoning/rpc");

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "qRM", "client.ini", isRFVerbose);
    iCub->opc->isVerbose = false;
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
        "quit \n"
        "exploreUnknowneEntity entity_type entity_name \n" +
        "exploreEntityByName entity_name \n" +
        "executeSharedPlan  ('action' name_plan 'sharedplan') (<arg 1>  <arg 2>  ...  <arg n>)  (<role 1>  <role 2 >  ... <role n>) \n" +
        "executeAction ('predicate' action_name) ('agent' agent_name) ('object' object_name) ('recipient' adjective_name) \n" +
        "learnSharedPlan sharedplan_name\n" +
        "populateOPC\n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "populateABMGive") {
        if (command.size() == 2)
        {
            populateABMGive(command.get(1).asInt());
        }
        else
        {
            populateABMGive();
        }
        reply.addString("populating ABM");
        return true;
    }
    else if (command.get(0).asString() == "populateABMTake") {
        if (command.size() == 2)
        {
            populateABMTake(command.get(1).asInt());
        }
        else
        {
            populateABMTake();
        }
        reply.addString("populating ABM");
        return true;
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
    else if (command.get(0).asString() == "exploreUnknownEntity") {
        yInfo() << " exploreUnknownEntity";
        reply = exploreUnknownEntity(command);
    }
    else if (command.get(0).asString() == "exploreEntityByName") {
        yInfo() << " exploreEntityByName";
        reply = exploreEntityByName(command);
    }
    else if (command.get(0).asString() == "executeSharedPlan") {
        yInfo() << " executeSharedPlan";
        reply = executeSharedPlan(command);
    }
    else if (command.get(0).asString() == "executeAction") {
        yInfo() << " executeAction";
        reply = executeAction(command);
    }
    else if (command.get(0).asString() == "learnSharedPlan") {
        yInfo() << " learnSharedPlan";
        reply = learnSharedPlan(command);
    }
    else {
        yInfo() << helpMessage;
        reply.addString("wrong command");
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
    bMessenger.addString(grammarToString(MainGrammar).c_str());

    Bottle bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(MainGrammar));

    while (!fGetaReply)
    {
        //   Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        yInfo() << " Reply from Speech Recog : " << bSpeechRecognized.toString();

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << " Check " << MainGrammar;
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
            if (((*it)->isType(EFAA_OPC_ENTITY_RTOBJECT) || (*it)->isType(EFAA_OPC_ENTITY_AGENT)) && (dynamic_cast<Object*>(*it))->m_present==1.0)
            {
                presentObjects.push_back(dynamic_cast<Object*>(*it));
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
            if (((*it)->isType(EFAA_OPC_ENTITY_RTOBJECT) || (*it)->isType(EFAA_OPC_ENTITY_AGENT)) && (dynamic_cast<Object*>(*it))->m_present==1.0)
            {
                presentObjects.push_back(dynamic_cast<Object*>(*it));
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

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarYesNo), 20);

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
    osResponse << " So, you said that " << (((sAgent == "I")) ? "you " : ((sAgent == "You") ? "I" : sAgent));
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
        bAnswer; //response from speech recog without transfer information, including raw sentence

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarYesNo), 20);

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

    Agent* agent = iCub->opc->addOrRetrieveEntity<Agent>("Carol");
    agent->m_ego_position[0] = -1.4;
    agent->m_ego_position[2] = 0.60;
    agent->m_present = 1.0;
    agent->m_color[0] = 200;
    agent->m_color[1] = 50;
    agent->m_color[2] = 50;
    iCub->opc->commit();

    return true;
}


/*
* Ask the name of an unknown entity
* input: exploreUnknownEntity entityType entityName (eg: exploreUnknownEntity agent unknown_25)
* ask through speech the name of an unknwon entity
*/
Bottle qRM::exploreUnknownEntity(Bottle bInput)
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
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameAgent), 20);

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

        Agent* agentToChange = dynamic_cast<Agent*>(iCub->opc->getEntity(sNameTarget));
        iCub->changeName(agentToChange,sName);
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

        Object* objectToChange = dynamic_cast<Object*>(iCub->opc->getEntity(sNameTarget));
        iCub->changeName(objectToChange,sName);
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

        RTObject* objectToChange = dynamic_cast<RTObject*>(iCub->opc->getEntity(sNameTarget));
        iCub->changeName(objectToChange,sName);
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
Bottle qRM::exploreEntityByName(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 2)
    {
        yInfo() << " qRM::exploreEntityByName | Problem in input size.";
        bOutput.addString("Problem in input size");
        return bOutput;
    }

    string sNameTarget = bInput.get(1).toString();
    yInfo() << " Entity to find: " << sNameTarget;

    // check if the entity is already present in the OPC
    if (iCub->opc->isConnected())
    {
        iCub->opc->checkout();
        list<Entity*> lEntities = iCub->opc->EntitiesCache();

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
        yWarning() << " in qRM::exploreEntityByName | OPC not Connected";
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
        list<Entity*> lEntities = iCub->opc->EntitiesCache();

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
            Entity* e = iCub->opc->getEntity(sNameBestEntity);
            iCub->changeName(e, sNameBestEntity);
            yInfo() << " name changed: " << sNameBestEntity << " is now " << sNameTarget;
            bOutput.addString("name changed");
        }
    }

    iCub->opc->commit();

    return bOutput;
}

/*
* executeSharedPlan  ('action' name_action 'sharedplan') (<arg 1>  <arg 2>  ...  <arg n>)  (<role 1>  <role 2 >  ... <role n>)
*/

Bottle qRM::executeSharedPlan(Bottle bInput)
{
    Bottle bOutput,
        bPlanFromReasoning;

    Port2abmReasoning.write(bInput, bPlanFromReasoning);


    yInfo() << " " << bPlanFromReasoning.toString() << "\n";

    if (bPlanFromReasoning.get(0).toString() == "NACK" || bPlanFromReasoning.get(0).toString() == "nack")
    {
        yInfo() << " in qRM::executeSharedPlan | error from reasoning";
        bOutput = bPlanFromReasoning;
        return bOutput;
    }

    if (bPlanFromReasoning.size() < 2)
    {
        yInfo() << " in qRM::executeSharedPlan | wrong size of bottle from reasoning";
        bOutput.addString("NACK");
        bOutput.addString("in qRM::executeSharedPlan | wrong size of bottle from reasoning");
        return bOutput;
    }

    Bottle bPlan = *bPlanFromReasoning.get(1).asList();
    string sPredicate,
        sAgent,
        sObject,
        sRecipient,
        sEffect;


    for (int i = 0; i < bPlan.size(); i++)
    {
        if (bPlan.get(i).isList())
        {
            Bottle bTemp = *bPlan.get(i).asList();
            executeAction(bTemp);
        }
    }

    bOutput.addString("done");
    return bOutput;

}



/*
* dialogue for learning a shared plan through ABM and ABMReasoning
* input: "learnSharedPlan" shared_plan_name
*/
Bottle qRM::learnSharedPlan(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 2)
    {
        yInfo() << " in qRM::learnSharedPlan:: wrong size of input";
        bOutput.addString("nack");
        bOutput.addString("in qRM::learnSharedPlan:: wrong size of input");
        return bOutput;
    }

    string sNameSp = bInput.get(1).toString();


    list<pair<string, string> > lpArg;

    iCub->getABMClient()->sendActivity("sharedplan",
        sNameSp,
        "qRM",
        lpArg,
        true);

    string sSentence = "Ok, can you show me how to " + sNameSp;
    yInfo() << " " << sSentence;
    iCub->say(sSentence);

    bool bSPonGoing = true;

    Bottle bRecognized,
        bAnswer,
        bSemantic;

    string sPredicate;
    string sAgent;
    string sObject;
    string sRecipient;

    bool fActionStarted = false;

    Bottle ABMR_end_Action;
    ABMR_end_Action.addString("addLastActivity");
    ABMR_end_Action.addString("action");
    Bottle bOPCUpdateLocation;
    bOPCUpdateLocation.addString("updateOpcObjectLocation");
    bOPCUpdateLocation.addString("OPC");


    while (bSPonGoing)
    {
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(grammarAction));

        if (bRecognized.get(0).asInt() == 0)
        {
            yWarning() << " error in qRM::exploreEntity | askNameAgent | Error in speechRecog";
            bOutput.addString("error");
            bOutput.addString("error in speechRecog");
            return bOutput;
        }

        bAnswer = *bRecognized.get(1).asList();
        /*
        * Human had an ongoing action it might be ended.
        */
        if (fActionStarted)
        {
            iCub->getABMClient()->sendActivity("action",
                sPredicate,
                "qRM",
                lpArg,
                false);
            Port2abmReasoning.write(ABMR_end_Action);
            Port2abmReasoning.write(bOPCUpdateLocation);
            fActionStarted = false;
        }

        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        /*
        * Human signals the end of the SP
        */
        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo() << " in qRM::learnSharedPlan | end of SP.";
            bSPonGoing = false;
        }
        else
        {
            /*
            * Plan is not finished
            */

            bSemantic = *bAnswer.get(1).asList();

            /*
            * TODO
            * Use reservoir to treat the sentence and get PAOR.
            */

            sPredicate = bSemantic.check("predicate", Value("none")).asString();
            sAgent = bSemantic.check("agent", Value("none")).asString();
            sObject = bSemantic.check("object", Value("none")).asString();
            sRecipient = bSemantic.check("recipient", Value("none")).asString();

            if (sPredicate == "none")
            {
                yInfo() << " in qRM::learnSharedPlan:: problem of recognition of action: no predicate";
                iCub->say("What did you said bro ?");
            }
            else
            {

                lpArg.clear();
                lpArg.push_back(pair<string, string>(sPredicate, "prediacte"));
                lpArg.push_back(pair<string, string>(sAgent, "agent"));
                lpArg.push_back(pair<string, string>(sObject, "object"));
                lpArg.push_back(pair<string, string>(sRecipient, "recipient"));

                iCub->getABMClient()->sendActivity("action",
                    sPredicate,
                    "qRM",
                    lpArg,
                    true);

                /*
                * The iCub is agent: ask how to to ABMR, then execute it
                *
                */
                if (sAgent == "you" || sAgent == "iCub")
                {
                    Bottle bActionSent,
                        bActionGet;
                    bActionSent.addString("executeAction");
                    bActionSent.addString(sPredicate);
                    bActionSent.addString(sRecipient);
                    bActionSent.addString(sObject);
                    bActionSent.addString(sAgent);

                    Port2abmReasoning.write(bActionSent, bActionGet);

                    executeAction(bActionGet);
                }
                else
                {
                    /*
                    * the Human will do the action
                    */
                    fActionStarted = true;
                }
            }
        }
    }


    iCub->getABMClient()->sendActivity("sharedplan",
        sNameSp,
        "qRM",
        lpArg,
        false);


    Bottle ABMR_end_SP;
    ABMR_end_SP.addString("addLastActivity");
    ABMR_end_SP.addString("sharedplan");
    Port2abmReasoning.write(ABMR_end_SP);



    return bOutput;
}


/*
* ask the ABMR for the action to perform, and execute it with ARE
* bInput: ("predicate" action_name) ("agent" agent_name) ("object" object_name) ("recipient" adjective_name)
*/
Bottle qRM::executeAction(Bottle bInput)
{
    Bottle bOutput;
    Bottle bEffect;


    Bottle ABMR_end_Action;
    ABMR_end_Action.addString("addLastActivity");
    ABMR_end_Action.addString("action");
    Bottle bOPCUpdateLocation;
    bOPCUpdateLocation.addString("updateOpcObjectLocation");
    bOPCUpdateLocation.addString("OPC");

    string sEffect;

    string sPredicate = bInput.check("predicate", Value("none")).asString();
    string sAgent = bInput.check("agent", Value("none")).asString();
    string sObject = bInput.check("object", Value("none")).asString();
    string sRecipient = bInput.check("recipient", Value("none")).asString();

    list<pair<string, string > > lpArg;
    lpArg.push_back(pair<string, string>(sPredicate, "prediacte"));
    lpArg.push_back(pair<string, string>(sAgent, "agent"));
    lpArg.push_back(pair<string, string>(sObject, "object"));
    lpArg.push_back(pair<string, string>(sRecipient, "recipient"));

    if (bInput.findGroup("effect").size() != 2)
    {
        yInfo() << " qRM::executeSharedPlan:: no effect of action";
        sEffect = "none";
    }
    else
    {
        bEffect = *bInput.findGroup("effect").get(1).asList();
        sEffect = bEffect.toString();
    }


    yInfo() << " predicate: " << sPredicate;
    yInfo() << " agent:     " << sAgent;
    yInfo() << " object:    " << sObject;
    yInfo() << " recipient: " << sRecipient;
    yInfo() << " effect:    " << sEffect << "\n";

    //            if (sPredicate == "none" || sAgent == "none" || sObject == "none" || sRecipient == "none")

    if (sAgent == "iCub")
    {
        iCub->opc->checkout();
        if (bEffect.get(0).toString() == "absolute")
        {

            if (sObject == "none")
            {
                yInfo() << " Error in qRM::executeSharedPlan. Object is set at 'none'";
            }
            else
            {

                Entity *EntToGrasp = iCub->opc->getEntity(sObject);
                if (EntToGrasp == NULL)
                {
                    ostringstream osSentence;
                    osSentence << " Franckly my dear, I don't think I'll be able to " << sPredicate << " the " << sObject << " to the " << sRecipient;
                    yInfo() << " " << osSentence.str();
                    iCub->say(osSentence.str());
                }
                else if (EntToGrasp->isType("object"))
                {
                    Object *toGrasp = dynamic_cast<Object*>(iCub->opc->getEntity(sObject));
                    if (toGrasp->m_present==1.0)
                    {
                        yarp::sig::Vector coordToGrasp = toGrasp->m_ego_position;
                        iCub->getABMClient()->sendActivity("action",
                            sPredicate,
                            "qRM",
                            lpArg,
                            true);
                        yInfo() << " trying to grasp at location: " << coordToGrasp.toString();
                        iCub->take(coordToGrasp);

                        coordToGrasp[0] = bEffect.get(1).asDouble();
                        coordToGrasp[1] = bEffect.get(2).asDouble();

                        yInfo() << " trying to drop at location: " << coordToGrasp.toString();
                        iCub->release(coordToGrasp);
                        iCub->getABMClient()->sendActivity("action",
                            sPredicate,
                            "qRM",
                            lpArg,
                            false);
                        Port2abmReasoning.write(ABMR_end_Action);
                        Port2abmReasoning.write(bOPCUpdateLocation);
                    }
                }
                else if (EntToGrasp->isType("RTObject"))
                {
                    RTObject *toGrasp = dynamic_cast<RTObject*>(iCub->opc->getEntity(sObject));
                    if (toGrasp->m_present==1.0)
                    {
                        yarp::sig::Vector coordToGrasp = toGrasp->m_ego_position;
                        iCub->getABMClient()->sendActivity("action",
                            sPredicate,
                            "qRM",
                            lpArg,
                            true);
                        yInfo() << " trying to grasp at location: " << coordToGrasp.toString();
                        iCub->take(coordToGrasp);

                        coordToGrasp[0] = bEffect.get(1).asDouble();
                        coordToGrasp[1] = bEffect.get(2).asDouble();

                        yInfo() << " trying to drop at location: " << coordToGrasp.toString();
                        iCub->release(coordToGrasp);
                        iCub->getABMClient()->sendActivity("action",
                            sPredicate,
                            "qRM",
                            lpArg,
                            false);
                        Port2abmReasoning.write(ABMR_end_Action);
                        Port2abmReasoning.write(bOPCUpdateLocation);
                    }
                }
            }
        }
        else if (bEffect.get(0).toString() == "relative")
        {
            if (sObject == "none")
            {
                yInfo() << " Error in qRM::executeSharedPlan. Object is set at 'none'";
            }
            else
            {
                Entity *EntToGrasp = iCub->opc->getEntity(sObject);
                if (EntToGrasp == NULL)
                {
                    ostringstream osSentence;
                    osSentence << " Francly my dear, I don't think I'll be able to " << sPredicate << " the " << sObject << " to the " << sRecipient;
                    yInfo() << " " << osSentence.str();
                    iCub->say(osSentence.str());
                }
                else if (EntToGrasp->isType("object"))
                {
                    Object *toGrasp = iCub->opc->addEntity<Object>(sObject);
                    yarp::sig::Vector coordToGrasp = toGrasp->m_ego_position;
                    iCub->getABMClient()->sendActivity("action",
                        sPredicate,
                        "qRM",
                        lpArg,
                        true);
                    yInfo() << " trying to grasp at location: " << coordToGrasp.toString();
                    iCub->take(coordToGrasp);

                    coordToGrasp[0] += bEffect.get(1).asDouble();
                    coordToGrasp[1] += bEffect.get(2).asDouble();

                    yInfo() << " trying to drop at location: " << coordToGrasp.toString();
                    iCub->release(coordToGrasp);
                    iCub->getABMClient()->sendActivity("action",
                        sPredicate,
                        "qRM",
                        lpArg,
                        false);
                    Port2abmReasoning.write(ABMR_end_Action);
                    Port2abmReasoning.write(bOPCUpdateLocation);
                }
                else if (EntToGrasp->isType("RTObject"))
                {
                    RTObject *toGrasp = dynamic_cast<RTObject*>(iCub->opc->getEntity(sObject));
                    yarp::sig::Vector coordToGrasp = toGrasp->m_ego_position;
                    iCub->getABMClient()->sendActivity("action",
                        sPredicate,
                        "qRM",
                        lpArg,
                        true);
                    yInfo() << " trying to grasp at location: " << coordToGrasp.toString();
                    iCub->take(coordToGrasp);

                    coordToGrasp[0] += bEffect.get(1).asDouble();
                    coordToGrasp[1] += bEffect.get(2).asDouble();

                    yInfo() << " trying to drop at location: " << coordToGrasp.toString();
                    iCub->release(coordToGrasp);
                    iCub->getABMClient()->sendActivity("action",
                        sPredicate,
                        "qRM",
                        lpArg,
                        false);
                    Port2abmReasoning.write(ABMR_end_Action);
                    Port2abmReasoning.write(bOPCUpdateLocation);
                }
            }

        }
    }
    else if (sAgent != "none")
    {
        ostringstream osSentence;
        osSentence << sAgent << ", you should " << sPredicate << " the " << sObject << " to the " << sRecipient << ".";
        string sentence = osSentence.str();

        iCub->say(sentence);
        yInfo() << "\n " << sentence << "\n";
        Time::delay(2.);
    }
    else
    {
        yInfo() << " in qRM::executeAction :: problem: doesn't know agent: " << sAgent;
    }


    return bOutput;
}




/*
*   Populate the ABM with predifined situations
*
*/
void qRM::populateABMGive(int iRepetition)
{
    vector<string> vObject;
    vObject.push_back("doudou");
    vObject.push_back("thing");
    vObject.push_back("bottle");
    vObject.push_back("phone");


    vector<string> vAgent;
    vAgent.push_back("larry");
    vAgent.push_back("robert");
    vAgent.push_back("john");
    vAgent.push_back("mike");
    vAgent.push_back("stuart");


    for (int ii = 0; ii < iRepetition; ii++){

        Time::delay(0.3);
        iCub->opc->clear();
        Time::delay(0.3);
        iCub->opc->checkout();
        Time::delay(0.3);

        string sAg1 = vAgent[Random::uniform(0, vAgent.size() - 1)];
        string sAg2 = sAg1;

        while (sAg2 == sAg1){
            sAg2 = vAgent[Random::uniform(0, vAgent.size() - 1)];
        }
        string sObj = vObject[Random::uniform(0, vObject.size() - 1)];

        Action* have = iCub->opc->addOrRetrieveEntity<Action>("have");

        yInfo() << "start action with: " << sAg1 << ", " << sAg2 << " and the " << sObj;

        Agent* ag1 = iCub->opc->addOrRetrieveEntity<Agent>(sAg1);
        double robert_X = -0.1 - 0.5 * Random::uniform();
        double robert_Y = -0.7 + 2 * Random::uniform();
        ag1->m_ego_position[0] = robert_X;
        ag1->m_ego_position[1] = robert_Y;
        ag1->m_ego_position[2] = 0.30;
        ag1->m_present = 1.0;
        ag1->m_color[0] = Random::uniform(0, 180);
        ag1->m_color[1] = Random::uniform(0, 80);
        ag1->m_color[2] = Random::uniform(180, 250);
        iCub->opc->commit(ag1);

        Agent* ag2 = iCub->opc->addOrRetrieveEntity<Agent>(sAg2);
        double larry_X = -0.1 - 0.5 * Random::uniform();
        double larry_Y = -1.2 + 2 * Random::uniform();
        ag2->m_ego_position[0] = larry_X;
        ag2->m_ego_position[1] = larry_Y;
        ag2->m_ego_position[2] = 0.30;
        ag2->m_present = 1.0;
        ag2->m_color[0] = Random::uniform(100, 180);
        ag2->m_color[1] = Random::uniform(80, 180);
        ag2->m_color[2] = Random::uniform(0, 80);
        iCub->opc->commit(ag2);

        Object* obj = iCub->opc->addOrRetrieveEntity<Object>(sObj);
        obj->m_ego_position[0] = larry_X + 0.1 * Random::uniform();
        obj->m_ego_position[1] = larry_Y + 0.1 * Random::uniform();
        obj->m_ego_position[2] = 0.0;
        obj->m_present = 1.0;
        obj->m_color[0] = Random::uniform(0, 80);
        obj->m_color[1] = Random::uniform(80, 180);
        obj->m_color[2] = Random::uniform(180, 250);
        iCub->opc->commit(obj);

        yInfo() << " begin action " << ii;

        iCub->opc->addRelation(ag2, have, obj);

        // send the result of recognition to the ABM
        list<pair<string, string> > lArgument;
        string sentence;
        sentence = "Give me the " + sObj;
        sentence += " slowly please.";
        lArgument.push_back(pair<string, string>(sentence, "sentence"));
        lArgument.push_back(pair<string, string>("give", "predicate"));
        lArgument.push_back(pair<string, string>(sAg2, "agent"));
        lArgument.push_back(pair<string, string>(sObj, "object"));
        lArgument.push_back(pair<string, string>(sAg1, "adj1"));
        lArgument.push_back(pair<string, string>("slowly", "adj2"));
        lArgument.push_back(pair<string, string>(sAg1, "speaker"));
        lArgument.push_back(pair<string, string>("none", "subject"));
        lArgument.push_back(pair<string, string>(sAg2, "addressee"));
        iCub->getABMClient()->sendActivity("action",
            "sentence",
            "recog",
            lArgument,
            true);

        Time::delay(1.0);

        lArgument.clear();

        lArgument.push_back(pair<string, string>(sAg2, "agent"));
        lArgument.push_back(pair<string, string>("give", "predicate"));
        lArgument.push_back(pair<string, string>(sObj, "object"));
        lArgument.push_back(pair<string, string>(sAg1, "recipient"));

        iCub->getABMClient()->sendActivity("action",
            "give",
            "action",
            lArgument,
            true);

        yInfo() << " in delay of action";
        Time::delay(4 + 4 * Random::uniform());

        obj->m_ego_position[0] = robert_X + 0.1 * Random::uniform();
        obj->m_ego_position[1] = robert_Y + 0.1 * Random::uniform();
        iCub->opc->commit(obj);

        iCub->opc->removeRelation(ag2, have, obj);
        iCub->opc->addRelation(ag1, have, obj);

        Time::delay(3);

        iCub->getABMClient()->sendActivity("action",
            "give",
            "action",
            lArgument,
            false);

        yInfo() << " end action " << ii;

        Time::delay(1.);
    }

    for (int ii = 0; ii < iRepetition; ii++){

        Time::delay(0.3);
        iCub->opc->clear();
        Time::delay(0.3);
        iCub->opc->checkout();
        Time::delay(0.3);

        string sAg1 = vAgent[Random::uniform(0, vAgent.size() - 1)];
        string sAg2 = sAg1;

        while (sAg2 == sAg1){
            sAg2 = vAgent[Random::uniform(0, vAgent.size() - 1)];
        }
        string sObj = vObject[Random::uniform(0, vObject.size() - 1)];
        Action* have = iCub->opc->addOrRetrieveEntity<Action>("have");

        yInfo() << "start action with: " << sAg1 << ", " << sAg2 << " and the " << sObj;

        Agent* ag1 = iCub->opc->addOrRetrieveEntity<Agent>(sAg1);
        double robert_X = -0.1 - 0.5 * Random::uniform();
        double robert_Y = -0.7 + 2 * Random::uniform();
        ag1->m_ego_position[0] = robert_X;
        ag1->m_ego_position[1] = robert_Y;
        ag1->m_ego_position[2] = 0.30;
        ag1->m_present = 1.0;
        ag1->m_color[0] = Random::uniform(0, 180);
        ag1->m_color[1] = Random::uniform(0, 80);
        ag1->m_color[2] = Random::uniform(180, 250);
        iCub->opc->commit(ag1);

        Agent* ag2 = iCub->opc->addOrRetrieveEntity<Agent>(sAg2);
        double larry_X = -0.1 - 0.5 * Random::uniform();
        double larry_Y = -1.2 + 2 * Random::uniform();
        ag2->m_ego_position[0] = larry_X;
        ag2->m_ego_position[1] = larry_Y;
        ag2->m_ego_position[2] = 0.30;
        ag2->m_present = 1.0;
        ag2->m_color[0] = Random::uniform(100, 180);
        ag2->m_color[1] = Random::uniform(80, 180);
        ag2->m_color[2] = Random::uniform(0, 80);
        iCub->opc->commit(ag2);

        Object* obj = iCub->opc->addOrRetrieveEntity<Object>(sObj);
        obj->m_ego_position[0] = larry_X + 0.1 * Random::uniform();
        obj->m_ego_position[1] = larry_Y + 0.1 * Random::uniform();
        obj->m_ego_position[2] = 0.0;
        obj->m_present = 1.0;
        obj->m_color[0] = Random::uniform(0, 80);
        obj->m_color[1] = Random::uniform(80, 180);
        obj->m_color[2] = Random::uniform(180, 250);
        iCub->opc->commit(obj);

        yInfo() << " begin action " << ii;

        iCub->opc->addRelation(ag2, have, obj);

        // send the result of recognition to the ABM
        list<pair<string, string> > lArgument;
        string sentence;
        sentence = "Give me the " + sObj;
        sentence += " quickly please.";
        lArgument.push_back(pair<string, string>(sentence, "sentence"));
        lArgument.push_back(pair<string, string>("give", "predicate"));
        lArgument.push_back(pair<string, string>(sAg2, "agent"));
        lArgument.push_back(pair<string, string>(sObj, "object"));
        lArgument.push_back(pair<string, string>(sAg1, "adj1"));
        lArgument.push_back(pair<string, string>("quickly", "adj2"));
        lArgument.push_back(pair<string, string>(sAg1, "speaker"));
        lArgument.push_back(pair<string, string>("none", "subject"));
        lArgument.push_back(pair<string, string>(sAg2, "addressee"));
        iCub->getABMClient()->sendActivity("action",
            "sentence",
            "recog",
            lArgument,
            true);

        Time::delay(1.0);

        lArgument.clear();

        lArgument.push_back(pair<string, string>(sAg2, "agent"));
        lArgument.push_back(pair<string, string>("give", "predicate"));
        lArgument.push_back(pair<string, string>(sObj, "object"));
        lArgument.push_back(pair<string, string>(sAg1, "recipient"));

        iCub->getABMClient()->sendActivity("action",
            "give",
            "action",
            lArgument,
            true);

        yInfo() << " in delay of action";
        Time::delay(1 + 3 * Random::uniform());

        obj->m_ego_position[0] = robert_X + 0.1 * Random::uniform();
        obj->m_ego_position[1] = robert_Y + 0.1 * Random::uniform();
        iCub->opc->commit(obj);

        iCub->opc->removeRelation(ag2, have, obj);
        iCub->opc->addRelation(ag1, have, obj);

        Time::delay(3);

        iCub->getABMClient()->sendActivity("action",
            "give",
            "action",
            lArgument,
            false);

        yInfo() << " end action " << ii;

        Time::delay(1.);
    }

}



/*
*   Populate the ABM with predifined situations
*
*/
void qRM::populateABMTake(int iRepetition)
{
    vector<string> vObject;
    vObject.push_back("doudou");
    vObject.push_back("thing");
    vObject.push_back("bottle");
    vObject.push_back("phone");


    vector<string> vAgent;
    vAgent.push_back("larry");
    vAgent.push_back("robert");
    vAgent.push_back("john");
    vAgent.push_back("mike");
    vAgent.push_back("stuart");


    for (int ii = 0; ii < iRepetition; ii++){

        Time::delay(0.3);
        iCub->opc->clear();
        Time::delay(0.3);
        iCub->opc->checkout();
        Time::delay(0.3);

        string sAg1 = vAgent[Random::uniform(0, vAgent.size() - 1)];
        string sObj = vObject[Random::uniform(0, vObject.size() - 1)];

        Action* have = iCub->opc->addOrRetrieveEntity<Action>("have");

        yInfo() << "start action with: " << sAg1 << " and the " << sObj;

        Agent* ag1 = iCub->opc->addOrRetrieveEntity<Agent>(sAg1);
        double XAG= -0.1 - 0.5 * Random::uniform();
        double YAG = -0.7 + 2 * Random::uniform();
        ag1->m_ego_position[0] = XAG;
        ag1->m_ego_position[1] = YAG;
        ag1->m_ego_position[2] = 0.30;
        ag1->m_present = 1.0;
        ag1->m_color[0] = Random::uniform(0, 180);
        ag1->m_color[1] = Random::uniform(0, 80);
        ag1->m_color[2] = Random::uniform(180, 250);
        iCub->opc->commit(ag1);



        double theta = 3.14 * 2 * Random::uniform();
        double length = 0.3 + 0.2*Random::uniform();

        double xObj = cos(theta) * length + XAG;
        double yObj = sin(theta) * length + YAG;

        Object* obj = iCub->opc->addOrRetrieveEntity<Object>(sObj);
        obj->m_ego_position[0] = xObj;
        obj->m_ego_position[1] = yObj;
        obj->m_ego_position[2] = 0.0;
        obj->m_present = 1.0;
        obj->m_color[0] = Random::uniform(0, 80);
        obj->m_color[1] = Random::uniform(80, 180);
        obj->m_color[2] = Random::uniform(180, 250);
        iCub->opc->commit(obj);

        yInfo() << " begin action " << ii;


        // send the result of recognition to the ABM
        list<pair<string, string> > lArgument;
        Time::delay(1.0);

        lArgument.push_back(pair<string, string>(sAg1, "agent"));
        lArgument.push_back(pair<string, string>("take", "predicate"));
        lArgument.push_back(pair<string, string>(sObj, "object"));
        lArgument.push_back(pair<string, string>("none", "recipient"));

        iCub->getABMClient()->sendActivity("action",
            "take",
            "action",
            lArgument,
            true);

        yInfo() << " in delay of action";
        Time::delay( 8 * Random::uniform());

        obj->m_ego_position[0] = XAG + 0.1 * Random::uniform();
        obj->m_ego_position[1] = YAG + 0.1 * Random::uniform();
        iCub->opc->commit(obj);

        iCub->opc->addRelation(ag1, have, obj);

        Time::delay(2);

        iCub->getABMClient()->sendActivity("action",
            "take",
            "action",
            lArgument,
            false);

        yInfo() << " end action " << ii;

        Time::delay(1.);
    }
}

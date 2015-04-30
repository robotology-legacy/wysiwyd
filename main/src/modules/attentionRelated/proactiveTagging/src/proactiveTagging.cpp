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

    nameMainGrammar = rf.findFileByName(rf.check("nameMainGrammar", Value("mainLoopGrammar.xml")).toString());
    nameGrammarSentenceTemporal = rf.findFileByName(rf.check("nameGrammarSentenceTemporal", Value("GrammarSentenceTemporal.xml")).toString());
    nameGrammarYesNo = rf.findFileByName(rf.check("nameGrammarYesNo", Value("nodeYesNo.xml")).toString());
    nameGrammarAskManner = rf.findFileByName(rf.check("nameGrammarAskManner", Value("askManner.xml")).toString());

    GrammarSentenceTemporal = rf.findFileByName(rf.check("GrammarSentenceTemporal", Value("GrammarSentenceTemporal.xml")).toString());
    nameGrammarNodeTestAP = rf.findFileByName(rf.check("nameGrammarNodeTestAP", Value("nameGrammarNodeTestAP.xml")).toString());
    nameGrammarNodeTrainAP = rf.findFileByName(rf.check("nameGrammarNodeTrainAP", Value("nameGrammarNodeTrainAP.xml")).toString());
    nameGrammarNodeTrainSD = rf.findFileByName(rf.check("nameGrammarNodeTrainSD", Value("nameGrammarNodeTrainSD.xml")).toString());


    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    bool    bEveryThingisGood = true;

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

 /*   Bottle bAnswer,
        bRecognized;

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameMainGrammar), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        cout << bRecognized.get(1).toString() << endl;
        return "none";
    }

    bAnswer = *bRecognized.get(1).asList();
    cout << "bAnswer is : " << bAnswer.toString() << endl;

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarNodeTestAP), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        cout << bRecognized.get(1).toString() << endl;
        return "none";
    }

    bAnswer = *bRecognized.get(1).asList();
    cout << "bAnswer is : " << bAnswer.toString() << endl;


    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarSentenceTemporal), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        cout << bRecognized.get(1).toString() << endl;
        return "none";
    }

    bAnswer = *bRecognized.get(1).asList();
    cout << "bAnswer is : " << bAnswer.toString() << endl;


    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarNodeTestAP), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        cout << bRecognized.get(1).toString() << endl;
        return "none";
    }

    bAnswer = *bRecognized.get(1).asList();
    cout << "bAnswer is : " << bAnswer.toString() << endl;


    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarNodeTrainAP), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        cout << bRecognized.get(1).toString() << endl;
        return "none";
    }

    bAnswer = *bRecognized.get(1).asList();
    cout << "bAnswer is : " << bAnswer.toString() << endl;

*/









    populateOpc();
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
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "help") {
        cout << helpMessage;
        reply.addString("ok");
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool proactiveTagging::updateModule() {
    return true;
}

void    proactiveTagging::nodeSentenceTemporal()
{
    ostringstream osError;          // Error message

    iCub->say("Yep ?");
    cout << "Yep ? " << endl;

    Bottle bOutput;

    //bool fGetaReply = false;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bMessenger; //to be send TO speech recog

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarSentenceTemporal), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        cout << bRecognized.get(1).toString() << endl;
        return;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)


    if (bAnswer.get(0).asString() == "stop")
    {
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
    }


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
    osResponse << "So, you said that " << (sAgent == "I") ? "you " : ((sAgent == "You") ? "I" : sAgent);
    osResponse << " will " << sVerb << "to the " << sLocation << " " << sAdverb << " ?";

    iCub->say(osResponse.str().c_str());
    cout << "osResponse" << endl;
    if (!nodeYesNo())
    {
        nodeSentenceTemporal();
        return;
    }

    iCub->getABMClient()->sendActivity("action", sVerb, "proactiveTagging", lArgument, true);

    cout << "Wait for end signal" << endl;

    nodeYesNo();        // wait for end of action

    iCub->getABMClient()->sendActivity("action", sVerb, "proactiveTagging", lArgument, false);

    iCub->say("Ok ! Another ?");

    cout << "Another ? " << endl;
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



bool proactiveTagging::nodeYesNo()
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
        cout << bRecognized.get(1).toString() << endl;
        return false;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

    if (bAnswer.get(0).asString() == "yes")        return true;

    return false;
}



bool proactiveTagging::populateOpc(){
    iCub->opc->update();
    iCub->opc->commit();

    Agent* agent = iCub->opc->addAgent("Carol");
    agent->m_ego_position[0] = -1.4;
    agent->m_ego_position[2] = 0.60;
    agent->m_present = 1;
    agent->m_color[0] = 200;
    agent->m_color[1] = 50;
    agent->m_color[2] = 50;

    Object* book = iCub->opc->addObject("book");
    book->m_ego_position[0] = -1.2;
    book->m_ego_position[2] = 0.4;
    book->m_present = 1;

    Action* verb = iCub->opc->addAction("write");
    iCub->opc->commit();


    bool relaTest = iCub->opc->addRelation(agent, verb, book);

    cout << "relatest is : " << relaTest << endl;

    checkRelations();

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

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarAskManner), 20);

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
#include "ears.h"
#include "wrdac/subsystems/subSystem_recog.h"

bool ears::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("ears")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "ears", "client.ini", isRFVerbose);
    iCub->opc->isVerbose = false;
    if (!iCub->connect())
    {
        yInfo() << " iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    portToBehavior.open("/" + moduleName + "/behavior:o");
    portTarget.open("/" + moduleName + "/target:o");

    MainGrammar = rf.findFileByName(rf.check("MainGrammar", Value("MainGrammar.xml")).toString());
    bShouldListen = true;

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    return true;
}

bool ears::interruptModule() {
    // speechRecognizer is in a long loop, which prohibits closure of ears
    // so interrupt the speechRecognizer
    bShouldListen = false;
    iCub->getRecogClient()->interruptSpeechRecognizer();

    portToBehavior.interrupt();
    portTarget.interrupt();
    rpc.interrupt();

    return true;
}


bool ears::close() {
    bShouldListen = false;
    iCub->getRecogClient()->interruptSpeechRecognizer();

    if(iCub) {
        iCub->close();
        delete iCub;
    }

    portToBehavior.interrupt();
    portToBehavior.close();

    portTarget.interrupt();
    portTarget.close();

    rpc.interrupt();
    rpc.close();

    return true;
}


bool ears::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "listen")
    {
        // yInfo() <<
        if (command.size() == 2)
        {
            if (command.get(1).asString() == "on")
            {
                yDebug() << "should listen on";
                bShouldListen = true;
                reply.addString("ack");
            }
            else if (command.get(1).asString() == "off")
            {
                yDebug() << "should listen off";
                bShouldListen = false;
                reply.addString("ack");
            }
            else if (command.get(1).asString() == "offShouldWait")
            {
                yDebug() << "should listen offShouldWait";
                bShouldListen = false;
                LockGuard lg(m);
                reply.addString("ack");
            }
            else {
                reply.addString("nack");
                reply.addString("Send either listen on or listen off");
            }
        }
    }
    else {
        yInfo() << helpMessage;
        reply.addString("wrong command");
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool ears::updateModule() {
    if (bShouldListen)
    {
        LockGuard lg(m);
        yDebug() << "bListen";
        Bottle bRecognized, //received FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic; // semantic information of the content of the recognition
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(MainGrammar), 1, true);

        if (bRecognized.get(0).asInt() == 0)
        {
            yDebug() << "ears::updateModule -> speechRecognizer did not recognize anything";
            return true;
        }

        bAnswer = *bRecognized.get(1).asList();

        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo() << " in abmHandler::node1 | stop called";
            return true;
        }
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        bSemantic = *(*bAnswer.get(1).asList()).get(1).asList();
        cout << bSemantic.toString() << endl;
        string sObject, sAction;
        string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

        string sObjectType, sCommand;
        if(sQuestionKind == "SENTENCEOBJECT") {
            sAction = bSemantic.check("predicateObject", Value("none")).asString();
            sObjectType = "object";
            sObject = bSemantic.check("object", Value("none")).asString();

            sCommand = "followingOrder";

        } else if(sQuestionKind == "SENTENCEBODYPART") {
            sAction = bSemantic.check("predicateBodypart", Value("none")).asString();
            sObject = bSemantic.check("bodypart", Value("none")).asString();
            sObjectType = "bodypart";

            sCommand = "followingOrder";

        } else if(sQuestionKind == "SENTENCENARRATIVE") {
            sAction = "narrate";
            sObjectType = "";
            sObject = "";

            sCommand = "followingOrder";
        } else if (sQuestionKind == "SENTENCEKS") {
            sCommand = "followingOrder";
            sAction = bSemantic.check("predicateKS", Value("none")).asString();
            sObjectType = bSemantic.check("KS", Value("none")).asString();
            sObject = "";

        } else{
            yError() << "[ears] Unknown predicate";

        }

        Bottle &bToTarget = portTarget.prepare();
        bToTarget.clear();
        bToTarget.addString(sAction);
        bToTarget.addString(sObjectType);
        bToTarget.addString(sObject);
        portTarget.write();

        Bottle bCondition;
        bCondition.addString(sCommand);
        bCondition.addString(sAction);
        bCondition.addString(sObjectType);
        bCondition.addString(sObject);

        portToBehavior.write(bCondition);

        yDebug() << "Sending " + bCondition.toString();
    } else {
        yDebug() << "Not bListen";
    }

    return true;
}

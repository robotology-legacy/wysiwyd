#include "ears.h"
#include "wrdac/subsystems/subSystem_recog.h"

bool ears::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("ears")).asString().c_str();
    setName(moduleName.c_str());
    onPlannerMode = rf.check("plans",Value("false")).asBool();
    yDebug()<< "PLANS ENABLED: " << onPlannerMode;

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

    portTarget.open("/" + moduleName + "/target:o");
    portToSpeechRecognizer.open("/" + moduleName + "/speech:o");

    MainGrammar = rf.findFileByName(rf.check("MainGrammar", Value("MainGrammar.xml")).toString());
    bShouldListen = true;

    if (!onPlannerMode) {
        portToBehavior.open("/" + moduleName + "/behavior:o");
    }

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    return true;
}

bool ears::interruptModule() {
    // speechRecognizer is in a long loop, which prohibits closure of ears
    // so interrupt the speechRecognizer
    yDebug() << "interrupt ears";
    bShouldListen = false;
    if(Network::connect("/" + getName() + "/speech:o", "/speechRecognizer/rpc")) {
        Bottle bMessenger, bReply;
        bMessenger.addString("interrupt");
        // send the message
        portToSpeechRecognizer.write(bMessenger, bReply);
        if(bReply.get(1).asString() != "OK") {
            yError() << "speechRecognizer was not interrupted";
            yDebug() << "Reply from speechRecognizer:" << bReply.toString();
        }
    }

    yDebug() << "interrupted speech recognizer";
    portToSpeechRecognizer.interrupt();
    portToBehavior.interrupt();
    portTarget.interrupt();
    rpc.interrupt();

    yDebug() << "interrupt done";

    return true;
}


bool ears::close() {
    yDebug() << "close ears";

    if(iCub) {
        iCub->close();
        delete iCub;
    }
    yDebug() << "closed icub";

    portToSpeechRecognizer.interrupt();
    portToSpeechRecognizer.close();

    portTarget.interrupt();
    portTarget.close();

    if (!onPlannerMode) {
        portToBehavior.interrupt();
        portToBehavior.close();
    }

    yDebug() << "closing rpc port";
    rpc.interrupt();
    rpc.close();

    yDebug() << "end of close. bye!";
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
    else if (command.get(0).asString() == "dummy")
    {
        // sends a test bottle to planner
        Bottle &bToTarget = portTarget.prepare();
        bToTarget.clear();
        Bottle bAux;
        bAux.clear();
        Bottle bAux2;
        bAux2.clear();
        bToTarget.addString("new");
        bAux.addString("dummy2");
        bAux.addInt(1);
        bAux2.addString("sObjectType");
        bAux2.addString("sObject");
        bAux.addList()=bAux2;
        bToTarget.addList()=bAux;
        portTarget.write();
        yDebug() << "Sending " + bToTarget.toString();
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

        if(bAnswer.get(1).asList()->get(1).isList()) {
            bSemantic = *(*bAnswer.get(1).asList()).get(1).asList();
        }
        yDebug() << bSemantic.toString();
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
            sAction = "show";
            sObjectType = "kinematic structure";
            sObject = "";
        } else if (sQuestionKind == "SENTENCEKSC") {
            sCommand = "followingOrder";
            sAction = "show";
            sObjectType = "kinematic structure correspondence";
            sObject = "";
        } else if (sQuestionKind == "SENTENCERECOGNISE") {
            sCommand = "recognitionOrder";
            sAction = "";
            sObjectType = "";
            sObject = "";
        } else if (sQuestionKind == "SENTENCEDONE") {
            sCommand = "followingOrder";
            sAction = "end";
            sObjectType = "";
            sObject = "";
        } else if (sQuestionKind == "SENTENCEGAMESTART") {
            sCommand = "followingOrder";
            sAction = "game";
            sObjectType = "start";
            sObject = "";
        } else if (sQuestionKind == "SENTENCEGAMEEND") {
            sCommand = "followingOrder";
            sAction = "game";
            sObjectType = "end";
            sObject = "";
        } else {
            yError() << "[ears] Unknown predicate";
            // return true;
        }
        //send rpc data to planner
        if (onPlannerMode) {
            Bottle &bToTarget = portTarget.prepare();
            bToTarget.clear();
            Bottle bAux;
            bAux.clear();
            Bottle bAux2;
            bAux2.clear();
            bToTarget.addString("new");
            bAux.addString(sAction);
            bAux.addInt(1);
            bAux2.addString(sObjectType);
            bAux2.addString(sObject);
            bAux.addList()=bAux2;
            bToTarget.addList()=bAux;
            portTarget.write();
            yDebug() << "Sending " + bToTarget.toString();
        } else {
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
        }
    } else {
        yDebug() << "Not bListen";
        yarp::os::Time::delay(0.5);
    }

    return true;
}

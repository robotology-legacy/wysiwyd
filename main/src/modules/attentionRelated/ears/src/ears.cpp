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
    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    portToBehavior.open("/" + moduleName + "/behavior:o");
    while (!Network::connect(portToBehavior.getName(),"/BehaviorManager/trigger:i ")) {
        yWarning() << " Behavior is not reachable";
        yarp::os::Time::delay(0.5);
    }

    portTarget.open("/" + moduleName + "/target:o");

    MainGrammar = rf.findFileByName(rf.check("MainGrammar", Value("MainGrammar.xml")).toString());

    bShouldListen = true;

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";
    
    return true;
}


bool ears::close() {
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
    LockGuard lg(mutex);
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
                bShouldListen = true;
                reply.addString("ack");
            }
            else if (command.get(1).asString() == "off")
            {
                bShouldListen = false;
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
    LockGuard lg(mutex);
    if (bShouldListen)
    {
        yDebug() << "bListen";
        Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic; // semantic information of the content of the recognition
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(MainGrammar), 1, true);
        //bShouldListen=true;

        if (bRecognized.get(0).asInt() == 0)
        {
            yWarning() << " error in ears::updateModule | Error in speechRecog";
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
        //string sPredicate = bSemantic.check("predicate", Value("none")).asString();

        string sObjectType, sCommand;
        if(sQuestionKind == "SENTENCEOBJECT") {
            sObject = bSemantic.check("object", Value("none")).asString();
            sAction = bSemantic.check("predicateObject", Value("none")).asString();
            sCommand = "followingingOrder";            sObjectType = "object";
        } else if(sQuestionKind == "SENTENCEBODYPART") {
            sObject = bSemantic.check("bodypart", Value("none")).asString();
            sCommand = "touchingOrder";
            sObjectType = "bodypart";
        } else if(sQuestionKind == "SENTENCENARRATIVE") {
            sCommand = "followingingOrder"; 
            sAction = "narrate";
            sObjectType = "";
            sObject = "";
        }else{
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
        //bCondition.addString(sAction);
        bCondition.addString(sObjectType);
        bCondition.addString(sObject);

        portToBehavior.write(bCondition);
 
        yDebug() << "Sending " + bCondition.toString();
    } else {
        yDebug() << "Not bListen";
    }

    return true;
}


/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string ears::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        yInfo() << "Error in ears::grammarToString. Couldn't open file : " << sPath << ".";
        return "Error in ears::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

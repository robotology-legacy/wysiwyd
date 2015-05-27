#include "abmInteraction.h"


/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string abmInteraction::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        cout << "Error in abmInteraction::grammarToString. Couldn't open file : " << sPath << "." << endl;
        return "Error in abmInteraction::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

bool abmInteraction::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("abmInteraction")).asString().c_str();
    setName(moduleName.c_str());

    nameGrammarHumanFeedback = rf.findFileByName(rf.check("nameGrammarHumanFeedback", Value("hFeedback.xml")).toString());
    nameGrammarYesNo = rf.findFileByName(rf.check("nameGrammarYesNo", Value("nodeYesNo.xml")).toString());

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    bool    bEveryThingisGood = true;
    bool    tryAgain = false ; //at start, iCub will explain and speaks
    int     bestRank = 0 ;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "abmInteraction", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= false;
    if (!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        Time::delay(1.0);
    }

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    if (!iCub->getRecogClient())
    {
        cout << "WARNING SPEECH RECOGNIZER NOT CONNECTED" << endl;
    }
    if (!iCub->getABMClient())
    {
        cout << "WARNING ABM NOT CONNECTED" << endl;
    }

    nodeFeedback();

    return true;
}


bool abmInteraction::close() {
    iCub->close();
    delete iCub;

    return true;
}


bool abmInteraction::respond(const Bottle& command, Bottle& reply) {
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
bool abmInteraction::updateModule() {
    //  mainLoop();
    return true;
}

void    abmInteraction::nodeFeedback()
{
    ostringstream osError;          // Error message
    Bottle bOutput;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bMessenger; //to be send TO speech recog
    ostringstream osResponse;

    if(tryAgain = false){
        iCub->say("Note this kinematic structure between 1 and 10 please");
        yInfo() << " iCub says : Note this kinematic structure between 1 and 10 please" ;
    } else {
        iCub->say("Can you repeat your feedback please?");
        yInfo() << "iCub says : Can you repeat your feedback please?" ;
    }

    if(bestRank != 0) {
        osResponse.str("");
        osResponse << "The current best structure is shown at left. The rank is " << bestRank ;
        iCub->say(osResponse.str().c_str()) ;
        yInfo() << "iCub says : " << osResponse ;
    }

    //Method to call ABM and remember a kinematic structure from an instance
    //if first time, just one, otherwise the current best structure and the new one

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarHumanFeedback), 20);

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
        cout << osError.str() << endl;
    }

    yInfo() << "bRecognized " << bRecognized.toString();
    cout << bRecognized.get(1).toString() << endl;

    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    if(sQuestionKind == "FEEDBACK") {
        yInfo() << "FEEDBACK received from Human!" ;
    } else {
        yError() << " The sentence type is not recognized, waiting for FEEDBACK" ;
        return ;
    }

    // semantic is the list of the semantic elements of the sentence except the type ef sentence
    bSemantic = *bAnswer.get(1).asList()->get(1).asList();

    string sFeedback10 = bSemantic.check("feedback10", Value("0")).asString();
    int iFeedback10    = atoi(sFeedback10.c_str()) ;

    osResponse.str("");
    osResponse << "So for you, this kinematic structure has a score of " << sFeedback10 << ", right?";
    iCub->say(osResponse.str().c_str());
    yInfo() << "iCub says : " << osResponse ;

    if (!nodeYesNo())
    {
        osResponse.str("");
        osResponse << "Oups, I am sorry" ;
        tryAgain = true ;
        iCub->say(osResponse.str().c_str());
        yInfo() << "iCub says : " << osResponse ;
        nodeFeedback();
        
        return;
    } else {
        tryAgain = false ;
    }

    if (iFeedback10 > bestRank) {
        bestRank = iFeedback10 ;

        //update ABM

        osResponse.str("");
        osResponse << "Yes, I have improved my skills" ;
        iCub->say(osResponse.str().c_str());
        yInfo() << "iCub says : " << osResponse ;

    } else {

        //update ABM

        osResponse.str("");
        osResponse << "Erf, too bad" ;
        iCub->say(osResponse.str().c_str());
        yInfo() << "iCub says : " << osResponse ;

    }

    iCub->say("Ok ! Another ?");

    cout << "Another ? " << endl;
    if (nodeYesNo())
    {
        nodeFeedback();
        return;
    }
    else
    {
        return;
    }
}



bool abmInteraction::nodeYesNo()
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


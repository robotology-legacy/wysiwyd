#include "qRM.h"


/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string qRM::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        cout << "Error in qRM::grammarToString. Couldn't open file : " << sPath << "." << endl;
        return "Error in qRM::grammarToString. Couldn't open file";
    }

    string sLine;
    while( getline(isGrammar, sLine) )
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

bool qRM::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("qRM")).asString().c_str();
    setName(moduleName.c_str());


    nameMainGrammar     = rf.getContextPath().c_str();
    nameMainGrammar    += rf.check("nameMainGrammar",  Value("/MainGrammar.xml")).toString().c_str();

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();



    bool    bEveryThingisGood = true;

    // Open port2abm
    port2abmName = "/";
    port2abmName += getName() + "/toABM";

    if (!Port2ABM.open(port2abmName.c_str())) {           
        cout << getName() << ": Unable to open port " << port2abmName << endl;  
        bEveryThingisGood &= false;
    }

    // Open port2speech
    port2SpeechRecogName = "/";
    port2SpeechRecogName += getName() + "/toSpeechRecog";

    if (!Port2SpeechRecog.open(port2SpeechRecogName.c_str())) {           
        cout << getName() << ": Unable to open port " << port2SpeechRecogName << endl;  
        bEveryThingisGood &= false;
    }

    // Open port2reasoning
    port2abmReasoningName = "/";
    port2abmReasoningName += getName() + "/toAbmR";

    if (!Port2abmReasoning.open(port2abmReasoningName.c_str())) {           
        cout << getName() << ": Unable to open port " << port2abmReasoningName << endl;  
        bEveryThingisGood &= false;
    }


    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName,"qRM","client.ini",isRFVerbose);
    iCub->opc->isVerbose &= false;
    if (!iCub->connect())
    {
        cout<<"iCubClient : Some dependencies are not running..."<<endl;
        Time::delay(1.0);
    }

    
    bEveryThingisGood &= Network::connect(port2abmName.c_str(), "/autobiographicalMemory/request:i");
    bEveryThingisGood &= Network::connect(port2SpeechRecogName.c_str(), "/speechRecognizer/rpc");
    bEveryThingisGood &= Network::connect(port2abmReasoningName.c_str(), "/abmReasoning/rpc");

 //   calibrationThread = new AutomaticCalibrationThread(100,"ical");
 //   calibrationThread->start();
 //   calibrationThread->suspend();

    rpc.open ( ("/"+moduleName+"/rpc").c_str());
    attach(rpc);

    mainLoop();

    return true;
}


bool qRM::close() {
    iCub->close();
    delete iCub;

    return true;
}


bool qRM::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) + 
        " commands are: \n" +  
        "help \n" + 
        "quit \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="calib")
    {
        cout << "calibration of the RT" << endl;
        if (command.size() == 2)
        {
            cout << "default : left hand" << endl;
            reply = calibrationRT();
        }
        else
        {
            cout << "using " << command.get(1).asString() << " hand" << endl;
            reply = calibrationRT(command.get(1).asString());
        }
    }


    return true;
}

/* Called periodically every getPeriod() seconds */
bool qRM::updateModule() {
    mainLoop();
    return true;
}


Bottle qRM::calibrationRT()
{
    return calibrationRT("left");
}


Bottle qRM::calibrationRT(string side)
{
    Bottle bOutput;
    string sOutput;

    if(iCub->getABMClient())
    {
        list<string> roles;
        list<string> arguments;
        roles.push_back("table");
        arguments.push_back("table");
        iCub->getABMClient()->sendActivity("activity","calibration","navigation",arguments, roles, true);
    }

    //Calibrate
    iCub->say("Now I will self calibrate.");
    iCub->look("cursor_0");
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


    if(iCub->getABMClient())
    {
        list<string> roles;
        list<string> arguments;
        roles.push_back("table");
        arguments.push_back("table");
        iCub->getABMClient()->sendActivity("activity","calibration","navigation",arguments, roles, false);
    }

    bOutput.addString("calibration to the reactable endded using the hand "+side);
    return bOutput;
}


void    qRM::mainLoop()
{
    ostringstream osError;          // Error message

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


    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger,bSpeechRecognized);

        cout << "Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << nameMainGrammar;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
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
        cout << osError.str() << endl;
    }


    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    // semantic is the list of the semantic elements of the sentence except the type ef sentence
    bSemantic = *bAnswer.get(1).asList()->get(1).asList();


    if (sQuestionKind == "LOOK")
    {
        string sObject = bSemantic.check("words", Value("none")).asString();
        list<string>  vRole,
            vArgument;
        vArgument.push_back(sObject);
        vRole.push_back("word");

        vector<Object*>      presentObjects;
        iCub->opc->checkout();
        list<Entity*> entities = iCub->opc->EntitiesCache();
        presentObjects.clear();
        for(list<Entity*>::iterator it=entities.begin(); it !=entities.end(); it++)
        {
            //!!! ONLY RT_OBJECT and AGENTS ARE TRACKED !!!
            if ( ( (*it)->isType(EFAA_OPC_ENTITY_RTOBJECT) || (*it)->isType(EFAA_OPC_ENTITY_AGENT) ) && ((Object*)(*it))->m_present )
            {
                presentObjects.push_back((Object*)(*it));
            }
        }


        double maxSalience = 0;
        string nameTrackedObject = "none";
        for(vector<Object*>::iterator it = presentObjects.begin(); it!= presentObjects.end(); it++)
        {
            if (maxSalience < (*it)->m_saliency )
            {
                maxSalience = (*it)->m_saliency;
                nameTrackedObject = (*it)->name();
            }
        }


        cout<<"Most salient is : "<<nameTrackedObject<<" with saliency="<<maxSalience << endl;
        vArgument.push_back(nameTrackedObject);

        vRole.push_back("focus");

        iCub->getABMClient()->sendActivity("says","look","sentence",vArgument,vRole, true);

    }
    else if (sQuestionKind == "SHOW")
    {
        string sWord = bSemantic.check("words", Value("none")).asString();
        bSendReasoning.addString("askWordKnowledge");
        bSendReasoning.addString("getObjectFromWord");
        bSendReasoning.addString(sWord);
        Port2abmReasoning.write(bSendReasoning, bMessenger);

        list<string>  vRole,
            vArgument;
        vArgument.push_back(sWord);
        vRole.push_back("word");

        cout << "bMessenger is : " << bMessenger.toString() << endl;

        iCub->getABMClient()->sendActivity("says","show","sentence",vArgument,vRole, true);

    }
    else if (sQuestionKind == "WHAT")
    {
        bSendReasoning.addString("askWordKnowledge");
        bSendReasoning.addString("getWordFromObject");

        vector<Object*>      presentObjects;
        iCub->opc->checkout();
        list<Entity*> entities = iCub->opc->EntitiesCache();
        presentObjects.clear();
        for(list<Entity*>::iterator it=entities.begin(); it !=entities.end(); it++)
        {
            //!!! ONLY RT_OBJECT and AGENTS ARE TRACKED !!!
            if ( ( (*it)->isType(EFAA_OPC_ENTITY_RTOBJECT) || (*it)->isType(EFAA_OPC_ENTITY_AGENT) ) && ((Object*)(*it))->m_present )
            {
                presentObjects.push_back((Object*)(*it));
            }
        }


        double maxSalience = 0;
        string nameTrackedObject = "none";
        for(vector<Object*>::iterator it = presentObjects.begin(); it!= presentObjects.end(); it++)
        {
            if (maxSalience < (*it)->m_saliency )
            {
                maxSalience = (*it)->m_saliency;
                nameTrackedObject = (*it)->name();
            }
        }

        cout<<"Most salient is : "<<nameTrackedObject<<" with saliency="<<maxSalience << endl;

        list<string>  vRole,
            vArgument;
        vArgument.push_back(nameTrackedObject);
        vRole.push_back("word");


        bSendReasoning.addString(nameTrackedObject);
        Port2abmReasoning.write(bSendReasoning, bMessenger);

        iCub->getABMClient()->sendActivity("says","what","sentence",vArgument,vRole, true);


        cout << "bMessenger is : " << bMessenger.toString() << endl;

    }


}





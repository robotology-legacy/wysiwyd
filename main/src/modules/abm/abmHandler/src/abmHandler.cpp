/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Grégoire Pointeau
* email:   gregoire.pointeau@inserm.fr
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* wysiwyd/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include <abmHandler.h>


using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


abmHandler::~abmHandler()
{
}



/*
* Configure method. Receive a previously initialized
* resource finder object. Use it to configure your module.
* If you are migrating from the old Module, this is the
* equivalent of the "open" method.
*/

bool abmHandler::configure(yarp::os::ResourceFinder &rf) {

    iCurrentInstance = -1;
    sCurrentPronoun = "none";
    sCurrentActivity = "none";
    psCurrentComplement.first = "none";
    psCurrentComplement.second = "none";

    bool    bEveryThingisGood = true;
    bool    bOptionnalModule = true;
    moduleName = rf.check("name",
        Value("/abmHandler"),
        "module name (string)").asString();

    sKeyWord = rf.check("keyword", Value("history")).toString().c_str();

    nameGrammarNode1 = rf.findFileByName(rf.check("nameGrammarNode1", Value("nameGrammarNode1.xml")).toString().c_str());
    nameGrammarNode2 = rf.findFileByName(rf.check("nameGrammarNode2", Value("nameGrammarNode2.xml")).toString().c_str());
    nameGrammarNode3 = rf.findFileByName(rf.check("nameGrammarNode3", Value("nameGrammarNode3.xml")).toString().c_str());

    /*
    * before continuing, set the module name before getting any other parameters,
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());


    // Open handler port
    handlerPortName = "/";
    handlerPortName += getName() + "/rpc";         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        bEveryThingisGood = false;
    }

    // Open port2abm
    port2abmName = "/";
    port2abmName += getName() + "/toABM";

    if (!Port2ABM.open(port2abmName.c_str())) {
        cout << getName() << ": Unable to open port " << port2abmName << endl;
        bEveryThingisGood = false;
    }

    // Open port2speech
    port2SpeechRecogName = "/";
    port2SpeechRecogName += getName() + "/toSpeechRecog";

    if (!Port2SpeechRecog.open(port2SpeechRecogName.c_str())) {
        cout << getName() << ": Unable to open port " << port2SpeechRecogName << endl;
        bEveryThingisGood = false;
    }

    // Open port2reasoning
    port2abmReasoningName = "/";
    port2abmReasoningName += getName() + "/toAbmR";

    if (!Port2abmReasoning.open(port2abmReasoningName.c_str())) {
        cout << getName() << ": Unable to open port " << port2abmReasoningName << endl;
        bEveryThingisGood = false;
    }


    // Open port2OPCManager
    port2OPCmanagerName = "/";
    port2OPCmanagerName += getName() + "/toOPCManager";

    if (!Port2OPCManager.open(port2OPCmanagerName.c_str())) {
        cout << getName() << ": Unable to open port " << port2OPCmanagerName << endl;
        bEveryThingisGood = false;
    }

    // Open port2ispeak
    port2iSpeakName = "/";
    port2iSpeakName += getName() + "/toiSpeak";

    if (!Port2iSpeak.open(port2iSpeakName.c_str())) {
        cout << getName() << ": Unable to open port " << port2iSpeakName << endl;
        bEveryThingisGood = false;
    }

    // Open port2ispeak
    port2BodySchemaName = "/" + getName() + "/toBodySchema";

    if (!Port2BodySchema.open(port2BodySchemaName.c_str())) {
        cout << getName() << ": Unable to open port " << port2BodySchemaName << endl;
        bEveryThingisGood = false;
    }


    attach(handlerPort);                  // attach to port


    //------------------------//
    //      iCub Client
    //------------------------//
    string ttsSystem = SUBSYSTEM_SPEECH;
    iCub = new ICubClient(moduleName.c_str(), "abmHandler", "client.ini", true);
    //  iCub->getSpeechClient()->SetOptions("iCubina-fr");
    iCub->say("test Greg!", false);
    //  iCub->opc->isVerbose = false;

    // Connect iCub Client, and ports
    //      bEveryThingisGood &= !iCub->connect()  
    //bEveryThingisGood &= Network::connect(port2abmName.c_str(), "/autobiographicalMemory/request:i");

    bEveryThingisGood &= Network::connect(port2abmName.c_str(), "/autobiographicalMemory/rpc");
    bOptionnalModule &= Network::connect(port2SpeechRecogName.c_str(), "/speechRecognizer/rpc");
    bOptionnalModule &= Network::connect(port2abmReasoningName.c_str(), "/abmReasoning/rpc");
    bOptionnalModule &= Network::connect(port2OPCmanagerName, "/opcManager/rpc");
    bOptionnalModule &= Network::connect(port2iSpeakName, "/iSpeak");
    bOptionnalModule &= Network::connect(port2BodySchemaName, "/bodySchema/rpc");

    bOptionnalModule &= Network::connect("/mainLoop/speechGrammar/keyword:o", handlerPortName.c_str());

    if (!bOptionnalModule)
    {
        cout << endl << "Some dependencies are not running (ICubClient or port(s) connections)" << endl << endl;
    }

    //Deal with settings of the speech synthesizer
    /*if (ttsSystem == SUBSYSTEM_SPEECH_ISPEAK)
    {
    string ttsOptions = rf.check("ttsOptions",Value("iCub")).asString().c_str();
    ( (SubSystem_Speech_iSpeak*) iCub->getSubSystem("speech"))->SetOptions(ttsOptions);
    }*/

    //iCub->opc->checkout();

    if (!bEveryThingisGood || !bOptionnalModule)
        cout << endl << "Some dependencies are not running (ICubClient or port(s) connections)" << endl << endl;
    else
        cout << endl << endl << "----------------------------------------------" << endl << endl << "abmHandler ready !" << endl << endl;

    node1();

    return bEveryThingisGood;
}

bool abmHandler::interruptModule() {
    return true;
}

bool abmHandler::close() {
    handlerPort.close();
    Port2ABM.close();
    Port2SpeechRecog.close();
    Port2abmReasoning.close();
    Port2OPCManager.close();
    Port2iSpeak.close();
    iCub->close();
    delete iCub;

    return true;
}

bool abmHandler::respond(const Bottle& command, Bottle& reply) {
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
    else if (command.get(0).asString() == sKeyWord.c_str()) {
        reply = node1();
    }


    return true;
}

/* Called periodically every getPeriod() seconds */
bool abmHandler::updateModule() {
    return true;
}

double abmHandler::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}



/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string abmHandler::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        cout << "Error in abmHandler::grammarToString. Couldn't open file : " << sPath << "." << endl;
        return "Error in abmHandler::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}



/* Node 1: general question
* (When was ... )
* Send nameGrammarNode1 in loop every 5 second until a correct answer or a STOP signal
*/
Bottle abmHandler::node1()
{
    sCurrentNode = "Node 1";
    sCurrentGrammarFile = nameGrammarNode1;
    ostringstream osError;          // Error message
    osError << "Error in abmHandler | " << sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bBodySchema, //send rpc command to BodySchema
        bSpeak, // bottle for tts
        bTemp;

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());


    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        cout << "Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        if (!bSpeechRecognized.get(1).isList())
        {
            osError << "Grammar not recognized or speechRecognizer is in Legacy mode!";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
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
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return bOutput;
    }


    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    // semantic is the list of the semantic elements of the sentence except the type ef sentence
    bSemantic = *bAnswer.get(1).asList()->get(1).asList();


    // Do you know any ...
    if (sQuestionKind == "INFORMATION")
    {
        sCurrentActivity = bSemantic.check("knowledge", Value("none")).asString();

        if (sCurrentActivity == "location")
        {


        }
        else if (sCurrentActivity == "shared plan")
        {
            ostringstream osRequest;
            osRequest << "SELECT DISTINCT activityname FROM main WHERE activitytype = 'sharedplan'";
            bMessenger.clear();
            bMessenger.addString("request");
            bMessenger.addString(osRequest.str().c_str());

            Port2ABM.write(bMessenger, bAnswer);

            cout << "Reponse de ABM : \n" << bAnswer.toString() << endl;

            if (bAnswer.toString() == "NULL" || bAnswer.isNull())
            {
                iCurrentInstance = -1;
                osError.str("");
                osError << sCurrentNode << " :: Response from ABM :: Unknown Event";
                bOutput.addString(osError.str());
                cout << osError.str() << endl;
                return bOutput;
            }

            vector<string> lsSharedPlanKnown;
            ostringstream osSentence;
            osSentence << "I know " << bAnswer.size() << " shared plan : ";
            for (int i = 0; i < bAnswer.size(); i++)
            {
                lsSharedPlanKnown.push_back(bAnswer.get(i).toString());
                osSentence << lsSharedPlanKnown[i] << ", ";
            }
            osSentence << ".";

            cout << osSentence.str() << endl;
            iCub->say(osSentence.str());
            sLastSentence = osSentence.str();
            bSpeak.addString(osSentence.str());
            Port2iSpeak.write(bSpeak);


            return node3();


        }
        else if (sCurrentActivity == "game")
        {

        }

        return bOutput;
    }
    // When was the last time you ...
    else if (sQuestionKind == "TEMPORAL")
    {
        bool fTimeFirst = bSemantic.check("time_value", Value("last")).asString() == "first";
        sCurrentActivity = bSemantic.check("activity", Value("none")).asString();
        sCurrentPronoun = bSemantic.check("pronon", Value("none")).asString();
        bTemp = *bSemantic.get(3).asList()->get(1).asList();

        psCurrentComplement.first = bTemp.get(0).asString();
        psCurrentComplement.second = bTemp.get(1).asString();

        if (psCurrentComplement.first == "spatial")  (psCurrentComplement.first = "spatial1");

        if (sCurrentPronoun == "none" || sCurrentActivity == "none")
        {
            iCurrentInstance = -1;
            osError << "no pronoun or activity";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        ostringstream osRequest;
        osRequest << "SELECT main.instance, main.time FROM main, contentarg WHERE main.instance = contentarg.instance ";


        osRequest << " AND ( contentarg.argument = '" << psCurrentComplement.second << "' OR main.activitytype = '" << psCurrentComplement.second << "' OR main.activityname = '" << psCurrentComplement.second << "' ) ";

        if (sCurrentActivity != "met" && sCurrentActivity != "played")
        {
            osRequest << " AND contentarg.instance IN (SELECT instance FROM contentarg WHERE ";

            if ((sCurrentPronoun == "I" && sCurrentActivity != "learned") || (sCurrentPronoun == "you" && sCurrentActivity == "learned"))
                osRequest << " argument != 'icub' ) ";
            if ((sCurrentPronoun == "you" && sCurrentActivity != "learned") || (sCurrentPronoun == "I" && sCurrentActivity == "learned"))
                osRequest << " argument = 'icub' ) ";
        }

        fTimeFirst ? osRequest << " ORDER BY main.instance LIMIT 1" : osRequest << " ORDER BY main.instance DESC LIMIT 1";

        bMessenger.clear();
        bMessenger.addString("request");
        bMessenger.addString(osRequest.str().c_str());

        Port2ABM.write(bMessenger, bAnswer);

        cout << "Reponse de ABM : \n" << bAnswer.toString() << endl;
        if (bAnswer.toString() == "NULL" || bAnswer.isNull())
        {
            iCurrentInstance = -1;
            osError.str("");
            osError << sCurrentNode << " :: Response from ABM :: Unknown Event";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        iCurrentInstance = atoi(bAnswer.get(0).asList()->get(0).asString().c_str());
        ostringstream osAnswer;
        osAnswer << "It was the " << dateToSpeech(bAnswer.get(0).asList()->get(1).asString().c_str());
        sLastSentence = osAnswer.str();
        bSpeak.addString(osAnswer.str());

        Port2iSpeak.write(bSpeak);

        bSendReasoning.addString("imagine");
        bSendReasoning.addInt(iCurrentInstance);
        Port2abmReasoning.write(bSendReasoning);
        cout << "Result of Reasoning for imagination: " << bTemp.toString() << endl;

        cout << "iCurrentInstance = " << iCurrentInstance << endl;

        return node2();
    }

    // Can you remember the first/last time when you ...
    else if (sQuestionKind == "REMEMBERING")
    {
        cout << "============= REMEMBERING ===================" << endl ;
        cout << bSemantic.toString().c_str() << endl ;

        bool fTimeFirst = bSemantic.check("time_value", Value("last")).asString() == "first";
        sCurrentActivity = bSemantic.check("activity_past", Value("none")).asString();
        sCurrentPronoun = bSemantic.check("pronoun", Value("none")).asString();

        bBodySchema.clear();
        if(sCurrentActivity == "babbled"){
            sCurrentActivity = "babbling";
        }

        cout << "first time? = " << fTimeFirst << " ; activity = " << sCurrentActivity  << " ; sCurrentPronoun = " << sCurrentPronoun << endl ;

        if (sCurrentPronoun == "none" || sCurrentActivity == "none")
        {
            iCurrentInstance = -1;
            osError << "no pronoun or activity";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        ostringstream osRequest;
        osRequest << "SELECT DISTINCT main.instance, main.time FROM main, contentarg WHERE main.activityname = '"<< sCurrentActivity << "' AND main.instance = contentarg.instance AND main.begin = TRUE AND contentarg.instance IN (SELECT instance FROM contentarg WHERE ";
        if (sCurrentPronoun == "you") {
            osRequest << " argument = 'icub' AND role = 'agent1') ";
        } else { //IMPORTANT : don't have recognition then so I is everything but the iCub
            osRequest << " argument != 'icub' AND role = 'agent1') ";
        }

        fTimeFirst ? osRequest << " ORDER BY main.instance LIMIT 1" : osRequest << " ORDER BY main.instance DESC LIMIT 1";

        cout << "REQUEST : " << osRequest.str() << endl ;

        bMessenger.clear();
        bMessenger.addString("request");
        bMessenger.addString(osRequest.str().c_str());

        bAnswer.clear();
        Port2ABM.write(bMessenger, bAnswer);

        cout << "Response of ABM: \n ==>" << bAnswer.toString() << "<==" << endl;

        if (bAnswer.toString() == "NULL" || bAnswer.isNull() || bAnswer.toString() == "")
        {
            iCurrentInstance = -1;
            osError.str("");
            osError << sCurrentNode << " :: Response from ABM :: Unknown Event";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        iCurrentInstance = atoi(bAnswer.get(0).asList()->get(0).asString().c_str());
        ostringstream osAnswer;
        osAnswer << "It was the " << dateToSpeech(bAnswer.get(0).asList()->get(1).asString().c_str());
        sLastSentence = osAnswer.str();

        bSpeak.clear();
        bSpeak.addString(osAnswer.str());
        Port2iSpeak.write(bSpeak);

        //give the instance to hyung jin : iCurrentInstance


        //return bOutput ;
        return node1() ;
    }

    // Can you remember the first/last time when you ...
    else if (sQuestionKind == "ACTING")
    {
        cout << "============= ACTING ===================" << endl ;
        cout << bSemantic.toString().c_str() << endl ;

        sCurrentActivity = bSemantic.check("activity", Value("none")).asString();

        bBodySchema.clear();
        if(sCurrentActivity == "babbling"){
            bBodySchema.addString("babblingLearning");
        }

        cout << "To BodySchema : " << bBodySchema.toString() << endl ;

        //Response from iCub through iSpeak
        ostringstream osAnswer;
        osAnswer << "Of course! Let me show you with my left arm";

        bSpeak.clear();
        bSpeak.addString(osAnswer.str());
        Port2iSpeak.write(bSpeak);


        //Port2BodySchema (testing no reply)
        //Port2BodySchema.write(bBodySchema);

        //waiting for reply
        //Port2BodySchema.write(bBodySchema, bAnswer);

        return node1() ;
    }

    else
    {
        string sError = "What da hell are you talking about Bro ?";
        cout << sError << endl;
        iCub->say(sError);
        sLastSentence = sError;
        bOutput.addString(sError);
        Port2iSpeak.write(bOutput);
        return bOutput;
    }
}

/*  Node 2 : details about number and agent
*       possibility to go back to node 1
*       repeat and stop allowed
*       Send nameGrammarNode2 in loop every 5 second until a correct answer or a STOP signal
*/
Bottle abmHandler::node2()
{
    sCurrentNode = "Node 2";
    sCurrentGrammarFile = nameGrammarNode2;
    ostringstream osError;          // Error message
    osError << "Error in abmHandler | " << sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSpeak, // to ommunicate with the tts
        bTemp;

    Bottle bOutput;
    string sSentence = "You want to know more about it ?";
    iCub->say(sSentence);
    bSpeak.addString(sSentence);
    cout << sSentence << endl;
    Port2iSpeak.write(bSpeak);

    bool fGetaReply = false;


    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());


    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        cout << "Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 3)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)
        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }

    }

    // STOP & REPEAT | not Bottle

    if (bAnswer.get(0).asString() == "stop")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return bOutput;
    }

    if (bAnswer.get(1).asList()->get(0).asString() == "repeat")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << sCurrentNode << " | REPEAT called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;

        bSpeak.clear();
        sLastSentence = "I said, " + sLastSentence;
        bSpeak.addString(sLastSentence);
        Port2iSpeak.write(bSpeak);

        return node2();
    }
    else if (bAnswer.get(1).asList()->get(0).asString() == "WHO")
    {
        // previous question was not about a game
        if (psCurrentComplement.first != "game")
        {
            ostringstream osAnswer;
            osAnswer << "What da hell are you taking about bro ?";
            sLastSentence = osAnswer.str();
            bSpeak.addString(osAnswer.str());

            Port2iSpeak.write(bSpeak);
            return node2();
        }

        bSemantic = *bAnswer.get(1).asList();

        // INSTANT WINNER:

        Bottle bNextTopic = *bSemantic.get(1).asList();

        if (bNextTopic.get(0).toString() == "instant_winner")
        {
            ostringstream osRequest;
            osRequest << "SELECT argument FROM contentarg WHERE role = 'winner' AND instance = " << iCurrentInstance;

            bMessenger.clear();
            bMessenger.addString("request");
            bMessenger.addString(osRequest.str().c_str());
            Port2ABM.write(bMessenger, bAnswer);

            cout << "Reponse de ABM : \n" << bAnswer.toString() << endl;
            if (bAnswer.toString() == "NULL" || bAnswer.isNull())
            {
                bSpeak.clear();
                bSpeak.addString("Can't remember... It might be me !");
                sLastSentence = bSpeak.toString();
                Port2iSpeak.write(bSpeak);

                bOutput = bSpeak;
                return node2();
            }
            string sWinner = bAnswer.get(0).asList()->get(0).toString();
            (sWinner == "icub") ? sWinner = "I" : sWinner = sWinner;
            ostringstream osAnswer;
            osAnswer << sWinner << " won the game";
            bSpeak.clear();
            bSpeak.addString(osAnswer.str());
            sLastSentence = osAnswer.str();
            cout << osAnswer.str() << endl;
            Port2iSpeak.write(bSpeak);

            bOutput = bAnswer;
            return node2();
        }
        else if (bNextTopic.get(0).toString() == "usual_winner")
        {

            ostringstream osRequest;
            osRequest << "SELECT contentarg.argument FROM main, contentarg WHERE contentarg.instance = main.instance AND main.activityname='" << psCurrentComplement.second << "' AND contentarg.role ='winner' AND main.begin = false";
            bMessenger.clear();
            bMessenger.addString("request");
            bMessenger.addString(osRequest.str().c_str());
            Port2ABM.write(bMessenger, bAnswer);

            cout << "Reponse de ABM : \n" << bAnswer.toString() << endl;
            if (bAnswer.toString() == "NULL" || bAnswer.isNull())
            {
                bSpeak.clear();
                bSpeak.addString("Can't remember... It might be me !");
                sLastSentence = bSpeak.toString();
                Port2iSpeak.write(bSpeak);

                bOutput = bSpeak;
                return node2();
            }

            //Get differents winners
            vector<pair<string, int> >  vpsWinner;
            for (int i = 0; i < bAnswer.size(); i++)
            {
                string sTempWinner = bAnswer.get(i).asList()->get(0).toString();
                bool bFound = false;
                for (vector<pair<string, int> >::iterator itV = vpsWinner.begin(); itV != vpsWinner.end(); itV++)
                {
                    if (!bFound && itV->first == sTempWinner)
                    {
                        itV->second++;
                        bFound = true;
                    }
                }
                if (!bFound)
                {
                    pair<string, int> pTemp(sTempWinner, 1);
                    vpsWinner.push_back(pTemp);
                }
            }


            // get the best
            string sWinnner;
            int max = 0;
            for (vector<pair<string, int> >::iterator itV = vpsWinner.begin(); itV != vpsWinner.end(); itV++)
            {
                if (itV->second > max)
                {
                    sWinnner = itV->first;
                    max = itV->second;
                }
            }
            if (sWinnner == "icub") sWinnner = "I";
            ostringstream osAnswer;
            osAnswer << sWinnner << " usualy wins.";

            bSpeak.clear();
            bSpeak.addString(osAnswer.str());
            sLastSentence = osAnswer.str();
            cout << osAnswer.str() << endl;
            Port2iSpeak.write(bSpeak);

            bOutput = bAnswer;
            return node2();

        }


    }


    if (!bAnswer.get(1).isList())
    {
        osError << "semantic is not bottle";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return bOutput;
    }


    //ELSE

    // semantic is the list of the semantic elements of the sentence except the type ef sentence
    bSemantic = *bAnswer.get(1).asList();
    string sNextTopic = bSemantic.get(0).toString();


    if (sNextTopic == "ELSE")
    {
        cout << "Human wants to talk about something else" << endl;
        return node1();
    }


    // IF HUIMAN WANT TO KNOW MORE

    if (sNextTopic == "MORE")
    {
        if (iCurrentInstance < 7)
        {
            osError << "doesn't know context (iCurrentInstance < 7)";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        Bottle bNextTopic = *bSemantic.get(1).asList();

        if (bNextTopic.get(0).toString() == "about_agent")
        {
            cout << sCurrentNode << " :: MORE :: about agent" << endl;

            ostringstream osRequest;
            osRequest << "SELECT name from agent WHERE presence = true AND instance = " << iCurrentInstance << endl;

            bMessenger.clear();
            bMessenger.addString("request");
            bMessenger.addString(osRequest.str().c_str());

            Port2ABM.write(bMessenger, bAnswer);

            cout << "Reponse de ABM : \n" << bAnswer.toString() << endl;
            if (bAnswer.toString() == "NULL" || bAnswer.isNull())
            {
                bSpeak.clear();
                bSpeak.addString("At least me... I guess...");
                sLastSentence = bSpeak.toString();
                Port2iSpeak.write(bSpeak);

                bOutput = bSpeak;
                return node2();
            }

            ostringstream osAnswer;
            for (int i = 0; i < bAnswer.get(0).asList()->size(); i++)
            {
                osAnswer << bAnswer.get(0).asList()->get(i).toString() << ", ";
            }
            osAnswer << " were present.";
            bSpeak.clear();
            bSpeak.addString(osAnswer.str());
            sLastSentence = osAnswer.str();
            Port2iSpeak.write(bSpeak);

            bOutput = bAnswer;
            return node2();
        }

        if (bNextTopic.get(0).toString() == "about_number")
        {
            cout << sCurrentNode << " :: MORE :: about number" << endl;

            ostringstream osRequest;


            // ACTIVITY IS MET
            if (sCurrentActivity == "met")
            {
                osRequest << "SELECT COUNT (DISTINCT instance) as nb_instance FROM contentarg WHERE instance IN (SELECT instance FROM contentarg WHERE argument = '" << psCurrentComplement.second << "') AND instance IN (SELECT instance FROM main WHERE begin = true)";
            }
            else
            {
                // get the MAIN information
                osRequest << "SELECT COUNT (DISTINCT main.instance) as nb_instance FROM main,contentarg WHERE contentarg.instance = main.instance AND contentarg.instance IN (SELECT main.instance FROM main,contentarg WHERE (contentarg.role = '" << psCurrentComplement.first << "' AND contentarg.argument = '" << psCurrentComplement.second << "') OR main.activityname = '" << psCurrentComplement.second << "') AND contentarg.instance IN (SELECT instance FROM contentarg WHERE argument ";


                if (sCurrentPronoun == "you")
                {
                    osRequest << " = 'icub')  AND contentarg.instance IN (SELECT instance FROM main WHERE begin = true)";
                }
                else
                {
                    osRequest << " != 'icub') AND contentarg.instance IN (SELECT instance FROM main WHERE begin = true)";
                }
            }

            bMessenger.clear();
            bMessenger.addString("request");
            bMessenger.addString(osRequest.str().c_str());

            Port2ABM.write(bMessenger, bAnswer);

            cout << "Reponse de ABM : \n" << bAnswer.toString() << endl;
            if (bAnswer.toString() == "NULL" || bAnswer.isNull())
            {
                iCurrentInstance = -1;
                bSpeak.clear();
                bSpeak.addString("At least once... I guess...");
                sLastSentence = bSpeak.toString();
                Port2iSpeak.write(bSpeak);

                bOutput = bSpeak;
                return node2();
            }

            ostringstream osAnswer;
            osAnswer << "It happend " << bAnswer.get(0).asList()->get(0).toString() << " times";
            bSpeak.clear();
            bSpeak.addString(osAnswer.str());
            sLastSentence = osAnswer.str();
            Port2iSpeak.write(bSpeak);

            return node2();
        }
    }

    return bOutput;
}


Bottle abmHandler::node3()
{
    sCurrentNode = "Node 3";
    sCurrentGrammarFile = nameGrammarNode3;
    ostringstream osError;          // Error message
    osError << "Error in abmHandler | " << sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSpeak, // to ommunicate with the tts
        bTemp;

    Bottle bOutput;

    string sMessage = "something else ?";
    iCub->say(sMessage);
    bSpeak.addString(sMessage);
    cout << sMessage << endl;
    Port2iSpeak.write(bSpeak);
    bSpeak.clear();

    bool fGetaReply = false;


    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());


    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 3)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        cout << "Reply from speech : " << bSpeechRecognized.toString() << endl;

        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)
        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }

    }




    // REPLY GET FROM SPEECH RECOGNIZER


    // STOP & REPEAT
    if (bAnswer.get(0).asString() == "stop")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << sCurrentNode << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return bOutput;
    }

    if (bAnswer.get(1).asList()->get(0).asString() == "repeat")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << sCurrentNode << " | REPEAT called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;

        bSpeak.clear();
        sLastSentence = "I said, " + sLastSentence;
        bSpeak.addString(sLastSentence);
        Port2iSpeak.write(bSpeak);

        return node3();
    }

    if (!bAnswer.get(1).isList())
    {
        osError << "semantic is not bottle";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return bOutput;
    }


    //ELSE

    // semantic is the list of the semantic elements of the sentence except the type ef sentence
    bSemantic = *bAnswer.get(1).asList();
    string sNextTopic = bSemantic.get(0).toString();


    if (sNextTopic == "ELSE")
    {
        cout << "something else" << endl;
        return node1();
    }

    // MORE DETAILS ABOUT A SHARED PLAN

    if (sNextTopic == "EXPLAIN")
    {

        Bottle bNextTopic = *bSemantic.get(1).asList();
        string sSP = bSemantic.check("shared_plan", Value("none")).asString();

        // GET THE INSTANCE OF THE FIRST ENCOUNTER

        ostringstream osRequest;
        osRequest << "SELECT instance FROM main WHERE activitytype = 'sharedplan' AND activityname = '" << sSP << "' LIMIT 1";
        bMessenger.clear();
        bMessenger.addString("request");
        bMessenger.addString(osRequest.str().c_str());

        Port2ABM.write(bMessenger, bAnswer);

        cout << "Reponse de ABM : \n" << bAnswer.toString() << endl;
        if (bAnswer.toString() == "NULL" || bAnswer.isNull())
        {
            iCurrentInstance = -1;
            osError.str("");
            osError << sCurrentNode << " :: Response from ABM :: Unknown Event";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        iCurrentInstance = atoi(bAnswer.get(0).asList()->get(0).toString().c_str());
        if (iCurrentInstance < 7)
        {
            iCurrentInstance = -1;
            osError.str("");
            osError << sCurrentNode << " :: Response from ABM :: Unknown Event";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        Bottle bToImagine;
        bToImagine.addString("imagine");
        bToImagine.addInt(iCurrentInstance);
        Port2abmReasoning.write(bToImagine);

        bAnswer.clear();

        // FIRST ENCOUNTER FOUND AND IMAGINED



        // GET THE ARGUMENT OF THE FIRST PLAN

        osRequest.str("");
        osRequest << "SELECT DISTINCT contentarg.role, contentarg.argument FROM main, contentarg WHERE main.instance = contentarg.instance AND main.begin = true AND main.instance  = " << iCurrentInstance;

        bMessenger.clear();
        bMessenger.addString("request");
        bMessenger.addString(osRequest.str().c_str());

        Port2ABM.write(bMessenger, bAnswer);

        cout << "Reponse de ABM : \n" << bAnswer.toString() << endl;
        if (bAnswer.toString() == "NULL" || bAnswer.isNull())
        {
            iCurrentInstance = -1;
            osError.str("");
            osError << sCurrentNode << " :: Response from ABM :: Unknown Event";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }

        Bottle bListRole,
            bListArgument;

        for (int i = 0; i < bAnswer.size(); i++)
        {
            bListRole.addString(bAnswer.get(i).asList()->get(0).toString());
            bListArgument.addString(bAnswer.get(i).asList()->get(1).toString());
        }


        // GET THE PLAN USING THE ARGUMENT OF THE FIRST ENCOUNTER

        Bottle bToabmReasoning,
            bAction,
            bExplanation;
        bToabmReasoning.addString("executeActivity");
        bToabmReasoning.addString("sharedplan");
        bToabmReasoning.addString(sSP);
        bToabmReasoning.addList().copy(bListArgument);
        bToabmReasoning.addList().copy(bListRole);
        bTemp.clear();
        Port2abmReasoning.write(bToabmReasoning, bTemp);
        cout << "Response from abmR : " << endl << bTemp.toString() << endl;

        if (bTemp.get(0).toString() != "ack")
        {
            iCurrentInstance = -1;
            osError.str("");
            osError << sCurrentNode << " :: Response from abmReasoning :: Unknown Event";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return bOutput;
        }
        bExplanation = *bTemp.get(1).asList();

        cout << endl << "bExplanation : " << bExplanation.toString() << endl << endl;

        ostringstream osAnswer;
        Time::delay(3);
        sLastSentence = "";

        for (int i = 0; i < bExplanation.size(); i++)
        {
            bAction = *bExplanation.get(i).asList();
            /* bAction :
            move absolut (0.0  0.0) (action put (left cross icub) (spatial1 object1 agent1))
            */
            cout << "Sub bottle " << i << "\t";
            cout << bExplanation.get(i).toString() << endl;
            osAnswer.str("");

            string sAgent = bAction.get(3).asList()->get(2).asList()->get(2).toString();
            string sAction = bAction.get(3).asList()->get(1).toString();
            string sObject = bAction.get(3).asList()->get(2).asList()->get(1).toString();
            string sArgu = bAction.get(3).asList()->get(2).asList()->get(0).toString();

            osAnswer << sAgent << " " << sAction << " the " << sObject << " to the " << sArgu;
            bListArgument.clear();
            bListRole.clear();

            bListArgument = *bAction.get(3).asList()->get(2).asList();
            bListRole = *bAction.get(3).asList()->get(3).asList();

            bToImagine.clear();
            bToImagine.addString("executeActivity");
            bToImagine.addString("action");
            bToImagine.addString(sAction);
            bToImagine.addList().copy(bListArgument);
            bToImagine.addList().copy(bListRole);

            cout << bToImagine.toString() << endl;

            Port2OPCManager.write(bToImagine);

            (i != bExplanation.size() - 1) ? osAnswer << ", then, " : osAnswer << ".";

            bSpeak.clear();
            bSpeak.addString(osAnswer.str());
            cout << "osAnswer : " << osAnswer.str() << endl;
            sLastSentence += osAnswer.str();
            Port2iSpeak.write(bSpeak);
            Time::delay(4);
        }
    }

    return node2();
}


// from a date on string format : "2013-06-27 11:25:36", get a sentence for the tts
string abmHandler::dateToSpeech(string sDate)
{
    string sOutput;

    char *cBuffer;
    cBuffer = (char*)sDate.c_str();
    unsigned int i = 0;
    int iLevel = 0;
    //  int iHH,iMM,iSS; //iYear,iMonth,iDay,
    string sYear, sMonth, sDay, sHH, sMM, sSS = "";
    //  bool bYear,bMonth,bDay,bHH,bMM = false;
    while (cBuffer[i] != '\0')
    {
        char cTemp = cBuffer[i];
        if (cTemp == ' ' || cTemp == '-' || cTemp == ':' || cTemp == '+')
        {
            iLevel++;
        }
        else if (cTemp != '"')
        {
            switch (iLevel)
            {
            case 0:
                sYear += cTemp;
                break;
            case 1:
                sMonth += cTemp;
                break;
            case 2:
                sDay += cTemp;
                break;
            case 3:
                sHH += cTemp;
                break;
            case 4:
                sMM += cTemp;
                break;
            case 5:
                sSS += cTemp;
                break;
            }
        }
        i++;
    }

    string sNameMonth;
    switch (atoi(sMonth.c_str()))
    {
    case 1:
        sNameMonth = "january";
        break;
    case 2:
        sNameMonth = "february";
        break;
    case 3:
        sNameMonth = "march";
        break;
    case 4:
        sNameMonth = "april";
        break;
    case 5:
        sNameMonth = "may";
        break;
    case 6:
        sNameMonth = "june";
        break;
    case 7:
        sNameMonth = "july";
        break;
    case 8:
        sNameMonth = "august";
        break;
    case 9:
        sNameMonth = "september";
        break;
    case 10:
        sNameMonth = "october";
        break;
    case 11:
        sNameMonth = "november";
        break;
    case 12:
        sNameMonth = "december";
        break;
    }

    ostringstream osOutput;
    osOutput << sDay << ", of " << sNameMonth << ", of " << sYear;
    sOutput = osOutput.str();
    return sOutput;
}

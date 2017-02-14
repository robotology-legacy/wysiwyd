/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Anne-Laure MEALIER
 * email:   anne-laure.mealier@inserm.fr
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

#include "reservoirHandler.h"
#include "wrdac/subsystems/subSystem_ARE.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;


/*
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the
 * equivalent of the "open" method.
 */

bool reservoirHandler::configure(ResourceFinder &rf) {

    iCurrentInstance = -1;
    bool    bEveryThingisGood = true;
    bool    bOptionnalModule = true;
    moduleName = rf.check("name",
        Value("reservoirHandler"),
        "module name (string)").asString();

    sKeyWord = rf.check("keyword", Value("grammar")).toString().c_str();
    cout << "**************Context path for grammars: " << rf.getContextPath() << endl;
    nameGrammarNodeType = rf.getContextPath().c_str();
    nameGrammarNodeType += rf.check("nameGrammarNodeType", Value("/nameGrammarNodeType.xml")).toString().c_str();
    nameGrammarNodeModality = rf.getContextPath().c_str();
    nameGrammarNodeModality += rf.check("nameGrammarNodeModality", Value("/nameGrammarNodeModality.xml")).toString().c_str();
    nameGrammarNodeTrainAP = rf.getContextPath().c_str();
    nameGrammarNodeTrainAP += rf.check("nameGrammarNodeTrainAP", Value("/nameGrammarNodeTrainAP.xml")).toString().c_str();
    nameGrammarNodeTestAP = rf.getContextPath().c_str();
    nameGrammarNodeTestAP += rf.check("nameGrammarNodeTestAP", Value("/nameGrammarNodeTestAP.xml")).toString().c_str();
    nameGrammarNodeTrainSD = rf.getContextPath().c_str();
    nameGrammarNodeTrainSD += rf.check("nameGrammarNodeTrainSD", Value("/nameGrammarNodeTrainSD.xml")).toString().c_str();
    nameGrammarYesNo = rf.getContextPath().c_str();
    nameGrammarYesNo += rf.check("nameGrammarYesNo", Value("/nameGrammarYesNo.xml")).toString().c_str();
    nameGrammarNodeInteraction = rf.getContextPath().c_str();
    nameGrammarNodeInteraction += rf.check("nameGrammarNodeInteraction", Value("/nameGrammarNodeInteraction.xml")).toString().c_str();

    fvector = rf.getContextPath().c_str();
    fvector += rf.check("vectorFile", Value("/vector.txt")).toString().c_str();

    cout << fvector << "        " << endl;
    pythonPath = rf.getContextPath().c_str();
    cout << "rf.getContextPath().c_str() : " << rf.getContextPath().c_str() << endl;
    pythonPath += rf.check("pythonPath", Value("/RAD/src/iCub_language")).toString().c_str();
    cout << "pythonPath : " << pythonPath << endl;

    /* Mode Action Performer => Meaning*/
    fileAPimputS = rf.getContextPath().c_str();
    fileAPimputS += rf.check("APimputS", Value("/AP_input_S.txt")).toString().c_str();
    fileAPoutputM = rf.getContextPath().c_str();
    fileAPoutputM += rf.check("APoutputM", Value("/AP_output_M.txt")).toString().c_str();
    fileXavierTrainAP = rf.getContextPath().c_str();
    fileXavierTrainAP += rf.check("xavierTrainAP", Value("/xavier_trainAP.txt")).toString().c_str();

    fileAP = rf.check("fileAP", Value("action_performer.py")).toString().c_str();

    /* Mode Scene Describer => Produce*/
    fileSRinputM = rf.getContextPath().c_str();
    fileSRinputM += rf.check("SRinputM.txt", Value("/SR_input_M.txt")).toString().c_str();
    fileSRoutputS = rf.getContextPath().c_str();
    fileSRoutputS += rf.check("SRoutputS", Value("/SR_output_S.txt")).toString().c_str();
    fileXavierTrain = rf.getContextPath().c_str();
    fileXavierTrain += rf.check("xavierTrain", Value("/xavier_train.txt")).toString().c_str();

    fileSD = rf.check("fileSD", Value("spatial_relation.py")).toString().c_str();

    sHand = rf.check("hand", Value("right")).toString().c_str();
    ZRTObjects = rf.check("ZRTObjects", Value("0.080")).asDouble();
    offsetGrasp = rf.check("offsetGrasp", Value("0.02")).asDouble();
    bMode = rf.check("Mode", Value("test")).toString().c_str();

    testAction = rf.check("action", Value("point")).toString().c_str();
    testObject = rf.check("object", Value("Cross")).toString().c_str();
    testLocation = rf.check("location", Value("right")).toString().c_str();


    /*
    * before continuing, set the module name before getting any other parameters,
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    // Open handler port
    string sName = getName();
    handlerPortName = "/" + sName + "/rpc";

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        bEveryThingisGood = false;
    }


    // Open port2speech
    port2SpeechRecogName = "/" + sName + "/toSpeechRecog";

    if (!Port2SpeechRecog.open(port2SpeechRecogName.c_str())) {
        cout << getName() << ": Unable to open port " << port2SpeechRecogName << endl;
        bEveryThingisGood = false;
    }


    // Open port2iSpeak
    port2iSpeakName = "/" + sName + "/toiSpeak";

    if (!Port2iSpeak.open(port2iSpeakName.c_str())) {
        cout << getName() << ": Unable to open port " << port2iSpeakName << endl;
        bOptionnalModule = false;
    }

    attach(handlerPort);                  // attach to port

    //------------------------//
    //      iCub Client
    //------------------------//

    // string ttsSystem = SUBSYSTEM_SPEECH;
    iCub = new ICubClient(moduleName.c_str(), "reservoirHandler", "client.ini", true);
    iCub->opc->isVerbose = false;

    char rep = 'n';
    while (rep != 'y'&&!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        break; //to debug
        Time::delay(1.0);
    }
    cout << "Connections done" << endl;
    iCub->opc->checkout();
    cout << "Checkout done" << endl;

    // Connect iCub Client, and ports
    bOptionnalModule &= Network::connect(port2SpeechRecogName.c_str(), "/speechRecognizer/rpc");
    bOptionnalModule &= Network::connect("/mainLoop/speechGrammar/keyword:o", handlerPortName.c_str());
    bOptionnalModule &= Network::connect(port2iSpeakName, "/iSpeak");

    if (!bOptionnalModule)
    {
        cout << endl << "Some dependencies are notrunning (ICubClient or port(s) connections)" << endl << endl;
    }

    if (!bEveryThingisGood || !bOptionnalModule)
        cout << endl << "Some dependencies are not running (ICubClient or port(s) connections)" << endl << endl;
    else
        cout << endl << endl << "----------------------------------------------" << endl << endl << "reservoirHandler ready !" << endl << endl;

    //populateOPC();
    nodeType();
    //testARE();
    return false;
    //    return bEveryThingisGood ;
}

bool reservoirHandler::testARE(){
    vector<string> testseq;

    testseq.push_back(testAction);
    testseq.push_back(testObject);
    testseq.push_back(testLocation);
    testseq.push_back("quickly");

    cout << "initialisation done" << endl;
    AREactions(testseq);

    return true;
}

bool reservoirHandler::populateOPC(){
    iCub->opc->update();
    iCub->opc->commit();
    RTObject* obj1 = iCub->opc->addOrRetrieveEntity<RTObject>("cube");

    Vector dimensionObject(3);
    dimensionObject[0] = 0.065;
    dimensionObject[1] = 0.065;
    dimensionObject[2] = 0.08;

    Vector color(3);
    color[0] = 50;
    color[1] = 100;
    color[2] = 50;

    Vector x(3);
    x[0] = -0.38;
    x[1] = 0.2;
    x[2] = 0.0016;

    //vGoal is : -0.350000   0.200000    0.001600
    obj1->m_ego_position = x;
    obj1->m_present = 1.0;
    obj1->m_dimensions = dimensionObject;
    obj1->m_color = color;

    color[0] = 0;
    color[1] = 100;
    color[2] = 200;
    RTObject* obj2 = iCub->opc->addOrRetrieveEntity<RTObject>("mouse");
    x[0] = -0.45;  //y position
    x[1] = 0.0;    //x position
    x[2] = 0.0016; //z position
    //vGoal is : -0.350000   0.200000    0.001600
    obj2->m_ego_position = x;
    obj2->m_present = 1.0;
    obj2->m_dimensions = dimensionObject;
    obj2->m_color = color;

    color[0] = 70;
    color[1] = 200;
    color[2] = 80;
    RTObject* obj3 = iCub->opc->addOrRetrieveEntity<RTObject>("croco");
    x[0] = -0.35;
    x[1] = -0.2;
    x[2] = 0.0016;
    //vGoal is : -0.350000   0.200000    0.001600
    obj3->m_ego_position = x;
    obj3->m_present = 1.0;
    obj3->m_dimensions = dimensionObject;
    obj3->m_color = color;

    Agent* coco = iCub->opc->addOrRetrieveEntity<Agent>("Michel");
    coco->m_ego_position[0] = -1.2;
    coco->m_ego_position[2] = 0.60;
    coco->m_present = 1.0;
    iCub->opc->commit();
    return false;
}

bool reservoirHandler::interruptModule(){

    handlerPort.close();
    Port2SpeechRecog.close();
    Port2iSpeak.close();

    return true;
}

bool reservoirHandler::close() {
    iCub->opc->close();
    iCub->close();
    return true;
}

bool reservoirHandler::respond(const Bottle& command, Bottle& reply) {
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
        nodeType();
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool reservoirHandler::updateModule() {


    return true;
}

double reservoirHandler::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}

/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string reservoirHandler::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        cout << "Error in reservoirHandler::grammarToString. Couldn't open file : " << sPath << "." << endl;
        return "Error in reservoirHandler::grammarToString. Couldn't open file";
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
*   produce or understand
*/
bool reservoirHandler::nodeType()
{
    sCurrentNode = "nodeType";
    sCurrentGrammarFile = nameGrammarNodeType;
    ostringstream osError;          // Error message
    osError << "Error in reservoirHandler | " << sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic; // semantic information of the content of the recognition

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say
    cout << "ICUB SAY : Do you want me to understand or produce language? " << endl;


    while (!fGetaReply)
    {

        bSpeechRecognized.clear();
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        //e.g. : Reply from Speech Recog : 1 ("I want you to produce language" (INFORMATION (type produce))) ACK
        cout << "In " << sCurrentNode << " Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }

        if (bSpeechRecognized.get(1).toString() == "UNKNOWN")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

    if (bAnswer.get(0).asString() == "stop the interaction")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return false;
    }

    bSemantic = *bAnswer.get(1).asList()->get(1).asList();
    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    if (sQuestionKind == "INFORMATION")
    {
        sCurrentActivity = bSemantic.check("type", Value("none")).asString();

        if (sCurrentActivity == "understand")
        {
            /*
                 * Mode Action Performer => Meaning
                 * Training
                 * 1. Robot generates random actions [meaning]
                 * 2. Human says a corresponding command [sentence]
                 * Testing
                 * 1. Human says a command [sentence]
                 * 2. Robot performs corresponding actions [meaning]
                 */

            cout << "iCub says : 'Set the objects'" << endl;
            iCub->say("I am ready...");
            iCub->say("Tell me a sentence");
            sCurrentType = "test";
            return nodeTestAP();
        }


        else if (sCurrentActivity == "produce")
        {
            /*
                 * Mode Scene Describer => Produce
                 * Training
                 * 1. Human arranges objects on the table [meaning]
                 * 2. Human describes the scene [sentence]
                 * Testing
                 * 1. Human arranges objects on the table [meaning]
                 * 2. Robot describes the scene [sentence]
                 */
            cout << "here" << endl;
            iCub->say("I am ready...");
            iCub->say("Do you want me to focus the description about object or location ?");
            return nodeModality();
        }

        else{
            return nodeType();
        }
    }

    return true;
}

bool reservoirHandler::nodeModality()
{
    sCurrentNode = "nodeModality";
    sCurrentGrammarFile = nameGrammarNodeModality;
    ostringstream osError;          // Error message
    osError << "Error in reservoirHandler | " << sCurrentNode << " :: ";

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic; // semantic information of the content of the recognition

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say

    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        cout << "In " << sCurrentNode << " Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }

        if (bSpeechRecognized.get(1).toString() == "UNKNOWN")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    if (bAnswer.get(0).asString() == "stop the interaction")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return false;
    }


    bSemantic = *bAnswer.get(1).asList()->get(1).asList();
    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    if (sQuestionKind == "CANONICAL")
    {
        sCurrentCanonical = bSemantic.check("focus", Value("none")).asString();

        if (sCurrentCanonical == "object")
        {
            iCub->say("Fine. Let's focus about object");
            cout << "iCub says : 'Fine. Let's focus about object'" << endl;
            sSentence_type = " :C";

        }
        else if (sCurrentCanonical == "location")
        {
            iCub->say("Oh, tricky! Let's go with locations");
            cout << "iCub says : 'Oh, tricky! Let's go with locations'" << endl;
            sSentence_type = " :N";
        }
        inbsentence = 2;
        sobjectFocusChanged = "";
        sCurrentType = "test";
        iCub->say("Ok, Set your initial situation, and show me your object of focus !");

        while (!nodeYesNo())
        {
        }
        return spatialRelation();
    }

    // to avoid saying "Go in test Mode"
    //sQuestionKind = "INFORMATION";
    //sCurrentType = bMode.c_str();

    if (sQuestionKind == "INFORMATION")
    {
        if (sCurrentActivity == "understand")
        {
            /* Mode Action Performer => Meaning */

            // Module test_mode

            iCub->say("Let me see");
            cout << "iCub says : 'Let me see...'" << endl;
            iCub->say("I see all the objects");
            cout << "iCub says : 'I see all the objects'" << endl;

            if (sCurrentType == "train")
            {
                /*
                 * Training
                 * 1. Robot generates random actions [meaning]
                 * 2. Human says a corresponding command [sentence]
                 */
                if (lMeaningsSentences.size() != 0)
                {
                    lMeaningsSentences.clear();
                }
                cout << "iCub says : 'What is this action ?'" << endl;
                if (sCurrentCanonical.length() != 0)
                {
                    return nodeTrainAP();
                }
                else
                {
                    return nodeModality();
                }
            }
            else if (sCurrentType == "test")
            {
                /*
                * Testing
                * 1. Human says a command [sentence]
                * 2. Robot performs corresponding actions [meaning]
                */
                iCub->say("I am ready...");
                iCub->say("Tell me a sentence");
                return nodeTestAP();
            }
        }
        else if (sCurrentActivity == "produce")
        {
            cout << "Dans le produce (Modality)" << endl;
            /* Mode Scene Describer => Produce sentence*/
            cout << sCurrentType << endl;
            if (sCurrentType == "test")
            {
                /*
                 * Testing
                 * 1. Human arranges objects on the table [meaning]
                 * 2. Robot describes the scene [sentence]
                 */
                cout << "Dans le test (modality)" << endl;

                iCub->say("I am ready...");
                iCub->say("Do you want me to focus the description about object or location ?");
                inbsentence = 2;
                sobjectFocusChanged = "";
                return nodeTestSD();
            }

            else if (sCurrentType == "train")
            {
                /*
                    * Training
                    * 1. Human arranges objects on the table [meaning]
                    * 2. Human describes the scene [sentence]
                    */
                if (lMeaningsSentences.size() != 0)
                {
                    lMeaningsSentences.clear();
                }

                inbsentence = 1;
                return nodeTestSD();
            }

            cout << "iCub says : 'The focus object is $OBJ_FOCUS'" << endl;
        }
    }

    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)


    if (bAnswer.get(0).asString() == "stop the interaction")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return false;
    }

    if (bAnswer.get(0).asString() == "return to the interaction")
    {
        return nodeModality();
    }

    return true;
}

bool reservoirHandler::nodeTrainAP()
{
    sCurrentNode = "nodeModality";
    sCurrentGrammarFile = nameGrammarNodeModality;
    ostringstream osError;          // Error message
    osError << "Error in reservoirHandler | " << sCurrentNode << " :: ";

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic; // semantic information of the content of the recognition

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        cout << "In " << sCurrentNode << " Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }

        if (bSpeechRecognized.get(1).toString() == "UNKNOWN")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    if (bAnswer.get(0).asString() == "stop the interaction")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return false;
    }

    bSemantic = *bAnswer.get(1).asList()->get(1).asList();
    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    cout << "iCub execute an action...'" << endl;
    cout << "iCub says : 'What is this action ?      '" << endl;

    // Do you know any ...
    if (sQuestionKind == "sentence")
    {
        /*
         * bAnswer.get(0).asString() => the circle is to the left of of the cross
         * bAnswer.toString() =>
         * "the circle is to the left of of the cross" (sentence (sentence1 ((object "the circle") (relative_complete ((spatial_relative ((relative to) (spatial "the left"))) (object "the cross"))))))
         */
        cout << "iCub says :  " << bAnswer.get(0).asString() << endl;
        iCub->say("Do you want continue or exit");
        sSentence = bAnswer.get(0).asString();
        cout << "sSentence" << sSentence << endl;
        return nodeTrainAP();
    }

    else if (sQuestionKind == "follow")
    {
        string continueExit = bSemantic.check("mode", Value("none")).asString();


        if (continueExit == "continue the interaction")
        {
            cout << "lMeaningsSentences " << endl;
            lMeaningsSentences.push_back(sSentence);
            return nodeTrainAP();
        }
        else if (continueExit == "exit the interaction")
        {
            trainSaveMeaningSentence(fileAPimputS.c_str());
            return nodeType();
        }
    }

    return true;
}

bool reservoirHandler::nodeTestAP()
{
    sCurrentNode = "nodeTestAP";
    sCurrentGrammarFile = nameGrammarNodeTestAP;
    ostringstream osError;          // Error message
    osError << "Error in reservoirHandler | " << sCurrentNode << " :: ";

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic; // semantic information of the content of the recognition

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say


    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        cout << "In " << sCurrentNode << " Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }

        if (bSpeechRecognized.get(1).toString() == "UNKNOWN")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    if (bAnswer.get(0).asString() == "stop the interaction")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return false;
    }

    bSemantic = *bAnswer.get(1).asList()->get(1).asList();
    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    // Do you know any ...
    if (sQuestionKind == "sentence")
    {
        /*
         * bAnswer.get(0).asString() => the circle is to the left of of the cross
         * bAnswer.toString() =>
         *  ("before you point to the mouse push the croco to the right" (sentence ((temporal "before you") (actionX (action1 ((verb1 point) (object mouse)))) (actionX (action2 (action21 ((verb2 push) (object croco) (location right))))))))
         */

        cout << "iCub says : 'I have understood      '" << bAnswer.get(0).asString() << endl;



        iCub->say("I have understood ", false);
        sentence += bAnswer.get(0).asString() + " ";
        iCub->say(bAnswer.get(0).toString());
        iCub->say("Is it ok ?");
        cout << "iCub says : 'Is it ok ? ... 'No or Yes" << endl;
        return nodeTestAP();
    }


    else if (sQuestionKind == "yesno")
    {
        string yesNo = bSemantic.check("agree", Value("none")).asString();


        if (yesNo == "it's right")
        {
            copyPastFile(fileXavierTrainAP.c_str(), fileAPimputS.c_str());
            cout << fileXavierTrainAP << endl;
            cout << fileAPimputS << endl;
            createTestwithTrainData(fileAPimputS.c_str(), sentence);
            callReservoir(fileAP);
            cout << fileAPoutputM << endl;
            string result = openResult(fileAPoutputM.c_str());
            iCub->say(result, false);

            int id = result.find(",");
            int idf = result.size() - id;

            fileVectorAP.open(fvector.c_str(), ios::out | ios::trunc);

            iCub->say("I will do the actions", false);


            // CREATE VECTOR FILE
            //force<push(You,circle)>;moved(circle)>
            mAssociation["put"] = "placed";
            mAssociation["take"] = "got";
            mAssociation["grasp"] = "hold";
            mAssociation["push"] = "moved";
            mAssociation["point"] = "stayed";


            if (result.find(",") != string::npos)
            {
                cout << "resut 1 : " << result.substr(0, id) << endl;
                cout << "resut 2 : " << result.substr(id + 1, idf) << endl;
                string firstCommand = result.substr(0, id);
                string secondCommand = result.substr(id + 1, idf);

                bool ok = AREactions(extractVocabulary(firstCommand));
                if (ok)
                    AREactions(extractVocabulary(secondCommand));

                sentence = " ";
            }
            else
            {
                cout << "result : " << result << endl;
                AREactions(extractVocabulary(result));
                sentence = " ";
            }

            cout << "iCub do the action..." << endl;
            fileVectorAP.close();

            //
            iCub->say("Do you have any questions", false);
            return nodeYesNoInteraction();
        }

        else if (yesNo == "not the good sentence")
        {
            sentence = " ";
            return nodeTestAP();
        }

        else if (sQuestionKind == "follow")
        {
            string continueExit = bSemantic.check("mode", Value("none")).asString();

            if (continueExit == "continue the interaction")
            {
                cout << "Humain say a command ...." << endl;
                sentence = " ";
                return nodeTestAP();
            }
            else if (continueExit == "exit the interaction")
            {
                return nodeType();
            }
        }
    }

    if (bAnswer.get(0).asString() == "change the interaction")
    {
        return nodeType();
    }

    return true;
}

bool reservoirHandler::nodeTestSD()
{
    sCurrentNode = "nodeTestSD";
    sCurrentGrammarFile = nameGrammarNodeTrainSD;
    ostringstream osError;          // Error message
    osError << "Error in reservoirHandler | " << sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic; // semantic information of the content of the recognition

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say

    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        cout << "In " << sCurrentNode << " Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }

        if (bSpeechRecognized.get(1).toString() == "UNKNOWN")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    bSemantic = *bAnswer.get(1).asList()->get(1).asList();
    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    if (sCurrentType == "train")
    {
        // Do you know any ...
        if (sQuestionKind == "sentence")
        {
            cout << "Human :  'Let me see, I have finished...'" << endl;
            cout << "iCub says : 'What is the corresponding sentence      '" << bAnswer.get(0).asString() << "    " << bAnswer.toString() << endl;
            /*
                 * bAnswer.get(0).asString() => the circle is to the left of of the cross
                 * bAnswer.toString() =>
                 * "the circle is to the left of of the cross" (sentence (sentence1 ((object "the circle") (relative_complete ((spatial_relative ((relative to) (spatial "the left"))) (object "the cross"))))))
                 */
            cout << "iCub says : 'I have recognized      '" << bAnswer.get(0).asString() << endl;
            cout << "iCub says : 'Is it ok ? ... 'continue or exit" << endl;
            string sSentence = bAnswer.get(0).asString();
            nodeTestSD();
        }

        else if (sQuestionKind == "follow")
        {
            string continueExit = bSemantic.check("mode", Value("none")).asString();


            if (continueExit == "continue the interaction")
            {
                cout << "lMeaningsSentences " << endl;
                lMeaningsSentences.push_back(sSentence + sSentence_type);
                nodeTestSD();

            }
            else if (continueExit == "exit the interaction")
            {
                trainSaveMeaningSentence(fileSRinputM.c_str());
                nodeType();
            }
        }

    }

    else if (sCurrentType == "test")
    {


    }

    if (bAnswer.get(0).asString() == "change the interaction")
    {
        return nodeType();
    }

    return true;
}

bool reservoirHandler::callReservoir(string fPython)
{
    //launch Xavier reservoir
    string command = "cd " + pythonPath + " && python " + fPython;
    bool bsys = system(command.c_str());
    cout << "cd " << pythonPath << " && python " << fPython << endl;
    return bsys;
}


//
string reservoirHandler::openResult(const char* fileNameIn)
{
    ifstream in;
    in.open(fileNameIn);
    string str;
    getline(in, str);
    cout << str << endl;
    in.close();

    return str;
}

vector<string> reservoirHandler::extractVocabulary(string sequence)
{

    string object, recipient, location, adverbs;

    //////////////////////////
    if (sequence.find("grasp") != string::npos || sequence.find("take") != string::npos || sequence.find("put") != string::npos)
    {
        recipient = "grasp";
    }
    else if (sequence.find("point") != string::npos)
    {
        recipient = "point";
    }

    else if (sequence.find("push") != string::npos)
    {
        recipient = "push";
    }

    ///////////////////////////
    if (sequence.find("circle") != string::npos)
    {
        object = "circle";
    }
    else if (sequence.find("cross") != string::npos)
    {
        object = "cross";
    }
    else if (sequence.find("triangle") != string::npos)
    {
        object = "triangle";
    }
    else if (sequence.find("square") != string::npos)
    {
        object = "square";
    }
    else if (sequence.find("eraser") != string::npos)
    {
        object = "eraser";
    }
    else if (sequence.find("croco") != string::npos)
    {
        object = "croco";
    }
    else if (sequence.find("cube") != string::npos)
    {
        object = "cube";
    }
    else if (sequence.find("mug") != string::npos)
    {
        object = "mug";
    }
    else if (sequence.find("mouse") != string::npos)
    {
        object = "mouse";
    }
    else if (sequence.find("rabbit") != string::npos)
    {
        object = "rabbit";
    }
    else if (sequence.find("wysiwyd sponge") != string::npos)
    {
        object = "wysiwyd sponge";
    }
    else if (sequence.find("white-trophy") != string::npos)
    {
        object = "white-trophy";
    }

    /////////////////////////////
    if (sequence.find("left") != string::npos)
    {
        location = "left";
    }
    else if (sequence.find("right") != string::npos)
    {
        location = "right";
    }
    else if (sequence.find("middle") != string::npos)
    {
        location = "middle";
    }
    else
        location = " ";

    /////////////////////////////
    if (sequence.find("quickly") != string::npos)
    {
        adverbs = "quickly";
    }
    else if (sequence.find("slowly") != string::npos)
    {
        adverbs = "slowly";
    }
    else
        adverbs = " ";

    vector<string> seq;
    seq.push_back(recipient);
    seq.push_back(object);
    if (location.size() != 0)
        seq.push_back(location);
    if (adverbs.size() != 0)
        seq.push_back(adverbs);

    cout << seq[0] << " " << seq[1] << " " << seq[2] << seq[3] << endl;

    return seq;
}

bool reservoirHandler::AREactions(vector<string> seq)
{
    string sPredicat, sObject, sLocation, sadverbs;
    float ftime;
    sPredicat = seq[0];
    sObject = seq[1];
    sLocation = seq[2];
    sadverbs = seq[3];

    if (sadverbs == "slowly"){
        ftime = 4.0;
    }
    else if (sadverbs == "quickly"){
        ftime = 0.0;
    }
    else{
        ftime = 2.0;
    }

    if (sPredicat == "none")
    {
        cout << "Error in reservoirHandler::AREactions | sPredicat == none" << endl;
        return false;
    }

    // GET LOCATION OF THE OBJECT IN THE OPC + OFFSET IN Z
    iCub->opc->update();
    RTObject *rtObject = dynamic_cast<RTObject*>(iCub->opc->getEntity(sObject));
    //Object *rtObject = iCub->opc->addEntity<Object>(sObject);
    Vector value(4);
    value = rtObject->m_ego_position;
    value[2] += offsetGrasp;
    cout << sObject << " is at: " << value.toString() << endl;

    bool success = true;

    if (rtObject->m_present == 1.0)
    {

        // GRASP BEGIN
        if (sPredicat == "put" || sPredicat == "take" || sPredicat == "grasp")
        {
            Bottle bHand(sHand);
            bHand.addString("still");
            cout << "sHand : " << sHand << endl;

            Object* Location = dynamic_cast<Object*>(iCub->opc->getEntity(sLocation));
            Vector vGoal = Location->m_ego_position;
            vGoal[2] += ZRTObjects;

            cout << "vGoal is : " << vGoal.toString() << endl;

            // DROP ON LOCATION
            if (sLocation != " ")
            {
                Time::delay(ftime);
                bool grasped = iCub->take(sObject, bHand);

                cout << (grasped ? "grasped!" : "missed!") << endl;

                success &= grasped;
                Time::delay(ftime);

                if (grasped){
                    Bottle opts("over still " + sHand);
                    bool dropped = iCub->release(vGoal, opts);
                    cout << (dropped ? "dropped!" : "missed!") << endl;
                    Time::delay(ftime);
                    iCub->home();
                    success &= dropped;
                }

            }
            // DROP WITHOUT LOCATION
            else
            {
                bool grasped = iCub->take(sObject, bHand);
                cout << (grasped ? "grasped!" : "missed!") << endl;
                Time::delay(ftime);
                success &= grasped;

                if (grasped)
                {
                    iCub->release(value, bHand);
                }
                Time::delay(ftime);
                iCub->home(bHand.toString());
            }
            // END GRASP
        }

        // PUSH
        else if (sPredicat == "push")
        {
            sLocation == "right" ? sHand = "left" : sHand = "right";

            Bottle bHand(sHand);
            cout << "sHand : " << sHand << endl;
            Time::delay(ftime);
            bool pushed = iCub->push(sObject, bHand);
            cout << (pushed ? "pushed!" : "missed!") << endl;
            Time::delay(ftime);
            success &= pushed;
            iCub->home();
        }

        // POINT
        else if (sPredicat == "point")
        {
            Time::delay(ftime);

            bool pointed = iCub->point(sObject);
            cout << (pointed ? "pointed!" : "missed!") << endl;

            success &= pointed;
            Time::delay(ftime);
            iCub->home(sHand);
        }

        if (sLocation != " "){
            if (sadverbs == " "){
                sVectorFileAP = "force<" + sPredicat + "(I," + sObject + "," + sLocation + ")>;result<" + mAssociation.find(sPredicat)->second + "(" + sObject + "," + sLocation + ")>";
            }
            else{
                sVectorFileAP = "force<" + sPredicat + "(I," + sObject + "," + sLocation + "," + sadverbs + ")>;result<" + mAssociation.find(sPredicat)->second + "(" + sObject + "," + sLocation + ")>";
            }
        }

        else if (sLocation == " "){
            if (sadverbs == " "){
                sVectorFileAP = "force<" + sPredicat + "(I," + sObject + ")>;result<" + mAssociation.find(sPredicat)->second + "(" + sObject + ")>";
            }
            else
            {
                sVectorFileAP = "force<" + sPredicat + "(I," + sObject + "," + sadverbs + ")>;result<" + mAssociation.find(sPredicat)->second + "(" + sObject + ")>";
            }
        }
        createVectorFile(sVectorFileAP);

    }
    else
    {
        iCub->say(sObject + " is not present", false);
        cout << sObject << " is not present ! " << endl;
        success = false;
    }

    cout << "Result of the action: " << (success ? "success!" : "missed!") << endl;

    return success;
}

//
/*
 * test mode : copy the train data in test file, add the end of the train data (if first test) and add the test sentence
 */
int reservoirHandler::copyTrainData(const char* fileNameIn, const char* fileNameOut)
{
    // Module Save
    ofstream file;
    if (inbsentence == 1){
        file.open(fileNameOut, ios::out | ios::trunc); // open the file in writing and erase the file if it is not empty
        file << "<train data>" << endl;
        inbsentence += 1;
        file.close();
    }
    if (sCurrentType == "train"){
        cout << "iCub says : 'Ok'" << endl;
        trainSaveMeaningSentence(fileNameOut);
    }
    else if (sCurrentType == "test"){
        cout << "iCub says : 'I will describe the situation'" << endl;
        inbsentence = 1;
        copyPastFile(fileNameIn, fileNameOut);
        createTestwithTrainData(fileNameOut, sdataTestSD);
    }
    file.close();

    return true;
}

int reservoirHandler::trainSaveMeaningSentence(const char* filename)
{
    ofstream file;
    file.open(filename, ios::out | ios::trunc);
    std::list<string>::iterator it;

    file << "<train data>" << endl;
    for (it = lMeaningsSentences.begin(); it != lMeaningsSentences.end(); ++it)
    {
        string meaning;
        if (sCurrentActivity == "produce")
        {
            meaning = "put trumpet left;put guitar right";
            file << meaning << ";" << *it << endl;
        }
        else
        {
            meaning = "left violin trumpet";
            file << *it << ";" << meaning << endl;
        }
    }
    file.close();

    return true;
}

int reservoirHandler::createTestwithTrainData(const char* filename, string sMeaningSentence)
{
    cout << "createTestwithTrainData(const char* filename, string sMeaningSentence " << endl;
    cout << filename << "    " << "sMeaningSentence" << endl;
    ofstream file;
    file.open(filename, ios::app);
    file << "</train data>" << endl;
    file << "<test data>" << endl;
    file << sMeaningSentence << endl;
    file << "</test data>" << endl;
    file.close();

    return true;
}

int reservoirHandler::copyPastFile(const char* fileNameIn, const char* fileNameOut)
{
    ifstream in;
    ofstream out;
    in.open(fileNameIn);
    out.open(fileNameOut, ios::out | ios::trunc);
    string str;
    while (getline(in, str))
    {
        out << str << endl;
    }
    in.close();
    out.close();

    return true;
}


//
std::list<int> reservoirHandler::nbCaracters(string ssequence)
{
    unsigned int pos = 0;
    std::list<int> lposElements;
    for (pos = 0; pos < ssequence.size(); ++pos)
    {
        if (ssequence[pos] == ','){
            lposElements.push_back(pos);
        }

    }
    return lposElements;
}


//
bool reservoirHandler::mainNodeInteraction()
{
    if (getline(fileVectorAPRead, svector))
    {
        iquestion = languageNodeInteraction();
    }
    else{
        iCub->say("I have no more information", false);
        fileVectorAPRead.close();
        iCub->say("Tell me an other sentence", false);
        return nodeTestAP();
    }
    return 0;
}

bool reservoirHandler::grammarNodeInteraction()
{
    cout << "################################################################" << endl;
    int id = svector.find(";");
    int idf = svector.size();
    string sforce = svector.substr(6, id - 7); //  => sforce=   "push(Ag1,object)"
    string sresult = svector.substr(id + 8, idf - 1 - (id + 8)); // =>   sresult=   "move(object)"

    string sverb, sagent1, sagent2, sobject, slocation, sadverb;

    if (iquestion == 1)  // => Case "What happened"  => Result part !!
    {
        int i = sresult.find("(");
        int iend = sresult.size();
        sverb = sresult.substr(0, i); // => "move"
        cout << "sverb : " << sverb << endl;

        sresult = sresult.substr(i + 1, iend - (i + 1) - 1); // =>  "object"
        cout << "sresult : " << sresult << endl;

        std::list<int> lposElements = nbCaracters(sresult);
        int nb = lposElements.size();


        if (nb == 0){            //  "object"
            cout << "nb elements " << nb << endl;
            sobject = sresult;
            sanswer = "The " + sobject + " " + sverb;
            iCub->say(sanswer, false);
            cout << sanswer << endl;
        }

        else if (nb == 1)   //  "object,location"
        {
            cout << "nb elements " << nb << endl;
            std::list<int>::iterator it;

            for (it = lposElements.begin(); it != lposElements.end(); ++it)
            {
                sobject = sresult.substr(0, *it);
                cout << "sagent1 : " << sobject << endl;
                slocation = sresult.substr(*it + 1, lposElements.size() - (*it + 1));
                cout << "sobject : " << slocation << endl;
            }
            sanswer = "The " + sobject + " " + sverb + " to the " + slocation;
            iCub->say(sanswer, false);
            cout << sanswer << endl;
        }
    }
    else if (iquestion == 2)  // "what did Anne do ?" => active form
    {
        int i = sforce.find("(");
        int iend = sforce.size();
        sverb = sforce.substr(0, i); // => "result"
        cout << "sverb : " << sverb << endl;

        sforce = sforce.substr(i + 1, iend - (i + 1) - 1); // =>  "Ag2,object"
        cout << "sforce : " << sforce << endl;

        std::list<int> lposElements = nbCaracters(sforce);
        int nb = lposElements.size();

        if (nb == 0){            //  "object"
            cout << "nb elements " << nb << endl;
            sobject = sforce;
            iCub->say(sanswer, false);
            cout << sanswer << endl;
        }

        else if (nb == 1)   //  "Ag2,object"
        {
            cout << "nb elements " << nb << endl;
            std::list<int>::iterator it;
            int cpt = 0;

            vector<int> tab(lposElements.size(), 0);
            for (it = lposElements.begin(); it != lposElements.end(); ++it)
            {
                tab[cpt] = *it;
                cpt++;
                cout << tab << endl;
            }

            sagent1 = sforce.substr(0, tab[0]);
            cout << "sagent1 : " << sagent1 << endl;
            sobject = sforce.substr(tab[0] + 1, sforce.size() - tab[0] + 1);
            cout << "sobject : " << sobject << endl;
            cout << "tab : " << tab[0] << endl;
            sanswer = sagent1 + " " + sverb + "ed the " + sobject;
            iCub->say(sanswer, false);
            cout << sanswer << endl;
        }
        else if (nb == 2)   //  "Ag1,object,location"
        {
            cout << "nb elements " << nb << endl;
            std::list<int>::iterator it;
            int cpt = 0;
            vector<int> tab(lposElements.size(), 0);
            for (it = lposElements.begin(); it != lposElements.end(); ++it)
            {
                tab[cpt] = *it;
                cpt++;
                cout << tab << endl;
            }
            sagent1 = sforce.substr(0, tab[0]);
            cout << "sagent1 : " << sagent1 << endl;

            sobject = sforce.substr(tab[0] + 1, tab[1] - (tab[0] + 1));
            cout << "sobject : " << sobject << endl;

            slocation = sforce.substr(tab[1] + 1, sforce.size() - tab[1] + 1);
            cout << "slocation : " << slocation << endl;
            cout << "tab : " << tab[0] << " " << tab[1] << endl;

            if (slocation == "left" || slocation == "right"){
                sanswer = sagent1 + " " + sverb + "ed the " + sobject + " to the " + slocation;
            }
            else
                sanswer = sagent1 + " " + sverb + "ed the " + sobject + " " + slocation;
            iCub->say(sanswer, false);
            cout << sanswer << endl;
        }
        else if (nb == 3)   //  "Ag1,object,location,adverb"
        {
            cout << "nb elements " << nb << endl;
            std::list<int>::iterator it;
            int cpt = 0;
            vector<int> tab(lposElements.size(), 0);
            for (it = lposElements.begin(); it != lposElements.end(); ++it)
            {
                tab[cpt] = *it;
                cpt++;
                cout << tab << endl;
            }
            sagent1 = sforce.substr(0, tab[0]);
            cout << "sagent1 : " << sagent1 << endl;

            sobject = sforce.substr(tab[0] + 1, tab[1] - (tab[0] + 1));
            cout << "sobject : " << sobject << endl;

            slocation = sforce.substr(tab[1] + 1, tab[2] - tab[1] + 1);
            cout << "slocation : " << slocation << endl;

            sadverb == sforce.substr(tab[2] + 1, sforce.size() - tab[2] + 1);
            cout << "sadverb : " << sadverb << endl;
            cout << "tab : " << tab[0] << " " << tab[1] << " " << tab[2] << endl;

            sanswer = sagent1 + " " + sverb + "ed the " + sobject + " to the " + slocation + " " + sadverb;

            iCub->say(sanswer, false);
            cout << sanswer << endl;
        }

    }
    else if (iquestion == 3)  // "how did that happen" => passive form
    {
        int i = sforce.find("(");
        int iend = sforce.size();
        string sverb = sforce.substr(0, i); // => "result"
        cout << "sverb : " << sverb << endl;

        sforce = sforce.substr(i + 1, iend - (i + 1) - 1); // =>  "Ag2,object"
        cout << "sforce : " << sforce << endl;

        std::list<int> lposElements = nbCaracters(sforce);
        int nb = lposElements.size();

        if (nb == 0){            //  "object"
            cout << "nb elements " << nb << endl;
            string object = sforce;
            sanswer = "The " + sobject + " has been " + sverb;
            iCub->say(sanswer, false);
        }

        else if (nb == 1)   //  "Ag2,object"
        {
            cout << "nb elements " << nb << endl;
            std::list<int>::iterator it;
            int cpt = 0;

            vector<int> tab(lposElements.size(), 0);
            for (it = lposElements.begin(); it != lposElements.end(); ++it)
            {
                tab[cpt] = *it;
                cpt++;
            }
            sagent1 = sforce.substr(0, tab[0]);
            cout << "sagent1 : " << sagent1 << endl;

            sobject = sforce.substr(tab[0] + 1, sforce.size() - tab[0] + 1);
            cout << "sobject : " << sobject << endl;
            cout << "tab : " << tab[0] << " " << tab[1] << endl;

            sanswer = "The " + sobject + " has been " + sverb + " by me";
            iCub->say(sanswer, false);
            cout << sanswer << endl;
        }
        else if (nb == 2)   //  "Ag2,object,location"
        {
            cout << "nb elements " << nb;
            std::list<int>::iterator it;
            int cpt = 0;
            vector<int> tab(lposElements.size(), 0);
            for (it = lposElements.begin(); it != lposElements.end(); ++it)
            {
                tab[cpt] = *it;
                cpt++;
            }
            sagent1 = sforce.substr(0, tab[0]);
            cout << "sagent1 : " << sagent1 << endl;

            sobject = sforce.substr(tab[0] + 1, tab[1] - (tab[0] + 1));
            cout << "sobject : " << sobject << endl;

            slocation = sforce.substr(tab[1] + 1, sforce.size() - tab[1] + 1);
            cout << "sobject : " << slocation << endl;

            cout << "tab : " << tab[0] << " " << tab[1] << endl;

            if (slocation == "left" || slocation == "right"){
                sanswer = "The " + sobject + " has been " + sverb + "ed by me to the " + slocation;
            }
            else
                sanswer = "The " + sobject + " has been " + sverb + "ed by me";

            cout << "sanswer : " << endl;
            iCub->say(sanswer, false);
            cout << sanswer << endl;
        }
        else if (nb == 3)   //  "Ag2,object,location,adverb"
        {
            cout << "nb elements " << nb;
            std::list<int>::iterator it;
            int cpt = 0;
            vector<int> tab(lposElements.size(), 0);
            for (it = lposElements.begin(); it != lposElements.end(); ++it)
            {
                tab[cpt] = *it;
                cpt++;
            }
            sagent1 = sforce.substr(0, tab[0]);
            cout << "sagent1 : " << sagent1 << endl;

            sobject = sforce.substr(tab[0] + 1, tab[1] - (tab[0] + 1));
            cout << "sobject : " << sobject << endl;

            slocation = sforce.substr(tab[1] + 1, tab[2] - tab[1] + 1);
            cout << "sobject : " << slocation << endl;

            sadverb == sforce.substr(tab[2] + 1, sforce.size() - tab[2] + 1);
            cout << "sadverb : " << sadverb << endl;
            cout << "tab : " << tab[0] << " " << tab[1] << " " << tab[2] << endl;

            sanswer = "The " + sobject + " has been " + sverb + "ed by me to the " + slocation + " " + sadverb;

            cout << "sanswer : " << endl;
            iCub->say(sanswer, false);
            cout << sanswer << endl;
        }
    }

    cout << "##############################################################" << endl;
    return languageNodeInteraction();
}

int reservoirHandler::languageNodeInteraction()
{
    sCurrentNode = "languageNodeInteraction";
    sCurrentGrammarFile = nameGrammarNodeInteraction;
    ostringstream osError;          // Error message
    osError << "Error in  LanguageActionAnalysis | " << sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic; // semantic information of the content of the recognition

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());


    while (!fGetaReply)
    {
        bSpeechRecognized.clear();
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        //e.g. : Reply from Speech Recog : 1 ("I want you to produce language" (INFORMATION (type produce))) ACK
        cout << "In " << sCurrentNode << " Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        if (bSpeechRecognized.get(1).toString() == "UNKNOWN")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)
        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    if (bAnswer.get(0).asString() == "stop the interaction")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        //return bOutput;
    }

    bAnswer = *bSpeechRecognized.get(1).asList();

    if (bAnswer.get(1).asList()->get(0).asString() == "answer")
    {
        if (bAnswer.get(1).asList()->get(1).asString() == "positive"){
            return languageNodeInteraction();
        }
        else if (bAnswer.get(1).asList()->get(1).asString() == "negative"){
            return nodeTestAP();
        }
    }

    if (bAnswer.get(1).asList()->get(0).toString() == "continue")
    {
        return mainNodeInteraction();
    }

    if (bAnswer.get(0).asString() == "what happened")
    {
        iquestion = 1;
        return grammarNodeInteraction();
    }
    else if (bAnswer.get(1).asList()->get(0).asString() == "whoagent")
    {
        cout << "bAnswer.get(0).asString() : " << bAnswer.get(1).asList()->get(0).asString() << endl;
        cout << "here    " << bAnswer.get(1).asString() << endl;
        iquestion = 2;
        bSemantic = *bAnswer.get(1).asList()->get(1).asList();
        sagent = bSemantic.check("agent", Value("none")).asString();
        return grammarNodeInteraction();
    }
    else if (bAnswer.get(0).asString() == "how did that happen")
    {
        iquestion = 3;
        return grammarNodeInteraction();
    }


    //else if (bAnswer.get(0).asString() == "no")
    //{
    //    iCub->say("go ahead");
    //}
    return languageNodeInteraction();
}

bool reservoirHandler::languageNodeInteractionSD()
{
    sCurrentNode = "languageNodeInteractionSD";
    sCurrentGrammarFile = nameGrammarNodeInteraction;
    ostringstream osError;          // Error message
    osError << "Error in  LanguageActionAnalysis | " << sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;


    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic; // semantic information of the content of the recognition

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    cout << endl << "In " << sCurrentNode << " 2 " << endl << endl;


    while (!fGetaReply)
    {
        bSpeechRecognized.clear();
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);



        for (int kk = 0; kk < bSpeechRecognized.size(); kk++)
        {
            cout << "element " << kk << " " << bSpeechRecognized.get(kk).toString() << endl;
        }



        //e.g. : Reply from Speech Recog : 1 ("I want you to produce language" (INFORMATION (type produce))) ACK
        cout << "In " << sCurrentNode << " Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        if (bSpeechRecognized.get(1).toString() == "UNKNOWN")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)
        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    if (bAnswer.get(0).asString() == "stop the interaction")
    {
        iCurrentInstance = -1;
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return false;
    }

    bAnswer = *bSpeechRecognized.get(1).asList();


    if (bAnswer.get(1).asList()->get(0).asString() == "answer")
    {
        //(yes (answer (positive yes)))
        cout << bAnswer.get(1).asList()->get(0).asString() << endl;
        cout << bAnswer.get(1).asList()->get(1).asString() << endl;

        Bottle bans = *bAnswer.get(1).asList()->get(1).asList();
        string pos = bans.check("positive", Value("none")).asString();

        if (pos == "yes"){
            return languageNodeInteractionSD();
        }
        else {
            cout << bAnswer.get(1).asList()->get(1).asString();
            cout << "I am here in negative way" << endl;
            sobjectFocusChanged = "";
            iCub->say("Do you want me to focus the description about object or location ?", false);
            return nodeModality();
        }
    }


    if (bAnswer.get(1).asList()->get(0).toString() == "happenedTo")
    {
        bSemantic = *bAnswer.get(1).asList()->get(1).asList();
        sobjectFocusChanged = bSemantic.check("object", Value("none")).asString();

        return spatialRelation();
    }

    return languageNodeInteractionSD();
}

bool reservoirHandler::launchSpatialRelation(){
    iCub->say("Do you have any Questions");
    cout << "Do you have any Questions" << endl;
    return languageNodeInteractionSD();
}

bool reservoirHandler::spatialRelation()
{
    iCub->opc->update();
    std::list<Entity*> PresentObjects = iCub->opc->EntitiesCache();
    std::vector<RTObject> PresentRtoBefore;

    for (std::list<Entity*>::iterator itE = PresentObjects.begin(); itE != PresentObjects.end(); itE++)
    {
        if ((*itE)->isType(EFAA_OPC_ENTITY_RTOBJECT))
        {
            RTObject rto;
            rto.fromBottle((*itE)->asBottle());
            if (rto.m_present == 1.0)
                PresentRtoBefore.push_back(rto);
        }
    }

    if (PresentObjects.size() < 2 && PresentObjects.size() > 3)
    {
        iCub->say("Dude, I was expecting 2 or 3 objects... Start again !", false);
        return nodeTestSD();
    }

    //get the focus object
    //string sObjectFocus = "circle";
    double maxSalience = 0.0;
    string sObjectFocus = "none";
    if (sobjectFocusChanged.empty())
    {
        //double maxSalience = 0.4;
        for (std::vector<RTObject>::iterator itRTO = PresentRtoBefore.begin(); itRTO != PresentRtoBefore.end(); itRTO++)
        {
            if (itRTO->m_saliency > maxSalience)
            {
                maxSalience = itRTO->m_saliency;
                sObjectFocus = itRTO->name();
            }
        }

        if (maxSalience == 0.)
        {
            iCub->say("I think I didn't get your focus object", false);
            return nodeTestSD();
        }

        string sSentence = "Ok, so you decided to focus on " + sObjectFocus;
        iCub->say(sSentence, false);
    }
    else{
        sObjectFocus = sobjectFocusChanged;
    }

    if (sSentence_type.size() == 0)
    {
        sSentence_type = " :C";
    }

    if (PresentRtoBefore.size() == 2)
    {
        int iFactor;
        (PresentRtoBefore[0].name() == sObjectFocus) ? iFactor = 1 : iFactor = -1;
        //double deltaX = iFactor*(PresentRtoBefore[1].m_ego_position[0] - PresentRtoBefore[0].m_ego_position[0]);
        double deltaY = iFactor*(PresentRtoBefore[1].m_ego_position[1] - PresentRtoBefore[0].m_ego_position[1]);

        string sLocation;
        (deltaY > 0) ? sLocation = "right" : sLocation = "left";
        string sRelative;
        (iFactor == 1) ? sRelative = (PresentRtoBefore[1].name()) : sRelative = (PresentRtoBefore[0].name());

        cout << "I understood :" << endl << sObjectFocus << "\t" << sLocation << "\t" << sRelative << endl;
        sdataTestSD = sLocation + " " + sObjectFocus + " " + sRelative + sSentence_type;

    }
    else    // case of 3 objects
    {
        RTObject rtFocus,
            rtRelative1,
            rtRelative2;
        bool bFirstRelative = true;
        for (unsigned int i = 0; i < 3; i++)
        {
            if (PresentRtoBefore[i].name() != sObjectFocus)
            {
                bFirstRelative ? rtRelative1 = PresentRtoBefore[i] : rtRelative2 = PresentRtoBefore[i];
                bFirstRelative = false;
            }
            else
            {
                rtFocus = PresentRtoBefore[i];
            }
        }

        iCub->say("Thinking of the situation", false);
        double deltaX1; // difference btw focus and relative1
        double deltaX2; // difference btw focus and relative2

        deltaX1 = rtRelative1.m_ego_position[1] - rtFocus.m_ego_position[1];
        deltaX2 = rtRelative2.m_ego_position[1] - rtFocus.m_ego_position[1];

        string sLocation1;
        string sLocation2;

        string sRelative1 = rtRelative1.name();
        string sRelative2 = rtRelative2.name();

        (deltaX1 > 0) ? sLocation1 = "right" : sLocation1 = "left";
        (deltaX2 > 0) ? sLocation2 = "right" : sLocation2 = "left";

        cout << "I understood : " << sLocation1 << "\t" << sObjectFocus << "\t" << sRelative1 << endl;
        cout << "and          : " << sLocation2 << "\t" << sObjectFocus << "\t" << sRelative2 << endl;

        sdataTestSD = sLocation1 + " " + sObjectFocus + " " + sRelative1 + ", " + sLocation2 + " " + sObjectFocus + " " + sRelative2 + sSentence_type;

    }

    // TO SEND TO XAVIER
    copyPastFile(fileXavierTrain.c_str(), fileSRinputM.c_str());
    cout << fileXavierTrain << endl;
    cout << fileSRinputM << endl;
    createTestwithTrainData(fileSRinputM.c_str(), sdataTestSD);

    callReservoir(fileSD);
    string result = openResult(fileSRoutputS.c_str());

    int size = sObjectFocus.size() + 8;
    if (sSentence_type == " :C"){
        string str = result.substr(size + 1, svector.size() - (size + 1));
        sConstrualLocation = str;
    }
    else{
        string str = result.substr(0, svector.size() - size);
        sConstrualLocation = str;
    }

    iCub->say(result, false);
    cout << "iCub says : 'I have understood  '" << result << endl;

    return launchSpatialRelation();
}

bool reservoirHandler::nodeYesNoInteraction()
{
    sCurrentNode = "nodeYesNoInteraction";
    sCurrentGrammarFile = nameGrammarYesNo;
    ostringstream osError;          // Error message
    osError << "Error in reservoirHandler | " << sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle  bMessenger,
        bSpeechRecognized,
        bAnswer;

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say

    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        cout << "In " << sCurrentNode << " Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }

        if (bSpeechRecognized.get(1).toString() == "UNKNOWN")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    if (bAnswer.get(0).asString() == "yes"){
        fileVectorAPRead.open(fvector.c_str());
        return mainNodeInteraction();
    }
    else if (bAnswer.get(0).asString() == "no"){
        fileVectorAPRead.close();
        return nodeTestAP();
    }
    return nodeYesNoInteraction();
}

bool reservoirHandler::nodeYesNo()
{
    sCurrentNode = "nodeYesNo";
    sCurrentGrammarFile = nameGrammarYesNo;
    ostringstream osError;          // Error message
    osError << "Error in reservoirHandler | " << sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle  bMessenger,
        bSpeechRecognized,
        bAnswer;

    bMessenger.addString("recog");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say

    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger, bSpeechRecognized);

        cout << "In " << sCurrentNode << " Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 2)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }

        if (bSpeechRecognized.get(1).toString() == "UNKNOWN")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            return false;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    return (bAnswer.get(0).asString() == "yes");

}

bool reservoirHandler::createVectorFile(string sVectorFile)
{
    cout << "createVectorFile :  " << sVectorFile << endl;
    fileVectorAP << sVectorFile << endl;

    return true;
}

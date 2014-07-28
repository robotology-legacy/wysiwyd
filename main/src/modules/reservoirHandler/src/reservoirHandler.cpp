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

#include <reservoirHandler.h>

reservoirHandler::reservoirHandler(ResourceFinder &rf)
{
    iCurrentInstance = -1;
}

reservoirHandler::~reservoirHandler()
{
    close();
}

/*
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the
 * equivalent of the "open" method.
 */

bool reservoirHandler::configure(ResourceFinder &rf) {

    bool	bEveryThingisGood = true;
    bool	bOptionnalModule  = true;
    moduleName            = rf.check("name",
                                     Value("reservoirHandler"),
                                     "module name (string)").asString();

    sKeyWord            = rf.check("keyword", Value("grammar")).toString().c_str();

    nameGrammarNodeType	 = rf.getContextPath().c_str();
    nameGrammarNodeType	+= rf.check("nameGrammarNodeType",  Value("/nameGrammarNodeType.xml")).toString().c_str();
    nameGrammarNodeModality	 = rf.getContextPath().c_str();
    nameGrammarNodeModality	+= rf.check("nameGrammarNodeModality",  Value("/nameGrammarNodeModality.xml")).toString().c_str();
    nameGrammarNodeTrainAP	 = rf.getContextPath().c_str();
    nameGrammarNodeTrainAP	+= rf.check("nameGrammarNodeTrainAP",  Value("/nameGrammarNodeTrainAP.xml")).toString().c_str();
    nameGrammarNodeTestAP	 = rf.getContextPath().c_str();
    nameGrammarNodeTestAP	+= rf.check("nameGrammarNodeTestAP",  Value("/nameGrammarNodeTestAP.xml")).toString().c_str();
    nameGrammarNodeTrainSD	 = rf.getContextPath().c_str();
    nameGrammarNodeTrainSD	+= rf.check("nameGrammarNodeTrainSD",  Value("/nameGrammarNodeTrainSD.xml")).toString().c_str();

    pythonPath 	= rf.check("pythonPath",  Value("/mnt/data/Data/BuildLinux/Robot/RAD/src/iCub_language/")).toString().c_str();

    /* Mode Action Performer => Meaning*/
    fileAPimputS	 = rf.getContextPath().c_str();
    fileAPimputS	+= rf.check("APimputS",  Value("/AP_input_S.txt")).toString().c_str();
    fileAPoutputM 	 = rf.getContextPath().c_str();
    fileAPoutputM 	+= rf.check("APoutputM",  Value("/AP_output_M.txt")).toString().c_str();
    fileXavierTrainAP 	 = rf.getContextPath().c_str();
    fileXavierTrainAP 	+= rf.check("xavierTrainAP",  Value("/xavier_trainAP.txt")).toString().c_str();

    fileAP      = rf.check("fileAP",  Value("action_performer.py")).toString().c_str();

    /* Mode Scene Describer => Produce*/
    fileSRinputM 	 = rf.getContextPath().c_str();
    fileSRinputM 	+= rf.check("SRinputM.txt",  Value("/SR_input_M.txt")).toString().c_str();
    fileSRoutputS 	 = rf.getContextPath().c_str();
    fileSRoutputS 	+= rf.check("SRoutputS",  Value("/SR_output_S.txt")).toString().c_str();
    fileXavierTrain 	 = rf.getContextPath().c_str();
    fileXavierTrain 	+= rf.check("xavierTrain",  Value("/xavier_train.txt")).toString().c_str();

    fileSD      = rf.check("fileSD",  Value("spatial_relation.py")).toString().c_str();
    /*
    * before continuing, set the module name before getting any other parameters,
    * specifically the port names which are dependent on the module name
    */

    setName(moduleName.c_str());

    // Open handler port
    string sName = getName();
    handlerPortName = "/"+ sName + "/rpc";

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        bEveryThingisGood = false;
    }


    // Open port2speech
    port2SpeechRecogName = "/"+ sName + "/toSpeechRecog";

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
    //		iCub Client
    //------------------------//

    string ttsSystem = SUBSYSTEM_SPEECH;
    iCub = new ICubClient(moduleName.c_str(),"reservoirHandler","client.ini",true);
    //iCub->connect();
    iCub->say("Hi!",false);

    // Connect iCub Client, and ports
    bOptionnalModule &= Network::connect(port2SpeechRecogName.c_str(), "/speechRecognizer/rpc");
    bOptionnalModule &= Network::connect("/mainLoop/speechGrammar/keyword:o", handlerPortName.c_str());
    bOptionnalModule &= Network::connect(port2iSpeakName, "/iSpeak");

    if (!bOptionnalModule)
    {
        cout << endl << "Some dependencies are notyarp connect /icubSim/cam/left /view/left running (ICubClient or port(s) connections)"<<endl<<endl;
    }

    if (!bEveryThingisGood || !bOptionnalModule)
        cout << endl << "Some dependencies are not running (ICubClient or port(s) connections)"<<endl<<endl;
    else
        cout << endl << endl << "----------------------------------------------" << endl << endl << "reservoirHandler ready !" << endl << endl;

    nodeType();

    return bEveryThingisGood ;
}

bool reservoirHandler::interruptModule(){
    handlerPort.interrupt();
    Port2SpeechRecog.interrupt();
    Port2iSpeak.interrupt();
    return true;
}

bool reservoirHandler::close() {
    handlerPort.close();
    Port2SpeechRecog.close();
    iCub->close();
    Port2iSpeak.close();
    delete iCub;

    return true;
}

bool reservoirHandler::respond(const Bottle& command, Bottle& reply) {
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
    else if (command.get(0).asString()==sKeyWord.c_str()) {
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
*	Get the context path of a .grxml grammar, and return it as a string
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
    while( getline(isGrammar, sLine) )
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

bool reservoirHandler::callReservoir(string fPython)
{

    //launch Xavier reservoir
    string command = "cd " + pythonPath + " && python " + fPython;
    //bool bsys = system(command.c_str());
    //cout << "cd " << pythonPath << " && python " << fPython << endl;
    //return bsys;


    FILE* file;
    int argc;
    char * argv[3];

    argc = 3;
    argv[0] = "/mnt/data/Data/BuildLinux/Robot/RAD/src/iCub_language/action_performer.py";
    argv[1] = "/home/anne/.local/share/yarp/contexts/reservoirHandler/AP_input_S.txt";
    argv[2] = "/home/anne/.local/share/yarp/contexts/reservoirHandler/AP_output_M.txt";

    Py_SetProgramName(argv[0]);
    Py_Initialize();
    PySys_SetArgv(argc, argv);
    file = fopen("/mnt/data/Data/BuildLinux/Robot/RAD/src/iCub_language/action_performer.py","r");

    PyRun_SimpleFile(file, "/mnt/data/Data/BuildLinux/Robot/RAD/src/iCub_language/action_performer");

    //return Py_Finalize();
    return true;
}

/* Node 1: general question
*	produce or understand
*/
bool reservoirHandler::nodeType()
{
    sCurrentNode = "nodeType";
    sCurrentGrammarFile = nameGrammarNodeType;
    ostringstream osError;			// Error message
    osError << "Error in reservoirHandler | "<< sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
            bMessenger, //to be send TO speech recog
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic, // semantic information of the content of the recognition
            bSendReasoning,	// send the information of recall to the abmReasoning
            bSpeak,	// bottle for tts
            bTemp;

    bMessenger.addString("recogBottle");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say
    cout << "ICUB SAY : Do you want me to understand or produce language? " << endl;


    while (!fGetaReply)
    {
        bSpeechRecognized.clear();
        Port2SpeechRecog.write(bMessenger,bSpeechRecognized);

        //e.g. : Reply from Speech Recog : 1 ("I want you to produce language" (INFORMATION (type produce))) ACK
        cout << "Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 3)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
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
        //return bOutput;
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

            cout << "iCub says : 'Set the objects'" << endl ;
            cout << "iCub says : 'Told a sentence to execute. Or go in train mode'" << endl ;

            nodeModality();
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

            // Module "move_and_tell"
            // ATTENTION_SELECTOR_track_agent_or_object "Reactable"
            // ATTENTION_SELECTOR_track_agent_or_object "You"
            cout << "iCub says : 'Let me see...'" << endl ;
            cout << "iCub says : 'I see all the objects...'" << endl ;

            // ATTENTION_SELECTOR_track_agent_or_object "You"
            cout << "iCub says : 'Told a sentence to execute. Or go in train mode....'" << endl ;

            nodeModality();
        }

        else
            return nodeType();
    }

    return true;
}

/*	Node 2 : details about number and agent
*		possibility to go back to node 1
*		repeat and stop allowed
*/

bool reservoirHandler::nodeModality()
{
    sCurrentNode = "nodeModality";
    sCurrentGrammarFile = nameGrammarNodeModality;
    ostringstream osError;			// Error message
    osError << "Error in reservoirHandler | "<< sCurrentNode << " :: ";

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
            bMessenger, //to be send TO speech recog
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic, // semantic information of the content of the recognition
            bSendReasoning,	// send the information of recall to the abmReasoning
            bSpeak,	// bottle for tts
            bTemp;

    bMessenger.addString("recogBottle");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say

    string sSentence = "You want to know more about it ?";
    iCub->say(sSentence);

    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger,bSpeechRecognized);

        cout << "Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 3)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    bSemantic = *bAnswer.get(1).asList()->get(1).asList();
    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();


    if (sQuestionKind == "CANONICAL")
    {
        sCurrentCanonical = bSemantic.check("focus", Value("none")).asString();

        if (sCurrentCanonical == "object")
        {
            cout << "iCub says : 'Fine. Let's focus about object'" << endl ;
            sSentence_type = " :C";
            cout << "IN "<<  sCurrentType << " mode" << endl;
            cout << "IN "<<  sCurrentActivity << " mode" << endl;
            cout << "IN "<<  sCurrentCanonical << " mode" << endl;

        }
        else if(sCurrentCanonical == "location")
        {

            cout << "iCub says : 'Oh, tricky! Let's go with locations'" << endl ;
            sSentence_type = " :N";
         }

        cout << "iCub says : 'Thinking of the situation'" << endl ;
        copyTrainData(fileXavierTrain.c_str(),fileSRinputM.c_str());
        cout << "iCub says : 'I have understood'" << endl ;
        nodeType();
    }


    if (sQuestionKind == "INFORMATION")
    {
        sCurrentType = bSemantic.check("modality", Value("none")).asString();


        if (sCurrentActivity == "understand")
        {
            /* Mode Action Performer => Meaning */

            // Module test_mode



            iCub->say("Let me see");
            cout << "iCub says : 'Let me see...'" << endl ;
            Time::delay(1.0);

            iCub->say("I see all the objects");
            Time::delay(1.0);
            cout << "iCub says : 'I see all the objects'" << endl ;

            iCub->say("Told a sentence to execute or go in train mode.");
            cout << "iCub says : 'Told a sentence to execute. Or go in train mode.'" << endl ;
            Time::delay(1.0);

            if (sCurrentType == "train")
            {
                /*
                 * Training
                 * 1. Robot generates random actions [meaning]
                 * 2. Human says a corresponding command [sentence]
                 */
                if (lMeaningsSentences.size()!=0)
                {
                    lMeaningsSentences.clear();
                }
                cout << "iCub says : 'What is this action ?'" << endl;
                return nodeTrainAP();
            }
            else if(sCurrentType == "test")
            {
                /*
                * Testing
                 * 1. Human says a command [sentence]
                 * 2. Robot performs corresponding actions [meaning]
                */

                cout << "go in test mode " << endl;
                sentence = " ";
                return nodeTestAP();
            }
        }
        else if (sCurrentActivity == "produce")
        {
            /* Mode Scene Describer => Produce sentence*/

            if (sCurrentType == "test")
            {
                /*
                 * Testing
                 * 1. Human arranges objects on the table [meaning]
                 * 2. Robot describes the scene [sentence]
                 */

                inbsentence=2;
                cout << "iCub says : 'Do you want me to focus the description about object or location'" << endl ;
                nodeTrainSD();

            }

            else if (sCurrentType == "train")
            {

                    /*
                    * Training
                    * 1. Human arranges objects on the table [meaning]
                    * 2. Human describes the scene [sentence]
                    */
                    if (lMeaningsSentences.size()!=0)
                    {
                        lMeaningsSentences.clear();
                    }

                    inbsentence=1;
                    nodeTrainSD();

             }
            //ATTENTION_SELECTOR_track $OBJ_FOCUS
            cout << "iCub says : 'The focus object is $OBJ_FOCUS'" << endl ;

            //ATTENTION_SELECTOR_track_agent_or_object "You"

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
        //return bOutput;
    }

    if (bAnswer.get(0).asString() == "return")
    {
        nodeModality();
    }

    return true;
}


bool reservoirHandler::nodeTrainAP()
{
    sCurrentNode = "nodeModality";
    sCurrentGrammarFile = nameGrammarNodeModality;
    ostringstream osError;			// Error message
    osError << "Error in reservoirHandler | "<< sCurrentNode << " :: ";

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
            bMessenger, //to be send TO speech recog
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic, // semantic information of the content of the recognition
            bSendReasoning,	// send the information of recall to the abmReasoning
            bSpeak,	// bottle for tts
            bTemp;

    bMessenger.addString("recogBottle");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say

    string sSentence = "You want to know more about it ?";
    iCub->say(sSentence);

    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger,bSpeechRecognized);

        cout << "Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 3)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
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

        cout << "Human says :  " << bAnswer.get(0).asString() << endl ;
        cout << "Human says :  'Do you want continue or exit'" << endl ;
        iCub->say("Do you want continue or exit");
        sSentence = bAnswer.get(0).asString();
        cout << "sSentence" << sSentence << endl;
        nodeTrainAP();
      }


      else if (sQuestionKind == "follow")
      {
          string continueExit = bSemantic.check("mode", Value("none")).asString();


          if (continueExit == "continue")
          {
            cout << "lMeaningsSentences " <<  endl;
            lMeaningsSentences.push_back(sSentence);
            nodeTrainAP();

          }
          else if (continueExit == "exit")
          {
            trainSaveMeaningSentence(fileAPimputS.c_str());
            nodeType();
          }
     }

    return true;
}

bool reservoirHandler::nodeTestAP()
{
    sCurrentNode = "nodeTestAP";
    sCurrentGrammarFile = nameGrammarNodeTestAP;
    ostringstream osError;			// Error message
    osError << "Error in reservoirHandler | "<< sCurrentNode << " :: ";

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
            bMessenger, //to be send TO speech recog
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic, // semantic information of the content of the recognition
            bSendReasoning,	// send the information of recall to the abmReasoning
            bSpeak,	// bottle for tts
            bTemp;

    bMessenger.addString("recogBottle");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say

    //string sSentence = "You want to know more about it ?";
    //iCub->say(sSentence);

    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger,bSpeechRecognized);

        cout << "Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 3)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    bSemantic = *bAnswer.get(1).asList()->get(1).asList();
    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    // Do you know any ...
    if (sQuestionKind == "sentence")
    {
        /*
         * bAnswer.get(0).asString() => the circle is to the left of of the cross
         * bAnswer.toString() =>
         * "the circle is to the left of of the cross" (sentence (sentence1 ((object "the circle") (relative_complete ((spatial_relative ((relative to) (spatial "the left"))) (object "the cross"))))))
         */

        cout << "iCub says : 'I have understood      '" << bAnswer.get(0).asString() << endl ;
        sentence += bAnswer.get(0).asString() + " ";
        cout << "iCub says : 'Is it ok ? ... 'No or Yes" << endl;

        cout << "sentence : " << sentence << endl;
        nodeTestAP();
      }


    else if (sQuestionKind == "yesno")
    {
        string yesNo = bSemantic.check("agree", Value("none")).asString();


        if (yesNo == "yes")
        {
          cout << "___________________________________________" << endl;
          copyPastFile(fileXavierTrainAP.c_str(), fileAPimputS.c_str());
          cout << fileXavierTrainAP << endl;
          cout << fileAPimputS << endl;
          createTestwithTrainData(fileAPimputS.c_str(), sentence);
          callReservoir(pythonPath + fileAPimputS);

          string result = openResult(fileAPoutputM.c_str());
          iCub->say(result);
          cout << result << endl;

          int id = result.find(",");
          int idf = result.size()-id;

          if (id != 0){
              cout << result.substr(0, id) << endl;
              cout << result.substr(id +1,idf) << endl;
          }

          cout <<  "iCub do the action..." << endl;

          nodeTestAP();

        }
        else if (yesNo == "no")
        {
          sentence = " ";
          nodeTestAP();
        }
      }

      else if (sQuestionKind == "follow")
      {
          string continueExit = bSemantic.check("mode", Value("none")).asString();


          if (continueExit == "continue")
          {
            cout << "Humain say a command ...." << endl;
            sentence = " ";
            nodeTestAP();

          }
          else if (continueExit == "exit")
          {
            nodeType();
          }
     }

     return true;
}



bool reservoirHandler::nodeTrainSD()
{
    sCurrentNode = "nodeTrainSD";
    sCurrentGrammarFile = nameGrammarNodeTrainSD;
    ostringstream osError;			// Error message
    osError << "Error in reservoirHandler | "<< sCurrentNode << " :: ";
    cout << endl << "In " << sCurrentNode << endl << endl;

    Bottle bOutput;

    bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
            bMessenger, //to be send TO speech recog
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic, // semantic information of the content of the recognition
            bSendReasoning,	// send the information of recall to the abmReasoning
            bSpeak,	// bottle for tts
            bTemp;

    bMessenger.addString("recogBottle");
    bMessenger.addString("grammarXML");
    bMessenger.addString(grammarToString(sCurrentGrammarFile).c_str());

    //to be replace by Say

    string sSentence = "You want to know more about it ?";
    iCub->say(sSentence);

    while (!fGetaReply)
    {
        Port2SpeechRecog.write(bMessenger,bSpeechRecognized);

        cout << "Reply from Speech Recog : " << bSpeechRecognized.toString() << endl;

        if (bSpeechRecognized.toString() == "NACK" || bSpeechRecognized.size() != 3)
        {
            osError << "Check " << sCurrentGrammarFile;
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }

        if (bSpeechRecognized.get(0).toString() == "0")
        {
            osError << "Grammar not recognized";
            bOutput.addString(osError.str());
            cout << osError.str() << endl;
            //return bOutput;
        }


        bAnswer = *bSpeechRecognized.get(1).asList();

        if (bAnswer.toString() != "" && !bAnswer.isNull())
        {
            fGetaReply = true;
        }
    }

    bSemantic = *bAnswer.get(1).asList()->get(1).asList();
    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    if(sCurrentType == "train")
    {
            // Do you know any ...
            if (sQuestionKind == "sentence")
            {
                cout << "Human :  'Let me see, I have finished...'" << endl;
                cout << "iCub says : 'What is the corresponding sentence      '" << bAnswer.get(0).asString() << "    " << bAnswer.toString() << endl ;
                /*
                 * bAnswer.get(0).asString() => the circle is to the left of of the cross
                 * bAnswer.toString() =>
                 * "the circle is to the left of of the cross" (sentence (sentence1 ((object "the circle") (relative_complete ((spatial_relative ((relative to) (spatial "the left"))) (object "the cross"))))))
                 */
                cout << "iCub says : 'I have recognized      '" << bAnswer.get(0).asString() << endl ;
                cout << "iCub says : 'Is it ok ? ... 'continue or exit" << endl;
                string sSentence = bAnswer.get(0).asString();
                nodeTrainSD();
              }

              else if (sQuestionKind == "follow")
              {
                  string continueExit = bSemantic.check("mode", Value("none")).asString();


                  if (continueExit == "continue")
                  {
                    cout << "lMeaningsSentences " <<  endl;
                    lMeaningsSentences.push_back(sSentence+ sSentence_type);
                    nodeTrainSD();

                  }
                  else if (continueExit == "exit")
                  {
                    trainSaveMeaningSentence(fileSRinputM.c_str());
                    nodeType();
                  }
             }

    }

    else if (sCurrentType == "test")
    {
        cout << "iCub says : 'Thinking of the situation'" << endl ;
        copyTrainData(fileXavierTrain.c_str(),fileSRinputM.c_str());
        callReservoir(pythonPath + fileSD);
        cout << "iCub says : 'I have understood'" << endl ;
        nodeType();
    }


    if (bAnswer.get(0).asString() == "return")
    {
        nodeModality();
    }

    return true;
 }



/*
 * test mode : copy the train data in test file, add the end of the train data (if first test) and add the test sentence
 */
int reservoirHandler::copyTrainData(const char* fileNameIn, const char* fileNameOut)
{
    // Module Save
    ofstream file;
    if(inbsentence==1){
        file.open(fileNameOut, ios::out | ios::trunc); // open the file in writing and erase the file if it is not empty
        file << "<train data>"<< endl;
        inbsentence+=1;
        file.close();
    }
    if (sCurrentType == "train"){
        cout << "iCub says : 'Ok'" << endl;
        trainSaveMeaningSentence(fileNameOut);
    }
    else if (sCurrentType == "test"){
        cout << "iCub says : 'I will describe the situation'" << endl ;
        inbsentence=1;
        copyPastFile(fileNameIn,fileNameOut);
        createTestwithTrainData(fileNameOut, "left circle cross, left circle eraser :C");
    }
    file.close();
    return true;
}



int reservoirHandler::trainSaveMeaningSentence(const char* filename)
{
    ofstream file;
    file.open(filename, ios::out | ios::trunc);
    std::list<string>::iterator it;

    file << "<train data>" <<endl;
    for(it=lMeaningsSentences.begin(); it!=lMeaningsSentences.end(); ++it)
    {
        string meaning;
        if(sCurrentActivity == "produce")
        {
            meaning = "put trumpet left;put guitar right";
            file << meaning << ";" << *it <<endl;
        }
        else
        {
            meaning = "left violin trumpet";
            file << *it << ";" << meaning <<endl;
        }
    }
    file.close();
    return true;
}


int reservoirHandler::createTestwithTrainData(const char* filename, string sMeaningSentence)
{
    ofstream file;
    file.open (filename, ios::app);
    file << "</train data>" <<endl;
    file << "<test data>" <<endl;
    file << sMeaningSentence <<endl;
    file << "</test data>" <<endl;
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
    while(getline(in,str))
    {
        out<<str<<endl;
    }
    in.close();
    out.close();

    return true;
}

string reservoirHandler::openResult(const char* fileNameIn)
{
    ifstream in;

    in.open(fileNameIn);
    string str;
    getline(in,str);

    string result = str;
    cout << result << endl;
    in.close();

    return result;
}

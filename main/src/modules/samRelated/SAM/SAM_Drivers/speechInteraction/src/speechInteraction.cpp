// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Uriel Martinez, Luke Boorman
* email:   uriel.martinez@sheffield.ac.uk
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* $WYSIWYD_ROOT/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/


#include "speechInteraction.h"
#include "wrdac/subsystems/subSystem_recog.h"

// Helper methods
inline bool caseInsCharCompareN(char a, char b) {
    return(toupper(a) == toupper(b));
}

bool caseInsCompare(const string& s1, const string& s2) {
    return((s1.size() == s2.size()) &&
        equal(s1.begin(), s1.end(), s2.begin(), caseInsCharCompareN));
}

speechInteraction::speechInteraction()
{
    speechType = 0;
}

speechInteraction::~speechInteraction()
{
}

void speechInteraction::triggerBehaviour(int index)
{
    Bottle &outputBottle = triggerBehaviourPort.prepare();
    outputBottle.clear();
    outputBottle.addInt(index+1);
    triggerBehaviourPort.write();	
}

void speechInteraction::sendSpeech(int index)
{
    string outputString = outputVocabs.at(index);
    
    Bottle &outputBottle = outputPort.prepare();
    outputBottle.clear();
    outputBottle.addString(outputString);
    outputPort.write();	
}

bool speechInteraction::matchVocab(string vocab, int *index)
{    
    if( caseInsCompare(vocab, "!SIL") )
        return false;
        
    for( int i = 0; i < nVocabs; i++ )
    {
        if( caseInsCompare(vocab, inputVocabs.at(i).c_str()) )
        {
            *index = i;
            return true;
        }
    }

    return false;
}

bool speechInteraction::updateModule()
{
    //bool fGetaReply = false;
    Bottle bSpeechRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer) ACK/NACK)
        bMessenger, //to be send TO speech recog
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bSpeak, // bottle for tts
        bTemp;

    Bottle bOutput;
    ostringstream osError;     // Error message
    Bottle bRecognized;
    string inputString;

    if( speechType == 0 )
    {
        Bottle *inputBottle = inputPort.read();
        inputString = inputBottle->toString();
    }
    else if( speechType == 1 )
    {
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNamePerson), 20);

        if( bRecognized.get(0).asInt() != 0 )
        {
            bAnswer = *bRecognized.get(1).asList();
            inputString = bAnswer.get(0).asString();
        }
    }

    cout << "RECEIVED TEXT: " << inputString << endl;

    inputString.erase(remove(inputString.begin(), inputString.end(), '\"'), inputString.end());
    
    int index;
    if( matchVocab(inputString, &index) )
    {
        if( index == 15 )
            triggerBehaviour(index);
        else if( index == 17 )
            triggerBehaviour(index);
        else if( index == 19 )  // ID in C++ (-1 from config file = 20 )
            triggerBehaviour(index);
        else
            sendSpeech(index);

    }
    
    return true;
}

bool speechInteraction::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("speechInteraction"), "module name (string)").asString();
    
    setName(moduleName.c_str());

    Property config;
    config.fromConfigFile(rf.findFile("from").c_str());

    Bottle &nGeneral = config.findGroup("number_of_vocabs");
    nVocabs = nGeneral.find("nvocabs").asInt();

    Bottle &speechGeneral = config.findGroup("speech_engine");
    speechType = speechGeneral.find("engine").asInt();
    
    cout << "Engine type: " << speechType << endl;

    Bottle &inputGeneral = config.findGroup("input_vocabs");
    
    string findVocab = "vocab_";
    ostringstream convert;    

    cout << "INPUT VOCABS" << endl;
    for( int i = 0; i < nVocabs; i++ )
    {
        convert << (i+1);
        findVocab = findVocab + convert.str();
        inputVocabs.push_back(inputGeneral.find(findVocab).asString().c_str());
        cout << findVocab << ": " << inputGeneral.find(findVocab).asString().c_str() << endl;
        findVocab = "vocab_";
        convert.str("");
    }


    Bottle &outputGeneral = config.findGroup("output_vocabs");

    cout << "OUTPUT VOCABS" << endl;
    for( int i = 0; i < nVocabs; i++ )
    {
        convert << (i+1);
        findVocab = findVocab + convert.str();
        outputVocabs.push_back(outputGeneral.find(findVocab).asString().c_str());
        cout << findVocab << ": " << outputGeneral.find(findVocab).asString().c_str() << endl;
        findVocab = "vocab_";
        convert.str("");
    }
           

    Bottle &portsGeneral = config.findGroup("ports");
    
    inputPortName = portsGeneral.find("input_port").asString().c_str();
    outputPortName = portsGeneral.find("output_port").asString().c_str();
    triggerBehaviourPortName = portsGeneral.find("behaviour_port").asString().c_str();

	inputOpen = inputPort.open(inputPortName.c_str());
	outputOpen = outputPort.open(outputPortName.c_str());
	behaviourPortOpen = triggerBehaviourPort.open(triggerBehaviourPortName.c_str());

	if(!outputOpen || !inputOpen || !behaviourPortOpen)
	{
		cout << "Could not open ports. Exiting" << endl;
		return false;
	}



    if( speechType == 1 )
    {
        cout << "DEBUG!!!!" << endl;
        //moduleName = rf.check("name", Value("speechInteraction"), "module name (string)").asString();  
        //setName(moduleName.c_str());    

        GrammarAskNamePerson = rf.findFileByName("GrammarAskNamePerson.xml");

        yInfo() << moduleName << " : finding configuration files...";
        cout << "Grammar file: " << GrammarAskNamePerson;

        //Create an iCub Client and check that all dependencies are here before starting
        //bool isRFVerbose = false;
        iCub = new ICubClient(moduleName,"speechInteraction","client.ini",true);
        iCub->opc->isVerbose = false;
        if (!iCub->connect())
        {
            yInfo() << " iCubClient : Some dependencies are not running...";
            Time::delay(1.0);
        }

        rpc.open(("/" + moduleName +"/rpc").c_str());
        attach(rpc);

        if (!iCub->getRecogClient())
        {
            yInfo() << " WARNING SPEECH RECOGNIZER NOT CONNECTED";
        }
    }

    return true;
}

bool speechInteraction::interruptModule()
{
    return true;
}

double speechInteraction::getPeriod()
{
    return 0.1;
}

string speechInteraction::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        string sErrorMessage = " Error in qRM::grammarToString. Couldn't open file : " + sPath;
        sErrorMessage += " .";
        yInfo() << sErrorMessage;
        return sErrorMessage;
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

bool speechInteraction::close()
{
    iCub->close();
    delete iCub;

    return true;
}


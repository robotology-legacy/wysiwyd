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


#include "abmActions.h"


abmActions::abmActions()
{
    speechType = 0;
    m_masterName = "abmActions";
}

abmActions::~abmActions()
{
}

void abmActions::triggerBehaviour(int index)
{
    Bottle &outputBottle = triggerBehaviourPort.prepare();
    outputBottle.clear();
    outputBottle.addInt(index+1);
    triggerBehaviourPort.write();	
}

void abmActions::sendSpeech(int index)
{
    string outputString = outputVocabs.at(index);
    
    Bottle &outputBottle = outputPort.prepare();
    outputBottle.clear();
    outputBottle.addString(outputString);
    outputPort.write();	
}

bool abmActions::matchVocab(string vocab, int *index)
{    
    if( boost::iequals(vocab, "!SIL") )
        return false;
        
    for( int i = 0; i < nVocabs; i++ )
    {
        if( boost::iequals(vocab, inputVocabs.at(i).c_str()) )
        {
            *index = i;
            return true;
        }
    }

    return false;
}

bool abmActions::updateModule()
{
   
//   const bool sw;

/*
    cout << "+++++++++++++++++++++++++ INIT +++++++++++++++++++++++" << endl;

    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>("abmInteraction", "subsystem"));
        SubABM->sendActivity("action", "waving", "action", lArgument, true);
    }

    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>("abmInteraction", "subsystem"));
        SubABM->sendActivity("action", "waving", "action", lArgument, false);
    }

    
    cout << "+++++++++++++++++++++++++ END +++++++++++++++++++++++" << endl;
    Time::delay(2);
*/

    if (ABMconnected)
    {
        list<pair<string, string> > lArgument;
//        lArgument.push_back(pair<string, string>("Human", "agent"));
//        lArgument.push_back(pair<string, string>("Action recognition", "about"));
        iCub->getABMClient()->sendActivity("agent", "Uriel", "name", lArgument, true);
//        iCub->getABMClient()->sendActivity("action", "put down", "action", lArgument, true);
    }
    Time::delay(2);

    return true;
}

bool abmActions::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("abmActions"), "module name (string)").asString();
    
    setName(moduleName.c_str());

    SubABM = new SubSystem_ABM("from_ABM_INTERACTION");
    ABMconnected = (SubABM->Connect());
    std::cout << ((ABMconnected) ? "ABM_INTERACTION connected to ABM" : "ABM_INTERACTION didn't connect to ABM") << std::endl;


    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "abmActions", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= false;
    if (!iCub->connect())
    {
        cout << "iCubClient : Some dpeendencies are not running..." << endl;
        Time::delay(1.0);
    }

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);


    if (!iCub->getABMClient())
    {
        cout << "WARNING ABM NOT CONNECTED" << endl;
    }

    rememberedInstance = rf.check("rememberedInstance", Value(1333)).asInt();
    agentName = rf.check("agentName", Value("Uriel")).asString().c_str();
    //img_provider_port = rf.check("img_provider_port", Value("/icub/camcalib/left/out/kinematic_structure")).asString().c_str();

    return true;
}

bool abmActions::interruptModule()
{
    return true;
}

double abmActions::getPeriod()
{
    return 0.1;
}

string abmActions::grammarToString(string sPath)
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

bool abmActions::close()
{
    iCub->close();
    delete iCub;

    return true;
}


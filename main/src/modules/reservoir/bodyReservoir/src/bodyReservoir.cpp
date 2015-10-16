/*
* Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
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

#include "bodyReservoir.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

bool bodyReservoir::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("bodyReservoir")).asString().c_str();
    setName(moduleName.c_str());

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();
    sAgentName = rf.check("agentName", Value("partner")).asString();

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "bodyReservoir", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    if (!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        Time::delay(1.0);
    }

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    if (!iCub->getRecogClient())
    {
        yWarning() << "WARNING SPEECH RECOGNIZER NOT CONNECTED";
    }
    if (!iCub->getABMClient())
    {
        yWarning() << "WARNING ABM NOT CONNECTED";
    }


    iCub->say("bodyReservoir is ready", false);
    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    return true;
}


bool bodyReservoir::interruptModule() {
    rpcPort.interrupt();

    yInfo() << "--Interrupting the synchronized yarp ports module...";
    return true;
}

bool bodyReservoir::close() {
    iCub->close();
    delete iCub;

    rpcPort.interrupt();
    rpcPort.close();
    return true;
}


bool bodyReservoir::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "'point' + object to point \n" +
        "'objectToDump' + object \n" +
        "'agentName' + object \n" +
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    }
    else if (command.get(0).asString() == "point") {
        if (command.size() == 2) {
            reply.addList() = pointObject(command.get(1).toString());
        }
        else {
            reply.addString("error in pointObject, wrong size of input (2 element: 'point' + object)");
        }
    }
    else if (command.get(0).asString() == "wave") {
        if (command.size() == 2) {
            reply.addList() = waveAtAgent(command.get(1).toString());
        }
        else {
            reply.addString("error in waveAtAgent, wrong size of input (2 element: 'wave' + agent)");
        }
    }
    else if (command.get(0).asString() == "agentName"){
        if (command.size() != 2)
        {
            reply.addString("error in bodyReservoir: Botte 'agentName' misses information (agent name)");
        }
        else
        {
            sAgentName = command.get(1).toString();
            yInfo() << " agent to dump: " + sAgentName;
            reply.addString("agent to dump: " + sAgentName);
        }
    }
    else {
        cout << helpMessage;
        reply.addString(helpMessage);
    }

    rpcPort.reply(reply);

    return true;
}

/* Called periodically every getPeriod() seconds */
bool bodyReservoir::updateModule() {

    return true;
}


Bottle bodyReservoir::pointObject(string sObject)
{

    Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>(sObject);
    string sHand;
    (obj1->m_ego_position[1] < 0) ? sHand = "left" : sHand = "right";
    Bottle bHand(sHand);

    Bottle   bOutput;

    list<pair<string, string> > lArgument;
    lArgument.push_back(pair<string, string>("iCub", "agent"));
    lArgument.push_back(pair<string, string>("point", "predicate"));
    lArgument.push_back(pair<string, string>(sObject, "object"));
    iCub->getABMClient()->sendActivity("action", "point", "action", lArgument, true);

    bool bSuccess = iCub->look(sObject);
    Time::delay(0.5);
    bSuccess &= iCub->point(sObject, bHand, true);

    lArgument.push_back(pair<string, string>((bSuccess ? "success" : "failed"), "status"));
    iCub->getABMClient()->sendActivity("action", "point", "action", lArgument, false);


    bOutput.addString("point");
    bOutput.addString(sObject);
    (bSuccess) ? bOutput.addString("success") : bOutput.addString("failed");

    return bOutput;
}

Bottle bodyReservoir::waveAtAgent(string sAgent)
{

    Agent* obj1 = iCub->opc->addOrRetrieveEntity<Agent>(sAgent);
    string sHand;
    (obj1->m_ego_position[1] < 0) ? sHand = "left" : sHand = "right";

    Bottle  bOutput;


    list<pair<string, string> > lArgument;
    lArgument.push_back(pair<string, string>("iCub", "agent"));
    lArgument.push_back(pair<string, string>("wave", "predicate"));
    lArgument.push_back(pair<string, string>(sAgent, "object"));
    iCub->getABMClient()->sendActivity("action", "wave", "action", lArgument, true);


    bool bSuccess = iCub->look(sAgent);
    Time::delay(0.5);
    bSuccess &= iCub->getARE()->waving(true);

    lArgument.push_back(pair<string, string>((bSuccess ? "success" : "failed"), "status"));
    iCub->getABMClient()->sendActivity("action", "point", "action", lArgument, false);

    bOutput.addString("waving");
    bOutput.addString(sAgent);
    (bSuccess) ? bOutput.addString("success") : bOutput.addString("failed");

    return bOutput;
}

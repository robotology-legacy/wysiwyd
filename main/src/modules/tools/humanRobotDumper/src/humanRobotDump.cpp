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

#include "humanRobotDump.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

bool humanRobotDump::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("humanRobotDump")).asString().c_str();
    setName(moduleName.c_str());

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();
    sAgentName = rf.check("agentName", Value("partner")).asString();

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "humanRobotDump", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    if (!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        Time::delay(1.0);
    }

    humanDump = false;
    robotDump = false;
    sObjectToDump = "none";
    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    portInfoDumper.open(("/" + moduleName + "/InfoDump").c_str());
    DumperPort.open(("/" + moduleName + "/humanDump").c_str());

    if (!iCub->getRecogClient())
    {
        yWarning() << "WARNING SPEECH RECOGNIZER NOT CONNECTED";
    }
    if (!iCub->getABMClient())
    {
        yWarning() << "WARNING ABM NOT CONNECTED";
    }

    if (!configureSWS(rf))
    {
        yWarning() << " WARNING SWS NOT CONFIGURED";
    }


    iCub->say("humanRobotDump is ready", false);
    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    return true;
}


bool humanRobotDump::interruptModule() {
    rpcPort.interrupt();
    portInfoDumper.interrupt();
    DumperPort.interrupt();

    m_oTriggerPort.interrupt();
    m_oSynchronizedDataPort.interrupt();

    yInfo() << "--Interrupting the synchronized yarp ports module...";

    return true;
}

bool humanRobotDump::close() {
    iCub->close();
    delete iCub;

    if (!closeSWS())
    {
        yWarning() << " WARNING DIDN'T CLOSE SWS";
    }

    rpcPort.interrupt();
    rpcPort.close();
    portInfoDumper.interrupt();
    portInfoDumper.close();
    DumperPort.interrupt();
    DumperPort.close();

    return true;
}


bool humanRobotDump::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "'point' + object to point \n" +
        "'humanDump' + (on/off) \n" +
        "'objectToDump' + object \n" +
        "'agentName' + object \n" +
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    }
    else if (command.get(0).asString() == "humanDump"){
        if (command.size() != 2)
        {
            reply.addString("error in humanRobotDump: Botte 'humanDump' misses information (on/off)");
        }
        else
        {
            if (command.get(1).asString() == "off")
            {
                humanDump = false;
                yInfo() << " stop humanDumping";
                reply.addString("stop humanDumping");
            }
            else if (command.get(1).asString() == "on")
            {
                humanDump = true;
                yInfo() << " start humanDumping";
                reply.addString("start humanDumping");
            }
        }
    }
    else if (command.get(0).asString() == "robotDump"){
        if (command.size() != 2)
        {
            reply.addString("error in humanRobotDump: Botte 'robotDump' misses information (on/off)");
        }
        else
        {
            if (command.get(1).asString() == "off")
            {
                robotDump = false;
                yInfo() << " stop robotDumping";
                reply.addString("stop robotDumping");
            }
            else if (command.get(1).asString() == "on")
            {
                robotDump = true;
                yInfo() << " start robotDumping";
                reply.addString("start robotDumping");
            }
        }
    }
    else if (command.get(0).asString() == "objectToDump"){
        if (command.size() != 2)
        {
            reply.addString("error in humanRobotDump: Botte 'objectToDump' misses information (object name)");
        }
        else
        {
            sObjectToDump = command.get(1).toString();
            yInfo() << " object to dump: " + sObjectToDump;
            reply.addString("object to dump: " + sObjectToDump);
        }
    }
    else if (command.get(0).asString() == "agentName"){
        if (command.size() != 2)
        {
            reply.addString("error in humanRobotDump: Botte 'agentName' misses information (agent name)");
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
bool humanRobotDump::updateModule() {

    if (humanDump)
    {
        DumpHumanObject();
    }
    if (robotDump)
    {
        updateSWS();
    }

    return true;
}


/*
*   Dump in the port DumperPort the human skeleton, and the object of the OPC: sObjectToDump
*
*/
void humanRobotDump::DumpHumanObject()
{
    Agent* ag = iCub->opc->addOrRetrieveEntity<Agent>(sAgentName);

    Bottle bDump;
    bDump.addList() = ag->m_body.asBottle();

    if (sObjectToDump != "none")
    {
        Object* ob = iCub->opc->addOrRetrieveEntity<Object>(sObjectToDump);
        bDump.addList() = ob->asBottle();
    }

    DumperPort.write(bDump);
}
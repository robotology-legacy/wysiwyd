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
#include "wrdac/subsystems/subSystem_ABM.h"
#include "wrdac/subsystems/subSystem_ARE.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

bool bodyReservoir::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("bodyReservoir")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << ": finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();
    sAgentName = rf.check("agentName", Value("partner")).asString();

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "bodyReservoir", "bodyReservoir.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    if (!iCub->connect())
    {
        yInfo() << "iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    Bottle &bRFInfo = rf.findGroup("lookPointing");

    delayLook = bRFInfo.check("delayLook", Value(2.)).asDouble();
    delayPoint = bRFInfo.check("delayPoint", Value(1.)).asDouble();
    delayHome = bRFInfo.check("delayHome", Value(1.5)).asDouble();

    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    portToDumper.open(("/" + moduleName + "/toDumper").c_str());
    if (!Network::connect(("/" + moduleName + "/toDumper"), "/humanRobotDump/rpc"))
    {
        yWarning() << " CANNOT CONNECT TO DUMPERS";
    }
    if (!iCub->getRecogClient())
    {
        yWarning() << " WARNING SPEECH RECOGNIZER NOT CONNECTED";
    }
    abm = true;
    if (!iCub->getABMClient())
    {
        abm = false;
        yWarning() << " WARNING ABM NOT CONNECTED";
    }

    iCub->lookStop();
    iCub->home();

    cout << "delayLook " << delayLook << endl;
    cout << "delayPoint " << delayPoint << endl;
    cout << "delayHome " << delayHome << endl;

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
        "loopPointing + nbLoop \n" +
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
    else if (command.get(0).asString() == "loopPointing") {
        if (command.size() == 2) {
            if (command.get(1).asInt() > 0){
                reply.addList() = loopPointing(command.get(1).asInt());
            }
        }
        else {
            reply.addList() = loopPointing();
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
    else if (command.get(0).asString() == "action"){
        if (command.size() != 3)
        {
            reply.addString("error in bodyReservoir: Botte 'action' misses information action_name object");
        }
        else
        {
            string ob = command.get(2).toString();
            bool bSuccess = false;

            yDebug() << "Point to" << ob;
            if (command.get(1).toString() == "point"){
                bSuccess = iCub->point(ob);
            }
            else if (command.get(1).toString() == "look"){
                bSuccess = iCub->look(ob);
            }
            bSuccess ? reply.addString("ok done") : reply.addString("failed");
        }
    }
    else {
        yInfo() << helpMessage;
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

    if (!Network::connect(("/bodyReservoir/toDumper"), "/humanRobotDump/rpc"))
    {
        yWarning() << " CANNOT CONNECT TO DUMPERS";
    }

    cout << sObject << endl;

    Bottle   bOutput;
    list<pair<string, string> > lArgument;
    lArgument.push_back(pair<string, string>("iCub", "agent"));
    lArgument.push_back(pair<string, string>("point", "predicate"));
    lArgument.push_back(pair<string, string>(sObject, "object"));
    if (abm) iCub->getABMClient()->sendActivity("action", "point", "action", lArgument, true);

    // SEND OBJECT TO DUMP
    Bottle bToDumper;
    bToDumper.addString("objectToDump");
    bToDumper.addString(sObject);

    Bottle bAnswer;
    portToDumper.write(bToDumper, bAnswer);
    yInfo() << bAnswer.toString();

    // SEND SUBACTION
    bToDumper.clear();
    bToDumper.addString("subaction");
    bToDumper.addString("look");

    portToDumper.write(bToDumper, bAnswer);
    yInfo() << bAnswer.toString();


    // SEND FLAG TO DUMPER
    bToDumper.clear();
    bToDumper.addString("robotDump");
    bToDumper.addString("point");

    portToDumper.write(bToDumper, bAnswer);
    yInfo() << bAnswer.toString();

    bool bSuccess = iCub->look(sObject);
    cout << "\t\t\t" << "IN DELAY LOOK" << endl;

    Time::delay(delayLook);
    iCub->lookStop();

    bSuccess &= iCub->point(sObject);
    // SEND SUBACTION
    bToDumper.clear();
    bToDumper.addString("subaction");
    bToDumper.addString("point");

    portToDumper.write(bToDumper, bAnswer);
    yInfo() << bAnswer.toString();

    cout << "\t\t\t" << "IN DELAY POINT" << endl;

    Time::delay(delayPoint);
    // SEND SUBACTION
    bToDumper.clear();
    bToDumper.addString("subaction");
    bToDumper.addString("back");

    portToDumper.write(bToDumper, bAnswer);
    yInfo() << bAnswer.toString();

    iCub->getARE()->home();
    cout << "\t\t\t" << "IN DELAY HOME" << endl;

    Time::delay(delayHome);

    lArgument.push_back(pair<string, string>((bSuccess ? "success" : "failed"), "status"));
    if (abm) iCub->getABMClient()->sendActivity("action", "point", "action", lArgument, false);

    // SEND FLAG TO DUMPER
    bToDumper.clear();
    bToDumper.addString("off");

    portToDumper.write(bToDumper, bAnswer);
    yInfo() << bAnswer.toString();

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
    if (abm) iCub->getABMClient()->sendActivity("action", "wave", "action", lArgument, true);


    bool bSuccess = iCub->look(sAgent);
    Time::delay(0.5);
    bSuccess &= iCub->getARE()->waving(true);

    lArgument.push_back(pair<string, string>((bSuccess ? "success" : "failed"), "status"));
    if (abm) iCub->getABMClient()->sendActivity("action", "point", "action", lArgument, false);

    bOutput.addString("waving");
    bOutput.addString(sAgent);
    (bSuccess) ? bOutput.addString("success") : bOutput.addString("failed");

    return bOutput;
}

/*
* For iNbLoop, the robot will generate 4 objects (up right, up left, bottom right, bottom left) with a bit of noise
* For each object, the robot will look at it, then point at it, then come back to it original position if bComeBack (true by defatul)
* Return the status of the actions.
*/
Bottle bodyReservoir::loopPointing(int iNbLoop)
{
    Bottle bOutput;
    Port  portToOpcPopulater;
    portToOpcPopulater.open("/bodyReservoir/toPopulater");
    if (!Network::connect("/bodyReservoir/toPopulater", "/opcPopulater/rpc"))
    {
        yWarning() << " CANNOT CONNECT TO OPC POPULATER";
        bOutput.addString("CANNOT CONNECT TO OPC POPULATER");
        return bOutput;
    }

    Bottle bToPopulater,
        bResultPopulate,
        bResultPointing;
    bToPopulater.addString("populateSpecific2");

    vector<string> vTarget;
    vTarget.push_back("bottom_right");
    vTarget.push_back("top_right");
    vTarget.push_back("bottom_left");
    vTarget.push_back("top_left");

    for (int ii = 0; ii < iNbLoop; ii++)
    {
        portToOpcPopulater.write(bToPopulater, bResultPopulate);
        yInfo() << " result of population: " << bResultPopulate.toString();
        Time::delay(0.5);

        for (vector<string>::iterator itTar = vTarget.begin(); itTar != vTarget.end(); itTar++)
        {
            yInfo() << " pointing " << *itTar << " for loop " << ii + 1 << "/" << iNbLoop;
            bResultPointing = pointObject(*itTar);
            if (bResultPointing.get(0).asString() == "failed"){
                yInfo() << "error in pointing " << *itTar << " " << ii;
                bOutput.addString("error in pointing " + *itTar + " " + ii);
            }
        }
    }

    return bOutput;
}

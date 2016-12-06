/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors:  Maxime Petit
 * email:  m.petit@imperial.ac.uk
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

#include "kinematicStructureInterface.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

bool kinematicStructureInterface::configure(yarp::os::ResourceFinder &rf)
{

    moduleName = rf.check("name", Value("kinematicStructureInterface")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << "findFileByName " << rf.findFileByName("kinematicStructureInterface.ini") ;
    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();
    robot  = rf.check("robot", Value("icub")).asString().c_str();

    //Create the ICubclient, check client.ini to know which subsystem to load (should be ARE)
    iCub = new ICubClient(moduleName, "kinematicStructureInterface", "client.ini");
    iCub->opc->isVerbose = false;
    if (!iCub->connect())
    {
        yInfo() << " iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready! \n \n ";

    return true;
}

bool kinematicStructureInterface::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "updateCorrespondence <bpName> <bpCorrespondence> \n" +
        "help \n" +
        "quit \n" ;

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        rpcPort.reply(reply);
        return false;
    } else if(command.get(0).asString() == "updateCorrespondence"){
        string bpName = "";
        string bpCorrespondence = "";
        if(command.size() == 3){
            bpName = command.get(1).asString();
            bpCorrespondence = command.get(2).asString();
            reply = updateCorrespondenceOpc(bpName, bpCorrespondence);
        } else {
            yError() << "to update the correspondence, need both bpName (string) and bpCorrespondence (string)!";
            reply.addString("nack");
        }
        rpcPort.reply(reply);
        return true;
    }
    else {
        cout << helpMessage;
        reply.addString("ok");
    }

    rpcPort.reply(reply);

    return true;
}

/* Simple method to update a bodypart m_kinectNode inside the opc */
Bottle kinematicStructureInterface::updateCorrespondenceOpc(string bpName, string bpCorrespondence){

    Bottle bOutput;

    //check that bpCorrespondence is one valid kinectNode? 
    if(find(begin(ALL_AVAILABLE_BODYPARTS), end(ALL_AVAILABLE_BODYPARTS), bpCorrespondence) == end(ALL_AVAILABLE_BODYPARTS)){
        yError() << "[updateCorrespondenceOpc] the bpCorrespondence (" << bpCorrespondence << ") is NOT a valid kinectNode!";
        bOutput.addString("ERROR");
        bOutput.addString("NOT a valid kinectNode");
        return bOutput;
    }

    //search through opc for the bodypart with bpName
    iCub->opc->checkout();
    Entity* e = iCub->opc->getEntity(bpName, true);
    if(!e) {
        yError() << "Could not get bodypart" << bpName;
        //iCub->say("Could not get bodypart" + bpName);
        bOutput.addString("ERROR");
        bOutput.addString("entity is unknown");
        return bOutput;
    }

    //Error if the name does NOT correspond to a bodypart
    if(!e->isType("bodypart")) {
        yError() << " error in kinematicStructureInterface::updateCorrespondenceOpc | for " << bpName << " | " << bpName << " is NOT a bodypart!" ;
        bOutput.addString("ERROR");
        bOutput.addString("NOT a bodypart");
        return bOutput;
    }
    Bodypart* BPentity = dynamic_cast<Bodypart*>(e);
    BPentity->m_kinectNode = bpCorrespondence;

    iCub->opc->commit(BPentity);
    bOutput.addString("ack");

    return bOutput;
}


/* Called periodically every getPeriod() seconds */
bool kinematicStructureInterface::updateModule() {


    return true;
}


/* interrupt ports when interrupting the module */
bool kinematicStructureInterface::interruptModule() {
    rpcPort.interrupt();
    iCub->close();

    return true;
}

/* close ports when closing the module */
bool kinematicStructureInterface::close() {

    rpcPort.interrupt();
    rpcPort.close();

    iCub->close();

    return true;
}

/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Maxime Petit
* email:   m.petit@imperial.ac.uk
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

#include <abmVision.h>
#include "wrdac/subsystems/subSystem_ABM.h"

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


abmVision::~abmVision()
{
}

/*
* Configure method. Receive a previously initialized
* resource finder object. Use it to configure your module.
* If you are migrating from the old Module, this is the
* equivalent of the "open" method.
*/

bool abmVision::configure(yarp::os::ResourceFinder &rf) {

    bool    bEveryThingisGood = true;
    bool    bOptionnalModule = true;
    moduleName = rf.check("name", Value("/abmVision"), "module name (string)").asString();

    /*
    * before continuing, set the module name before getting any other parameters,
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "abmVision", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    while (!iCub->connect())
    {
        yWarning() << "iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }
    iCub->opc->checkout();

    //configureOPC(rf);

    if (!iCub->getABMClient())
    {
        iCub->say("ABM not connected");
        yWarning() << "ABM NOT CONNECTED";
    }

    // Open handler port
    handlerPortName = "/";
    handlerPortName += getName() + "/rpc";         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {
        yInfo() << getName() << ": Unable to open port " << handlerPortName;
        bEveryThingisGood = false;
    }

    attach(handlerPort);                  // attach to port

    if (!bEveryThingisGood || !bOptionnalModule)
        yInfo() << " Some dependencies are not running (iCubClient, ABM?)";
    else
    {
        yInfo() << " ----------------------------------------------";
        yInfo() << " abmVision ready !";
    }

    return bEveryThingisGood;
}

bool abmVision::interruptModule() {
    return true;
}

bool abmVision::close() {
    handlerPort.close();

    delete iCub;

    return true;
}

bool abmVision::respond(const Bottle& command, Bottle& reply) {
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
        yInfo() << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString() == "get") {
        reply.addString("method TODO");
    }


    return true;
}

/* Called periodically every getPeriod() seconds */
bool abmVision::updateModule() {
    return true;
}

double abmVision::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}


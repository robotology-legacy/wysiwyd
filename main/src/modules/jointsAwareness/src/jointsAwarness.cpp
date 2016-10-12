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

#include "jointsAwareness.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

bool jointsAwareness::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("jointsAwareness")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << "findFileByName " << rf.findFileByName("jointsAwareness.ini") ;

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "jointsAwareness", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    arm   = rf.check("arm", Value("both")).asString().c_str();
    robot   = rf.check("robot", Value("icub")).asString().c_str();

    /*portToArm.open(("/" + moduleName + "/toArm:rpc").c_str());
    string portRobotArmName = "/" + robot + "/" + arm + "/rpc:i";

    yInfo() << "================> port controlling the arm : " << portRobotArmName;
    if (!Network::connect(portToArm.getName().c_str(),portRobotArmName))
    {
        yWarning() << "WARNING PORT TO CONTROL ARM (" << portRobotArmName << ") IS NOT CONNECTED";
    }*/


    if (!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        Time::delay(1.0);
    }

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    iCub->say("joints awareness is ready!", false);



    return true;
}

bool jointsAwareness::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n" ;

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    } else {
        cout << helpMessage;
        reply.addString("ok");
    }

    rpcPort.reply(reply);

    return true;
}


/* Called periodically every getPeriod() seconds */
bool jointsAwareness::updateModule() {
    return true;
}


bool jointsAwareness::interruptModule() {
    rpcPort.interrupt();

    return true;
}

bool jointsAwareness::close() {
    iCub->close();
    delete iCub;

    rpcPort.interrupt();
    rpcPort.close();

    return true;
}

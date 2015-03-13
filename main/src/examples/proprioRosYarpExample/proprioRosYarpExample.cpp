/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
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

#include <stdio.h>
#include <algorithm>
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "proprioRosYarpExample.h"

using namespace std;
using namespace yarp::os;

bool proprioRosYarpExample::configure(yarp::os::ResourceFinder &rf) {
    setName(rf.check("name", Value("proprioRosYarpExample"), "module name (string)").asString().c_str());

    string robot = rf.check("robot",Value("NAO")).asString().c_str();
    // change robot name to all lower case
    std::transform(robot.begin(), robot.end(), robot.begin(), ::tolower);

    // connect to ABM
    string abmName = rf.check("abm",Value("autobiographicalMemory")).asString().c_str();
    string abmLocal = "/"+getName()+"/abm:o";
    abm.open(abmLocal.c_str());
    string abmRemote = "/"+abmName+"/rpc";

    while (!Network::connect(abmLocal.c_str(),abmRemote.c_str())) {
        cout << "Waiting for connection to ABM..." << endl;
        Time::delay(1.0);
    }

    // attach to rpc port
    string handlerPortName = "/" + getName() + "/rpc";

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    attach(handlerPort);

    state_in.setReadOnly();

    string state_in_portname;
    if(robot=="nao") {
        state_in_portname = "/joint_states";
    } else if(robot=="baxter") {
        state_in_portname = "/robot/joint_states";
    } else {
        cerr << "Unknown robot name, abort" << endl;
        return false;
    }
    if (!state_in.open(state_in_portname+"@/"+robot+"_states")) {
        cerr << "Failed to open port" << endl;
        return 1;
    }

    pos_out.open("/" + robot + "/state:o");
    vel_out.open("/" + robot + "/velocities:o");

    // make providers known to ABM
    Bottle bCmd, bReply;
    bCmd.addString("addDataStreamProvider");
    bCmd.addString(pos_out.getName());

    abm.write(bCmd, bReply);

    if(bReply.get(0).toString()=="[ack]") {
        cout << "Added " << pos_out.getName() << " as DataStreamProvider to ABM" << endl;
    } else {
        cerr << "Could not add " << pos_out.getName() << " as DataStreamProvider to ABM" << endl;
        cerr << "Reason: " << bReply.toString() << endl;
    }

    yarp::os::Time::delay(0.2);

    bCmd.clear();
    bReply.clear();
    bCmd.addString("addDataStreamProvider");
    bCmd.addString(vel_out.getName());

    abm.write(bCmd, bReply);

    if(bReply.get(0).toString()=="[ack]") {
        cout << "Added " << vel_out.getName() << " as DataStreamProvider to ABM" << endl;
    } else {
        cerr << "Could not add " << vel_out.getName() << " as DataStreamProvider to ABM" << endl;
        cerr << "Reason: " << bReply.toString() << endl;
    }

    return true;
}

bool proprioRosYarpExample::respond(const Bottle& bCommand, Bottle& bReply) {
    if (bCommand.get(0).asString() == "test" )
    {
        bReply.addString("[ack]");
    }
    else
    {
        bReply.addString("[nack]");
    }
    return true;
}

double proprioRosYarpExample::getPeriod() {
    return 0.05;
}


bool proprioRosYarpExample::updateModule() {
    Bottle bMsgIn;
    if (!state_in.read(bMsgIn)) {
        cerr << "Failed to read msg" << endl;
    }
    else {
        Bottle *bPosIn, bPosOut, *bVelIn, bVelOut;
        bPosIn = bMsgIn.get(2).asList();
        bVelIn = bMsgIn.get(3).asList();

        for(int i=0; i<bPosIn->size(); i++) {
            bPosOut.addDouble(bPosIn->get(i).asDouble());
            bVelOut.addDouble(bVelIn->get(i).asDouble());
        }

        pos_out.write(bPosOut);
        vel_out.write(bVelOut);
    }

    return true;
}

bool proprioRosYarpExample::interruptModule() {
    Bottle bCmd, bReply;
    bCmd.addString("removeDataStreamProvider");
    bCmd.addString(pos_out.getName());

    abm.write(bCmd, bReply);

    if(bReply.get(0).toString()=="[ack]") {
        cout << "Removed " << pos_out.getName() << " as DataStreamProvider from ABM" << endl;
    } else {
        cerr << "Could not remove " << pos_out.getName() << " as DataStreamProvider from ABM" << endl;
        cerr << "Reason: " << bReply.toString() << endl;
    }

    yarp::os::Time::delay(0.2);

    bCmd.clear();
    bReply.clear();
    bCmd.addString("removeDataStreamProvider");
    bCmd.addString(vel_out.getName());

    abm.write(bCmd, bReply);

    if(bReply.get(0).toString()=="[ack]") {
        cout << "Removed " << vel_out.getName() << " as DataStreamProvider from ABM" << endl;
    } else {
        cerr << "Could not remove " << vel_out.getName() << " as DataStreamProvider from ABM" << endl;
        cerr << "Reason: " << bReply.toString() << endl;
    }

    pos_out.interrupt();
    vel_out.interrupt();
    abm.interrupt();
    handlerPort.interrupt();

    return true;
}

bool proprioRosYarpExample::close() {
    pos_out.interrupt();
    pos_out.close();
    vel_out.interrupt();
    vel_out.close();
    abm.interrupt();
    abm.close();
    handlerPort.interrupt();
    handlerPort.close();

    return true;
}

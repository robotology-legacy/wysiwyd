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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "proprioRosYarpExample.h"

using namespace std;
using namespace yarp::os;

bool proprioRosYarpExample::configure(yarp::os::ResourceFinder &rf) {
    setName(rf.check("name", Value("proprioRosYarpExample"), "module name (string)").asString().c_str());

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

    Port baxter_in;
    baxter_in.setReadOnly();
    if (!baxter_in.open("/robot/joint_states@/baxter_states")) {
        cerr << "Failed to open port" << endl;
        return 1;
    }

    Port pos_out, vel_out;
    pos_out.open("/baxter/state:o");
    vel_out.open("/baxter/velocities:o");

    while (true) {
        Bottle bMsgIn;
        if (!baxter_in.read(bMsgIn)) {
            cerr << "Failed to read msg" << endl;
            continue;
        }
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

bool proprioRosYarpExample::respond(const Bottle& bCommand, Bottle& bReply) {
    if (bCommand.get(0).asString() == "test" )
    {
        bReply.addString("ack");
    }
    else
    {
        bReply.addString("nack");
    }
    return true;
}

double proprioRosYarpExample::getPeriod() {
    return 0.1;
}


bool proprioRosYarpExample::updateModule() {
    return true;
}

bool proprioRosYarpExample::interruptModule() {
    abm.interrupt();
    handlerPort.interrupt();

    return true;
}

bool proprioRosYarpExample::close() {
    abm.interrupt();
    abm.close();
    handlerPort.interrupt();
    handlerPort.close();

    return true;
}

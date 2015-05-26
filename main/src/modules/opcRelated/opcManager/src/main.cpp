/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau
 * email:   greg.pointeau@gmail.com
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


/**
\defgroup opcManager opcManager


Module responsible for polling the OPCs: real and mental and update the belief of the agent.
Easy interaction via rpc



\author Maxime Petit, Grégoire Pointeau
*/

#include "opcManager.h" 

using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[]) {

    /* initialize yarp network */
    Network yarp;

    /* create your module */
    opcManager oManager;

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("opcManager");
    rf.setDefaultConfigFile("opcManager.ini");
    rf.configure(argc, argv);


    /* run the module: runModule() calls configure first and, if successful, it then runs */
    oManager.runModule(rf);

    return 0;
}



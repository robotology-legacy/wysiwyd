/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Anne-Laure MEALIER
 * email:   anne-laure.mealier@inserm.fr
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

#include "reservoirHandler.h"

int main(int argc, char * argv[]) {
    /* initialize yarp network */
    yarp::os::Network yarp;
    reservoirHandler module;


    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("reservoirHandler.ini"); //overridden by --from parameter
    rf.setDefaultContext("reservoirHandler/conf");   //overridden by --context parameter
    rf.configure(argc, argv);

    module.runModule(rf);

    return 0;
}


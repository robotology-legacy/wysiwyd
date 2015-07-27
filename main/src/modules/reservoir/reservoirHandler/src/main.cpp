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

#include <reservoirHandler.h>

int main(int argc, char * argv[]) {
    /* initialize yarp network */
    Network yarp;


    //system("mode con COLS=100");

    //test to launch a PDDL planner
    //system("cd ~/Robots/planner/problem/ && ../seq-sat-lama-2011/plan efaa-dom.pddl efaa-prob.pddl res_plan.txt") ;
    //system("cd ~/Robots/planner/problem/ && mv res_plan.txt* res_plan.txt") ;


    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("reservoirHandler.ini"); //overridden by --from parameter
    rf.setDefaultContext("reservoirHandler/conf");   //overridden by --context parameter
    rf.configure(argc, argv);

        /* create your module */
    reservoirHandler module(rf);
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

    return 0;
}


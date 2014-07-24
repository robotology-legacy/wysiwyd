// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau - Maxime Petit
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

/** 
* \defgroup abmReasoning abmReasoning
* @ingroup efaa_modules

Reasoning module based on the autobiographical memory

\section intro_sec Description 

Module to build, maintain and query a postgreSQL 8.3 database to use interaction history of the iCub. 

The commands it is possible to send (/efaa/abmReasoning/rpc) are:

@b findActivity <actionName> <begin|end|both> [<colums,to,select>] : find an action by his name. 2 OPC snapshot per action (begin and end). Could precide the column(s) you want (* by default).
@b findSharedPlan <sharedPlanName> <begin|end|both> [<colums,to,select>] : find an sharedPlan by his name. 2 OPC snapshot per sharedPlan (begin and end). Could precide the column(s) you want (* by default).
@b findOPC <#instance> : Give the OPC snapshot according to its instance
@b "help" to print the available command
@b "quit" to quit the module nicely
@b "sqlQueryTest" to run a fixed sqlQuery, testing the access to the ABM
@b "getActionConsequenceDrive" + 'actionName' + 'arg'
@b "printContextualKnowledge" + 'actionName' + 'arg' 
@b "printPDDLContextualKnowledge" + 'actionName' + 'arg'
@b "printPDDLContextualKnowledgeProblem" + 'actionName' + 'arg'
@b "testActionContextualKnowledge" + 'actionName' + 'arg' + 'objName'
@b "askLastAction"
@b "askLastActivity"
@b "addLastActivity"
@b "getIdFromActivity"
@b "findPossibleSharedPlan"
@b "availableSharedPlan"
@b "findAllSharedPlan"
@b "discrimminateAction"
@b "queryBehavior"
@b "saveKnowledge"
@b "resetKnowledge"
@b "discriminateUnknownActions"
@b "connectOPC"
@b "executeAction"
@b "executeActivity"
@b "updateLocation"


Available are : findActivity <actionName> <begin|end|both> [<colums,to,select>] | findSharedPlan <sharedPlanName> <begin|end|both> [<colums,to,select>] | help | sqlQueryTest | quit ");


\section lib_sec Libraries 
- YARP libraries
- \ref efaaHelpers

\section parameters_sec Parameters
None.

\section portsa_sec Ports Accessed
/efaa/abmReasoning/rpc \n
/efaa/abmReasoning/request:o \n
/efaa/abmReasoning/world/opc:rpc \n

\section portsc_sec Ports Created
- \c /efaa/abmReasoning/rpc \n This port must be used to communicate with the module \n

- \c /efaa/abmReasoning/request:o \n This port is used to communicate with the memory (autobiographicalMemory) \n

- \c /efaa/abmReasoning/world/opc:rpc \n This port is used to communicate with the OPC \

- \c /efaa/abmReasoning/interlocutor/toOPC \n This port is use to communicate between the interlocutor and the OPC

\section tested_os_sec Tested OS
Windows.
Linux

\section conf_file_sec Configuration Files 

* - \c autobiographicalMemory \c config.ini \n 
*   specifies the configuration file

\section tested_os_sec Tested OS
Windows

* Tool for reasoning on the Autobiographical Memory \n

* Contact : gregoire.pointeau@inserm.fr , maxime.petit@inserm.fr
* @author Gr\'egoire Pointeau, Maxime Petit

*/ 

#include <abmReasoning.h>

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
    rf.setDefaultConfigFile("abmReasoning.ini"); //overridden by --from parameter
    rf.setDefaultContext("abmReasoning");   //overridden by --context parameter
    rf.configure(argc, argv);
        /* create your module */
    abmReasoning module(rf); 
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

    return 0;
}


// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2012 EFAA Consortium, European Commission FP7 Project IST-270490
* Authors: Grégoire Pointeau, Maxime Petit
* email:   gregoire.pointeau@inserm.fr, maxime.petit@inserm.fr
* website: http://efaa.upf.edu/ 
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* $EFAA_ROOT/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

/** 
* \defgroup autobiographicalMemory autobiographicalMemory
* @ingroup efaa_modules

storage of the memory in the data base.

\section intro_sec Description 

Module to build, maintain and query a postgreSQL 8.3 database to use interaction history of the iCub. 

A database named 'ABM' need to be created on the local computer. (properties in the config file)

The commands it is possible to send (/efaa/autobiographicalMemory/request:i) are:

@b "new" Create a new database (erase the current one)

@b "snapshot" Create a snapshot of the OPC in the DB format of input bottle: "snapshot" (action name_action type_action) (arg1 arg2 argn) (role1 role 2 rolen) (begin 0/1)

@b "snapshotBE" Create a snapshot of the OPC in the DB optimized for the behaviors. format of input bottle: "snapshot" (action name_action type_action) (arg1 arg2 argn) (role1 role 2 rolen) (begin 0/1)

@b "snapshotSP" Create a snapshot of the OPC in the DB optimized for the shared plan. format of input bottle: "snapshot" (action name_action type_action) (arg1 arg2 argn) (role1 role 2 rolen) (begin 0/1)

@b "eraseInstance" Erase in the DB the data related to an instance. Input : "eraseInstance" instance_1 instance_2 instance_3 ... or "eraseInstance" (instance_begin instance_end)

@b "connectOPC" Connect the module to an OPC. input: "connectOPC OPC_name

@b "connect" Connect the module to the abmReasoning module

@b "request" Send a SQL request to the database. input: "request sql_request"

@b "load" Erase the content of the database and build a new one.

@b "resetKnowledge" Erase the content of the semantic memory

@b "quit" Quit the module

@b "close" Close the module

\section lib_sec Libraries 
- YARP libraries
- \ref efaaHelpers

\section parameters_sec Parameters
None.

\section portsa_sec Ports Accessed
/efaa/autobiographicalMemory/request:i \n
/efaa/autobiographicalMemory/to_reasoning \n
/efaa/autobiographicalMemory/world/opc:rpc \n

\section portsc_sec Ports Created
- \c /efaa/autobiographicalMemory/request:i \n This port must be used to communicate with the module \n

- \c /efaa/autobiographicalMemory/to_reasoning \n This port is used to communicate with the reasoning module (abmReasoning) \n

- \c /efaa/autobiographicalMemory/world/opc:rpc \n This port is used to communicate with the OPC \


\section conf_file_sec Configuration Files 

* - \c autobiographicalMemory \c config.ini \n 
*   specifies the configuration file

The configuration file passed through the option \e --from
should look like as follows:

\code 

//properties of the database - change according to your setup
[database_properties]
server					"127.0.0.1"
user					"postgres"
password				"rclab"
dataB					"ABM"

\endcode


\section tested_os_sec Tested OS
Windows

* Tool for reasoning on the Autobiographical Memory \n

* Contact : gregoire.pointeau@inserm.fr , maxime.petit@inserm.fr
* @author Gr\'egoire Pointeau, Maxime Petit

*/ 

#include <autobiographicalMemory.h>

int main(int argc, char * argv[]) {
    /* initialize yarp network */ 
    Network yarp;
    srand(time(NULL));

    //system("mode con COLS=100");


    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("autobiographicalMemory.ini"); //overridden by --from parameter
    rf.setDefaultContext("autobiographicalMemory");   //overridden by --context parameter
    rf.configure( argc, argv);
    /* create your module */
    autobiographicalMemory module(rf); 
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

    return 0;
}


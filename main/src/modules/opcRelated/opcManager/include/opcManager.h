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


#include "wrdac/clients/icubClient.h"
#include "wrdac/clients/opcEars.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

const double time_action = 0.500;
const double time_relation = 1.00;


/******************************************************************************************/

//World, list of Entities
class opcManager: public RFModule
{
private:
    OPCClient *realOPC;		// Real OPC
    OPCClient *mentalOPC;	// Mental OPC

    opcEars OPCEARS;

    Port handlerPort;      //a port to handle messages 
    Port portToAbmReasoning;
    string moduleName;
    string s_realOPC;				// name of the real OPC
    string s_mentalOPC;	// name of the mental OPC


public:

    Bottle connect(Bottle bInput);
    bool populate();                              // initialise the world with some objects
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();

    Bottle simulateActivity(Bottle bInput);			// Simulate an activity in the mental OPC
    Bottle simulateAction(Bottle bInput);			// Simulate an action in the mental OPC

    Bottle updateBelief(string sOPCname);			// update the beliefs of the agent present in an OPC  
    Bottle synchoniseOPCs();						// synchronise the mentalOPC with the content of the OPC

    Bottle getBeliefs(Bottle bInput);				// return the beliefs of an agent in an OPC given
    Bottle diffOPC();

};

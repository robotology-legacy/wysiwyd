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

const double time_action = 0.500;
const double time_relation = 1.00;


/******************************************************************************************/

//World, list of Entities
class opcManager : public yarp::os::RFModule
{
private:
   wysiwyd::wrdac::OPCClient *realOPC;		// Real OPC
   wysiwyd::wrdac::OPCClient *mentalOPC;	// Mental OPC

   wysiwyd::wrdac::opcEars OPCEARS;

    yarp::os::Port handlerPort;      //a port to handle messages 
	yarp::os::Port portToAbmReasoning;
    std::string moduleName;
	std::string s_realOPC;				// name of the real OPC
	std::string s_mentalOPC;	// name of the mental OPC


public:

	yarp::os::Bottle connect(yarp::os::Bottle bInput);
    bool populate();                              // initialise the world with some objects
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();

	yarp::os::Bottle simulateActivity(yarp::os::Bottle bInput);			// Simulate an activity in the mental OPC
	yarp::os::Bottle simulateAction(yarp::os::Bottle bInput);			// Simulate an action in the mental OPC

	yarp::os::Bottle updateBelief(std::string sOPCname);			// update the beliefs of the agent present in an OPC  
	yarp::os::Bottle synchoniseOPCs();						// synchronise the mentalOPC with the content of the OPC

	yarp::os::Bottle getBeliefs(yarp::os::Bottle bInput);				// return the beliefs of an agent in an OPC given
	yarp::os::Bottle diffOPC();

};

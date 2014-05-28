/* 
 * Copyright (C) 2011 EFAA Consortium, European Commission FP7 Project IST-270490
 * Authors: Maxime Petit, Grégoire Pointeau
 * email:   maxime.petit@inserm.fr, gregoire.pointeau@inserm.fr
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
\defgroup opcManager opcManager
 
@ingroup efaa_modules
 
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
   rf.configure(argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */
   oManager.runModule(rf);

   return 0;
}



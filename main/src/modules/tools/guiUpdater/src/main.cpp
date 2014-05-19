/* 
 * Copyright (C) 2011 EFAA Consortium, European Commission FP7 Project IST-270490
 * Authors: Stephane Lallee
 * email:   stephane.lallee@gmail.com
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
\defgroup guiUpdaterV2 guiUpdaterV2
 
@ingroup efaa_modules
 
Module responsible for polling the egosphere for various content (objects, skeletons, etc.) and 
updating icubGui based on this.

\section intro_sec Description 
 
The purpose of this module is to retrieve information from the 
online database implemented by the objectsPropertiesCollector 
module (OPC) and format/transmit this information to the gui
implemented by iCubGui (icub repo).
 
\section proto_sec Protocol 
 None.

\section lib_sec Libraries 
- YARP libraries
- \ref efaaHelpers

\section parameters_sec Parameters
--name <string> 
- To specify the module's name; all the open ports will be 
  tagged with the prefix /<moduleName>/. If not specified
  \e guiUpdater is assumed.
 
--period <int>
- To specify the thread period given in [ms]. By default, 200 
  [ms] is assumed.

--verbose
- To specify if debug information should be printed on the stdio
 
--displaySkeletons
- Agents skeleton will be displayed if availaible.

\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created
 
- \e /<moduleName>/world/opc:rpc to be connected to the 
  objectsPropertiesCollector port.
 
- \e /<moduleName>/gui to be connected to the iCubGui.
 
\section tested_os_sec Tested OS
Linux and Windows.

\author Stephane Lallee
*/ 

#include "world.h" 

using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[]) {

   /* initialize yarp network */ 
   Network yarp;

   /* create your module */
   GuiUpdaterModule world; 

   /* prepare and configure the resource finder */
   ResourceFinder rf;
   rf.setVerbose(true);
   rf.configure("EFAA_ROOT", argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */
   world.runModule(rf);

   return 0;
}



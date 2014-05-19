/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com
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
   rf.configure(argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */
   world.runModule(rf);

   return 0;
}



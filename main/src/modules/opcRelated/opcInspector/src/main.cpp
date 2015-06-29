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

#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include "opcInspector.h"

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[])
{
   Network yarp;
   if (!yarp.checkNetwork())
   {
       cout<<"yarp network is not available!"<<endl;
       return -1;
   }

   OpcInspector module; 

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.configure(argc,argv);
 
   module.runModule(rf);
   return 0;
}



/* 
* Copyright (C) 2015 WYSIWYD
* Authors: Tobias Fischer
* email:   t.fischer@imperial.ac.uk
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

/** 

\section intro_sec Description 


*/

#include <yarp/os/all.h>
#include "ABMAugmentionExample.h"
#include <time.h>

using namespace yarp::os;

int main(int argc, char * argv[]) {
    srand(time(NULL));
    Network yarp;
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("ABMAugmentionExample");
    rf.setDefaultConfigFile("ABMAugmentionExample.ini");
    rf.configure(argc, argv);

    ABMAugmentionExample mod;
    mod.runModule(rf);
    return 0;
}


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

#include <time.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include "PerspectiveTaking.h"

using namespace yarp::os;

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char * argv[]) {
    srand(time(NULL));
    Network yarp;
    if (!yarp.checkNetwork()) {
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("perspectiveTaking");
    rf.setDefaultConfigFile("perspectiveTaking.ini");
    rf.configure(argc, argv);

    perspectiveTaking mod;
    mod.runModule(rf);
    return 0;
}

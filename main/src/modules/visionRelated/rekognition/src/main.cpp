/*
* Copyright (C) 2015 WYSIWYD
* Authors: Tobias Fischer and Maxime Petit
* email:   t.fischer@imperial.ac.uk and m.petit@imperial.ac.uk
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

#include <stdlib.h>
#include <time.h>

#include <yarp/os/all.h>
#include "rekognition.h"

using namespace yarp::os;

int main(int argc, char * argv[]) {
    srand(time(NULL));
    Network yarp;
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("rekognition");
    rf.setDefaultConfigFile("rekognition.ini");
    rf.configure(argc, argv);

    rekognition mod;
    mod.runModule(rf);
    return 0;
}

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
\defgroup mainLoop Main Loop


\section intro_sec Description 

Handle the integration of all sensors and reactions of the robot.
The robot is reacting to tactile stimuli, gestures, voice and joystick inputs.
It also includes the subscenarios of Pon, DJ, and Tic Tac Toe.

*/

#include <yarp/os/all.h>
#include "reactiveLayer.h"

using namespace std;
using namespace yarp::os;

int main(int argc, char * argv[])
{
    srand(time(NULL));
    Network::init();
    ReactiveLayer mod;
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("reactiveLayer/conf");
    rf.setDefaultConfigFile("default.ini");
    rf.configure( argc, argv);
    mod.runModule(rf);
    return 0;
}

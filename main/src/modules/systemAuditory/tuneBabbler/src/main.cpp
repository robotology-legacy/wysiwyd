/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Jordi Ysard
 * email:   jordiysard@gmail.com
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



#include "soundGenerator.h"

using namespace std;
using namespace yarp::os;

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP network seems unavailable!";
        return 1;
    }
    babbler mod;
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("tuneBabbler");
    rf.setDefaultConfigFile("tuneBabbler.ini");
    rf.configure( argc, argv);
    return mod.runModule(rf);
}

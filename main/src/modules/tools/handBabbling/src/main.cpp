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

#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"
#include <time.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

int main()
{
	Network::init();
    srand(time(NULL));
    
    Property options;
    options.put("robot", "icub"); // typically from the command line.
    options.put("device", "remote_controlboard");

    Value& robotname = options.find("robot");
    string s("/");
    s += robotname.asString();
    s += "/right_arm/babbling";
    options.put("local", s.c_str());
    s.clear();
    s += "/";
    s += robotname.asString();
    s += "/right_arm";
    options.put("remote", s.c_str());
    

    PolyDriver dd(options);
    if (!dd.isValid()) {
        cout << "Device not available.  Here are the known devices:\n"<< endl;
        cout << Drivers::factory().toString().c_str() << endl;;
        Network::fini();
        return 0;
    }

    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    IPidControl *pid;
    IAmplifierControl *amp;
    IControlLimits *lim;

    bool ok;
    ok = dd.view(pos);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(pid);
    ok &= dd.view(amp);
    ok &= dd.view(lim);

     dd.close();


     int jnts = 0;
     pos->getAxes(&jnts);
     printf("Working with %d axes\n", jnts);
     double *tmp = new double[jnts];
     
     // limits

     vector<pair<int, int> > vLimitJoints;
     for (int i = 0; i < jnts; i++) {
         double min, max;
         lim->getLimits(i, &min, &max);
         vLimitJoints.push_back(pair<int, int> ((int)min, (int)max));
     }

     // get value of the arm
     // we move : 4 5 6 7 8 9 10 11 12 13 14 15 
     // we don't move: 0 1 2 3
    
     

     while(true)
     {
         bool motionDone = false;
         for (int i = 4; i < jnts; i++) {
             tmp[i] = (rand()&(vLimitJoints[jnts].second-vLimitJoints[jnts].first) )+ vLimitJoints[jnts].first;
             pos->positionMove(i,tmp[i]);
         }
         pos->checkMotionDone(&motionDone);
         while (!motionDone)    {}
     }

    return 0;
}


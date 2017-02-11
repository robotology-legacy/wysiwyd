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
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <time.h>
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

int main()
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP network seems unavailable!";
        return 1;
    }
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

    if (!ok) {
        cout << "Device not able to acquire views" << endl;
        Network::fini();
        dd.close();
        return 0;
    }


    int jnts = 0;
    pos->getAxes(&jnts);
    printf("Working with %d axes\n", jnts);


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
    
    vector<bool> mask;
    mask.resize(jnts);
    mask[0]    = false;
    mask[1]    = false;
    mask[2]    = false;
    mask[3]    = false;
    mask[4] = false;
    mask[5] = false;
    mask[6] = false;
    mask[7] = false;
    mask[8] = false;
    mask[9] = false;
    mask[10] = false;
    mask[11] = true;
    mask[12] = true;
    mask[13] = true;
    mask[14] = true;
    mask[15] = true;

    Vector tmp;
    tmp.resize(jnts,0.0);

    for (int i = 4; i < jnts; i++)
    {
        pos->positionMove(i, 0.0);
    }

    bool initDone = false;
    while (!initDone)
    {
        initDone = true;
        for (int i = 4; i < jnts; i++)
        {
            bool jntMotionDone = false;
            pos->checkMotionDone(i, &jntMotionDone);
            initDone &= jntMotionDone;
        }
    }



    while(true)
    {
        cout << "Moving to new posture..." << endl;

        for (int i = 4; i < jnts; i++)
        {
            if (mask[i])
            {
                double newValue = yarp::os::Random::uniform(vLimitJoints[i].first, vLimitJoints[i].second);
                tmp[i] = newValue;
                pos->positionMove(i, tmp[i]);
            }
        }

        cout << "Waiting for posture to be reached... ("<<tmp.toString(3,3)<<" ) ..." ;

        bool motionDone = false;
        while (!motionDone)
        {
            motionDone = true;
            for (int i = 4; i < jnts; i++)
            {
                if (mask[i])
                {
                    bool jntMotionDone = false;
                    pos->checkMotionDone(i, &jntMotionDone);
                    motionDone &= jntMotionDone;
                }
            }
        }
        Time::delay(15.0);
        cout << "ok" << endl;
    }

    dd.close();

    return 0;
}


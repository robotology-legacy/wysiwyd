/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project
 * Authors: Stephane Lallee
 * email:   stephane.lallee@gmail.com
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
#include <iostream>
#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/Drivers.h>
#include <vector>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
        return false;

    ResourceFinder rf;
    rf.configure(argc,argv);

    string moduleName = rf.check("name",Value("vector2cmd")).asString().c_str();
    string robotName = rf.check("robot",Value("icub")).asString().c_str();

    if (!rf.check("part"))
    {
        cout<<"A part specification is mandatory (e.g --part left_arm)"<<endl;
        return -1;
    }

    //Open motor interfaces
    string partUsed = rf.find("part").asString().c_str();
    string remoteName = "/";
    remoteName += robotName;
    remoteName += "/";
    remoteName += partUsed;

    string localName = "/";
    localName += moduleName;
    localName += "/";
    localName += partUsed;

    Property options;
    options.put("robot", "icub");
    options.put("device", "remote_controlboard");
    options.put("remote", remoteName.c_str());
    options.put("local", localName.c_str());

     PolyDriver driver(options);
     if (!driver.isValid()) {
         cout<<"Device not available."<<endl;
         Network::fini();
         return -1;
     }
     
    IPositionControl *pos;
    driver.view(pos);
    int axesCount;

    if (pos == NULL)
    {
        cout<<"Problem while acquiring the view"<<endl;
        Network::fini();
        return -1;
    }
    pos->getAxes(&axesCount);

    BufferedPort<Bottle> input;
    input.open( ("/"+moduleName+"/"+partUsed+":i").c_str());

    while (1)
    {
        Bottle *b = input.read(false);
        if (b != NULL)
        {
            if (b->size() == axesCount)
            {
                cerr<<"Moving to "<<b->toString().c_str()<<endl;
                for(int i=0; i<axesCount;i++)
                {
                    pos->positionMove(i,b->get(i).asDouble());
                }
            }
            else
            {
                cerr<<"Wrong vector size..."<<endl;
            }
        }
    }

    return 0;
}



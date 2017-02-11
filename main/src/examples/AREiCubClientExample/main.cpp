/*
* Copyright(C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
* Authors: Ugo Pattacini
* email : ugo.pattacini@iit.it
* Permission is granted to copy, distribute, and / or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* wysiwyd / license / gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU General
* Public License for more details
*/

#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wysiwyd::wrdac;

int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP network seems unavailable!"<<endl;
        return -1;
    }

    ICubClient iCub("AREiCubClientExample","icubClient","example_ARE.ini");

    // we connect just to ARE (skip connecting to OPC)
    if (!iCub.connectSubSystems())
    {
        cout<<"ARE seems unavailabe!"<<endl;
        return -1;
    }

    RpcClient port;
    port.open("/create_object");
    if (!Network::connect(port.getName().c_str(),"/icubSim/world"))
    {
        cout<<"Unable to connect to the World!"<<endl;
        port.close();
        return -1;
    }

    // object location in the iCub frame
    Vector x(4);
    x[0]=-0.35;
    x[1]=-0.05;
    x[2]=-0.05;
    x[3]=1.0;

    // corresponding location in the world frame
    Matrix T(4,4);
    T(0,0)=0.0;  T(0,1)=-1.0; T(0,2)=0.0; T(0,3)=0.0;
    T(1,0)=0.0;  T(1,1)=0.0;  T(1,2)=1.0; T(1,3)=0.5976;
    T(2,0)=-1.0; T(2,1)=0.0;  T(2,2)=0.0; T(2,3)=-0.026;
    T(3,0)=0.0;  T(3,1)=0.0;  T(3,2)=0.0; T(3,3)=1.0;
    Vector wx=T*x;
    x.pop_back();

    // create a static object in the simulated world
    Bottle cmd,reply;
    double radius=0.025;
    cmd.addString("world"); cmd.addString("mk"); cmd.addString("ssph");
    cmd.addDouble(radius);
    cmd.addDouble(wx[0]); cmd.addDouble(wx[1]); cmd.addDouble(wx[2]);
    cmd.addDouble(1.0);   cmd.addDouble(0.0);   cmd.addDouble(0.0);
    port.write(cmd,reply);
    port.close();

    /*iCub.home();
    cout<<"pointing at the object ... "<<endl;
    iCub.point(x);      // automatic selection of the hand
    Time::delay(2.0);
    iCub.home();

    cout<<"try to grasp ... ";
    x[2]+=1.2*radius;
    Bottle options("right");     // force the use of the right hand
    bool ok=iCub.grasp(x,options);
    cout<<(ok?"grasped!":"missed!")<<endl;
    iCub.home();

    if (ok)
    {
        cout<<"releasing ... "<<endl;
        iCub.release(x);
        iCub.home();
    }*/

    cout<<"shutting down ... "<<endl;
    iCub.close();
    return 0;
}



/*
* Copyright(C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
* Authors: Tobias Fischer, Maxime Petit (Based on ARE/KARMA iCubClient examples of Ugo Pattacini and Nguyen Dong Hai Phuong)
* email : t.fischer@imperial.ac.uk, m.petit@imperial.ac.uk
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

    ICubClient iCub("ARE_KARMAiCubClientExample","icubClient","example_ARE_KARMA.ini");

    // we connect just to ARE (skip connecting to OPC)
    // we need connect to KARMA also
    if (!iCub.connectSubSystems())
    {
        cout<<"KARMA seems unavailabe!"<<endl;
        return -1;
    }

    // object location in the iCub frame
    Vector x(4);
    x[0]=-0.25;
    x[1]=-0.20;
    x[2]=-0.05;
    x[3]=1.0;

    // corresponding location in the world frame
    Matrix T(4,4);
    T(0,0)=0.0;  T(0,1)=-1.0; T(0,2)=0.0; T(0,3)=0.0;
    T(1,0)=0.0;  T(1,1)=0.0;  T(1,2)=1.0; T(1,3)=0.5976;
    T(2,0)=-1.0; T(2,1)=0.0;  T(2,2)=0.0; T(2,3)=-0.026;
    T(3,0)=0.0;  T(3,1)=0.0;  T(3,2)=0.0; T(3,3)=1.0;
    Vector wx=T*x; //if want to create the object in the simulator
    x.pop_back();

    bool ok ;

    /*iCub.home();    // Home by using ARE
    cout<<"try to pushFront with KARMA ..."<<endl;
    ok = iCub.pushKarmaFront(x,-0.35);                  //WARNING: second argument is the desired x[0] that we want and should be less than original x[0]
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    iCub.home();    // Home by using ARE
    x[1] =x[1] - 0.1;                        //offset on lateral to draw?
    cout<<"try to pull with KARMA...";
    ok = iCub.drawKarma(x,0,0.1,0.1);        //draw of 0.1m but the objects is considered as -0.35 so final: -0.25
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);
    x[1] = x[1] + 0.1; //go back to original value
    */


    //----------------------------> ARE related action
    iCub.home();     // Home by using ARE
    cout<<"try to point using ARE..."<<endl;
    ok = iCub.point(x);      // automatic selection of the hand
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    iCub.home();     // Home by using ARE
    cout<<"try to take from side using ARE..."<<endl;
    Bottle bOptions("side");     // param1: side
    ok = iCub.take(x, bOptions);     // automatic selection of the hand
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    iCub.home();     // Home by using ARE
    cout<<"try to take from above using ARE..."<<endl;
    bOptions.clear();
    bOptions.addString("above");
    ok = iCub.take(x, bOptions);     // automatic selection of the hand
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    iCub.home();     // Home by using ARE
    cout<<"try to push using ARE..."<<endl;
    ok = iCub.push(x);     // automatic selection of the hand
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    iCub.home();     // Home by using ARE
    cout<<"try to push away using ARE..."<<endl;
    bOptions.clear();
    bOptions.addString("away");
    ok = iCub.push(x,bOptions);     // automatic selection of the hand
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);
    iCub.home();

    cout<<"shutting down ... "<<endl;
    iCub.close();
    return 0;
}



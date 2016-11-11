/*
* Copyright(C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
* Authors: Nguyen Dong Hai Phuong
* email : phuong.nguyen@iit.it
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

    ICubClient iCub("KARMAiCubClientExample","icubClient","example_ARE_KARMA.ini");

    // we connect just to ARE (skip connecting to OPC)
    // we need connect to KARMA also
    if (!iCub.connectSubSystems())
    {
        cout<<"KARMA seems unavailabe!"<<endl;
        return -1;
    }

    // object location in the iCub frame
    Vector x(4);
    x[0]=-0.35;
    x[1]=0.05;
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

    iCub.home();    // Home by using ARE
    x[1] =x[1] - 0.1;
    cout<<"try to push with KARMA...";
    bool ok = iCub.pushKarma(x,180,0.2);
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    iCub.home();    // Home by using ARE
    cout<<"try to pushLeft with KARMA by right hand..."<<endl;
    ok = iCub.pushKarmaLeft(x,-.2,"right");
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    iCub.home();    // Home by using ARE
    cout<<"try to pushRight with KARMA by unknown hand..."<<endl;
    ok = iCub.pushKarmaRight(x,.2);
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    iCub.home();    // Home by using ARE
    x[0] = -0.25;
    x[1] = 0.1;
    cout<<"try to pushFront with KARMA ..."<<endl;
    ok = iCub.pushKarmaFront(x,-0.4);
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    iCub.home();    // Home by using ARE
    x[0] = -0.45;
    x[1] =  0.1;
    x[2] = -0.05;
    cout<<"try to pull with KARMA...";
    ok = iCub.drawKarma(x,0,0,0.2);
    cout<<(ok?"success":"failed")<<endl;
    Time::delay(4.0);

    cout<<"shutting down ... "<<endl;
    iCub.close();
    return 0;
}



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
        yError()<<"[KARMA_OPC_Example] YARP network seems unavailable!";
        return -1;
    }

    ICubClient iCub("KARMA_OPC_Example","icubClient","example_KARMA_OPC.ini");

    ResourceFinder rfClient;
    rfClient.setVerbose(true);
    rfClient.setDefaultContext("icubClient");
    rfClient.setDefaultConfigFile("example_KARMA_OPC.ini");
    rfClient.configure(0, NULL);

    string target;
    if (rfClient.check("target"))
    {
        target = rfClient.find("target").asString();
        yInfo("target name set to %s", target.c_str());
    }
    else
    {
        target = "octopus";
        yInfo("target name set to default, i.e. %s", target.c_str());
    }


    // we connect just to ARE (skip connecting to OPC)
    // we need connect to KARMA also
    if (!iCub.connectSubSystems())
    {
        yError()<<"[KARMA_OPC_Example] KARMA seems unavailabe!";
        return -1;
    }

    bool connected = iCub.connectOPC();
    yInfo()<<"connected to OPC port";
    yInfo()<<(connected?"success":"failed");
    bool ok = false;
    iCub.home();    // Home by using ARE

    yInfo()<<"[KARMA_OPC_Example] try to pushLeft target with KARMA ...";
    //iCub.look(target);
    ok = iCub.pushKarmaLeft(target,-0.2);
    yInfo()<<(ok?"success":"failed");
    Time::delay(1.0);
    iCub.home();    // Home by using ARE
    Time::delay(10.0);

    yInfo()<<"[KARMA_OPC_Example] try to pushRight target with KARMA ...";
    //iCub.look(target);
    ok = iCub.pushKarmaRight(target, 0.2);
    yInfo()<<(ok?"success":"failed");
    Time::delay(1.0);
    iCub.home();    // Home by using ARE
    Time::delay(10.0);

    yInfo()<<"[KARMA_OPC_Example] try to pushFront target with KARMA ...";
    //iCub.look(target);
    ok = iCub.pushKarmaFront(target,-0.45);
    yInfo()<<(ok?"success":"failed");
    Time::delay(1.0);
    iCub.home();    // Home by using ARE
    Time::delay(10.0);

    yInfo()<<"[KARMA_OPC_Example] try to pullBack target with KARMA ...";
    //iCub.look(target);
    ok = iCub.pullKarmaBack(target,-0.25);
    yInfo()<<(ok?"success":"failed");
    Time::delay(1.0);
    iCub.home();    // Home by using ARE
    Time::delay(10.0);

    yInfo()<<"[KARMA_OPC_Example] shutting down ... ";
    iCub.close();
    return 0;
}



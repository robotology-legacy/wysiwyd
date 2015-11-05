/*
* Copyright(C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <wrdac/clients/icubClient.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wysiwyd::wrdac;

int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP network seems unavailable!";
        return 1;
    }

    OPCClient world("test_are_attention/opc");
    if (!world.connect("OPC"))
    {
        yError()<<"OPC seems unavailabe!";
        return 1;
    }
    world.clear();

    ICubClient icub("test_are_attention","icubClient","test_are_attention.ini");
    if (!icub.connect())
    {
        yError()<<"One or more functionalities seem unavailabe!";
        world.close();
        return 1;
    }

    // attention starts up in auto mode => stop it
    icub.lookStop();

    Object *object=world.addOrRetrieveEntity<Object>("object");
    object->m_present=true;
    object->m_ego_position[0]=-0.35;
    object->m_ego_position[1]=-0.2;
    object->m_ego_position[2]=-0.1;
    world.commit(object);

    yInfo()<<"homing ... ";
    icub.home();

    yInfo()<<"looking at the object ... ";
    icub.look(object->name());
    Time::delay(3.0);
    icub.lookStop();

    yInfo()<<"pointing at the object ... ";
    icub.point(object->name());

    yInfo()<<"homing ... ";
    icub.home();

    yInfo()<<"shutting down ... ";
    icub.close();
    world.clear();
    world.close();

    return 0;
}



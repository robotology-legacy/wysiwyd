/*
* Copyright(C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
* Authors: Gregoire Pointeau
* email : gregoire.pointeau@inserm.fr
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

#include <csignal>
#include <vector>
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <wrdac/clients/icubClient.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

namespace
{
    volatile std::sig_atomic_t gSignalStatus;
}

void signal_handler(int signal)
{
    gSignalStatus=signal;
}


int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP network seems unavailable!";
        return 1;
    }

    OPCClient world("test_cartesian_sim/opc");
    if (!world.connect("OPC"))
    {
        yError()<<"OPC seems unavailabe!";
        return 1;
    }
    world.clear();

    ICubClient icub("test_cartesian_sim","icubClient","test_cartesian_sim.ini");
    if (!icub.connect())
    {
        yError()<<"One or more functionalities seem unavailabe!";
        world.close();
        return 1;
    }

    double X_obj = -0.4;
    double Y_obj = 0.5;
    double Z_obj = 0.1;
    double X_ag = -0.4;
    double Y_ag = 0.5;
    double Z_ag = 0.5;

    // populate OPC: begin
    icub.opc->clear();

    Object* obj1 = icub.opc->addOrRetrieveEntity<Object>("bottom_left");
    obj1->m_ego_position[0] = X_obj;
    obj1->m_ego_position[1] = -1.* Y_obj;
    obj1->m_ego_position[2] = Z_obj;
    obj1->m_present = 1;
    obj1->m_color[0] = Random::uniform(0, 80);
    obj1->m_color[1] = Random::uniform(80, 180);
    obj1->m_color[2] = Random::uniform(180, 250);

    Object* obj2 = icub.opc->addOrRetrieveEntity<Object>("top_left");
    obj2->m_ego_position[0] = X_ag;
    obj2->m_ego_position[1] = -1.* Y_ag;
    obj2->m_ego_position[2] = Z_ag;
    obj2->m_present = 1;
    obj2->m_color[0] = Random::uniform(0, 180);
    obj2->m_color[1] = Random::uniform(0, 80);
    obj2->m_color[2] = Random::uniform(180, 250);

    Object* obj3 = icub.opc->addOrRetrieveEntity<Object>("top_right");
    obj3->m_ego_position[0] = X_ag;
    obj3->m_ego_position[1] = Y_ag;
    obj3->m_ego_position[2] = Z_ag;
    obj3->m_present = 1;
    obj3->m_color[0] = Random::uniform(100, 180);
    obj3->m_color[1] = Random::uniform(80, 180);
    obj3->m_color[2] = Random::uniform(0, 80);

    Object* obj4 = icub.opc->addOrRetrieveEntity<Object>("bottom_right");
    obj4->m_ego_position[0] = X_obj;
    obj4->m_ego_position[1] = Y_obj;
    obj4->m_ego_position[2] = Z_obj;
    obj4->m_present = 1;
    obj4->m_color[0] = Random::uniform(100, 180);
    obj4->m_color[1] = Random::uniform(0, 80);
    obj4->m_color[2] = Random::uniform(180, 250);    

    vector<Object*> vObject;
    vObject.push_back(obj1);
    vObject.push_back(obj2);
    vObject.push_back(obj3);
    vObject.push_back(obj4);
    icub.opc->commit();
    // populate OPC: end

    // catch ctrl-c sequence
    std::signal(SIGINT,signal_handler);

    // loop pointing
    for (int i = 0; i < 5; i++)
    {
        for (vector<Object*>::iterator itOb = vObject.begin(); itOb != vObject.end(); itOb++)
        {
            string obj_name=(*itOb)->name();
            string obj_pos=(*itOb)->m_ego_position.toString(3,3).c_str();

            yInfo() << "Object " << obj_name << " is in position (" << obj_pos << ") [m]";

            if (gSignalStatus==SIGINT)
            {
                yWarning("SIGINT detected: closing ...");
                break;
            }

            yInfo() << "\t\t" << "looking at it for a while ...";
            icub.look(obj_name);
            Time::delay(2.0);

            if (gSignalStatus==SIGINT)
            {
                yWarning("SIGINT detected: closing ...");
                break;
            }

            yInfo() << "\t\t" << "pointing initiated";
            icub.point(obj_name);
            yInfo() << "\t\t" << "pointing complete";

            if (gSignalStatus==SIGINT)
            {
                yWarning("SIGINT detected: closing ...");
                break;
            }

            yInfo() << "\t\t" << "homing initiated";
            icub.home();
            yInfo() << "\t\t" << "homing complete";
        }

        if (gSignalStatus==SIGINT)
            break;
    }

    return 0;
}



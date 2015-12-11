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
using namespace std;

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
    yInfo()<<"stopping attention";
    icub.lookStop();
   
    double X_obj = -0.4;
    double Y_obj = 0.5;
    double Z_obj = 0.1;
    double X_ag = -0.4;
    double Y_ag = 0.5;
    double Z_ag = 0.5;


    double delayLook = 2.;
    double delayPoint = 1.;
    double delayHome = 1.5;

    int iLoop = 5;

    //POPULATE OPC:
    icub.opc->clear();

    Object* obj1 = icub.opc->addOrRetrieveEntity<Object>("bottom_left");
    obj1->m_ego_position[0] = X_obj;
    obj1->m_ego_position[1] = -1.* Y_obj;
    obj1->m_ego_position[2] = Z_obj;
    obj1->m_present = 1;
    obj1->m_color[0] = Random::uniform(0, 80);
    obj1->m_color[1] = Random::uniform(80, 180);
    obj1->m_color[2] = Random::uniform(180, 250);
    icub.opc->commit(obj1);

    Object* obj2 = icub.opc->addOrRetrieveEntity<Object>("top_left");
    obj2->m_ego_position[0] = X_ag;
    obj2->m_ego_position[1] = -1.* Y_ag;
    obj2->m_ego_position[2] = Z_ag;
    obj2->m_present = 1;
    obj2->m_color[0] = Random::uniform(0, 180);
    obj2->m_color[1] = Random::uniform(0, 80);
    obj2->m_color[2] = Random::uniform(180, 250);
    icub.opc->commit(obj2);

    Object* obj3 = icub.opc->addOrRetrieveEntity<Object>("top_right");
    obj3->m_ego_position[0] = X_ag;
    obj3->m_ego_position[1] = Y_ag;
    obj3->m_ego_position[2] = Z_ag;
    obj3->m_present = 1;
    obj3->m_color[0] = Random::uniform(100, 180);
    obj3->m_color[1] = Random::uniform(80, 180);
    obj3->m_color[2] = Random::uniform(0, 80);
    icub.opc->commit(obj3);


    Object* obj4 = icub.opc->addOrRetrieveEntity<Object>("bottom_right");
    obj4->m_ego_position[0] = X_obj;
    obj4->m_ego_position[1] = Y_obj;
    obj4->m_ego_position[2] = Z_obj;
    obj4->m_present = 1;
    obj4->m_color[0] = Random::uniform(100, 180);
    obj4->m_color[1] = Random::uniform(0, 80);
    obj4->m_color[2] = Random::uniform(180, 250);
    icub.opc->commit(obj4);


    vector<string>  vObject;
    vObject.push_back("bottom_right");
    vObject.push_back("bottom_left");
    vObject.push_back("top_right");
    vObject.push_back("top_left");


    // END POPULATING OPC

    //  BEGINNING LOOP POINTING
    for (int ii = 0; ii < iLoop; ii++){
        for (auto itOb = vObject.begin(); itOb != vObject.end(); itOb++){

            Object* obj1 = icub.opc->addOrRetrieveEntity<Object>(*itOb);
            string sHand;
            (obj1->m_ego_position[1] < 0) ? sHand = "left" : sHand = "right";
            Bottle bHand(sHand);

            cout << *itOb << ": " << obj1->m_ego_position.toString() << endl;


            bool bSuccess = icub.look(*itOb);
            cout << "\t\t\t" << "IN DELAY LOOK" << endl;

            Time::delay(delayLook);
            icub.lookStop();

            bSuccess &= icub.point(*itOb, bHand);
            cout << "\t\t\t" << "IN DELAY POINT" << endl;
            Time::delay(delayPoint);


            icub.getARE()->home();
            cout << "\t\t\t" << "IN DELAY HOME" << endl;
            Time::delay(delayHome);

        }

    }
    return 0;
}



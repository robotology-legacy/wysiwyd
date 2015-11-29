/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Magnus Johnsson and Zahra Gharaee
 * email:   magnus@magnusjohnsson.se zahra.gharaee@gmail.com
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

#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <wrdac/clients/icubClient.h>

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

class verbRec : public RFModule {
private:
        double period;
        Port Port_out; // a port to receive input
        Port Port_in; // a port for output

    float input[47];
    float output[11];

    int r_has_obj[4];
    int a_has_obj[4];

    int timer[4][2];    // temporary

    string objectNames[4];

    int nbrOfObj;

public:
        bool configure(yarp::os::ResourceFinder &rf);

        bool interruptModule()
        {
        cout<<"Interrupting the module verbRec, for port cleanup"<<endl;
            return true;
        }

        bool close();

        double getPeriod()
        {
            return period;
        }

    void setTimer(char ch, int obj);    // temporary

        bool updateModule();
        bool respond(const Bottle& cmd, Bottle& reply);

    void actionRec(const Bottle& command);
    void readData(const Bottle& command, float* input);

    void egoCenterTransformation(float obj[][3], float r_hand[][3], float l_hand[][3], float i_obj[][3], float i_rh[][3], float i_lh[][3]);
    bool gluInvertMatrix(float m[], float invm[]);

    bool wave(float r_hand[3], float l_hand[3], float prev_rh[3], float prev_lh[3]);
    bool move(float obj[], float prev_obj[]);
    bool push(float obj[][3], float prev_obj[], float r_hand[][3], float l_hand[][3]);
    bool pull(float obj[][3], float prev_obj[], float r_hand[][3], float l_hand[][3]);
    bool grasp(float obj[], float prev_obj[], float r_hand[], float l_hand[]);
    bool have(float obj[], int objNbr, float spine[]);
    bool give(float obj[], float prev_obj[], float spine[], float prev_sp[]);
    bool take(float obj[], float prev_obj[], float spine[], float prev_sp[]);
    bool put(float pres_obj[]);
    bool lift(float pres_obj[]);
    bool point(float obj[][3], float prev_obj[][3], float r_arm[][3], float l_arm[][3], float prev_rh[], float prev_lh[],float fac[]);
};

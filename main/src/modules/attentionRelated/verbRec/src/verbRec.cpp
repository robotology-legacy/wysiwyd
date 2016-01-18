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


#include "verbRec.h"

bool 
verbRec::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("verbRec")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    for (int i=0; i<11; i++)
        output[i] = 0;

    //temporary
    for (int i=0; i<4; i++)
        for (int j=0; j<2; j++)
            timer[i][j] = 0;

    Port_rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(Port_rpc);

    return true;
}


bool 
verbRec::close() 
{
    /* optional, close port explicitly */
    cout<<"Calling close function\n";
    Port_rpc.close();
        
    return true;
}


bool 
verbRec::respond(const Bottle& command, Bottle& reply) 
{
        // prepare a message
        Bottle botWrite; 

        actionRec(command);

        // have for Robot/Agent
        if (output[5]) { // at least one of the agent or the robot has objects
                char str_a[50];
                char str_r[50];
                strcpy(str_a,"");   
                strcpy(str_r,"");

        // the agent has at least one object ...
        for (int i=0; i< nbrOfObj; i++)
            if (a_has_obj[i]) {
                strcat(str_a,"Agent has ");
                break;
            }
                // the agent has the objects ...
        for (int i=0; i< nbrOfObj; i++)
                    if (a_has_obj[i]) {
                        strcat(str_a, objectNames[i].c_str());
                            strcat(str_a, " ");
                    }

        // the robot has at least one object ...
        for (int i=0; i< nbrOfObj; i++)
            if (r_has_obj[i]) {
                strcat(str_r,"Robot has ");
                break;
            }
                // the robot has the objects ...
        for (int i=0; i< nbrOfObj; i++)
                    if (r_has_obj[i]) {
                        strcat(str_r, objectNames[i].c_str());
                            strcat(str_r, " ");
                    }

                char str2[100];     
                strcpy(str2,str_a);     
                botWrite.addString(strcat(str2,str_r));
        }        

    // the agent is waving
        if (output[0])
            botWrite.addString("Agent waving");
        
        char str[50];

    for (int i=0; i<nbrOfObj; i++) {
        if (output[1] == i+1) { // object i+1 moving
            strcpy(str, objectNames[i].c_str());
            botWrite.addString(strcat(str, " moving"));
        }
        strcpy(str,"Agent pushes ");    // the agent pushes object i+1
        if (output[2] == i+1)
                botWrite.addString(strcat(str, objectNames[i].c_str()));
        strcpy(str,"Agent pulls "); // the agent pulls object i+1
        if (output[3] == i+1)
                botWrite.addString(strcat(str, objectNames[i].c_str()));
        strcpy(str,"Agent grasps ");    // the agent grasps object i+1
        if (output[4] == i+1)
                botWrite.addString(strcat(str, objectNames[i].c_str()));
        strcpy(str,"Agent gives "); // the agent gives object i+1
        if (output[6] == i+1)
            botWrite.addString(strcat(str, objectNames[i].c_str()));
        strcpy(str,"Agent takes "); // the agent takes object i+1
        if (output[7] == i+1)
                botWrite.addString(strcat(str, objectNames[i].c_str()));
        strcpy(str,"Agent puts ");  // the agent puts object i+1
        if (output[8] == i+1)
                botWrite.addString(strcat(str, objectNames[i].c_str()));
        strcpy(str,"Agent lifts "); // the agent lifts object i+1
        if (output[9] == i+1)
                botWrite.addString(strcat(str, objectNames[i].c_str()));
        strcpy(str,"Agent points at "); // the agent points at object i+1
        if (output[10] == i+1)
                botWrite.addString(strcat(str, objectNames[i].c_str()));
    }

        yInfo() << "\t\t\t" << botWrite.toString();

        // send the message
        reply=botWrite;
        return true;
}

/* Called periodically every getPeriod() seconds */
bool verbRec::updateModule() 
{
        cout<< " updateModule... "<<endl;
        return true;
}

void 
verbRec::actionRec(const Bottle& command) 
{
    // reset output
        for (int i=0; i<11; i++)
            output[i] = 0;

    // temporary
    for (int i=0; i<4; i++)
        if (timer[i][0] > 0) {
            output[i+6] = (float)timer[i][1];
            timer[i][0]--;
        }

        readData(command, input);

        // preprocess input, smoothing etc
        static float spine[5][3] = { {input[24], input[25], input[26]}, {input[24], input[25], input[26]}, {input[24], input[25], input[26]},
            {input[24], input[25], input[26]}, {input[24], input[25], input[26]} };
    
        static float r_shoulder[5][3] = { {input[21], input[22], input[23]}, {input[21], input[22], input[23]}, {input[21], input[22], input[23]},
            {input[21], input[22], input[23]}, {input[21], input[22], input[23]} };
    
        static float l_shoulder[5][3] = { {input[18], input[19], input[20]}, {input[18], input[19], input[20]}, {input[18], input[19], input[20]},
            {input[18], input[19], input[20]}, {input[18], input[19], input[20]} };
    
        static float r_elbow[5][3] = { {input[3], input[4], input[5]}, {input[3], input[4], input[5]}, {input[3], input[4], input[5]},
            {input[3], input[4], input[5]}, {input[3], input[4], input[5]} };
    
        static float l_elbow[5][3] = { {input[0], input[1], input[2]}, {input[0], input[1], input[2]}, {input[0], input[1], input[2]},
            {input[0], input[1], input[2]}, {input[0], input[1], input[2]} };
    
        static float r_hand[5][3] = { {input[9], input[10], input[11]}, {input[9], input[10], input[11]}, {input[9], input[10], input[11]},
            {input[9], input[10], input[11]}, {input[9], input[10], input[11]} };
    
        static float l_hand[5][3] = { {input[6], input[7], input[8]}, {input[6], input[7], input[8]}, {input[6], input[7], input[8]},
            {input[6], input[7], input[8]}, {input[6], input[7], input[8]} };

    static float objs[4][5][3] = {{ {input[31], input[32], input[33]}, {input[31], input[32], input[33]}, {input[31], input[32], input[33]},
            {input[31], input[32], input[33]}, {input[31], input[32], input[33]} }, { {input[35], input[36], input[37]}, {input[35], input[36], input[37]}, {input[35], input[36], input[37]},
            {input[35], input[36], input[37]}, {input[35], input[36], input[37]} }, { {input[39], input[40], input[41]}, {input[39], input[40], input[41]}, {input[39], input[40], input[41]},
            {input[39], input[40], input[41]}, {input[39], input[40], input[41]} }, { {input[43], input[44], input[45]}, {input[43], input[44], input[45]}, {input[43], input[44], input[45]},
            {input[43], input[44], input[45]}, {input[43], input[44], input[45]} }};

    static float prev_objs[4][3] = {{input[31], input[32], input[33]}, {input[35], input[36], input[37]}, {input[39], input[40], input[41]}, {input[43], input[44], input[45]}};
    
        static float prev_sp[3] = {input[24], input[25], input[26]};
    
        static float prev_rh[3] = {input[9], input[10], input[11]};
        static float prev_lh[3] = {input[6], input[7], input[8]};
    
    static float pres_objs[4][10] = {{input[34], input[34],input[34], input[34], input[34], input[34], input[34],input[34], input[34], input[34]}, {input[38], input[38],input[38], input[38], input[38], input[38], input[38],input[38], input[38], input[38]}, {input[42], input[42],input[42], input[42], input[42], input[42], input[42],input[42], input[42], input[42]}, {input[46], input[46],input[46], input[46], input[46], input[46], input[46],input[46], input[46], input[46]}};
    
        for (int i=0; i<9; i++) {
            pres_objs[0][i] = pres_objs[0][i+1];
            pres_objs[1][i] = pres_objs[1][i+1];
            pres_objs[2][i] = pres_objs[2][i+1];
            pres_objs[3][i] = pres_objs[3][i+1];
        }
        pres_objs[0][9] = input[34];
        pres_objs[1][9] = input[38];
        pres_objs[2][9] = input[42];
        pres_objs[3][9] = input[46];
    
        for (int i=0; i<4; i++) {
            spine[i][0] = spine[i+1][0];            spine[i][1] = spine[i+1][1];            spine[i][2] = spine[i+1][2];
        
            r_hand[i][0] = r_hand[i+1][0];          r_hand[i][1] = r_hand[i+1][1];          r_hand[i][2] = r_hand[i+1][2];
            l_hand[i][0] = l_hand[i+1][0];          l_hand[i][1] = l_hand[i+1][1];          l_hand[i][2] = l_hand[i+1][2];
        
            r_shoulder[i][0] = r_shoulder[i+1][0];  r_shoulder[i][1] = r_shoulder[i+1][1];  r_shoulder[i][2] = r_shoulder[i+1][2];
            l_shoulder[i][0] = l_shoulder[i+1][0];  l_shoulder[i][1] = l_shoulder[i+1][1];  l_shoulder[i][2] = l_shoulder[i+1][2];
        
            r_elbow[i][0] = r_elbow[i+1][0];        r_elbow[i][1] = r_elbow[i+1][1];        r_elbow[i][2] = r_elbow[i+1][2];
            l_elbow[i][0] = l_elbow[i+1][0];        l_elbow[i][1] = l_elbow[i+1][1];        l_elbow[i][2] = l_elbow[i+1][2];   
    }

        spine[4][0] = input[24];        spine[4][1] = input[25];        spine[4][2] = input[26];
    
        l_elbow[4][0] = input[0];       l_elbow[4][1] = input[1];       l_elbow[4][2] = input[2];
        r_elbow[4][0] = input[3];       r_elbow[4][1] = input[4];       r_elbow[4][2] = input[5];
    
        l_hand[4][0] = input[6];        l_hand[4][1] = input[7];        l_hand[4][2] = input[8];
        r_hand[4][0] = input[9];        r_hand[4][1] = input[10];       r_hand[4][2] = input[11];
    
        l_shoulder[4][0] = input[18];   l_shoulder[4][1] = input[19];   l_shoulder[4][2] = input[20];
        r_shoulder[4][0] = input[21];   r_shoulder[4][1] = input[22];   r_shoulder[4][2] = input[23];
    
        float msp[3]={0, 0, 0};
    
        float mrh[3]={0, 0, 0};
        float mlh[3]={0, 0, 0};
    
        float mrs[3]={0, 0, 0};
        float mls[3]={0, 0, 0};
    
        float mre[3]={0, 0, 0};
        float mle[3]={0, 0, 0};
    
        for (int i=0; i<5; i++) {
        
            msp[0] += spine[i][0];   msp[1] += spine[i][1];   msp[2] += spine[i][2];  // X:[i][0] Y[i][1] Z[i][2]
        
            mrh[0] += r_hand[i][0];  mrh[1] += r_hand[i][1];  mrh[2] += r_hand[i][2];  // X:[i][0] Y[i][1] Z[i][2]
            mlh[0] += l_hand[i][0];  mlh[1] += l_hand[i][1];  mlh[2] += l_hand[i][2];
        
            mrs[0] += r_shoulder[i][0];  mrs[1] += r_shoulder[i][1];  mrs[2] += r_shoulder[i][2];
            mls[0] += l_shoulder[i][0];  mls[1] += l_shoulder[i][1];  mls[2] += l_shoulder[i][2];
        
            mre[0] += r_elbow[i][0];  mre[1] += r_elbow[i][1];  mre[2] += r_elbow[i][2];
            mle[0] += l_elbow[i][0];  mle[1] += l_elbow[i][1];  mle[2] += l_elbow[i][2];  
    }
    
        msp[0] = msp[0]/5;  msp[1] = msp[1]/5;  msp[2] = msp[2]/5;
    
        mrh[0] = mrh[0]/5;  mrh[1] = mrh[1]/5;  mrh[2] = mrh[2]/5;
        mlh[0] = mlh[0]/5;  mlh[1] = mlh[1]/5;  mlh[2] = mlh[2]/5;
    
        mrs[0] = mrs[0]/5;  mrs[1] = mrs[1]/5;  mrs[2] = mrs[2]/5;
        mls[0] = mls[0]/5;  mls[1] = mls[1]/5;  mls[2] = mls[2]/5;
    
        mre[0] = mre[0]/5;  mre[1] = mre[1]/5;  mre[2] = mre[2]/5;
        mle[0] = mle[0]/5;  mle[1] = mle[1]/5;  mle[2] = mle[2]/5;
    
        float r_arm[3][3] = {{mrs[0],mrs[1],mrs[2]},{mre[0],mre[1],mre[2]},{mrh[0],mrh[1],mrh[2]}};
        float l_arm[3][3] = {{mls[0],mls[1],mls[2]},{mle[0],mle[1],mle[2]},{mlh[0],mlh[1],mlh[2]}};

    float mObjs[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

        // reset vectors recording what objects the robot and the agent have
        for (int i=0; i<4; i++) {
            r_has_obj[i] = 0;
            a_has_obj[i] = 0;
        }

    /************* verb recognition ****************/
        if (wave(mrh, mlh, prev_rh, prev_lh))
                output[0]=1;

        /************************** OBJECT(k+1) **************************/
    for (int k=0; k<nbrOfObj; k++) {

            if ((pres_objs[k][9] == 1) && (pres_objs[k][8] == 1)) // object k+1 is and was on reactable
            {
                for (int i=0; i<4; i++)
                for (int j=0; j<3; j++)
                        objs[k][i][j] = objs[k][i+1][j];

            for (int j=0; j<3; j++)
                objs[k][4][j] = input[4*k+(j+31)];
            
            for (int i=0; i<5; i++)
                for (int j=0; j<3; j++)
                        mObjs[k][j] += objs[k][i][j];
     
            for (int j=0; j<3; j++)
                    mObjs[k][j] = mObjs[k][j]/5;
        
                if (move(mObjs[k],prev_objs[k]))
                    output[1]=(float)(k+1);
     
                if (push(objs[k], prev_objs[k], r_hand, l_hand))
                    output[2]=(float)(k+1);
     
                if (pull(objs[k], prev_objs[k], r_hand, l_hand))
                    output[3]=(float)(k+1);
     
                if (grasp(mObjs[k], prev_objs[k], mrh, mlh))
                    output[4]=(float)(k+1);

            if (have(mObjs[k], k, msp))
                output[5] = 1;

                if (give(mObjs[k], prev_objs[k], msp, prev_sp)) {
                    output[6]=(float)(k+1);
                setTimer('g',k+1);  // temporary
            }
        
                if (take(mObjs[k], prev_objs[k], msp, prev_sp)) {
                    output[7]=(float)(k+1);
                setTimer('t',k+1);  // temporary
            }
     
                if (put(pres_objs[k])) {
                    output[8]=(float)(k+1);
                setTimer('p',k+1);  // temporary
            }
            
            for (int j=0; j<3; j++)
                    prev_objs[k][j] = mObjs[k][j];
            }
     
            else if ((pres_objs[k][9] == 1) && (pres_objs[k][8] == 0)) // object k+1 is but was NOT on reactable
            {
                for (int i=0; i<5; i++)
                for (int j=0; j<3; j++)
                        objs[k][i][j] = input[4*k+(j+31)];

            for (int j=0; j<3; j++) {
                    prev_objs[k][j] = input[4*k+(j+31)];
                    mObjs[k][j] = input[4*k+(j+31)];
            }
        }

        else
            if (lift(pres_objs[k])) {
                    output[9]=(float)(k+1);
                setTimer('l',k+1);  // temporary
            }
    }

        /************** POINT ***************/
    for (int k=0; k<nbrOfObj; k++) {        
            if (pres_objs[k][9] == 0)     //object k+1 is not on reactable
                mObjs[k][0] = -1000;
    }
        
        float fac[1] = {0};
        if (point(mObjs, prev_objs, r_arm, l_arm, prev_rh, prev_lh, fac))
            output[10]=fac[0];
    
    for (int j=0; j<3; j++) {
            prev_rh[j] = mrh[j];
            prev_lh[j] = mlh[j];
            prev_sp[j] = msp[j];
    }
}


void 
verbRec::setTimer(char ch, int obj) // temporary
{
    if (ch == 'g') {
        timer[0][0] = 20;
        timer[0][1] = obj;
    }
    if (ch == 't') {
        timer[1][0] = 20;
        timer[1][1] = obj;
    }
    if (ch == 'p') {
        timer[2][0] = 20;
        timer[2][1] = obj;
    }
    if (ch == 'l') {
        timer[3][0] = 20;
        timer[3][1] = obj;
    }
}


void 
verbRec::readData(const Bottle& command, float* input) 
{
        Bottle bSkeleton = *command.get(2).asList();
        Bottle bPartner = *command.get(3).asList();
        nbrOfObj = command.size()-5;

        for (int i=0; i<47; i++)    // -10000 means no data for the element
            input[i] = -10000;  

        for (int i=0; i<bSkeleton.size(); i++) {
            string str = (*bSkeleton.get(i).asList()).get(0).toString().c_str();
            if (str == "elbowLeft") {
                input[0] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(0).toString().c_str());
                input[1] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(1).toString().c_str());
                input[2] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(2).toString().c_str());
            }
            if (str == "elbowRight") {
                    input[3] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(0).toString().c_str());
                    input[4] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(1).toString().c_str());
                    input[5] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(2).toString().c_str());
            }
        if (str == "handLeft") {
                input[6] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(0).toString().c_str());
                input[7] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(1).toString().c_str());
                input[8] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(2).toString().c_str());
        }
        if (str == "handRight") {
                input[9] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(0).toString().c_str());
                input[10] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(1).toString().c_str());
                input[11] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(2).toString().c_str());
        }
        if (str == "head") {
                input[12] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(0).toString().c_str());
                input[13] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(1).toString().c_str());
                input[14] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(2).toString().c_str());
        }
        if (str == "shoulderCenter") {
                input[15] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(0).toString().c_str());
                input[16] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(1).toString().c_str());
                input[17] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(2).toString().c_str());
        }
        if (str == "shoulderLeft") {
                input[18] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(0).toString().c_str());
                input[19] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(1).toString().c_str());
                input[20] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(2).toString().c_str());
        }
        if (str == "shoulderRight") {
                input[21] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(0).toString().c_str());
                input[22] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(1).toString().c_str());
                input[23] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(2).toString().c_str());
        }
        if (str == "spine") {
                input[24] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(0).toString().c_str());
                input[25] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(1).toString().c_str());
                input[26] = (float)atof((*(*bSkeleton.get(i).asList()).get(1).asList()).get(2).toString().c_str());
        }
        }

        if (bPartner.get(0).toString() == "partner") {
            input[27] = (float)atof(bPartner.get(1).toString().c_str());
            input[28] = (float)atof(bPartner.get(2).toString().c_str());
            input[29] = (float)atof(bPartner.get(3).toString().c_str());
            input[30] = (float)atof(bPartner.get(4).toString().c_str());
        }

        // The number of objects can vary between 0 and 4. Objects 5, 6, ..., n will be ignored if present.
    if (nbrOfObj > 4)
        nbrOfObj = 4;

        for (int i=0; i<nbrOfObj; i++) {
            objectNames[i] = (*command.get(i+4).asList()).get(0).toString();
            input[31+4*i] = (float)atof((*command.get(i+4).asList()).get(1).toString().c_str());
            input[32+4*i] = (float)atof((*command.get(i+4).asList()).get(2).toString().c_str());
            input[33+4*i] = (float)atof((*command.get(i+4).asList()).get(3).toString().c_str());
            input[34+4*i] = (float)atof((*command.get(i+4).asList()).get(4).toString().c_str());
        }
}

void
verbRec::egoCenterTransformation(float obj[][3], float r_hand[][3], float l_hand[][3], float i_obj[][3], float i_rh[][3], float i_lh[][3])
{
        float n_y_axis[3] = {0, 1, 0};
    
        // Shoulder axis of agent: Y-axis of new Coordinate System:
        float lsh[3] = {input[18], input[19], 0}; // Left Shoulder
        float rsh[3] = {input[21], input[22], 0}; // Right Shoulder
        float n_lr_shoulder = sqrt(pow((rsh[0]-lsh[0]),2)+pow((rsh[1]-lsh[1]),2)+pow((rsh[2]-lsh[2]),2));
    
        float v_lr_shoulder[3] = {0, 0, 0};
        v_lr_shoulder[0] = (rsh[0]-lsh[0])/n_lr_shoulder;
        v_lr_shoulder[1] = (rsh[1]-lsh[1])/n_lr_shoulder;
        v_lr_shoulder[2] = (rsh[2]-lsh[2])/n_lr_shoulder;

        // dot product
        float rd = v_lr_shoulder[0]*n_y_axis[0]+v_lr_shoulder[1]*n_y_axis[1]+v_lr_shoulder[2]*n_y_axis[2];
    
        // cross product
        float rc[3] = {0, 0, 0};
        rc[0] = v_lr_shoulder[1]*n_y_axis[2]-v_lr_shoulder[2]*n_y_axis[1];
        rc[1] = v_lr_shoulder[2]*n_y_axis[0]-v_lr_shoulder[0]*n_y_axis[2];
        rc[2] = v_lr_shoulder[0]*n_y_axis[1]-v_lr_shoulder[1]*n_y_axis[0];
    
        float n_rc = sqrt(pow(rc[0],2)+pow(rc[1],2)+pow(rc[2],2));
    
        // angles between vectors
        float theta = atan2(n_rc,rd);
    
        // Transformation Matrix (Rotation/Translation)
        float T[16]  = {cos(theta), -sin(theta), 0, input[15], sin(theta),  cos(theta), 0, input[16], 0,0,1, input[17], 0, 0, 0, 1};
        float iT[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
        gluInvertMatrix(T,iT);
    
        for (int i=0; i<5; i++) {
            i_obj[i][0] = iT[0]*obj[i][0] + iT[1]*obj[i][1] + iT[2]*obj[i][2] + iT[3];
            i_obj[i][1] = iT[4]*obj[i][0] + iT[5]*obj[i][1] + iT[6]*obj[i][2] + iT[7];
            i_obj[i][2] = iT[8]*obj[i][0] + iT[9]*obj[i][1] + iT[10]*obj[i][2]+ iT[11];
        
            i_rh[i][0] = iT[0]*r_hand[i][0] + iT[1]*r_hand[i][1] + iT[2]*r_hand[i][2] + iT[3];
            i_rh[i][1] = iT[4]*r_hand[i][0] + iT[5]*r_hand[i][1] + iT[6]*r_hand[i][2] + iT[7];
            i_rh[i][2] = iT[8]*r_hand[i][0] + iT[9]*r_hand[i][1] + iT[10]*r_hand[i][2]+ iT[11];

            i_lh[i][0] = iT[0]*l_hand[i][0] + iT[1]*l_hand[i][1] + iT[2]*l_hand[i][2] + iT[3];
            i_lh[i][1] = iT[4]*l_hand[i][0] + iT[5]*l_hand[i][1] + iT[6]*l_hand[i][2] + iT[7];
            i_lh[i][2] = iT[8]*l_hand[i][0] + iT[9]*l_hand[i][1] + iT[10]*l_hand[i][2]+ iT[11];  
    }    
}

bool
verbRec::gluInvertMatrix(float m[], float invm[])
{
        double inv[16], det;
        int i;
    
        inv[0] = m[5]  * m[10] * m[15] -
        m[5]  * m[11] * m[14] -
        m[9]  * m[6]  * m[15] +
        m[9]  * m[7]  * m[14] +
        m[13] * m[6]  * m[11] -
        m[13] * m[7]  * m[10];
    
        inv[4] = -m[4]  * m[10] * m[15] +
        m[4]  * m[11] * m[14] +
        m[8]  * m[6]  * m[15] -
        m[8]  * m[7]  * m[14] -
        m[12] * m[6]  * m[11] +
        m[12] * m[7]  * m[10];
    
        inv[8] = m[4]  * m[9] * m[15] -
        m[4]  * m[11] * m[13] -
        m[8]  * m[5] * m[15] +
        m[8]  * m[7] * m[13] +
        m[12] * m[5] * m[11] -
        m[12] * m[7] * m[9];
    
        inv[12] = -m[4]  * m[9] * m[14] +
        m[4]  * m[10] * m[13] +
        m[8]  * m[5] * m[14] -
        m[8]  * m[6] * m[13] -
        m[12] * m[5] * m[10] +
        m[12] * m[6] * m[9];
    
        inv[1] = -m[1]  * m[10] * m[15] +
        m[1]  * m[11] * m[14] +
        m[9]  * m[2] * m[15] -
        m[9]  * m[3] * m[14] -
        m[13] * m[2] * m[11] +
        m[13] * m[3] * m[10];
    
        inv[5] = m[0]  * m[10] * m[15] -
        m[0]  * m[11] * m[14] -
        m[8]  * m[2] * m[15] +
        m[8]  * m[3] * m[14] +
        m[12] * m[2] * m[11] -
        m[12] * m[3] * m[10];
    
        inv[9] = -m[0]  * m[9] * m[15] +
        m[0]  * m[11] * m[13] +
        m[8]  * m[1] * m[15] -
        m[8]  * m[3] * m[13] -
        m[12] * m[1] * m[11] +
        m[12] * m[3] * m[9];
    
        inv[13] = m[0]  * m[9] * m[14] -
        m[0]  * m[10] * m[13] -
        m[8]  * m[1] * m[14] +
        m[8]  * m[2] * m[13] +
        m[12] * m[1] * m[10] -
        m[12] * m[2] * m[9];
    
        inv[2] = m[1]  * m[6] * m[15] -
        m[1]  * m[7] * m[14] -
        m[5]  * m[2] * m[15] +
        m[5]  * m[3] * m[14] +
        m[13] * m[2] * m[7] -
        m[13] * m[3] * m[6];
    
        inv[6] = -m[0]  * m[6] * m[15] +
        m[0]  * m[7] * m[14] +
        m[4]  * m[2] * m[15] -
        m[4]  * m[3] * m[14] -
        m[12] * m[2] * m[7] +
        m[12] * m[3] * m[6];
    
        inv[10] = m[0]  * m[5] * m[15] -
        m[0]  * m[7] * m[13] -
        m[4]  * m[1] * m[15] +
        m[4]  * m[3] * m[13] +
        m[12] * m[1] * m[7] -
        m[12] * m[3] * m[5];
    
        inv[14] = -m[0]  * m[5] * m[14] +
        m[0]  * m[6] * m[13] +
        m[4]  * m[1] * m[14] -
        m[4]  * m[2] * m[13] -
        m[12] * m[1] * m[6] +
        m[12] * m[2] * m[5];
    
        inv[3] = -m[1] * m[6] * m[11] +
        m[1] * m[7] * m[10] +
        m[5] * m[2] * m[11] -
        m[5] * m[3] * m[10] -
        m[9] * m[2] * m[7] +
        m[9] * m[3] * m[6];
    
        inv[7] = m[0] * m[6] * m[11] -
        m[0] * m[7] * m[10] -
        m[4] * m[2] * m[11] +
        m[4] * m[3] * m[10] +
        m[8] * m[2] * m[7] -
        m[8] * m[3] * m[6];
    
        inv[11] = -m[0] * m[5] * m[11] +
        m[0] * m[7] * m[9] +
        m[4] * m[1] * m[11] -
        m[4] * m[3] * m[9] -
        m[8] * m[1] * m[7] +
        m[8] * m[3] * m[5];
    
        inv[15] = m[0] * m[5] * m[10] -
        m[0] * m[6] * m[9] -
        m[4] * m[1] * m[10] +
        m[4] * m[2] * m[9] +
        m[8] * m[1] * m[6] -
        m[8] * m[2] * m[5];
        
        det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
        
        if (det == 0)
            return false;
    
        det = 1.0 / det;
    
        for (i = 0; i < 16; i++)
            invm[i] = inv[i] * det;
    
        return true;
}

/**************** functions for verb recognition *****************/

bool
verbRec::wave(float r_hand[3], float l_hand[3], float prev_rh[3], float prev_lh[3])
{
        static bool obj_wave = false;
    static float mov_thr = 1.0f;    
    
        float drh = 100*sqrt(pow((r_hand[0]-prev_rh[0]),2)+pow((r_hand[1]-prev_rh[1]),2)+pow((r_hand[2]-prev_rh[2]),2));
        float dlh = 100*sqrt(pow((l_hand[0]-prev_lh[0]),2)+pow((l_hand[1]-prev_lh[1]),2)+pow((l_hand[2]-prev_lh[2]),2));
    
        if ( ((drh > mov_thr) && (r_hand[2] > input[14])) || ((dlh > mov_thr) && (l_hand[2] > input[14])) )
            obj_wave = true;
        else
            obj_wave = false;
    
        return obj_wave;
}

bool
verbRec::move(float obj[], float prev_obj[])
{ 
        static bool  obj_move = false;
        static float mov_thr = 0.1f;
    
        float dobj = 100.*sqrt(pow((obj[0]-prev_obj[0]),2)+pow((obj[1]-prev_obj[1]),2)+pow((obj[2]-prev_obj[2]),2));
    
        if (dobj > mov_thr)
            obj_move = true;
        else
            obj_move = false;
    
        return obj_move;
}

bool
verbRec::push(float obj[][3], float prev_obj[], float r_hand[][3], float l_hand[][3])
{    
        static bool obj_push = false;
        static float dis_thr = 0.1f;
    
        float i_obj[5][3] = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};
        float i_rh[5][3]  = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};
        float i_lh[5][3]  = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};
    
        float mrh[3]  = {0, 0, 0};
        float mlh[3]  = {0, 0, 0};
        float mobj[3] = {0, 0, 0};
    
        egoCenterTransformation(obj,r_hand,l_hand,i_obj, i_rh, i_lh);
    
        for (int i=0; i<4; i++) {
            mrh[0] += i_rh[i][0];
            mrh[1] += i_rh[i][1];
            mrh[2] += i_rh[i][2];
        
            mlh[0] += i_lh[i][0];
            mlh[1] += i_lh[i][1];
            mlh[2] += i_lh[i][2];
        
            mobj[0] += i_obj[i][0];
        mobj[1] += i_obj[i][1];
            mobj[2] += i_obj[i][2];   
    }
    
        mrh[0] = mrh[0]/5;     mrh[1] = mrh[1]/5;     mrh[2] = mrh[2]/5;
        mlh[0] = mlh[0]/5;     mlh[1] = mlh[1]/5;     mlh[2] = mlh[2]/5;
        mobj[0] = mobj[0]/5;   mobj[1] = mobj[1]/5;   mobj[2] = mobj[2]/5;
    
        float dobj = sqrt(pow((mobj[0]-prev_obj[0]),2)+pow((mobj[1]-prev_obj[1]),2)+pow((mobj[2]-prev_obj[2]),2));

        if ( (dobj > dis_thr) && ((i_obj[4][0]-i_obj[0][0]) < 0 ) && ((i_obj[4][1]-i_obj[0][1]) < 0) )
            obj_push = true;
        else
            obj_push = false;
    
        return obj_push;
}

bool
verbRec::pull(float obj[][3], float prev_obj[], float r_hand[][3], float l_hand[][3])
{    
        static bool obj_pull = false;
        static float dis_thr = 0.1f;
    
        float i_obj[5][3] = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};
        float i_rh[5][3]  = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};
        float i_lh[5][3]  = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};
        
        float mrh[3] = {0, 0, 0};
        float mlh[3] = {0, 0, 0};
        float mobj[3] = {0, 0, 0};
    
        egoCenterTransformation(obj,r_hand,l_hand,i_obj, i_rh, i_lh);
    
        for (int i=0; i<4; i++) {
            mrh[0] += i_rh[i][0];
            mrh[1] += i_rh[i][1];
            mrh[2] += i_rh[i][2];
            
            mlh[0] += i_lh[i][0];
            mlh[1] += i_lh[i][1];
            mlh[2] += i_lh[i][2];
            
            mobj[0] += i_obj[i][0];
            mobj[1] += i_obj[i][1];
            mobj[2] += i_obj[i][2];   
    }
    
        mrh[0] = mrh[0]/5;     mrh[1] = mrh[1]/5;     mrh[2] = mrh[2]/5;
        mlh[0] = mlh[0]/5;     mlh[1] = mlh[1]/5;     mlh[2] = mlh[2]/5;
        mobj[0] = mobj[0]/5;   mobj[1] = mobj[1]/5;   mobj[2] = mobj[2]/5;
    
        float dobj = sqrt(pow((mobj[0]-prev_obj[0]),2)+pow((mobj[1]-prev_obj[1]),2)+pow((mobj[2]-prev_obj[2]),2));

        if ( (dobj > dis_thr) && ((i_obj[4][0]-i_obj[0][0]) > 0 ) && ((i_obj[4][1]-i_obj[0][1]) > 0) )
            obj_pull = true;
        else
            obj_pull = false;
    
        return obj_pull;
}

bool
verbRec::grasp(float obj[], float prev_obj[], float r_hand[], float l_hand[])
{
        static bool obj_grasp = false;
        static float dis_thr = 0.5f;    
    
        float drh = sqrt(pow((r_hand[0]-obj[0]),2)+pow((r_hand[1]-obj[1]),2)+pow((r_hand[2]-obj[2]),2));
        float dlh = sqrt(pow((l_hand[0]-obj[0]),2)+pow((l_hand[1]-obj[1]),2)+pow((l_hand[2]-obj[2]),2));
        float dobj = sqrt(pow((obj[0]-prev_obj[0]),2)+pow((obj[1]-prev_obj[1]),2)+pow((obj[2]-prev_obj[2]),2));

        if ((dobj == 0) && ((drh < dis_thr) || (dlh < dis_thr)))
            obj_grasp = true;
        else
            obj_grasp = false;
    
        return obj_grasp;
}

bool 
verbRec::have(float obj[], int objNbr, float spine[])
{
        static float dis_thr = 0.5f;

        if (sqrt(pow(obj[0],2)+pow(obj[1],2)+pow(obj[2],2)) < dis_thr ) {
        r_has_obj[objNbr] = 1;      
        return true;
    }

        if (sqrt(pow(spine[0]-obj[0],2)+pow(spine[1]-obj[1],2)+pow(spine[2]-obj[2],2)) < dis_thr ) {
        a_has_obj[objNbr] = 1;      
        return true;
    }

    return false;
}

bool
verbRec::give(float obj[], float prev_obj[], float spine[], float prev_sp[])
{
        static bool robot_give = false;
        static bool agent_give = false;
    
        static float dis_thr_r = 0.5f;
        static float dis_thr_a = 1.0f;
    
        // ROBOT gives
        float prev_nobj = sqrt(pow(prev_obj[0],2)+pow(prev_obj[1],2)+pow(prev_obj[2],2));
        float nobj = sqrt(pow(obj[0],2)+pow(obj[1],2)+pow(obj[2],2));
    
        if ( (prev_nobj < dis_thr_r) && (nobj > dis_thr_r) )
            robot_give = true;
        else
            robot_give = false;
    
        // AGENT gives
        float prev_nobj_sp = sqrt(pow(prev_obj[0]-prev_sp[0],2)+pow(prev_obj[1]-prev_sp[1],2)+pow(prev_obj[2]-prev_sp[2],2));
        float nobj_sp = sqrt(pow(obj[0]-spine[0],2)+pow(obj[1]-spine[1],2)+pow(obj[2]-spine[2],2));
    
        if ( (prev_nobj_sp < dis_thr_a) && (nobj_sp > dis_thr_a) )
            agent_give = true;
        else
            agent_give = false;
    
        return agent_give;
        //return robot_give;
}

bool
verbRec::take(float obj[], float prev_obj[], float spine[], float prev_sp[])
{
        static bool robot_take = false;
        static bool agent_take = false;
    
        static float dis_thr_r = 0.5f;
        static float dis_thr_a = 1.0f;
    
        // ROBOT takes
        float prev_nobj = sqrt(pow(prev_obj[0],2)+pow(prev_obj[1],2)+pow(prev_obj[2],2));
        float nobj = sqrt(pow(obj[0],2)+pow(obj[1],2)+pow(obj[2],2));
        
        if ( (prev_nobj > dis_thr_r) && (nobj < dis_thr_r) )
            robot_take = true;
        else
            robot_take = false;
    
        // AGENT takes
        float prev_nobj_sp = sqrt(pow(prev_obj[0]-prev_sp[0],2)+pow(prev_obj[1]-prev_sp[1],2)+pow(prev_obj[2]-prev_sp[2],2));
        float nobj_sp = sqrt(pow(obj[0]-spine[0],2)+pow(obj[1]-spine[1],2)+pow(obj[2]-spine[2],2));
    
        if ( (prev_nobj_sp > dis_thr_a) && (nobj_sp < dis_thr_a) )
            agent_take = true;
        else
            agent_take = false;
    
        return agent_take;
        //return robot_take;
}

bool
verbRec::put(float pres_obj[])
{
        bool obj_put = false;
    
        int l_pres = 9;
        int prev_pres_obj = 0;
   
        for (int i=0; i<l_pres; i++)
            prev_pres_obj += (int)pres_obj[i];
    
        if ((prev_pres_obj <l_pres) && (pres_obj[l_pres] == 1))
            obj_put = true;
        else
            obj_put = false;
    
        return obj_put;
}

bool
verbRec::lift(float pres_obj[])
{
        bool obj_lift = false;
    
        int l_pres = 9;
        int prev_pres_obj = 0;
    
        for (int i=0; i<l_pres; i++)
            prev_pres_obj += (int)pres_obj[i];

        if ( (prev_pres_obj > 0) && (pres_obj[l_pres] == 0) )
            obj_lift = true;
        else
            obj_lift = false;
    
        return obj_lift;
}

bool
verbRec::point(float obj[][3], float prev_obj[][3], float r_arm[][3], float l_arm[][3], float prev_rh[], float prev_lh[], float fac[])
{
    bool obj_point = false;

    if (nbrOfObj > 0) {
    
            ///////////// Object Movement
            float m_obj[4] = {0, 0, 0, 0};
            for (int i=0; i<nbrOfObj; i++)
                m_obj[i] = sqrt(pow((obj[i][0]-prev_obj[i][0]),2)+pow((obj[i][1]-prev_obj[i][1]),2)+pow((obj[i][2]-prev_obj[i][2]),2));
    
            ///////////// Hand Movement (Wrist:2)
            float m_rh = 0;
            float m_lh = 0;
        
            m_rh = sqrt(pow((r_arm[2][0]-prev_rh[0]),2)+pow((r_arm[2][1]-prev_rh[1]),2)+pow((r_arm[2][2]-prev_rh[2]),2));
            m_lh = sqrt(pow((l_arm[2][0]-prev_lh[0]),2)+pow((l_arm[2][1]-prev_lh[1]),2)+pow((l_arm[2][2]-prev_lh[2]),2));
    
            // Normal Values of RIGHT/LEFT hand (shoulder:0, elbow:1, wrist:2), Line connects elbow to wrist
            // Normal Vectors of RIGHT/LEFT hand (shoulder:0, elbow:1, wrist:2) , Line connects elbow to wrist
    
            // Left Hand
            float nl_ew = sqrt(pow((l_arm[1][0]-l_arm[2][0]),2)+pow((l_arm[1][1]-l_arm[2][1]),2)+pow((l_arm[1][2]-l_arm[2][2]),2));
            float vl_ew[3] = {(l_arm[1][0]-l_arm[2][0])/nl_ew, (l_arm[1][1]-l_arm[2][1])/nl_ew, (l_arm[1][2]-l_arm[2][2])/nl_ew};
    
            float tl = -l_arm[1][2]/vl_ew[2];
            float Xl = vl_ew[0]*tl + l_arm[1][0];
            float Yl = vl_ew[1]*tl + l_arm[1][1];
    
            float dis_l[2] = {100,0};
            for (int i=0; i<nbrOfObj; i++) {
                if ( (sqrt(pow(obj[i][0]-Xl,2)+pow(obj[i][1]-Yl,2)) < dis_l[0]) && (m_obj[i] == 0) )
                {
                        dis_l[0] = sqrt(pow(obj[i][0]-Xl,2)+pow(obj[i][1]-Yl,2));
                        dis_l[1] = (float)(i+1); 
            }
            }
    
            // Right Hand
            float nr_ew = sqrt(pow((r_arm[1][0]-r_arm[2][0]),2)+pow((r_arm[1][1]-r_arm[2][1]),2)+pow((r_arm[1][2]-r_arm[2][2]),2));
            float vr_ew[3] = {(r_arm[1][0]-r_arm[2][0])/nr_ew, (r_arm[1][1]-r_arm[2][1])/nr_ew, (r_arm[1][2]-r_arm[2][2])/nr_ew};
    
            float tr = -r_arm[1][2]/vr_ew[2];
            float Xr = vr_ew[0]*tr + r_arm[1][0];
            float Yr = vr_ew[1]*tr + r_arm[1][1];
    
            float dis_r[2] = {100,0};
            for (int i=0; i<nbrOfObj; i++) {
                if ( (sqrt(pow(obj[i][0]-Xr,2)+pow(obj[i][1]-Yr,2)) < dis_r[0]) && (m_obj[i] == 0) )
                {
                        dis_r[0] = sqrt(pow(obj[i][0]-Xr,2)+pow(obj[i][1]-Yr,2));
                        dis_r[1] = (float)(i+1); 
            }
            }
    
            /************************ COMPARISON **************************/
            if ( (l_arm[1][2] > 0) && (l_arm[2][2] > 0) && (m_lh < 0.001f) )
            {
                if (dis_l[0] < dis_r[0])
                {
                        fac[0] = dis_l[1];
                        obj_point = true;
                }
            }

            if ( (r_arm[1][2] > 0) && (r_arm[2][2] > 0) && (m_rh < 0.001f) )
            {
                if (dis_r[0] < dis_l[0])
                {
                        fac[0] = dis_r[1];
                        obj_point = true;
                }
            }
    }

    return obj_point;
}

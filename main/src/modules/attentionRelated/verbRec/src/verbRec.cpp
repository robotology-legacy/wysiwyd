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

bool verbRec::configure(yarp::os::ResourceFinder &rf)
{
        string moduleName = rf.check("name", Value("verbRec")).asString().c_str();
        setName(moduleName.c_str());

        yInfo() << moduleName << " : finding configuration files...";
        period = rf.check("period", Value(0.1)).asDouble();

    count=0;

    for (int i=0; i<11; i++)
        output[i] = 0;

        /* optional, attach a port to the module
           so that messages received from the port are redirected
           to the respond method */
        Port_in.open(("/" + moduleName + "In").c_str());
        attach(Port_in);
        Port_out.open(("/" + moduleName + "Out").c_str());
        attach(Port_out);

    Network::connect("/humanRobotDump/humanDump"/*"/input_data"*/,("/" + moduleName + "In").c_str());  // temporary

    Network::connect(("/" + moduleName + "Out").c_str(), "/output_data");  // temporary Change so that it only opens the output port later. 

        return true;
}


bool verbRec::close() {
    /* optional, close port explicitly */
    cout<<"Calling close function\n";
        Port_in.close();
        Port_out.close();
    return true;
}


bool verbRec::respond(const Bottle& command, Bottle& reply) {
    cout<<"Got something, echo is on"<<endl;

        if (command.get(0).asString()=="quit")
            return false;
        else {
            reply=command;

        // prepare a message
            Bottle botWrite; 
        char out[100];
        whatVerbs(command, out);

        // the agent is waving
        if (output[0])
            botWrite.addString("Agent waving");
        
        char str[50];

        // object 1 moving
        if (output[1] == 1) {
            strcpy(str, objectNames[0].c_str());
            botWrite.addString(strcat(str, " moving"));
        }
        // object 2 moving
        if (output[1] == 2) {
            strcpy(str, objectNames[1].c_str());
            botWrite.addString(strcat(str, " moving"));
        }
        // object 3 moving      
        if (output[1] == 3) {
            strcpy(str, objectNames[2].c_str());
            botWrite.addString(strcat(str, " moving"));
        }
        // object 4 moving      
        if (output[1] == 4) {
            strcpy(str, objectNames[3].c_str());
            botWrite.addString(strcat(str, " moving"));
        }

        strcpy(str,"Agent pushes ");
        // the agent pushes object 1
        if (output[2] == 1)
            botWrite.addString(strcat(str, objectNames[0].c_str()));
        strcpy(str,"Agent pushes ");
        // the agent pushes object 2
        if (output[2] == 2)
            botWrite.addString(strcat(str, objectNames[1].c_str()));
        strcpy(str,"Agent pushes ");
        // the agent pushes object 3
        if (output[2] == 3)
            botWrite.addString(strcat(str, objectNames[2].c_str()));
        strcpy(str,"Agent pushes ");
        // the agent pushes object 4
        if (output[2] == 4)
            botWrite.addString(strcat(str, objectNames[3].c_str()));

        strcpy(str,"Agent pulls ");
        // the agent pulls object 1
        if (output[3] == 1)
            botWrite.addString(strcat(str, objectNames[0].c_str()));
        strcpy(str,"Agent pulls ");
        // the agent pulls object 2
        if (output[3] == 2)
            botWrite.addString(strcat(str, objectNames[1].c_str()));
        strcpy(str,"Agent pulls ");
        // the agent pulls object 3
        if (output[3] == 3)
            botWrite.addString(strcat(str, objectNames[2].c_str()));
        strcpy(str,"Agent pulls ");
        // the agent pulls object 4
        if (output[3] == 4)
            botWrite.addString(strcat(str, objectNames[3].c_str()));

        strcpy(str,"Agent grasps ");
        // the agent grasps object 1
        if (output[4] == 1)
            botWrite.addString(strcat(str, objectNames[0].c_str()));
        strcpy(str,"Agent grasps ");
        // the agent grasps object 2
        if (output[4] == 2)
            botWrite.addString(strcat(str, objectNames[1].c_str()));
        strcpy(str,"Agent grasps ");
        // the agent grasps object 3
        if (output[4] == 3)
            botWrite.addString(strcat(str, objectNames[2].c_str()));
        strcpy(str,"Agent grasps ");
        // the agent grasps object 4
        if (output[4] == 4)
            botWrite.addString(strcat(str, objectNames[3].c_str()));

/*      strcpy(str,"Robot has ");
        // the robot has object 1
        if (output[5] == 1)
            botWrite.addString(strcat(str, objectNames[0].c_str()));
        strcpy(str,"Robot has ");
        // the robot has object 2
        if (output[5] == 2)
            botWrite.addString(strcat(str, objectNames[1].c_str()));
        strcpy(str,"Robot has ");
        // the robot has object 3
        if (output[5] == 3)
            botWrite.addString(strcat(str, objectNames[2].c_str()));
        strcpy(str,"Robot has ");
        // the robot has object 4
        if (output[5] == 4)
            botWrite.addString(strcat(str, objectNames[3].c_str()));*/

        // have for Robot/Agent
        if (output[5]) { // at least one of the agent or the robot has objects
            char str_a[50];
            char str_r[50];
            strcpy(str_a,"");   
            strcpy(str_r,"");
            // the agent has ...
            if ((output[5] == 1) || (output[5] == 11)) { // the agent has objects
                strcat(str_a,"Agent has ");
                // ... object 1
                if (a_has_obj[0]) {
                    strcat(str_a, objectNames[0].c_str());
                    strcat(str_a, " ");
                }
                // ... object 2
                if (a_has_obj[1]) {
                    strcat(str_a, objectNames[1].c_str());
                    strcat(str_a, " ");
                }
                // ... object 3
                if (a_has_obj[2]) {
                    strcat(str_a, objectNames[2].c_str());
                    strcat(str_a, " ");
                }
                // ... object 4
                if (a_has_obj[3]) {
                    strcat(str_a, objectNames[3].c_str());
                    strcat(str_a, " ");
                }
            }
            // the robot has ...
            if ((output[5] == 10) || (output[5] == 11)) { // the robot has object
                strcpy(str_r,"Robot has "); 
                // ... object 1
                if (r_has_obj[0]) {
                    strcat(str_r, objectNames[0].c_str());
                    strcat(str_r, " ");
                }
                // ... object 2
                if (r_has_obj[1]) {
                    strcat(str_r, objectNames[1].c_str());
                    strcat(str_r, " ");
                }
                // ... object 3
                if (r_has_obj[2]) {
                    strcat(str_r, objectNames[2].c_str());
                    strcat(str_r, " ");
                }
                // ... object 4
                if (r_has_obj[3]) {
                    strcat(str_r, objectNames[3].c_str());
                    strcat(str_r, " ");
                }
            }
            char str2[100];     
            strcpy(str2,str_a);     
            botWrite.addString(strcat(str2,str_r));
        }

        strcpy(str,"Robot or agent gives ");
        // the robot or agent gives object 1
        if (output[6] == 1)
            botWrite.addString(strcat(str, objectNames[0].c_str()));
        strcpy(str,"Robot or agent gives ");
        // the robot or agent gives object 2
        if (output[6] == 2)
            botWrite.addString(strcat(str, objectNames[1].c_str()));
        strcpy(str,"Robot or agent gives ");
        // the robot or agent gives object 3
        if (output[6] == 3)
            botWrite.addString(strcat(str, objectNames[2].c_str()));
        strcpy(str,"Robot or agent gives ");
        // the robot or agent gives object 4
        if (output[6] == 4)
            botWrite.addString(strcat(str, objectNames[3].c_str()));

        strcpy(str,"Robot or agent takes ");
        // the robot or agent takes object 1
        if (output[7] == 1)
            botWrite.addString(strcat(str, objectNames[0].c_str()));
        strcpy(str,"Robot or agent takes ");
        // the robot or agent takes object 2
        if (output[7] == 2)
            botWrite.addString(strcat(str, objectNames[1].c_str()));
        strcpy(str,"Robot or agent takes ");
        // the robot or agent takes object 3
        if (output[7] == 3)
            botWrite.addString(strcat(str, objectNames[2].c_str()));
        strcpy(str,"Robot or agent takes ");
        // the robot or agent takes object 4
        if (output[7] == 4)
            botWrite.addString(strcat(str, objectNames[3].c_str()));

        strcpy(str,"Agent puts ");
        // the agent puts object 1
        if (output[8] == 1)
            botWrite.addString(strcat(str, objectNames[0].c_str()));
        strcpy(str,"Agent puts ");
        // the agent puts object 2
        if (output[8] == 2)
            botWrite.addString(strcat(str, objectNames[1].c_str()));
        strcpy(str,"Agent puts ");
        // the agent puts object 3
        if (output[8] == 3)
            botWrite.addString(strcat(str, objectNames[2].c_str()));
        strcpy(str,"Agent puts ");
        // the agent puts object 4
        if (output[8] == 4)
            botWrite.addString(strcat(str, objectNames[3].c_str()));

        strcpy(str,"Agent lifts ");
        // the agent lifts object 1
        if (output[9] == 1)
            botWrite.addString(strcat(str, objectNames[0].c_str()));
        strcpy(str,"Agent lifts ");
        // the agent lifts object 2
        if (output[9] == 2)
            botWrite.addString(strcat(str, objectNames[1].c_str()));
        strcpy(str,"Agent lifts ");
        // the agent lifts object 3
        if (output[9] == 3)
            botWrite.addString(strcat(str, objectNames[2].c_str()));
        strcpy(str,"Agent lifts ");
        // the agent lifts object 4
        if (output[9] == 4)
            botWrite.addString(strcat(str, objectNames[3].c_str()));

        strcpy(str,"Agent points at ");
        // the agent points at object 1
        if (output[10] == 1)
            botWrite.addString(strcat(str, objectNames[0].c_str()));
        strcpy(str,"Agent points at ");
        // the agent points at object 2
        if (output[10] == 2)
            botWrite.addString(strcat(str, objectNames[1].c_str()));
        strcpy(str,"Agent points at ");
        // the agent points at object 3
        if (output[10] == 3)
            botWrite.addString(strcat(str, objectNames[2].c_str()));
        strcpy(str,"Agent points at ");
        // the agent points at object 4
        if (output[10] == 4)
            botWrite.addString(strcat(str, objectNames[3].c_str()));

            // send the message
            Port_out.write(botWrite);   
    }
        return true;
}

/* Called periodically every getPeriod() seconds */
bool verbRec::updateModule() {
    count++;
        cout<<"["<<count<<"]"<< " updateModule... "<<endl;
        return true;
}

void verbRec::whatVerbs(const Bottle& command, char* out) {

    // reset output
    for (int i=0; i<11; i++)
        output[i] = 0;

    readData(command, input);

    // temporary, compensate calibration
    /*if (count == 1)
    {
        input[2]=input[2]+0.2;
        input[5]=input[5]+0.2;
        input[8]=input[8]+0.2;
        input[11]=input[11]+0.2;
        input[14]=input[14]+0.2;
        input[17]=input[17]+0.2;
        input[20]=input[20]+0.2;
        input[23]=input[23]+0.2;
        input[26]=input[26]+0.2;  }*/

    cout<<"input converted = ";  // temporary
    for (int i=0; i<47; i++)
        cout<< input[i] <<" ";  // temporary
    cout<<endl;  // temporary
    cout<<endl;  // temporary

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

    static float obj1[5][3] = { {input[31], input[32], input[33]}, {input[31], input[32], input[33]}, {input[31], input[32], input[33]},
        {input[31], input[32], input[33]}, {input[31], input[32], input[33]} };
    
    static float obj2[5][3] = { {input[35], input[36], input[37]}, {input[35], input[36], input[37]}, {input[35], input[36], input[37]},
        {input[35], input[36], input[37]}, {input[35], input[36], input[37]} };
    
    static float obj3[5][3] = { {input[39], input[40], input[41]}, {input[39], input[40], input[41]}, {input[39], input[40], input[41]},
        {input[39], input[40], input[41]}, {input[39], input[40], input[41]} };
    
    static float obj4[5][3] = { {input[43], input[44], input[45]}, {input[43], input[44], input[45]}, {input[43], input[44], input[45]},
        {input[43], input[44], input[45]}, {input[43], input[44], input[45]} };

    static float prev_obj1[3] = {input[31], input[32], input[33]};
    static float prev_obj2[3] = {input[35], input[36], input[37]};
    static float prev_obj3[3] = {input[39], input[40], input[41]};
    static float prev_obj4[3] = {input[43], input[44], input[45]};
    
    static float prev_sp[3] = {input[24], input[25], input[26]};
    
    static float prev_rh[3] = {input[9], input[10], input[11]};
    static float prev_lh[3] = {input[6], input[7], input[8]};
    
    static float pres_obj1[10] = {input[34], input[34],input[34], input[34], input[34], input[34], input[34],input[34], input[34], input[34]};
    static float pres_obj2[10] = {input[38], input[38],input[38], input[38], input[38], input[38], input[38],input[38], input[38], input[38]};
    static float pres_obj3[10] = {input[42], input[42],input[42], input[42], input[42], input[42], input[42],input[42], input[42], input[42]};
    static float pres_obj4[10] = {input[46], input[46],input[46], input[46], input[46], input[46], input[46],input[46], input[46], input[46]};
    
    for (int i=0; i<9; i++) {
        pres_obj1[i] = pres_obj1[i+1];
        pres_obj2[i] = pres_obj2[i+1];
        pres_obj3[i] = pres_obj3[i+1];
        pres_obj4[i] = pres_obj4[i+1];
    }
    pres_obj1[9] = input[34];
    pres_obj2[9] = input[38];
    pres_obj3[9] = input[42];
    pres_obj4[9] = input[46];
    
    for (int i=0; i<4; i++) {
        
        spine[i][0] = spine[i+1][0];            spine[i][1] = spine[i+1][1];            spine[i][2] = spine[i+1][2];
        
        r_hand[i][0] = r_hand[i+1][0];          r_hand[i][1] = r_hand[i+1][1];          r_hand[i][2] = r_hand[i+1][2];
        l_hand[i][0] = l_hand[i+1][0];          l_hand[i][1] = l_hand[i+1][1];          l_hand[i][2] = l_hand[i+1][2];
        
        r_shoulder[i][0] = r_shoulder[i+1][0];  r_shoulder[i][1] = r_shoulder[i+1][1];  r_shoulder[i][2] = r_shoulder[i+1][2];
        l_shoulder[i][0] = l_shoulder[i+1][0];  l_shoulder[i][1] = l_shoulder[i+1][1];  l_shoulder[i][2] = l_shoulder[i+1][2];
        
        r_elbow[i][0] = r_elbow[i+1][0];        r_elbow[i][1] = r_elbow[i+1][1];        r_elbow[i][2] = r_elbow[i+1][2];
        l_elbow[i][0] = l_elbow[i+1][0];        l_elbow[i][1] = l_elbow[i+1][1];        l_elbow[i][2] = l_elbow[i+1][2];   }

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
        mle[0] += l_elbow[i][0];  mle[1] += l_elbow[i][1];  mle[2] += l_elbow[i][2];  }
    
    msp[0] = msp[0]/5;  msp[1] = msp[1]/5;  msp[2] = msp[2]/5;
    
    mrh[0] = mrh[0]/5;  mrh[1] = mrh[1]/5;  mrh[2] = mrh[2]/5;
    mlh[0] = mlh[0]/5;  mlh[1] = mlh[1]/5;  mlh[2] = mlh[2]/5;
    
    mrs[0] = mrs[0]/5;  mrs[1] = mrs[1]/5;  mrs[2] = mrs[2]/5;
    mls[0] = mls[0]/5;  mls[1] = mls[1]/5;  mls[2] = mls[2]/5;
    
    mre[0] = mre[0]/5;  mre[1] = mre[1]/5;  mre[2] = mre[2]/5;
    mle[0] = mle[0]/5;  mle[1] = mle[1]/5;  mle[2] = mle[2]/5;
    
    float r_arm[3][3] = {{mrs[0],mrs[1],mrs[2]},{mre[0],mre[1],mre[2]},{mrh[0],mrh[1],mrh[2]}};
    float l_arm[3][3] = {{mls[0],mls[1],mls[2]},{mle[0],mle[1],mle[2]},{mlh[0],mlh[1],mlh[2]}};

    // reset vectors recording what objects the robot and the agent have
    for (int i=0; i<4; i++) {
        r_has_obj[i] = 0;
        a_has_obj[i] = 0;
    }

// ************* start recognize verbs **************** //
        if (wave(mrh, mlh, prev_rh, prev_lh))
            output[0]=1;

    output[5] = (float)have();

    /************************** OBJECT(1) **************************/
     if ((pres_obj1[9] == 1) && (pres_obj1[8] == 1)) //Object is on reactable and was on reactable
     {
     for (int i=0; i<4; i++) {
     obj1[i][0] = obj1[i+1][0];
     obj1[i][1] = obj1[i+1][1];
     obj1[i][2] = obj1[i+1][2];  }
     
     obj1[4][0] = input[31];
     obj1[4][1] = input[32];
     obj1[4][2] = input[33];
     
     float mObj1[3]={0, 0, 0};
     for (int i=0; i<5; i++) {
     mObj1[0] += obj1[i][0];
     mObj1[1] += obj1[i][1];
     mObj1[2] += obj1[i][2]; }
     
     mObj1[0] = mObj1[0]/5;
     mObj1[1] = mObj1[1]/5;
     mObj1[2] = mObj1[2]/5;
     
     if (move(mObj1,prev_obj1))
     {output[1]=1;}
     
     if (push(obj1, prev_obj1, r_hand, l_hand))
     {output[2]=1;}
     
     if (pull(obj1, prev_obj1, r_hand, l_hand))
     {output[3]=1;}
     
     if (grasp(mObj1, prev_obj1, mrh, mlh))
     {output[4]=1;}
     
//     if (have(mObj1, prev_obj1))
//     {a_has_obj[0] = 1;}
//     {output[5]=1;}
     
        if (give(mObj1, prev_obj1, msp, prev_sp))
        {output[6]=1;}
        
        if (take(mObj1, prev_obj1, msp, prev_sp))
        {output[7]=1;}
     
     if (put(pres_obj1))
     {output[8]=1;}
     
     if (lift(pres_obj1, mrh, mlh, prev_rh, prev_lh))
     {output[9]=1;}
     
     if (point(mObj1, prev_obj1,r_arm,l_arm))
     {output[10]=1;}
     
     
     prev_obj1[0] = mObj1[0];
     prev_obj1[1] = mObj1[1];
     prev_obj1[2] = mObj1[2];
     }
     
     if ((pres_obj1[9] == 1) && (pres_obj1[8] == 0)) //Object is on reactable and was NOT on reactable
     {
     for (int i=0; i<5; i++) {
     obj1[i][0] = input[31];
     obj1[i][1] = input[32];
     obj1[i][2] = input[33];  }
     
     prev_obj1[0] = input[31];
     prev_obj1[1] = input[32];
     prev_obj1[2] = input[33];
     }
     
     
    /************************** OBJECT(2) **************************/
     if ((pres_obj2[9] == 1) && (pres_obj2[8] == 1))
     {
     for (int i=0; i<4; i++) {
     obj2[i][0] = obj2[i+1][0];
     obj2[i][1] = obj2[i+1][1];
     obj2[i][2] = obj2[i+1][2];  }
     
     obj2[4][0] = input[35];
     obj2[4][1] = input[36];
     obj2[4][2] = input[37];
     
     float mObj2[3]={0, 0, 0};
     for (int i=0; i<5; i++) {
     mObj2[0] += obj2[i][0];
     mObj2[1] += obj2[i][1];
     mObj2[2] += obj2[i][2]; }
     
     mObj2[0] = mObj2[0]/5;
     mObj2[1] = mObj2[1]/5;
     mObj2[2] = mObj2[2]/5;
     
     
     if (move(mObj2,prev_obj2))
     {output[1]=2;}
     
     if (push(obj2, prev_obj2, r_hand, l_hand))
     {output[2]=2;}
     
     if (pull(obj2, prev_obj2, r_hand, l_hand))
     {output[3]=2;}
     
     if (grasp(mObj2, prev_obj2, mrh, mlh))
     {output[4]=2;}
     
//     if (have(mObj2, prev_obj2))
//     {a_has_obj[1] = 1;}
//     {output[5]=2;}
     
            if (give(mObj2, prev_obj2, msp, prev_sp))
            {output[6]=2;}
            
            if (take(mObj2, prev_obj2, msp, prev_sp))
            {output[7]=2;}
     
     if (put(pres_obj2))
     {output[8]=2;}
     
     if (lift(pres_obj2, mrh, mlh, prev_rh, prev_lh))
     {output[9]=2;}
     
     if (point(mObj2, prev_obj2,r_arm,l_arm))
     {output[10]=2;}
     
     prev_obj2[0] = mObj2[0];
     prev_obj2[1] = mObj2[1];
     prev_obj2[2] = mObj2[2];
     }
     
     
     if ((pres_obj2[9] == 1) && (pres_obj2[8] == 0))
     {
     for (int i=0; i<5; i++) {
     obj2[i][0] = input[35];
     obj2[i][1] = input[36];
     obj2[i][2] = input[37];  }
     
     prev_obj2[0] = input[35];
     prev_obj2[1] = input[36];
     prev_obj2[2] = input[37];
     }
    
    
    /************************** OBJECT(3) **************************/
    if ((pres_obj3[9] == 1) && (pres_obj3[8] == 1))
    {
        for (int i=0; i<4; i++) {
            obj3[i][0] = obj3[i+1][0];
            obj3[i][1] = obj3[i+1][1];
            obj3[i][2] = obj3[i+1][2];  }
        
        obj3[4][0] = input[39];
        obj3[4][1] = input[40];
        obj3[4][2] = input[41];
        
        float mObj3[3]={0, 0, 0};
        for (int i=0; i<5; i++) {
            mObj3[0] += obj3[i][0];
            mObj3[1] += obj3[i][1];
            mObj3[2] += obj3[i][2]; }
        
        mObj3[0] = mObj3[0]/5;
        mObj3[1] = mObj3[1]/5;
        mObj3[2] = mObj3[2]/5;
        
        
        if (move(mObj3,prev_obj3))
        {output[1]=3;}
        
        if (push(obj3, prev_obj3, r_hand, l_hand))
        {output[2]=3;}
        
        if (pull(obj3, prev_obj3, r_hand, l_hand))
        {output[3]=3;}
        
        if (grasp(mObj3, prev_obj3, mrh, mlh))
        {output[4]=3;}
        
//        if (have(mObj3, prev_obj3))
//        {a_has_obj[2] = 1;}
//        {output[5]=3;}
        
            if (give(mObj3, prev_obj3, msp, prev_sp))
            {output[6]=3;}
            
            if (take(mObj3, prev_obj3, msp, prev_sp))
            {output[7]=3;}
        
        if (put(pres_obj3))
        {output[8]=3;}
        
        if (lift(pres_obj3, mrh, mlh, prev_rh, prev_lh))
        {output[9]=3;}
        
        if (point(mObj3, prev_obj3,r_arm,l_arm))
        {output[10]=3;}
        
        
        prev_obj3[0] = mObj3[0];
        prev_obj3[1] = mObj3[1];
        prev_obj3[2] = mObj3[2];
    }
    
    if ((pres_obj3[9] == 1) && (pres_obj3[8] == 0))
    {
        
        for (int i=0; i<5; i++) {
            obj3[i][0] = input[39];
            obj3[i][1] = input[40];
            obj3[i][2] = input[41];  }
        
        prev_obj3[0] = input[39];
        prev_obj3[1] = input[40];
        prev_obj3[2] = input[41];
    }
    
    /************************** OBJECT(4) **************************/
     if ((pres_obj4[9] == 1) && (pres_obj4[8] == 1))
     {
     for (int i=0; i<4; i++) {
     obj4[i][0] = obj4[i+1][0];
     obj4[i][1] = obj4[i+1][1];
     obj4[i][2] = obj4[i+1][2];  }
     
     obj4[4][0] = input[43];
     obj4[4][1] = input[44];
     obj4[4][2] = input[45];
     
     float mObj4[3]={0, 0, 0};
     for (int i=0; i<5; i++) {
     mObj4[0] += obj4[i][0];
     mObj4[1] += obj4[i][1];
     mObj4[2] += obj4[i][2]; }
     
     mObj4[0] = mObj4[0]/5;
     mObj4[1] = mObj4[1]/5;
     mObj4[2] = mObj4[2]/5;
     
     if (move(mObj4,prev_obj4))
     {output[1]=4;}
     
     if (push(obj4, prev_obj4, r_hand, l_hand))
     {output[2]=4;}
     
     if (pull(obj4, prev_obj4, r_hand, l_hand))
     {output[3]=4;}
     
     if (grasp(mObj4, prev_obj4, mrh, mlh))
     {output[4]=4;}
     
//      if (have(mObj4, prev_obj4))
//      {a_has_obj[3] = 1;}
//      {output[5]=4;}
     
            if (give(mObj4, prev_obj4, msp, prev_sp))
            {output[6]=4;}
            
            if (take(mObj4, prev_obj4, msp, prev_sp))
            {output[7]=4;}
     
     if (put(pres_obj4))
     {output[8]=4;}
     
     if (lift(pres_obj4, mrh, mlh, prev_rh, prev_lh))
     {output[9]=4;}
     
     if (point(mObj4, prev_obj4,r_arm,l_arm))
     {output[10]=4;}
     
     prev_obj4[0] = mObj4[0];
     prev_obj4[1] = mObj4[1];
     prev_obj4[2] = mObj4[2];
     }
     
     if ((pres_obj4[9] == 1) && (pres_obj4[8] == 0))
     {
     for (int i=0; i<5; i++) {
     obj4[i][0] = input[43];
     obj4[i][1] = input[44];
     obj4[i][2] = input[45];  }
     
     prev_obj4[0] = input[43];
     prev_obj4[1] = input[44];
     prev_obj4[2] = input[45];
     }
    
    prev_rh[0] = mrh[0];
    prev_rh[1] = mrh[1];
    prev_rh[2] = mrh[2];
    
    prev_lh[0] = mlh[0];
    prev_lh[1] = mlh[1];
    prev_lh[2] = mlh[2];

        prev_sp[0] = msp[0];
        prev_sp[1] = msp[1];
        prev_sp[2] = msp[2];

    // have
/*  bool a_has = false;
    bool r_has = false;
    for (int i=0; i<4; i++) {
        if (r_has_obj[i])
            r_has = true;
        if (a_has_obj[i])
            a_has = true;
    }
    if (a_has && !r_has)
        output[5] = 1;
    if (!a_has && r_has)
        output[5] = 10;
    if (a_has && r_has)
        output[5] = 11;*/


/*  if (wave())
        output[0] = 1;

    int object = 0;
    if (take(&object)) {
        if (object == 1)
            output[1] = 1;
        if (object == 2)
            output[1] = 2;
        if (object == 3)
            output[1] = 3;
        else
            output[1] = 4;
    }

    output[2] = put();

    object = 0;
    if (push(&object)) {
        if (object == 1)
            output[3] = 1;
        if (object == 2)
            output[3] = 2;
        if (object == 3)
            output[3] = 3;
        else
            output[3] = 4;
    }
    object = 0;
    if (pull(&object)) {
        if (object == 1)
            output[4] = 1;
        if (object == 2)
            output[4] = 2;
        if (object == 3)
            output[4] = 3;
        else
            output[4] = 4;
    }
    object = 0;
    if (point(&object)) {
        if (object == 1)
            output[5] = 1;
        if (object == 2)
            output[5] = 2;
        if (object == 3)
            output[5] = 3;
        else
            output[5] = 4;
    }
    object = 0;
    if (move(&object)) {
        if (object == 1)
            output[6] = 1;
        if (object == 2)
            output[6] = 2;
        if (object == 3)
            output[6] = 3;
        else
            output[6] = 4;
    }

    output[7] = lift();
    output[8] = have(); 

    object = 0;
    if (grasp(&object)) {
        if (object == 1)
            output[9] = 1;
        if (object == 2)
            output[9] = 2;
        if (object == 3)
            output[9] = 3;
        else
            output[9] = 4;
    }
    object = 0;
    if (give(&object)) {
        if (object == 1)
            output[10] = 1;
        if (object == 2)
            output[10] = 2;
        if (object == 3)
            output[10] = 3;
        else
            output[10] = 4;
    }*/

// ************* end recognize verbs **************** //

    cout<<"output as a float array = ";  // temporary
    for (int i=0; i<11; i++)
        cout<< output[i] <<" ";
    cout<<endl;  // temporary
    cout<<endl;  // temporary
}

void verbRec::readData(const Bottle& command, float* input) {

    Bottle bSkeleton = *command.get(2).asList();
    Bottle bPartner = *command.get(3).asList();
    int nbrOfObj = command.size()-5;

    for (int i=0; i<47; i++)    // -10000 means no data for the element
        input[i] = -10000;  

    for (int i=0; i<bSkeleton.size(); i++) {
        ConstString str = (*bSkeleton.get(i).asList()).get(0).toString();
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

    // the number of objects can vary between 0 and 4
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
    //const float PI = 3.1415927;
    
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
        i_lh[i][2] = iT[8]*l_hand[i][0] + iT[9]*l_hand[i][1] + iT[10]*l_hand[i][2]+ iT[11];  }
    
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

// **************** functions for verb recognition ***************** //

bool
verbRec::wave(float r_hand[3], float l_hand[3], float prev_rh[3], float prev_lh[3])
{    
    static bool obj_wave = false;
    static float mov_thr = 0.1f;
    
    float drh = sqrt(pow((r_hand[0]-prev_rh[0]),2)+pow((r_hand[1]-prev_rh[1]),2)+pow((r_hand[2]-prev_rh[2]),2));
    float dlh = sqrt(pow((l_hand[0]-prev_lh[0]),2)+pow((l_hand[1]-prev_lh[1]),2)+pow((l_hand[2]-prev_lh[2]),2));
    
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
        mobj[2] += i_obj[i][2];   }
    
    mrh[0] = mrh[0]/5;     mrh[1] = mrh[1]/5;     mrh[2] = mrh[2]/5;
    mlh[0] = mlh[0]/5;     mlh[1] = mlh[1]/5;     mlh[2] = mlh[2]/5;
    mobj[0] = mobj[0]/5;   mobj[1] = mobj[1]/5;   mobj[2] = mobj[2]/5;
    
//    float drh = sqrt(pow((mrh[0]-mobj[0]),2)+pow((mrh[1]-mobj[1]),2)+pow((mrh[2]-mobj[2]),2));
//    float dlh = sqrt(pow((mlh[0]-mobj[0]),2)+pow((mlh[1]-mobj[1]),2)+pow((mlh[2]-mobj[2]),2));
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
        mobj[2] += i_obj[i][2];   }
    
    mrh[0] = mrh[0]/5;     mrh[1] = mrh[1]/5;     mrh[2] = mrh[2]/5;
    mlh[0] = mlh[0]/5;     mlh[1] = mlh[1]/5;     mlh[2] = mlh[2]/5;
    mobj[0] = mobj[0]/5;   mobj[1] = mobj[1]/5;   mobj[2] = mobj[2]/5;
    
//    float drh = sqrt(pow((mrh[0]-mobj[0]),2)+pow((mrh[1]-mobj[1]),2)+pow((mrh[2]-mobj[2]),2));
//    float dlh = sqrt(pow((mlh[0]-mobj[0]),2)+pow((mlh[1]-mobj[1]),2)+pow((mlh[2]-mobj[2]),2));
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
    static float dis_thr = 0.5;    
    
    float drh = sqrt(pow((r_hand[0]-obj[0]),2)+pow((r_hand[1]-obj[1]),2)+pow((r_hand[2]-obj[2]),2));
    float dlh = sqrt(pow((l_hand[0]-obj[0]),2)+pow((l_hand[1]-obj[1]),2)+pow((l_hand[2]-obj[2]),2));
    float dobj = sqrt(pow((obj[0]-prev_obj[0]),2)+pow((obj[1]-prev_obj[1]),2)+pow((obj[2]-prev_obj[2]),2));

    if ((dobj == 0) && ((drh < dis_thr) || (dlh < dis_thr)))
        obj_grasp = true;
    else
        obj_grasp = false;
    
    return obj_grasp;
}

/*bool
verbRec::have(float obj[], float prev_obj[])
{
    static bool obj_have = false;
    static float dis_thr = 0.5;
    
    float prev_nobj = sqrt(pow(prev_obj[0],2)+pow(prev_obj[1],2)+pow(prev_obj[2],2));
    float nobj = sqrt(pow(obj[0],2)+pow(obj[1],2)+pow(obj[2],2));
   
    if ( (prev_nobj < dis_thr) && (nobj < dis_thr) )
        obj_have = true;
    else
        obj_have = false;
    
    return obj_have;
}*/

int 
verbRec::have()
{
        static float dis_thr = 0.5;
    int ret = 0;
    
    for (int i=0; i<4; i++) {
        r_has_obj[i] = 0;
        a_has_obj[i] = 0;
    }
    
        // robot
        if ((input[34] == 1) && (sqrt(pow(input[31],2)+pow(input[32],2)+pow(input[33],2)) < dis_thr) ) {
        r_has_obj[0] = 1;
        ret = 10;
    }
    
        if ((input[38] == 1) && (sqrt(pow(input[35],2)+pow(input[36],2)+pow(input[37],2)) < dis_thr) ) {
        r_has_obj[1] = 1;
        ret = 10;
    }
    
        if ((input[42] == 1) && (sqrt(pow(input[39],2)+pow(input[40],2)+pow(input[41],2)) < dis_thr) ) {
        r_has_obj[2] = 1;
        ret = 10;
    }    
        if ((input[44] == 1) && (sqrt(pow(input[43],2)+pow(input[44],2)+pow(input[45],2)) < dis_thr) ) {
        r_has_obj[3] = 1;
        ret = 10;
    }
    
        // agent
        float diff_1 = sqrt(pow(input[24]-input[31],2)+pow(input[25]-input[32],2)+pow(input[26]-input[33],2));
        if ((input[34] == 1) && (diff_1 < dis_thr) ) {
        a_has_obj[0] = 1;
        if (ret<10)
            ret = 1;
        else
            ret = 11;
        }
     
        float diff_2 = sqrt(pow(input[24]-input[35],2)+pow(input[25]-input[36],2)+pow(input[26]-input[37],2));
        if ((input[38] == 1) && (diff_2 < dis_thr) ) {
        a_has_obj[1] = 1;
        if (ret<10)
            ret = 1;
        else
            ret = 11;
        }
     
        float diff_3 = sqrt(pow(input[24]-input[39],2)+pow(input[25]-input[40],2)+pow(input[26]-input[41],2));
        if ((input[42] == 1) && (diff_3 < dis_thr) ) {
        a_has_obj[2] = 1;
        if (ret<10)
            ret = 1;
        else
            ret = 11;
        }
     
        float diff_4 = sqrt(pow(input[24]-input[43],2)+pow(input[25]-input[44],2)+pow(input[26]-input[45],2));
        if ((input[44] == 1) && (diff_4 < dis_thr) ) {
        a_has_obj[3] = 1;
        if (ret<10)
            ret = 1;
        else
            ret = 11;
        }

        return ret;
}

/*bool
verbRec::give(float obj[], float prev_obj[])
{
    static bool obj_give = false;
    static float dis_thr = 0.5;
    
    float prev_nobj = sqrt(pow(prev_obj[0],2)+pow(prev_obj[1],2)+pow(prev_obj[2],2));
    float nobj = sqrt(pow(obj[0],2)+pow(obj[1],2)+pow(obj[2],2));
    
    if ( (prev_nobj > dis_thr) && (nobj < dis_thr) )
        obj_give = true;
    else
        obj_give = false;
    
    return obj_give;
}*/

bool
verbRec::give(float obj[], float prev_obj[], float spine[], float prev_sp[])
{
    static bool robot_give = false;
    static bool agent_give = false;
    static float dis_thr = 0.5;
    
    
    // ROBOT gives
    float prev_nobj = sqrt(pow(prev_obj[0],2)+pow(prev_obj[1],2)+pow(prev_obj[2],2));
    float nobj = sqrt(pow(obj[0],2)+pow(obj[1],2)+pow(obj[2],2));
    
    
    if ( (prev_nobj > dis_thr) && (nobj < dis_thr) )
        robot_give = true;
    else
        robot_give = false;
    
    // AGENT gives
    float prev_nobj_sp = sqrt(pow(prev_obj[0]-prev_sp[0],2)+pow(prev_obj[1]-prev_sp[1],2)+pow(prev_obj[2]-prev_sp[2],2));
    float nobj_sp = sqrt(pow(obj[0]-spine[0],2)+pow(obj[1]-spine[1],2)+pow(obj[2]-spine[2],2));
    
    
    if ( (prev_nobj_sp > dis_thr) && (nobj_sp < dis_thr) )
        agent_give = true;
    else
        agent_give = false;
    
    
    
    return agent_give;
}

/*bool
verbRec::take(float obj[], float prev_obj[])
{
    static bool obj_take = false;
    static float dis_thr = 0.5;
    
    float prev_nobj = sqrt(pow(prev_obj[0],2)+pow(prev_obj[1],2)+pow(prev_obj[2],2));
    float nobj = sqrt(pow(obj[0],2)+pow(obj[1],2)+pow(obj[2],2));
    
    if ( (prev_nobj < dis_thr) && (nobj > dis_thr) )
        obj_take = true;
    else
        obj_take = false;
    
    return obj_take;
}*/

bool
verbRec::take(float obj[], float prev_obj[], float spine[], float prev_sp[])
{
    static bool robot_take = false;
    static bool agent_take = false;
    
    static float dis_thr = 0.5;
    
    // ROBOT takes
    float prev_nobj = sqrt(pow(prev_obj[0],2)+pow(prev_obj[1],2)+pow(prev_obj[2],2));
    float nobj = sqrt(pow(obj[0],2)+pow(obj[1],2)+pow(obj[2],2));
    
    if ( (prev_nobj < dis_thr) && (nobj > dis_thr) )
        robot_take = true;
    else
        robot_take = false;
    
    // AGENT takes
    float prev_nobj_sp = sqrt(pow(prev_obj[0]-prev_sp[0],2)+pow(prev_obj[1]-prev_sp[1],2)+pow(prev_obj[2]-prev_sp[2],2));
    float nobj_sp = sqrt(pow(obj[0]-spine[0],2)+pow(obj[1]-spine[1],2)+pow(obj[2]-spine[2],2));
    
    if ( (prev_nobj_sp < dis_thr) && (nobj_sp > dis_thr) )
        agent_take = true;
    else
        agent_take = false;
    
    
    return agent_take;
}

bool
verbRec::put(float pres_obj[])
{
    static bool obj_put = false;
    
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
verbRec::lift(float pres_obj[], float r_hand[], float l_hand[], float prev_rh[], float prev_lh[])
{
    static bool obj_lift = false;
//    static float mov_thr = 0.1;
    
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
verbRec::point(float obj[], float prev_obj[],float r_arm[][3], float l_arm[][3])
{
    static bool obj_point = false;
    //static float mov_thr = 0.01;
    
    ///////////// Object
    float dobj = sqrt(pow((obj[0]-prev_obj[0]),2)+pow((obj[1]-prev_obj[1]),2)+pow((obj[2]-prev_obj[2]),2));
    
    // Normal Values of RIGHT/LEFT hand (shoulder:0, elbow:1, wrist:2), Line connects elbow to wrist
    float nr_ew = sqrt(pow((r_arm[1][0]-r_arm[2][0]),2)+pow((r_arm[1][1]-r_arm[2][1]),2)); //+pow((r_arm[1][2]-r_arm[2][2]),2));
    float nl_ew = sqrt(pow((l_arm[1][0]-l_arm[2][0]),2)+pow((l_arm[1][1]-l_arm[2][1]),2)); //+pow((l_arm[1][2]-l_arm[2][2]),2));
    
    // Normal Vectors of RIGHT/LEFT hand (shoulder:0, elbow:1, wrist:2) , Line connects elbow to wrist
    float vr_ew[2] = {(r_arm[1][0]-r_arm[2][0])/nr_ew, (r_arm[1][1]-r_arm[2][1])/nr_ew}; //, (r_arm[1][2]-r_arm[2][2])/nr_ew};
    float vl_ew[2] = {(l_arm[1][0]-l_arm[2][0])/nl_ew, (l_arm[1][1]-l_arm[2][1])/nl_ew}; //, (l_arm[1][2]-l_arm[2][2])/nl_ew};
    
    // Intersection of the Object with RIGHT/LEFT hand line
    float tr[2] = {(obj[0]-r_arm[1][0])/vr_ew[0], (obj[1]-r_arm[1][1])/vr_ew[1]};
    float tl[2] = {(obj[0]-l_arm[1][0])/vl_ew[0], (obj[1]-l_arm[1][1])/vl_ew[1]};
    
    if ( (dobj == 0) && ((abs(tr[0]-tr[1]) == 0) || (abs(tl[0]-tl[1]) == 0)) ) // Object is Stationary AND positioned in (RIGHT OR LEFT) hand line
        obj_point = true;
    else
        obj_point = false;
    
    printf("<< dobj = %f >>\t", dobj);
    printf("<< tr = [%f %f] >>\t", tr[0],tr[1]);
    printf("<< tl = [%f %f] >>\t\n\n", tl[0],tl[1]);
    printf("<< dtr = %f >>\t", abs(tr[0]-tr[1]));
    printf("<< dtl = %f >>\t", abs(tl[0]-tl[1]));
    
    // float tsr[2] = {(r_arm[2][0]-r_arm[1][0])/vr_ew[0], (r_arm[2][1]-r_arm[1][1])/vr_ew[1]};
    //  printf("<< tsr = %f >>\t\n", abs(tsr[0]-tsr[1]));
    
    
    return obj_point;
    
}

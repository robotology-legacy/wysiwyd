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

		if (output[0])
			botWrite.addString("Agent waving");
		
		char str[50];

		strcpy(str,"Agent puts ");
		if (output[2] == 1)
			botWrite.addString(strcat(str, objectNames[0].c_str())/*"Agent puts object 1 "*/);
		strcpy(str,"Agent puts ");
		if (output[2] == 2)
			botWrite.addString(strcat(str, objectNames[1].c_str())/*"Agent puts object 2 "*/);
		strcpy(str,"Agent puts ");
		if (output[2] == 3)
			botWrite.addString(strcat(str, objectNames[2].c_str())/*"Agent puts object 3 "*/);
		strcpy(str,"Agent puts ");
		if (output[2] == 4)
			botWrite.addString(strcat(str, objectNames[3].c_str())/*"Agent puts object 4 "*/);

		if (output[6] == 1) {
			strcpy(str, objectNames[0].c_str());
			botWrite.addString(strcat(str, " moving")/*"Object 1 moving "*/);
		}
		if (output[6] == 2) {
			strcpy(str, objectNames[1].c_str());
			botWrite.addString(strcat(str, " moving")/*"Object 2 moving "*/);
		}		
		if (output[6] == 3) {
			strcpy(str, objectNames[2].c_str());
			botWrite.addString(strcat(str, " moving")/*"Object 3 moving "*/);
		}		
		if (output[6] == 4) {
			strcpy(str, objectNames[3].c_str());
			botWrite.addString(strcat(str, " moving")/*"Object 4 moving "*/);
		}

		strcpy(str,"Agent lifts ");
		if (output[7] == 1)
			botWrite.addString(strcat(str, objectNames[0].c_str())/*"Agent lifts object 1 "*/);
		strcpy(str,"Agent lifts ");
		if (output[7] == 2)
			botWrite.addString(strcat(str, objectNames[1].c_str())/*"Agent lifts object 2 "*/);
		strcpy(str,"Agent lifts ");
		if (output[7] == 3)
			botWrite.addString(strcat(str, objectNames[2].c_str())/*"Agent lifts object 3 "*/);
		strcpy(str,"Agent lifts ");
		if (output[7] == 4)
			botWrite.addString(strcat(str, objectNames[3].c_str())/*"Agent lifts object 4 "*/);


		////////////// Have for Robot/Agent
		if (output[8]) { // at least one of the agent or the robot has objects
			char str_a[50];
			char str_r[50];
	  		strcpy(str_a,"");	
	  		strcpy(str_r,"");
			if ((output[8] == 1) || (output[8] == 11)) { // the agent has objects
	  			strcat(str_a,"Agent has ");	
				if (a_has_obj[0]) {
					strcat(str_a, objectNames[0].c_str()/*"object 1 "*/);
					strcat(str_a, " ");
				}
				if (a_has_obj[1]) {
					strcat(str_a, objectNames[1].c_str()/*"object 2 "*/);
					strcat(str_a, " ");
				}
				if (a_has_obj[2]) {
					strcat(str_a, objectNames[2].c_str()/*"object 3 "*/);
					strcat(str_a, " ");
				}
				if (a_has_obj[3]) {
					strcat(str_a, objectNames[3].c_str()/*"object 4 "*/);
					strcat(str_a, " ");
				}
			}
			if ((output[8] == 10) || (output[8] == 11)) { // the robot has object
				strcpy(str_r,"Robot has ");	
				if (r_has_obj[0]) {
					strcat(str_r, objectNames[0].c_str()/*"object 1 "*/);
					strcat(str_r, " ");
				}
				if (r_has_obj[1]) {
					strcat(str_r, objectNames[1].c_str()/*"object 2 "*/);
					strcat(str_r, " ");
				}
				if (r_has_obj[2]) {
					strcat(str_r, objectNames[2].c_str()/*"object 3 "*/);
					strcat(str_r, " ");
				}
				if (r_has_obj[3]) {
					strcat(str_r, objectNames[3].c_str()/*"object 4 "*/);
					strcat(str_r, " ");
				}
			}
			char str2[100];		
			strcpy(str2,str_a);		
			botWrite.addString(strcat(str2,str_r));
		}

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

	for (int i=0; i<11; i++)
		output[i] = 0;

	readData(command, input);

	cout<<"input converted = ";  // temporary
	for (int i=0; i<47; i++)
		cout<< input[i] <<" ";  // temporary
	cout<<endl;  // temporary
	cout<<endl;  // temporary

// ************* start recognize verbs **************** //

	if (wave())
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
	}

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

	for (int i=0; i<47; i++)	// -10000 means no data for the element
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
		//objects.addString(command.get(i+4).asString()/*asList()*/);
		objectNames[i] = (*command.get(i+4).asList()).get(0).toString();
		input[31+4*i] = (float)atof((*command.get(i+4).asList()).get(1).toString().c_str());
		input[32+4*i] = (float)atof((*command.get(i+4).asList()).get(2).toString().c_str());
		input[33+4*i] = (float)atof((*command.get(i+4).asList()).get(3).toString().c_str());
		input[34+4*i] = (float)atof((*command.get(i+4).asList()).get(4).toString().c_str());
	}
}


// **************** functions for verb recognition ***************** //

bool verbRec::wave()
{
	static float movement_threshold = 0.2;
	static bool right_hand_movement = false;
	
	static float prev_right_hand_x[5] = {input[9], input[9], input[9], input[9], input[9]};
	static float prev_right_hand_y[5] = {input[10], input[10], input[10], input[10], input[10]};
	static float prev_right_hand_z[5] = {input[11], input[11], input[11], input[11], input[11]};

	static float rhx_prev = input[9];
	static float rhy_prev = input[10];
	static float rhz_prev = input[11];

	// smoothing	
	for (int i=0; i<4; i++) {
		prev_right_hand_x[i] = prev_right_hand_x[i+1];
		prev_right_hand_y[i] = prev_right_hand_y[i+1];
		prev_right_hand_z[i] = prev_right_hand_z[i+1];
	}
	prev_right_hand_x[4] = input[9];
	prev_right_hand_y[4] = input[10];
	prev_right_hand_z[4] = input[11];

	// moving average
	float rhx = 0;
	float rhy = 0;
	float rhz = 0;
	for (int i=0; i<5; i++) {
		rhx += prev_right_hand_x[i];
		rhy += prev_right_hand_y[i];
		rhz += prev_right_hand_z[i];
	}

	// the agent's right hand is moving
/*	if (sqrt(sqr(rhx-rhx_prev) + sqr(rhy-rhy_prev) + sqr(rhz-rhz_prev)) > movement_threshold)*/
	if (sqrt(pow((rhx-rhx_prev),2) + pow((rhy-rhy_prev),2) + pow((rhz-rhz_prev),2)) > movement_threshold)
		right_hand_movement = true;

	rhx_prev = rhx;
	rhy_prev = rhy;
	rhz_prev = rhz;

	// the right hand higher than the head
	if ((input[11] > input[14]) && right_hand_movement)
		return true;
	else
		return false;
}

bool verbRec::move(int* object)
{
	if (move_obj1() && (input[31] != -1000)) 
		*object = 1;
     	else if (move_obj2() && (input[35] != -1000)) 
		*object = 2;
     	else if (move_obj3() && (input[39] != -1000)) 
		*object = 3;
     	else if (move_obj4() && (input[43] != -1000)) 
		*object = 4;
	else
		return false;

	return true;
}

bool verbRec::move_obj1()
{

static float mov_thr = 0.05;
    
/*static*/ bool obj1_movement = false;

static float pres_obj1[2] = {input[34], input[34]};

static float prev_obj1_x[5] = {input[31], input[31], input[31], input[31], input[31]};
static float prev_obj1_y[5] = {input[32], input[32], input[32], input[32], input[32]};
static float prev_obj1_z[5] = {input[33], input[33], input[33], input[33], input[33]};

static float obj1x_prev = input[31];
static float obj1y_prev = input[32];
static float obj1z_prev = input[33];


pres_obj1[0] = pres_obj1[1];
pres_obj1[1] = input[34];

if ((pres_obj1[0] == 1) && (pres_obj1[1] == 1)) {

// smoothing
for (int i=0; i<4; i++) {
    prev_obj1_x[i] = prev_obj1_x[i+1];
    prev_obj1_y[i] = prev_obj1_y[i+1];
    prev_obj1_z[i] = prev_obj1_z[i+1];
    
}

prev_obj1_x[4] = input[31];
prev_obj1_y[4] = input[32];
prev_obj1_z[4] = input[33];


// moving average
float vx = 0;
float vy = 0;
float vz = 0;
for (int i=0; i<5; i++) {
    vx += prev_obj1_x[i];
    vy += prev_obj1_y[i];
    vz += prev_obj1_z[i];
}
vx = vx/5;
vy = vy/5;
vz = vz/5;

// evaluate if the motion is not because of noise
if (sqrt(pow(vx-obj1x_prev,2) + pow(vy-obj1y_prev,2) + pow(vz-obj1z_prev,2)) > mov_thr)
    obj1_movement = true;

obj1x_prev = vx;
obj1y_prev = vy;
obj1z_prev = vz;

}


if ((pres_obj1[0] = 0) && (pres_obj1[1] = 1)) {
    
    for (int i=0; i<4; i++) {
        prev_obj1_x[i] = input[31];
        prev_obj1_y[i] = input[32];
        prev_obj1_z[i] = input[33];  }

obj1x_prev = input[31];
obj1y_prev = input[32];
obj1z_prev = input[33];


}

if (obj1_movement)
   return true;
else
   return false;
}



bool verbRec::move_obj2()
{
 
static float mov_thr = 0.05;
    
/*static*/ bool obj2_movement = false;

static float pres_obj2[2] = {input[38], input[38]};

static float prev_obj2_x[5] = {input[35], input[35], input[35], input[35], input[35]};
static float prev_obj2_y[5] = {input[36], input[36], input[36], input[36], input[36]};
static float prev_obj2_z[5] = {input[37], input[37], input[37], input[37], input[37]};

static float obj2x_prev = input[35];
static float obj2y_prev = input[36];
static float obj2z_prev = input[37];


pres_obj2[0] = pres_obj2[1];
pres_obj2[1] = input[38];

if ((pres_obj2[0] == 1) && (pres_obj2[1] == 1)) {
    
    // smoothing
    for (int i=0; i<4; i++) {
        prev_obj2_x[i] = prev_obj2_x[i+1];
        prev_obj2_y[i] = prev_obj2_y[i+1];
        prev_obj2_z[i] = prev_obj2_z[i+1];
        
    }
    
    prev_obj2_x[4] = input[35];
    prev_obj2_y[4] = input[36];
    prev_obj2_z[4] = input[37];
    
    
    // moving average
    float vx = 0;
    float vy = 0;
    float vz = 0;
    for (int i=0; i<5; i++) {
        vx += prev_obj2_x[i];
        vy += prev_obj2_y[i];
        vz += prev_obj2_z[i];
    }
    vx = vx/5;
    vy = vy/5;
    vz = vz/5;
    
    // evaluate if the motion is not because of noise
    if (sqrt(pow(vx-obj2x_prev,2) + pow(vy-obj2y_prev,2) + pow(vz-obj2z_prev,2)) > mov_thr)
        obj2_movement = true;
        
        obj2x_prev = vx;
        obj2y_prev = vy;
        obj2z_prev = vz;
        
        }


if ((pres_obj2[0] == 0) && (pres_obj2[1] == 1)) {
    
    for (int i=0; i<4; i++) {
        prev_obj2_x[i] = input[35];
        prev_obj2_y[i] = input[36];
        prev_obj2_z[i] = input[37];  }
        
    
    obj2x_prev = input[35];
    obj2y_prev = input[36];
    obj2z_prev = input[37];
    
    
}

if (obj2_movement)
   return true;
else
   return false;
}


bool verbRec::move_obj3()
{
 
static float mov_thr = 0.05;
    
/*static*/ bool obj3_movement = false;

static float pres_obj3[2] = {input[42], input[42]};

static float prev_obj3_x[5] = {input[39], input[39], input[39], input[39], input[39]};
static float prev_obj3_y[5] = {input[40], input[40], input[40], input[40], input[40]};
static float prev_obj3_z[5] = {input[41], input[41], input[41], input[41], input[41]};

static float obj3x_prev = input[39];
static float obj3y_prev = input[40];
static float obj3z_prev = input[41];


pres_obj3[0] = pres_obj3[1];
pres_obj3[1] = input[42];

if ((pres_obj3[0] == 1) && (pres_obj3[1] == 1)) {
    
    // smoothing
    for (int i=0; i<4; i++) {
        prev_obj3_x[i] = prev_obj3_x[i+1];
        prev_obj3_y[i] = prev_obj3_y[i+1];
        prev_obj3_z[i] = prev_obj3_z[i+1];
        
    }
    
    prev_obj3_x[4] = input[39];
    prev_obj3_y[4] = input[40];
    prev_obj3_z[4] = input[41];
    
    
    // moving average
    float vx = 0;
    float vy = 0;
    float vz = 0;
    for (int i=0; i<5; i++) {
        vx += prev_obj3_x[i];
        vy += prev_obj3_y[i];
        vz += prev_obj3_z[i];
    }
    vx = vx/5;
    vy = vy/5;
    vz = vz/5;
    
    // evaluate if the motion is not because of noise
    if (sqrt(pow(vx-obj3x_prev,2) + pow(vy-obj3y_prev,2) + pow(vz-obj3z_prev,2)) > mov_thr)
        obj3_movement = true;
        
        obj3x_prev = vx;
        obj3y_prev = vy;
        obj3z_prev = vz;
        
        }


if ((pres_obj3[0] == 0) && (pres_obj3[1] == 1)) {
    
    for (int i=0; i<4; i++) {
         prev_obj3_x[i] = input[39];
         prev_obj3_y[i] = input[40];
         prev_obj3_z[i] = input[41];  }
    
    obj3x_prev = input[39];
    obj3y_prev = input[40];
    obj3z_prev = input[41];
    
    
}

if (obj3_movement)
   return true;
else
   return false;
}


bool verbRec::move_obj4()
{
    
static float mov_thr = 0.05;
    
/*static*/ bool obj4_movement = false;

static float pres_obj4[2] = {input[46], input[46]};

static float prev_obj4_x[5] = {input[43], input[43], input[43], input[43], input[43]};
static float prev_obj4_y[5] = {input[44], input[44], input[44], input[44], input[44]};
static float prev_obj4_z[5] = {input[45], input[45], input[45], input[45], input[45]};

static float obj4x_prev = input[43];
static float obj4y_prev = input[44];
static float obj4z_prev = input[45];

pres_obj4[0] = pres_obj4[1];
pres_obj4[1] = input[46];

if ((pres_obj4[0] == 1) && (pres_obj4[1] == 1)) {
    
    // smoothing
    for (int i=0; i<4; i++) {
        prev_obj4_x[i] = prev_obj4_x[i+1];
        prev_obj4_y[i] = prev_obj4_y[i+1];
        prev_obj4_z[i] = prev_obj4_z[i+1];
        
    }
    
    prev_obj4_x[4] = input[43];
    prev_obj4_y[4] = input[44];
    prev_obj4_z[4] = input[45];
    
    
    // moving average
    float vx = 0;
    float vy = 0;
    float vz = 0;
    for (int i=0; i<5; i++) {
        vx += prev_obj4_x[i];
        vy += prev_obj4_y[i];
        vz += prev_obj4_z[i];
    }
    vx = vx/5;
    vy = vy/5;
    vz = vz/5;
    
    // evaluate if the motion is not because of noise
    if (sqrt(pow(vx-obj4x_prev,2) + pow(vy-obj4y_prev,2) + pow(vz-obj4z_prev,2)) > mov_thr)
        obj4_movement = true;
        
        obj4x_prev = vx;
        obj4y_prev = vy;
        obj4z_prev = vz;
        
        }


if ((pres_obj4[0] == 0) && (pres_obj4[1] == 1)) {
    
    for (int i=0; i<4; i++)  {
          prev_obj4_x[i] = input[43];
          prev_obj4_y[i] = input[44];
          prev_obj4_z[i] = input[45];  }
    
    obj4x_prev = input[43];
    obj4y_prev = input[44];
    obj4z_prev = input[45];
    
    
}

if (obj4_movement)
  return true;
else
  return false;
}

// **************************************************************************************** //

bool verbRec::take(int* object)
{
	// todo
	return false;
}

/*bool verbRec::put(int* object)
{
	// todo
	return false;
}*/

int verbRec::put()

{
    
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
    
    
    int prev_obj1 = 0;
    int prev_obj2 = 0;
    int prev_obj3 = 0;
    int prev_obj4 = 0;
    
    for (int i=0; i<9; i++) {
        prev_obj1 += pres_obj1[i];
        prev_obj2 += pres_obj2[i];
        prev_obj3 += pres_obj3[i];
        prev_obj4 += pres_obj4[i];
    }
    
    // if any of the 4 Objects is lifted
    int obj=0;
    if ((prev_obj1 <9) && (pres_obj1[9] == 1))
    {obj=1;}
    
    else if  ((prev_obj2 <9) && (pres_obj2[9] == 1))
    {obj=2;}
    
    else if  ((prev_obj3 <9) && (pres_obj3[9] == 1))
    {obj=3;}
    
    else if  ((prev_obj4 <9) && (pres_obj4[9] == 1))
    {obj=4;}    

    else
    {obj=0;}
    
    return obj;
}




bool verbRec::push(int* object)
{
	// todo
	return false;
}

bool verbRec::pull(int* object)
{
	// todo
	return false;
}

bool verbRec::point(int* object)
{
	// todo
	return false;
}

/*bool verbRec::lift(int* object)
{
	// todo
	return false;
}*/

int verbRec::lift()
{
    
    float mov_thr = 0.05;
    
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
    
    
    int prev_obj1 = 0;
    int prev_obj2 = 0;
    int prev_obj3 = 0;
    int prev_obj4 = 0;
    
    for (int i=0; i<9; i++) {
        prev_obj1 += pres_obj1[i];
        prev_obj2 += pres_obj2[i];
        prev_obj3 += pres_obj3[i];
        prev_obj4 += pres_obj4[i];
    }
    
    
    static float r_hand_z[5] = {input[11],input[11],input[11],input[11],input[11]};
    static float l_hand_z[5] = {input[8],input[8],input[8],input[8],input[8]};
    
    static float pre_rhz = input[11];
    static float pre_lhz = input[8];
    
    
    
    for (int i=0; i<4; i++) {
        r_hand_z[i] = r_hand_z[i+1];
        l_hand_z[i] = l_hand_z[i+1];
    }
    
    r_hand_z[4] = input[11];
    l_hand_z[4] = input[8];
    
    // moving average
    float rhz = 0;
    float lhz = 0;
    for (int i=0; i<5; i++) {
        rhz += r_hand_z[i];
        lhz += l_hand_z[i];
    }
    rhz = rhz/5;
    lhz = lhz/5;
    
    
    
    // if any of the 4 Objects is lifted
    int obj = 0;
    
    if ((prev_obj1 <9) && (pres_obj1[9] == 0) && ((sqrt(rhz-pre_rhz) > mov_thr) || (sqrt(lhz-pre_lhz) > mov_thr)))
        obj = 1;
    
    else if ((prev_obj2 <9) && (pres_obj2[9] == 0) && ((sqrt(rhz-pre_rhz) > mov_thr) || (sqrt(lhz-pre_lhz) > mov_thr)))
        obj = 2;
    
    else if ((prev_obj3 <9) && (pres_obj3[9] == 0) && ((sqrt(rhz-pre_rhz) > mov_thr) || (sqrt(lhz-pre_lhz) > mov_thr)))
        obj = 3;
    
    else if ((prev_obj4 <9) && (pres_obj4[9] == 0) && ((sqrt(rhz-pre_rhz) > mov_thr) || (sqrt(lhz-pre_lhz) > mov_thr)))
        obj = 4;
    
    else
        obj = 0;
    
    
    pre_lhz = lhz;
    pre_rhz = rhz;
    
    return obj;
}

/*bool verbRec::have(int* object)
{
	// todo
	return false;
}*/

int verbRec::have(/*int ra, int obj*/)
{
    	static float dis_thr = 0.5;
    	//int ra_obj[2][4] = {{0,0,0,0},{0,0,0,0}};
	int ret = 0;
	
	for (int i=0; i<4; i++) {
		r_has_obj[i] = 0;
		a_has_obj[i] = 0;
	}
    
    	//////// ROBOT
    	if ((input[34] == 1) && (sqrt(pow(input[31],2)+pow(input[32],2)+pow(input[33],2)) < dis_thr) ) {
        	//ra_obj[0][0] = 1;
		r_has_obj[0] = 1;
		ret = 10;
	}
    
    	if ((input[38] == 1) && (sqrt(pow(input[35],2)+pow(input[36],2)+pow(input[37],2)) < dis_thr) ) {
        	//ra_obj[0][1] = 1;
		r_has_obj[1] = 1;
		ret = 10;
	}
    
    	if ((input[42] == 1) && (sqrt(pow(input[39],2)+pow(input[40],2)+pow(input[41],2)) < dis_thr) ) {
        	//ra_obj[0][2] = 1;
		r_has_obj[2] = 1;
		ret = 10;
	}    
    	if ((input[44] == 1) && (sqrt(pow(input[43],2)+pow(input[44],2)+pow(input[45],2)) < dis_thr) ) {
        	//ra_obj[0][3] = 1;
		r_has_obj[3] = 1;
		ret = 10;
	}
    
     	////// Agent
     	float diff_1 = sqrt(pow(input[24]-input[31],2)+pow(input[25]-input[32],2)+pow(input[26]-input[33],2));
     	if ((input[34] == 1) && (diff_1 < dis_thr) ) {
     		//ra_obj[1][0] = 1;
		a_has_obj[0] = 1;
		if (ret<10)
			ret = 1;
		else
			ret = 11;
     	}
     
     	float diff_2 = sqrt(pow(input[24]-input[35],2)+pow(input[25]-input[36],2)+pow(input[26]-input[37],2));
     	if ((input[38] == 1) && (diff_2 < dis_thr) ) {
     		//ra_obj[1][1] = 1;
		a_has_obj[1] = 1;
		if (ret<10)
			ret = 1;
		else
			ret = 11;
     	}
     
     	float diff_3 = sqrt(pow(input[24]-input[39],2)+pow(input[25]-input[40],2)+pow(input[26]-input[41],2));
     	if ((input[42] == 1) && (diff_3 < dis_thr) ) {
     		//ra_obj[1][2] = 1;
		a_has_obj[2] = 1;
		if (ret<10)
			ret = 1;
		else
			ret = 11;
     	}
     
     	float diff_4 = sqrt(pow(input[24]-input[43],2)+pow(input[25]-input[44],2)+pow(input[26]-input[45],2));
     	if ((input[44] == 1) && (diff_4 < dis_thr) ) {
     		//ra_obj[1][3] = 1;
		a_has_obj[3] = 1;
		if (ret<10)
			ret = 1;
		else
			ret = 11;
    	}
    	
	//return ra_obj[ra][obj];
    	return ret;
}

bool verbRec::grasp(int* object)
{
	// todo
	return false;
}

bool verbRec::give(int* object)
{
	// todo
	return false;
}


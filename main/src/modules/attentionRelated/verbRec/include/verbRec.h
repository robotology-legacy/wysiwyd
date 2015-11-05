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

#include <math.h>
#include <iostream>
#include <iomanip>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

class verbRec : public RFModule {
private:
    	double period;
    	Port Port_out; // a port to receive input
    	Port Port_in; // a port for output
    	int count;

	float input[47];
	float output[11];

	int r_has_obj[4];
	int a_has_obj[4];

	ConstString objectNames[4];

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

    	bool updateModule();
    	bool respond(const Bottle& cmd, Bottle& reply);

	void whatVerbs(const Bottle& command, char* output);
	void readData(const Bottle& command, float* input);

	bool wave();

	bool move_obj1();
	bool move_obj2();
	bool move_obj3();
	bool move_obj4();
	bool move(int* object);


	bool take(int* object);
//	bool put(int* object);
	int put();
	bool push(int* object);
	bool pull(int* object);
	bool point(int* object);

//	bool lift(int* object);
	int lift();
//	bool have(int* object);
	int have(/*int ra, int obj*/);
	bool grasp(int* object);
	bool give(int* object);
};

#include "verbRec.h"

bool verbRec::configure(yarp::os::ResourceFinder &rf)
{
    	string moduleName = rf.check("name", Value("verbRec")).asString().c_str();
    	setName(moduleName.c_str());

    	yInfo() << moduleName << " : finding configuration files...";
    	period = rf.check("period", Value(0.1)).asDouble();


	count=0;
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
// ************************ //
    	// read the input
//	Bottle botRead;
//    	Port_in.read(botRead);
//    	printf("Got message: %s\n", botRead.toString().c_str());

	// prepare a message
//        Bottle botWrite; 
//        botWrite.addString("testing");
//	count++;
//        botWrite.addInt(count);
        // send the message
//        Port_out.write(botWrite);

// ************************ //

        if (command.get(0).asString()=="quit")
            return false;
        else {
        	reply=command;

		// prepare a message
        	Bottle botWrite; 
		char out[100];
		whatVerbs(command, out);
        	botWrite.addString(out/*"Hej hej"*//*whatVerbs(command)*/);
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
//	cout<<"input = "<< command.get(0).asString() <<endl;

//	float input[47];
	float output[11] = {0,0,0,0,0,0,0,0,0,0,0};

	readData(command, input);

	cout<<"input converted = ";  // temporary
	for (int i=0; i<47; i++)
		cout<< input[i] <<" ";  // temporary
	cout<<endl;  // temporary
	cout<<endl;  // temporary

// ************* start recognize verbs **************** //

	if (wave())
		output[0] = 1;

// ************* end recognize verbs **************** //

	cout<<"output as a float array = ";  // temporary
	for (int i=0; i<11; i++)
		cout<< output[i] <<" ";
	cout<<endl;  // temporary
	cout<<endl;  // temporary

	// this will be the output converted to a string or a bottle
	if (output[0]) {
		out[0] = 'W';
		out[1] = 'a';
		out[2] = 'w';
		out[3] = 'i';
		out[4] = 'n';
		out[5] = 'g';
		out[6] = '\0';
	}
	else {
		out[0] = 'N';
		out[1] = 'o';
		out[2] = ' ';
		out[3] = 'a';
		out[4] = 'c';
		out[5] = 't';
		out[6] = 'i';
		out[7] = 'o';
		out[8] = 'n';
		out[9] = '\0';
	}
}

// reimplement this later
void verbRec::readData(const Bottle& command, float* input) {
	char line[1024];
    	char str[1024];

//	fgets(line,1024,fp);

//	line = command.get(0).asString(); 
	strcpy(line, command.toString().c_str());

	int i=0;
	int j=0;
	int nbrOfSC=0;

	// find the third (
	while(nbrOfSC < 3) {
		if (line[i] == '(')
			nbrOfSC++;
		i++;
	}

	// elbow left x coord
	nbrOfSC=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[0] = (float)atof(str);

//		printf("%f\n", input[0]);

	// elbow left y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[1] = (float)atof(str);

//		printf("%f\n", input[1]);

	// elbow left z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[2] = (float)atof(str);

//		printf("%f\n", input[2]);

	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == '(')
			nbrOfSC++;
		i++;
	}

	// elbow right x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[3] = (float)atof(str);

//		printf("%f\n", input[3]);

	// elbow right y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[4] = (float)atof(str);

//		printf("%f\n", input[4]);

	// elbow right z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[5] = (float)atof(str);

//		printf("%f\n", input[5]);

	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == '(')
			nbrOfSC++;
		i++;
	}

	// hand left x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[6] = (float)atof(str);

//		printf("%f\n", input[6]);

	// hand left y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[7] = (float)atof(str);

//		printf("%f\n", input[7]);

	// hand left z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[8] = (float)atof(str);

//		printf("%f\n", input[8]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == '(')
			nbrOfSC++;
		i++;
	}

	// hand right x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[9] = (float)atof(str);

//		printf("%f\n", input[9]);

	// hand right y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[10] = (float)atof(str);

//		printf("%f\n", input[10]);

	// hand right z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[11] = (float)atof(str);

//		printf("%f\n", input[11]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == '(')
			nbrOfSC++;
		i++;
	}

	// head x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[12] = (float)atof(str);

//		printf("%f\n", input[12]);

	// head y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[13] = (float)atof(str);

//		printf("%f\n", input[13]);

	// head z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[14] = (float)atof(str);

//		printf("%f\n", input[14]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == '(')
			nbrOfSC++;
		i++;
	}

	// shoulderCenter x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[15] = (float)atof(str);

//		printf("%f\n", input[15]);

	// shoulderCenter y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[16] = (float)atof(str);

//		printf("%f\n", input[16]);

	// shoulderCenter z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[17] = (float)atof(str);

//		printf("%f\n", input[17]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == '(')
			nbrOfSC++;
		i++;
	}

	// shoulderLeft x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[18] = (float)atof(str);

//		printf("%f\n", input[18]);

	// shoulderLeft y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[19] = (float)atof(str);

//		printf("%f\n", input[19]);

	// shoulderLeft z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[20] = (float)atof(str);

//		printf("%f\n", input[20]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == '(')
			nbrOfSC++;
		i++;
	}

	// shoulderRight x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[21] = (float)atof(str);

//		printf("%f\n", input[21]);

	// shoulderRight y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[22] = (float)atof(str);

//		printf("%f\n", input[22]);

	// shoulderRight z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[23] = (float)atof(str);

//		printf("%f\n", input[23]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == '(')
			nbrOfSC++;
		i++;
	}

	// spine x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[24] = (float)atof(str);

//		printf("%f\n", input[24]);

	// spine y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[25] = (float)atof(str);

//		printf("%f\n", input[25]);

	// spine z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[26] = (float)atof(str);

//		printf("%f\n", input[26]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
	}

	// partner x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[27] = (float)atof(str);

//		printf("%f\n", input[27]);

	// partner y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[28] = (float)atof(str);

//		printf("%f\n", input[28]);

	// partner z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[29] = (float)atof(str);

//		printf("%f\n", input[29]);

	// partner presence
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[30] = (float)atof(str);

//		printf("%f\n", input[30]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
	}

	// object 1 (cross) x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[31] = (float)atof(str);

//		printf("%f\n", input[31]);

	// object 1 (cross) y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[32] = (float)atof(str);

//		printf("%f\n", input[32]);

	// object 1 (cross) z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[33] = (float)atof(str);

//		printf("%f\n", input[33]);

	// object 1 (cross) presence
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[34] = (float)atof(str);

//		printf("%f\n", input[34]);

	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
	}

	// object 2 (circle) x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[35] = (float)atof(str);

//		printf("%f\n", input[35]);

	// object 2 (circle) y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[36] = (float)atof(str);

//		printf("%f\n", input[36]);

	// object 2 (circle) z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[37] = (float)atof(str);

//		printf("%f\n", input[37]);

	// object 2 (circle) presence
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[38] = (float)atof(str);

//		printf("%f\n", input[38]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
	}

	// object 3 (croco) x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[39] = (float)atof(str);

//		printf("%f\n", input[39]);

	// object 3 (croco) y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[40] = (float)atof(str);

//		printf("%f\n", input[40]);

	// object 3 (croco) z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[41] = (float)atof(str);

//		printf("%f\n", input[41]);

	// object 3 (croco) presence
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[42] = (float)atof(str);

//		printf("%f\n", input[42]);


	nbrOfSC=0;
	while(nbrOfSC < 2) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
	}

	// object 4 (mouse) x coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[43] = (float)atof(str);

//		printf("%f\n", input[43]);

	// object 4 (mouse) y coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[44] = (float)atof(str);

//		printf("%f\n", input[44]);

	// object 4 (mouse) z coord
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ' ')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[45] = (float)atof(str);

//		printf("%f\n", input[45]);

	// object 4 (mouse) presence
	nbrOfSC=0;
	j=0;
	while(nbrOfSC < 1) {
		str[j] = line[i];
		if (line[i] == ')')
			nbrOfSC++;
		i++;
		j++;
	}
	str[j] = '\0';
	input[46] = (float)atof(str);

//		printf("%f\n", input[46]);
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


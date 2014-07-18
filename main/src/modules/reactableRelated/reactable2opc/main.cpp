/*
//Stephane : the main is in rt2objCol.cpp
//I cannot manage to compile with it in a separate file. This module was wrongly though from the begining. Someone should recode it from scracth.

#include "rt2objCol.h"

bool loadingEvent;
int main(int argc, char* argv[])
{
	if (argc >= 2 && strcmp(argv[1], "-h") == 0)
	{
		std::cout << "usage: Reactable2OPC [port]\n";
		return 0;
	}

	int port = 3333;
	if (argc >= 2) port = atoi(argv[1]);

	Network yarp;
	if (!yarp.checkNetwork())
		return false;

	loadingEvent = false;

	Reactable2OPC *dump = new Reactable2OPC();
	TuioClient client(dump->OSC_INPUT_TABLE_PORT);
	client.addTuioListener(dump);
	client.connect(true);
	delete dump;

	return 0;
}


*/
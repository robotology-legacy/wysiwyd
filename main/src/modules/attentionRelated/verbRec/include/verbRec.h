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
};

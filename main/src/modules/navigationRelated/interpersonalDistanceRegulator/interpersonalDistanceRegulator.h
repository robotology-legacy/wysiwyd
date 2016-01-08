#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/SVD.h>
#include "wrdac/clients/icubClient.h"
#include "wrdac/subsystems/subSystem_iKart.h"
#include <map>
#include "interpersonalDistanceRegulator_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wysiwyd::wrdac;

class InterpersonalDistanceRegulator: public RFModule, public interpersonalDistanceRegulator_IDL
{
public:
	//Thrift methods
	virtual bool pause() { isPaused = true; return true; }
	virtual bool resume() { isPaused = false; return true; }
	virtual bool quit() { isQuitting = true; return true; }

private:

	bool isPaused, isQuitting;

	//Clients
	SubSystem_iKart* ikart;
	OPCClient* opc;

	//Parameters
	double preferedDistanceToPeople;
	double fwdSpeed, backwdSpeed, turningSpeed;

	//Module related
	Agent*	partner;
    double period;
    Port    rpc;

public:
	bool configure(yarp::os::ResourceFinder &rf);
	bool updateModule();
	bool respond(const Bottle& cmd, Bottle& reply);

    bool interruptModule()
	{
		opc->interrupt();

        return true;
    }

    bool close()
    {
		opc->close();
		ikart->Close();
		delete ikart;
		delete opc;
        return true;
    }

    double getPeriod()
    {
        return period;
    }

};

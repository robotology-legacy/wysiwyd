#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;


class IIE : public RFModule {
private:

    ICubClient *iCub;
    double      period;
    string      grammarToString(string sPath);
    Port        rpc;

    yarp::os::Port Port2Supervisor;       // a port to communicate with the reasoning module
    string Port2SupervisorName;

public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule()
    {
        return true;
    }

    bool close();

    double getPeriod()
    {
        return period;
    }
    
    bool updateModule();

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

#include "wrdac/clients/icubClient.h"
#include <map>
#include <fstream>
#include <string>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;


class jointsAwareness : public RFModule {
private:

    //conf options
    wysiwyd::wrdac::ICubClient  *iCub;
    double      period;

    string arm, robot;

    yarp::os::Port   rpcPort;

public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();;

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

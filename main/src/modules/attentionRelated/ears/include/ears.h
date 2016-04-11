#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

class ears : public RFModule {
private:
    ICubClient *iCub;
    double      period;
    Port        rpc;
    Port        portToBehavior;
    BufferedPort<Bottle>        portTarget;

    Mutex m;
    bool bShouldListen;

    std::string      MainGrammar;

public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool    updateModule();
    bool    populateSpecific1(Bottle bInput);

    bool    addUnknownEntity(Bottle bInput);
    bool    setSaliencyEntity(Bottle bInput);

    bool    populateABM(Bottle bInput);


    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

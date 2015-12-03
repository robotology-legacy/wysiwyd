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
    string      target;    

    bool bShouldListen;

    std::string      grammarToString(std::string sPath);
    std::string      MainGrammar;

    yarp::os::Mutex mutex;


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
    bool    populateSpecific1(Bottle bInput);

    bool    addUnknownEntity(Bottle bInput);
    bool    setSaliencyEntity(Bottle bInput);

    bool    populateABM(Bottle bInput);


    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

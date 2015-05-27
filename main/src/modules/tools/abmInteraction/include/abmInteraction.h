#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

class abmInteraction : public RFModule {
private:

    ICubClient *iCub;
    double      period;
    string      grammarToString(string sPath);
    Port        rpc;
    string      nameGrammarHumanFeedback;
    string      nameGrammarYesNo;
    bool        tryAgain;
    int         bestRank ;

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

    void nodeFeedback();
    bool nodeYesNo();

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

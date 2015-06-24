#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

class abmInteraction : public RFModule {
private:

    //rfh module
    ICubClient      *iCub;
    double          period;
    string          grammarToString(string sPath);
    Port            rpc;

    //global
    vector<string>     vAugmentedTime;
    vector<string>::iterator it_augmentedTime ;

    //conf options
    string          nameGrammarHumanFeedback;
    string          nameGrammarYesNo;
    string          img_provider_port ;
    string          agentName ;
    string          resume ;
    int             rememberedInstance;

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

    //spoken interaction node
    void nodeFeedback(bool tryAgain, std::pair<std::string, int> & bestTimeAndRank);
    bool nodeYesNo();
    
    bool createAugmentedTimeVector(std::pair<std::string, int> & bestTimeAndRank);
    bool createBestAugmentedTime(std::pair<std::string, int> & bestTimeAndRank);
    bool insertFeedback(int feedback, int instanceFeedback);

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

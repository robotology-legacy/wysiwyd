#include "wrdac/clients/icubClient.h"
#include <map>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

class learnPrimitive : public RFModule {
private:

    //conf options
    wysiwyd::wrdac::ICubClient  *iCub;
    double      period;

    std::string      grammarToString(std::string sPath);
    yarp::os::Port   rpcPort;
    yarp::os::Port   portToArm;

    std::string      GrammarYesNo;
    std::string      GrammarNameAction;

    std::map<std::string, int> mProtoActionEnd;
    std::map<std::string, double> mProtoActionSpeed;
    std::map<std::string, int> mBodyPartEnd;
    std::map<std::string, double> mBodyPartSpeed;

public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    //spoken interaction node
    //bool nodeYesNo();
    //grammar with proto-action, primitive and action. Can specify if you need something particular (when 1. "I will teach you how to") or if use to
    //combine during the learning :
    //-> proto-action cannot be (proactiveTagging is doing that)
    //-> primitive can only call proto so should use proto-action,
    //-> action can call whatever, included other action ("any")
    yarp::os::Bottle nodeNameAction(std::string actionTypeNeeded = "any");


    yarp::os::Bottle basicCommand(std::string sActionName, std::string sBodypartName, int maxAngle=10);
    yarp::os::Bottle learn();
    yarp::os::Bottle learnPrim();
    yarp::os::Bottle learnAction();
    bool updateProtoAction(yarp::os::ResourceFinder &rf);

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

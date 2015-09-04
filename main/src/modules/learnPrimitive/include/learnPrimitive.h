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
    std::string      GrammarDescribeAction;

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

    yarp::os::Bottle basicCommand(std::string sActionName, std::string sBodypartName, int maxAngle=10);
    bool updateProtoAction(yarp::os::ResourceFinder &rf);

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

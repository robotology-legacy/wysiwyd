#include "wrdac/clients/icubClient.h"
#include <map>
#include <fstream>
#include <string>
#include <Rcpp.h>
//#include <RcppArmadillo.h>
#include <RInside.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;


class learnPrimitive : public RFModule {
private:

    //conf options
    wysiwyd::wrdac::ICubClient  *iCub;
    double      period;

    yarp::os::Port   rpcPort;
    yarp::os::Port   portToArm;

    std::string      GrammarYesNo;
    std::string      GrammarNameAction;
    std::string      GrammarTypeAction;

    std::string      pathToIniFile ;

    std::map<std::string, int> mProtoActionEnd;
    std::map<std::string, double> mProtoActionSpeed;
    std::map<std::string, int> mBodyPartEnd;
    std::map<std::string, double> mBodyPartSpeed;

    std::vector<yarp::os::Bottle> vPrimitiveActionBottle;
    //   open    (hand)     ( (unfold thumb) (unfold index) (unfold middle) (unfold ring) )
    //   close   (hand)     ( (fold thumb) (fold index) (fold middle) (fold ring) )
    //   b.get(1) b.get(2)  b.get(3)
    //   name     arg        list of proto-action

    std::vector<yarp::os::Bottle> vActionBottle;
    //   point   (left)     ( (close hand) (unfold index) (goto left) )
    //   b.get(1) b.get(2)  b.get(3)
    //   name     arg        list of proto/prim/action

    RInside R;

    Rcpp::NumericMatrix createSquareMatrix(const int);


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


    yarp::os::Bottle protoCommand(std::string sActionName, std::string sBodypartName, int maxAngle=10);
    yarp::os::Bottle primitiveCommand(std::string sActionName, std::string sBodypartName);
    yarp::os::Bottle actionCommand(std::string sActionName, std::string sArgument);

    yarp::os::Bottle learn();
    yarp::os::Bottle learnPrim();
    yarp::os::Bottle learnAction();

    yarp::os::Bottle execute();

    bool updateProtoAction(yarp::os::ResourceFinder &rf);
    bool updatePrimitive(yarp::os::ResourceFinder &rf);
    bool updateAction(yarp::os::ResourceFinder &rf);

    bool saveToIniFile(string sType, string sName, string sArg, yarp::os::Bottle bActionDescription);

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);

    yarp::os::Bottle testRInside();
};

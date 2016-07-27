#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

class Planner : public RFModule {
private:

    ICubClient *iCub;
    double      period;
    Port        rpc;
    Port        portToBehavior;
    Port        toHomeo;
    BufferedPort<Bottle>        port_behavior_context;
    Bottle avaiPlansList;
    vector<Bottle> newPlan;
    vector<string> plan_list;
    vector<string> action_list;
    vector<int> priority_list;
    vector<string>::iterator actPt;
    vector<int>::iterator prioPt;
    Bottle grpPlans;
    Bottle gandalf;
    Bottle adhd;
    int id;
    int fulfill;
    // Bottle* current_goal;
    // string current_goal;
    string objectType;
    string object;

    // bool bShouldListen;

    std::string      grammarToString(std::string sPath);
    std::string      MainGrammar;

    yarp::os::Mutex mutex;
    bool following;
    bool ordering;
    Port behaviorsRPC;
    void run(Bottle args=Bottle());


public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool configurePlans(yarp::os::ResourceFinder &rf);

    // bool interruptModule()
    // {
    //     return true;
    // }

    bool freeze_all();

    bool unfreeze_all();

    bool checkKnown(const Bottle& command, Bottle& avaiPlansList, string foundPlan);
    
    bool close();

    // double getPeriod()
    // {
    //     return period;
    // }


    bool updateModule();

    bool triggerBehavior(Bottle cmd);

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

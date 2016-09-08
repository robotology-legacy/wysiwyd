#include "wrdac/clients/icubClient.h"

class Planner : public RFModule {
private:

    ICubClient *iCub;
    double      period;
    Port        rpc;
    Port        portToBehavior;
    Port        toHomeo;
    Port        getState;
    BufferedPort<Bottle>        port_behavior_context;
    Bottle avaiPlansList;
    vector<Bottle> newPlan;
    vector<string> plan_list;
    vector<string> action_list;
    vector<string> object_list;
    vector<string> type_list;
    vector<int> priority_list;
    vector<int> actionPos_list;
    vector<string>::iterator actPt;
    vector<int>::iterator prioPt;
    Bottle grpPlans;
    int id;
    int attemptCnt;

    // bool bShouldListen;

    yarp::os::Mutex mutex;
    bool ordering;
    bool fulfill;
    Port behaviorsRPC;
    void run(Bottle args=Bottle());


public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool configurePlans(yarp::os::ResourceFinder &rf);

    bool freeze_all();

    bool unfreeze_all();

    bool checkKnown(const Bottle& command, Bottle& avaiPlansList);
    
    bool exit();

    // double getPeriod()
    // {
    //     return period;
    // }


    bool updateModule();

    bool triggerBehavior(Bottle cmd);

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

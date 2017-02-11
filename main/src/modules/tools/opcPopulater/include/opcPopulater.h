#include "wrdac/clients/icubClient.h"
#include <time.h>

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

class opcPopulater : public RFModule {
private:

    ICubClient *iCub;
    double      period;
    Port        rpc;

    double X_obj;
    double Y_obj;
    double Z_obj;
    double X_ag;
    double Y_ag;
    double Z_ag;
    double noise;
    vector<double> spd1;
    vector<double> spd2;
    bool move;
    int iter;



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
    bool    populateEntityRandom(Bottle bInput);
    bool    populateRedBall();
    bool    populateMoving();
    bool    populateSpecific();
    bool    populateSpecific1(Bottle bInput);
    bool    populateSpecific2();
    bool    populateSpecific3();

    bool    addUnknownEntity(Bottle bInput);
    bool    setSaliencyEntity(Bottle bInput);
    bool    setValueEntity(Bottle bInput);

    bool    populateABM(Bottle bInput);
    bool    populateScenario1();
    bool    populateScenario2();
    bool    populateScenario3();
    bool    populateScenario4();
    bool    populateScenario5();
    bool    populateScenario6();
    bool    populateABMiCubStory(Bottle bInput);
    bool    storyFromPOV(Bottle bInput);


    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

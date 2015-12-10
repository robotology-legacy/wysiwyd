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
    bool    populateSpecific2();

    bool    addUnknownEntity(Bottle bInput);
    bool    setSaliencyEntity(Bottle bInput);

    bool    populateABM(Bottle bInput);
    bool    populateABMiCubStory(Bottle bInput);


    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

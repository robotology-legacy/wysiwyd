#include "wrdac/clients/icubClient.h"
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;


class proactiveTagging : public RFModule {
private:

    ICubClient  *iCub;

    double      period;

    string      grammarToString(string sPath);
    Port        rpcPort;

    string      GrammarAskNameAgent;
    string      GrammarAskNameObject;
    string      GrammarYesNo;

    double  thresholdDistinguishObjectsRatio; //ratio of saliency needed to detect if 1 object is more salient that the other
    double  thresholdSalienceDetection; //value of saliency needed to detect if 1 object is more salient that the other

    void        checkRelations();

    string        askManner(string agent, string verb, string object);
    Bottle        recogName(string entityType);

    //Configure
    void configureOPC(yarp::os::ResourceFinder &rf);

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


    Bottle  exploreUnknownEntity(Bottle bInput);
    Bottle  exploreEntityByName(Bottle bInput);

    bool updateModule();



    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

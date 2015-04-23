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

    string      nameMainGrammar;
    string      nameGrammarSentenceTemporal;
    string      nameGrammarYesNo;
    string      nameGrammarAskManner;
    string      GrammarSentenceTemporal	;
    string      nameGrammarNodeModality	;
    string      nameGrammarNodeTrainAP	;
    string      nameGrammarNodeTrainSD	;


    void        checkRelations();

    string        askManner(string agent, string verb, string object);

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
    bool    populateOpc();

    void    nodeSentenceTemporal();
    bool nodeYesNo();

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

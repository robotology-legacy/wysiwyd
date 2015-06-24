#include "automaticCalibrationThread.h"


class qRM : public RFModule {
private:

    ICubClient *iCub;
    double      period;
    string      grammarToString(string sPath);
    AutomaticCalibrationThread* calibrationThread;
    Port        rpc;
    string      nameMainGrammar;
    string      nameGrammarAskNameAgent;
    string      nameGrammarAskNameObject;
    string      nameGrammarSentenceTemporal;
    string      nameGrammarYesNo;
    string		nameGrammarNodeTrainAP;
    string      testMax1;
    string		testMax2;

    yarp::os::Port Port2abmReasoning;       // a port to communicate with the reasoning module

    string port2abmReasoningName;

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

    void    mainLoop();

    bool updateModule();
    bool    populateOpc();
    bool    populateSpecific(Bottle bInput);

    Bottle calibrationRT(std::string side);
    void    nodeSentenceTemporal();
    void    nodeTest();
    bool    nodeYesNo();

    Bottle  exploreEntity();

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

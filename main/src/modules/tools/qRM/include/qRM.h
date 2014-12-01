#include "automaticCalibrationThread.h"


class qRM : public RFModule {
private:

    ICubClient *iCub;
    double      period;
    string      grammarToString(string sPath);
    AutomaticCalibrationThread* calibrationThread;
    Port        rpc;
    string      nameMainGrammar;
    string      nameGrammarSentenceTemporal;
    string      nameGrammarYesNo;

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

    Bottle calibrationRT();
    Bottle calibrationRT(string side);

    void    nodeSentenceTemporal();
    bool nodeYesNo();

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

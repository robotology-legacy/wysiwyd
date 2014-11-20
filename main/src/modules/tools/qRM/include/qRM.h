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

    yarp::os::Port Port2SpeechRecog;        // a port to send grammar to the speech recog
    yarp::os::Port Port2ABM;                // a port to communicate with autobiographicalMemory
    yarp::os::Port Port2abmReasoning;       // a port to communicate with the reasoning module

    
    string port2SpeechRecogName;
    string port2abmName;
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
    
    void    nodeSentenceTemporal();

    bool updateModule();
    bool nodeYesNo();
    
    Bottle calibrationRT();
    Bottle calibrationRT(string side);


    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);
};

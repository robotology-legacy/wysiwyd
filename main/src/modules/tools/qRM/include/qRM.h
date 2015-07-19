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
    string      nameGrammarYesNo;

    string      grammarLearnSpAction;

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

    Bottle calibrationRT(std::string side);
    void    nodeSentenceTemporal();
    void    nodeTest();
    bool    nodeYesNo();

    Bottle  exploreUnknownEntity(Bottle bInput);
    Bottle  exploreEntityByName(Bottle bInput);

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);

    Bottle learnSharedPlan(Bottle bInput);
    Bottle executeSharedPlan(Bottle bInput);
    Bottle executeAction(Bottle bInput);



    double  thresholdDistinguishObjectsRatio; //ratio of saliency needed to detect if 1 object is more salient that the other
    double  thresholdSalienceDetection; //value of saliency needed to detect if 1 object is more salient that the other

};

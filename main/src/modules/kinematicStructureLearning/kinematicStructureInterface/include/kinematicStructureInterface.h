#include <string>
#include "wrdac/clients/icubClient.h"

class kinematicStructureInterface : public yarp::os::RFModule {
private:

    //conf options
    double      period;

    std::string robot, moduleName;

    yarp::os::Port   rpcPort;

    //update m_kinectNode of a bpName bodypart with bpCorrespondence
    yarp::os::Bottle updateCorrespondenceOpc(std::string bpName, std::string bpCorrespondence);

    //ICubClient to use ARE
    wysiwyd::wrdac::ICubClient  *iCub;


public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    //RPC & scenarios
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
};

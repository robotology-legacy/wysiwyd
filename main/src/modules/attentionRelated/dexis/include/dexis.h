
#ifndef _SG_H_
#define _SG_H_

#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;



class babbler : public RFModule {
private:

    Port        rpc;
    Port portOutput;


public:
    babbler() {  }
    virtual ~babbler() {  }
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule()
    {
        return true;
    }
    double getPeriod()
    {
        return 0.1;
    }

    bool close();
    void    mainLoop();
    bool updateModule();
    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);

};

#endif

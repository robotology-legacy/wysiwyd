
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
    yarp::os::BufferedPort<yarp::sig::Sound> portOutput;

    int elapsedCycles;
    unsigned int    samples;
    unsigned int    rate;
    unsigned int    channels;

    int             f;


    string port2output;

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

    void generateSound();
    int getNoisySinus(int wave_intensity, int random_offset);
    int newFrequency(int base_frequency);
    int getValue(std::string soundType);

    bool close();
    void    mainLoop();
    bool updateModule();
    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);

};

#endif

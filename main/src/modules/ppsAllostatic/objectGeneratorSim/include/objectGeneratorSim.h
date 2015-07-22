
#ifndef _PS_H_
#define _PS_H_

#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;



class objectGeneratorSim : public RFModule {
private:

    Port        rpc;
    yarp::os::BufferedPort<yarp::os::Bottle> portOutput;
    yarp::os::BufferedPort<yarp::os::Bottle> portOutputTarget;
    yarp::os::BufferedPort<yarp::os::Bottle> portInput;
    yarp::os::Port portSim;
    int elapsedCycles;
    yarp::os::Bottle    objects;
    yarp::os::Bottle    targets;
    yarp::os::Bottle    positions;
    yarp::os::Bottle    tpositions;
    //yarp::os::Bottle    id;
    int listSize[3];

    string port2output;
    yarp::os::Bottle    cmd;

public:
    objectGeneratorSim() {  }
    virtual ~objectGeneratorSim() {  }
    bool configure(yarp::os::ResourceFinder &rf); //Connect to icubSim
    bool interruptModule()
    {
        return true;
    }
    double getPeriod()
    {
        return 0.1;
    }
    void spamTable();
    void createObject(std::string ob, yarp::os::Bottle size, yarp::os::Bottle pos, yarp::os::Bottle colour, bool target=false);
    void createObject(yarp::os::Bottle pos,bool target=false); // Add new object to simulation and OPC
    int destroyObject(); // destroy an object in the scene and OPC
    int moveObject(); //move an object to xyz
    void getCoordinates(std::string object, int id,bool target=false); //send xyz to port

    bool close();
    //void    mainLoop();
    bool updateModule();
    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);

};

#endif

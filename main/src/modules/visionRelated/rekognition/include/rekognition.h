#ifndef REKOGNITION_H
#define REKOGNITION_H

#include <yarp/os/RFModule.h>

class rekognition: public yarp::os::RFModule {
protected:
    yarp::os::Port handlerPort;
    yarp::os::Port abmPort;

public:
    std::string api_addr_base;
    std::string api_key;
    std::string api_secret;

    std::string storing_path;

    bool close();
    bool interrupt();
    bool configure(yarp::os::ResourceFinder &rf);
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
    bool updateModule() { return true; }
};

#endif // REKOGNITION_H

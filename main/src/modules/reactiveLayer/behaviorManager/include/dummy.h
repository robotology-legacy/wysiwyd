#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/SVD.h>
#include <yarp/math/Rand.h>
#include "wrdac/clients/icubClient.h"
#include <map>

#include "behavior.h"

class Dummy: public Behavior
{
private:
    static int n_instances;

    void run(const Bottle &args) {
        yDebug() << "Dummmy::run start " + behaviorName;
        Time::delay(4);
        yDebug() << "Dummmy::run stop " + behaviorName;
    }
    int id;

public:
    Dummy(Mutex* mut, ResourceFinder &rf, std::string behaviorName): Behavior(mut, rf, behaviorName) {
        n_instances++;
        id = n_instances;
    }

    void configure() {
        external_port_name = "None";
        from_sensation_port_name = "None";        
    }


    void close_extra_ports() {
        ;
    }

};


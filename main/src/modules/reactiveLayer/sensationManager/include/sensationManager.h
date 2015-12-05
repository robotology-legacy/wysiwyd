#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>

#include "opcSensation.h"
#include "test.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wysiwyd::wrdac;



class SensationManager: public RFModule
{
private:
    vector<Sensation*> sensations;
    // vector<Port*> to_homeo_rpc;

    string moduleName;

    double period;

public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule()
    {
        return true;
    }

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    bool close();

};


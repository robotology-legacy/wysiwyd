#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/SVD.h>
#include <yarp/math/Rand.h>
#include <map>

#include "dummy.h"
#include "tagging.h"
#include "pointingOrder.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


// struct StimulusEmotionalResponse
// {
//     bool active;
//     Port *output_port;
//     Bottle rpc_command;
// };




class BehaviorManager: public RFModule
{
private:


    vector<Behavior*> behaviors;
    // vector<Port*> to_homeo_rpc;

    string moduleName;

    double period;

    Port rpc_in_port;

    ICubClient *iCub;

    // Port trigger_port;
    
    // vector<TriggerCallback*> trigger_under_ports;
    // vector<TriggerCallback*> trigger_over_ports;
    // vector<Port*> rpc_out_ports;

    // vector<BufferedPort<Bottle>*> sensation_input_ports;

    // vector<string> behavior_names;
    int behavior_to_trigger;
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

    bool updateModule();

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);

};


#include "cvz/core/all.h"
#include "cvz/gui/all.h"

#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

int main(int argc, char * argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        cout << "yarp network is not available!" << endl;
        return 0;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cvz");
    rf.setDefaultConfigFile("default.ini"); //overridden by --from parameter
    rf.configure(argc, argv);

    cvz::core::CvzStack stack;


    //Add head proprioception
    stack.addCvzFromConfigFile("stackExample_head.ini", "head");

    //Add vision
    stack.addCvzFromConfigFile("stackExample_vision.ini", "vision");

    //Add multimodal map
    stack.addCvzFromConfigFile("stackExample_multimodal.ini", "multimodal");

    //Connect the head and vision to the multimodal map
    stack.connectModalities("/head/multimodal", "/multimodal/gaze");
    stack.connectModalities("/vision/multimodal", "/multimodal/vision");



    //Make sure the graph is completed and all the connections are established
    stack.connectModalities();

    //Instantiate the module, each cvz will be spawn as a thread
    stack.configure(rf);

    //Run the module
    stack.runModule();
    return 0;
}
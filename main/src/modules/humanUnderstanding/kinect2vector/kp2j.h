#include <yarp/os/RFModule.h>
#include <yarp/math/Math.h>
#include <kinectWrapper/kinectWrapper_client.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace kinectWrapper;

class KP2JA: public RFModule
{
protected:
    //Kinect Related
    KinectWrapperClient client;
    Player closestPlayer;
    Matrix players;

    //Output
    BufferedPort<Bottle> portVector;

public:

    bool configure(ResourceFinder &rf);

    bool checkCalibration();

    bool close();

    double getPeriod();

    bool updateModule();

};

#include <yarp/os/RFModule.h>
#include <yarp/math/Math.h>

#include <kinectWrapper/kinectWrapper_client.h>

#include <wrdac/helpers.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/Odometry.h>

#include "CameraKinectWrapper.h"
#include "MapBuilder.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace kinectWrapper;
using namespace wysiwyd::wrdac;

class perspectiveTaking: public RFModule
{
protected:
    // Kinect related
    KinectWrapperClient client;

    // RTabmap related
    CameraKinectWrapper* camera;
    CameraThread* cameraThread;
    Rtabmap* rtabmap;
    MapBuilder* mapBuilder;
    OdometryThread* odomThread;
    RtabmapThread* rtabmapThread;

    // OPC related
    OPCClient* opc;
    Agent* partner;

    // RFH related
    Port   rfh;
    Port   handlerPort;

    unsigned int loopCounter;

    Matrix kinect2icub;
    Matrix icub2kinect;
    Eigen::Matrix4f kinect2icub_pcl;
    Eigen::Matrix4f yarp2pcl;

    yarp::os::ResourceFinder resfind;

public:
    bool configure(ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const Bottle& cmd, Bottle& reply);
    bool getRFHMatrix(const string& from, const string& to, Matrix& m);
    double getPeriod();
    bool updateModule();
};

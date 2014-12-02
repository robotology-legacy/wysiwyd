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

    // OPC/RFH related
    //OPCClient* opc;
    Port   rfh;
    Port   handlerPort;

    unsigned int loopCounter;

    Matrix kinect2icub;
    Matrix icub2kinect;
    vtkSmartPointer<vtkMatrix4x4> kinect2icub_vtk;

    yarp::os::ResourceFinder resfind;

    void yarp2vtkKinectMatrix(const yarp::sig::Matrix& kinect2icubYarp, vtkSmartPointer<vtkMatrix4x4> kinect2icubVTK);
    void doPerspectiveTransform(vtkSmartPointer<vtkMatrix4x4> m);

public:
    bool configure(ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const Bottle& cmd, Bottle& reply);
    bool getRFHMatrix(const string& from, const string& to, Matrix& m);
    double getPeriod();
    bool updateModule();
};

#ifndef CAMERAKINECTWRAPPER_H
#define CAMERAKINECTWRAPPER_H

#include <rtabmap/core/CameraRGBD.h>

#include <kinectWrapper/kinectWrapper_client.h>
#include <kinectWrapper/kinectTags.h>

using namespace kinectWrapper;
using namespace rtabmap;
using namespace yarp::sig;

class CameraKinectWrapper : public CameraRGBD
{
public:
    CameraKinectWrapper(KinectWrapperClient &c,
                        float imageRate = 0,
                        const Transform & localTransform = Transform::getIdentity(),
                        float fx = 0.0f,
                        float fy = 0.0f,
                        float cx = 0.0f,
                        float cy = 0.0f);
    virtual ~CameraKinectWrapper();
    static bool available() {return true;}
    bool init();
protected:
    virtual void captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);
private:
    double _depthFocal;
    KinectWrapperClient &client;
    ImageOf<PixelRgb> _rgb;
    ImageOf<PixelMono16> _depth;
    ImageOf<PixelFloat> _depthToDisplay;

    IplImage* depthTmp;
    IplImage* rgbTmp;
};

#endif // CAMERAKINECTWRAPPER_H

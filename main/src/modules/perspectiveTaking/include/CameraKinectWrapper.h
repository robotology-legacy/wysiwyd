/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef VPT_CAMERAKINECTWRAPPER
#define VPT_CAMERAKINECTWRAPPER

#include <rtabmap/core/CameraRGBD.h>

#include <kinectWrapper/kinectWrapper_client.h>
#include <kinectWrapper/kinectTags.h>

using namespace kinectWrapper;
using namespace rtabmap;
using namespace yarp::sig;

class CameraKinectWrapper : public CameraRGBD {
public:
    CameraKinectWrapper(KinectWrapperClient &c,
                        float imageRate = 0,
                        const Transform & localTransform = Transform::getIdentity());
    virtual ~CameraKinectWrapper();
    static bool available() { return true; }
    virtual bool init(const std::string & calibrationFolder = ".");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const {return "";} // unknown

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

    boost::mutex data_mutex;
};

#endif

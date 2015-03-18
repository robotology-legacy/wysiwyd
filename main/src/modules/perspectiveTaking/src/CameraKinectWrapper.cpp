/*
 *
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
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

#include <opencv/cv.h>

#include "CameraKinectWrapper.h"

using namespace std;
using namespace cv;
using namespace kinectWrapper;
using namespace yarp::os;
using namespace yarp::sig;


/*
 * CameraKinectWrapper is an interface class for using RTABMap with kinectWrapper.
 * RTABMap is expecting a camera interface to receive images. Usually, this is
 * done with OpenNI. For our purposes, we want to retrieve the images from
 * kinectServer, which is implemented here.
 */

CameraKinectWrapper::CameraKinectWrapper(KinectWrapperClient & c, float imageRate,
                                         const rtabmap::Transform & localTransform,
                                         float fx, float fy, float cx, float cy) :
    CameraRGBD(imageRate, localTransform, fx, fy, cx, cy),
    _depthFocal(0),
    client(c)
{
}

CameraKinectWrapper::~CameraKinectWrapper() {
    cvReleaseImage(&depthTmp);
    cvReleaseImage(&rgbTmp);
}

bool CameraKinectWrapper::init() {
    Property opt;
    client.getInfo(opt);

    int img_width=opt.find("img_width").asInt();
    int img_height=opt.find("img_height").asInt();
    int depth_width=opt.find("depth_width").asInt();
    int depth_height=opt.find("depth_height").asInt();

    //TODO: Change to receive focal length from kinect client
    //client.getFocalLength(_depthFocal);
    if(depth_width == 640)
        _depthFocal=525.0;
    else
        _depthFocal=525.0/2.0;
    _rgb.resize(img_width, img_height);
    _depth.resize(depth_width,depth_height);
    _depthToDisplay.resize(depth_width,depth_height);

    depthTmp=cvCreateImage(cvSize(depth_width,depth_height),IPL_DEPTH_32F,1);
    rgbTmp=cvCreateImage(cvSize(img_width,img_height),IPL_DEPTH_8U,3);

    return true;
}

void CameraKinectWrapper::captureImage(cv::Mat & rgb, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy) {
    data_mutex.lock();

    client.getDepth(_depth);
    client.getDepthImage(_depth,_depthToDisplay);
    client.getRgb(_rgb);

    cvConvertScale((IplImage*)_depthToDisplay.getIplImage(),depthTmp,1.0/255);
    cvCvtColor((IplImage*)_rgb.getIplImage(),rgbTmp,CV_BGR2RGB);

    rgb = cvarrToMat(rgbTmp, true);
    depth = cvarrToMat((IplImage*)_depth.getIplImage(), true);

    fx = _depthFocal;
    fy = _depthFocal;

    cx = float(depth.cols/2) - 0.5f;
    cy = float(depth.rows/2) - 0.5f;

    data_mutex.unlock();
}

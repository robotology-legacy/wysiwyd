// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Luke Boorman, Uriel Martinez
* email:   uriel.martinez@sheffield.ac.uk
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* $WYSIWYD_ROOT/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#ifndef __VISION_UTILS_H__
#define __VISION_UTILS_H__

/*
* @ingroup icub_module
*
* \defgroup modules visionUtils
*
* Utility functions for visionDriver.
*
* \author Luke Boorman, Uriel Martinez
*
* Copyright (C) 2015 WYSIWYD Consortium\n
* CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
*
*/


#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <vector>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace cv;
using namespace yarp::dev;
using namespace std;
using namespace cv::gpu;


class visionUtils
{
    private:
        Mat srcImage;
        Mat src_gray;
        int thresh;
        int max_thresh;
        int minContourSize; // minimum contour size
        bool useGPU;

        KalmanFilter KF;
        Mat_<float> measurement;
        Point2f middlePointV;
        Point2f kalmanMiddlePointV;

    public:
        visionUtils();
        ~visionUtils();
        void convertCvToYarp(cv::Mat MatImage, ImageOf<PixelRgb> &yarpImage);
        Rect checkRoiInImage(Mat, Rect);
        // Segment out face from haar cascade
        Mat segmentFace(Mat, Mat, bool, Mat *);
        Mat skeletonDetect(Mat, int, bool );
        // Segmentation tool, darws rect around region and can be updated to fit lines....
        vector<Rect> segmentLineBoxFit(Mat, int, int, Mat *,  std::vector<std::vector<cv::Point> > *, vector<RotatedRect> *, bool);
        
        // Draw rotatedRect
        Mat drawRotatedRect(Mat, RotatedRect, Scalar);
        
        bool isHandMoving(Point, Point, int);
        int updateArmPoints(Point2f , Point2f *, int);
        vector<Point2f> updateArmMiddlePoint(Point2f, Point2f *, int);

        // adaptive HSV
        std::vector<int> updateHSVAdaptiveSkin(std::vector<Mat>, bool);
        int drawHist(std::vector<Mat>, int);
        
        // Skin detection
        Mat skinDetect(Mat, Mat3b *, Mat *, std::vector<int>, int, int, int, int, bool);
        Mat cannySegmentation(Mat, int, bool);  
        
        // frame rate calc for perf measurement
        double getFrameRate(clock_t);     

        // predict next hand position using Kalman Filter
        vector<Point2f> getPredictedHandPosition(Point2f , int);
        void initKalmanFilterParameters(Point2f);
};

#endif // __VISION_UTILS_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------


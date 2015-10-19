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

#ifndef __VISION_DRIVER_H__
#define __VISION_DRIVER_H__

/*
* @ingroup icub_module
*
* \defgroup modules visionDriver
*
* Detects and tracks a face using OpenCV function and the gaze controller.
* The input image is from the left eye.
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

#include "visionUtils.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace cv;
using namespace yarp::dev;
using namespace std;
using namespace cv::gpu;


#define faceLimit 20

class visionDriver: public RFModule
{
    private:
	    string imageInPort;
	    string vectorOutPort;
	    string imageOutPort;
	    string hardware;
	    string format;
        string gazeOutPort;
        string faceCascadeFile;
        string bodyCascadeFile;
        string moduleName;
        
	    int format_int;
	    int hardware_int;
	    int isGPUavailable;
	    int poll;
	    bool displayFaces;
	    bool displayBodies;
        bool addFrameRate;
        clock_t startTime;
        
        std::vector<double> bodyPartLocations;
        bool bodyPosFound; // flag if any body parts are found -> wont transmit if nothing
        
        std::vector<double> storedFacePositions;
        bool storedFace;

        std::vector<double> storedBodyPositions;
        bool storedBody;
                
        Rect currentFaceRect;
        Mat faceSegMaskInv;
        
        // FLags
        bool faceSegFlag;
        bool bodySegFlag;

	    BufferedPort< ImageOf<PixelRgb> > faceTrack;	
	    BufferedPort< ImageOf<PixelRgb> > imageOut;

	    Port gazePort;	//x and y position for gaze controller
       
        String bodyPartPosName;
        Port bodyPartPosPort;
        
        std::vector<std::vector<cv::Point> > returnContours;
        std::vector<RotatedRect> armRotatedRects;
	    
	    bool inOpen;
	    bool outOpen;
	    bool imageOutOpen;

	    bool gazeOut;

    	int inCount;

    	Mat vectFaceArr;
    	Mat vectBodyArr;
    	
        Mat captureFrameBGR;
        Mat captureFrameFace;		
        Mat captureFrameBody;		
	    cv::gpu::GpuMat captureFrameGPU;
        cv::gpu::GpuMat grayscaleFrameGPU;
        cv::gpu::GpuMat objBufFaceGPU;
        cv::gpu::GpuMat objBufBodyGPU;
        
		int step;
        int maxSize;
        int biggestFace;
        int count;
        int noFaces;
        int noBodies;
        int faceSize;
        int bodySize;

        int d;
		bool inStatus;
        int boxScaleFactor; //Additional pixels for box sizing
		int neckScaleFactor;// additional neck scale factor for masking the neck region..... basically add pixels south
    	int pollTime;
        int sagittalSplit;  // split person in left and right
        Point bodyCentre; // calc centre of body
    	
    	int imgBlurPixels; //blur pixels for gauss smoothing
		std::vector< cv::Rect > facesOld;
		std::vector< cv::Rect > bodiesOld;

    	CascadeClassifier_GPU face_cascade;
    	CascadeClassifier_GPU body_cascade;
    	
        visionUtils *utilsObj;
       
        // Detect skin using default values for first go then update.......
		std::vector<int> hsvAdaptiveValues;

        // Arm / hand tracking
        bool firstLeftHandMovement;
        bool firstRightHandMovement;
        Point right_hand_position;
        Point left_hand_position;
        Point previous_right_hand_position;        
        Point previous_left_hand_position;        
        
        // Tracking of Arm points 
        Point2f previousLeftArmPoints;
        Point2f previousRightArmPoints;
        // Index to points used to fix tracking....
        vector<int> leftPointIndex[4];
        vector<int> rightPointIndex[4];
        bool calibratedLeftPoints;
        bool calibratedRightPoints;
        
        bool firstMiddlePointReady;
        int c_iter;
        int windowSize;
        

    public:
        visionDriver();
        ~visionDriver();
        bool updateModule();
        bool configure(ResourceFinder &);
        bool interruptModule();
        double getPeriod();
        void CVtoYarp(Mat, ImageOf<PixelRgb> &);
        Mat addText(string, Mat, Point, Scalar);
};

#endif // __VISION_DRIVER_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------


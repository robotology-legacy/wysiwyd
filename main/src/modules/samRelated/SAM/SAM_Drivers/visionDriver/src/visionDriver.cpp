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


#include "visionDriver.h"


visionDriver::visionDriver()
{
    imgBlurPixels=7; //gauss smoothing pixels
	faceSize = 400;
    bodySize = faceSize;
    boxScaleFactor = 20; // Expand face and body detected regions by this amount in pixels
	// LB additional neck scale factor for masking the neck region..... basically add pixels south
	neckScaleFactor = 40; // pixels south...

	displayFaces = false;//true;
	displayBodies = false; //true;
    utilsObj = new visionUtils();
    
    
    addFrameRate = true; // optional frameRate control
    
    faceSegFlag=false;
    bodySegFlag=false;
    firstLeftHandMovement = false;
    firstRightHandMovement = false;
    
    left_hand_position.x = 0;
    left_hand_position.y = 0;
    right_hand_position.x = 0;
    right_hand_position.y = 0;
    
    previous_left_hand_position.x = 0;
    previous_left_hand_position.y = 0;
    previous_right_hand_position.x = 0;
    previous_right_hand_position.y = 0;
    
    // Detect skin using default values for first go.......
	hsvAdaptiveValues.clear();
	
	// Compare point distances for tracking the same point....
	calibratedLeftPoints = false;
	calibratedRightPoints = false;
	storedFace = false;
	firstMiddlePointReady = false;
	
	windowSize = 5;
		
	namedWindow("Face / Body / Arms",WINDOW_NORMAL);	
}

visionDriver::~visionDriver()
{
}

bool visionDriver::updateModule()
{

    if (addFrameRate)
    {
        startTime = clock(); 
    }
    
    inCount = faceTrack.getInputCount();

    if(inCount == 0) // || outCount == 0)
    {
	    cout << "Awaiting input and output connections" << endl;
    }
    else
    {
	    ImageOf<PixelRgb> *yarpImage = faceTrack.read();
	    if (yarpImage!=NULL) 
	    {
            // PROCESS NEW IMAGE
		    //Alternative way of creating an openCV compatible image
		    //Takes approx twice as much time as uncomented implementation
		    //Also generates IplImage instead of the more useable format Mat
		    count = 0;
		    step = yarpImage->getRowSize() + yarpImage->getPadding();
		    Mat captureFrameRaw(yarpImage->height(),yarpImage->width(),CV_8UC3,yarpImage->getRawImage(),step);
            
            // resize input image from iCub cameras to 640x480
		    resize(captureFrameRaw,captureFrameRaw,Size(640,480));

		    cvtColor(captureFrameRaw,captureFrameBGR,CV_RGB2BGR);

            int height = captureFrameRaw.rows;
            //int width = captureFrameRaw.cols;

	        // Init bodyPartLocations vector if anything found... later sent out via yarp
	        for (int i = 0; i<12;i++) bodyPartLocations[i]=0.0; // 0 is no position found....
	        bodyPosFound=false; // set flag off -> set to true when body part position found

	        captureFrameFace=captureFrameBGR.clone();

	        Mat skinImage;
			Mat skinMaskDefault;
			Mat3b skinHSV;
			// Detect skin using default values.......
			std::vector<int> hsvDefault;
			// LB: this will always use the default values, to prevent runaway adaption!
            skinImage = utilsObj->skinDetect(captureFrameBGR, &skinHSV, &skinMaskDefault, hsvDefault, 400,7, 3, 0, displayFaces);

	        
		    // SET FACE AND BODY DETECTION cascades
		    // Haar cascades on GPU	
		    // Haar cascades on GPU	
		    captureFrameGPU.upload(captureFrameBGR);
		    cv::gpu::cvtColor(captureFrameGPU,grayscaleFrameGPU,CV_BGR2GRAY);
		    cv::gpu::equalizeHist(grayscaleFrameGPU,grayscaleFrameGPU);
		    // Face and Body
		    noFaces = face_cascade.detectMultiScale(grayscaleFrameGPU,objBufFaceGPU,1.2,5,Size(30,30));
		    noBodies = body_cascade.detectMultiScale(grayscaleFrameGPU,objBufBodyGPU,1.2,5,Size(150,150));

			// Check if face found
		    if(noFaces != 0)
		    {
			    noFaces = 1;

			    Mat3b face_HSV;
			    Mat allFaces;
                Mat allFacesSkin;
                // Get face cascadde info back from GPU
			    objBufFaceGPU.colRange(0,noFaces).download(vectFaceArr);

				ImageOf<PixelRgb>& faceImages = imageOut.prepare();
                int i = 0;

                    Rect* facesOld = vectFaceArr.ptr<Rect>();

                    //cout << "Got to face seg 0.4 ..." << endl;                                    
                    // LB - expand rectangle using additional pixels in boxScaleFactor
                    if (boxScaleFactor != 0)
                    {
                        facesOld[i]= Rect(facesOld[i].x-boxScaleFactor, facesOld[i].y-boxScaleFactor, facesOld[i].width+(boxScaleFactor*2), facesOld[i].height+(boxScaleFactor*2));

                        // LB - Check the extra sizes are not outside the original image size
                        // WARNING -> MIGHT produce distortions -> could reject image instead...
                        facesOld[i]=utilsObj->checkRoiInImage(captureFrameRaw, facesOld[i]); // LB: seg fault (need to pass rect inside of vector...)
                    }
					// Add extra pixels to bottom to remove neck skin region...
					if (neckScaleFactor !=0)
					{
						facesOld[i].height=facesOld[i].height+neckScaleFactor;
						facesOld[i]=utilsObj->checkRoiInImage(captureFrameRaw, facesOld[i]); // LB: seg fault (need to pass rect inside of vector...)
					}

					//required for rectangle faces in full image view
					Point pt1(facesOld[i].x + facesOld[i].width, facesOld[i].y + facesOld[i].height);
					Point pt2(facesOld[i].x, facesOld[i].y);
					rectangle(captureFrameFace,pt1,pt2,Scalar(0,255,0),1,8,0); 	
					
				    // Add values to body part pos vector (Face x(0),y(1),z(2))
                    bodyPartLocations[0]=int(facesOld[i].x+(facesOld[i].width/2));// Face  x
                    bodyPartLocations[1]=int(facesOld[i].y+(facesOld[i].height/2));// Face y
                    bodyPartLocations[2]=1.0;// Face z -> ++++++++++++++++++ SET AT DEFAULT 1 for NOW NEED TO UPDATE LATER...... STEREOVISION
                    bodyPosFound=true; // position found -> set flag to on	
					
					// Text face onto picture
					captureFrameFace=addText("Face", captureFrameFace, pt1, Scalar(0,255,0));
					
					// Send position of face in image to iKinGazeCtrl
					Bottle posGazeOutput;
					posGazeOutput.clear();
					posGazeOutput.addString("left");
					posGazeOutput.addDouble((int) bodyPartLocations[0]);
					posGazeOutput.addDouble((int) bodyPartLocations[1]);
					posGazeOutput.addDouble(1.0);
					gazePort.write(posGazeOutput);	
					
					storedFacePositions[0] = bodyPartLocations[0];
					storedFacePositions[1] = bodyPartLocations[1];
					storedFacePositions[2] = bodyPartLocations[2];
					storedFace = true;
					
				if(facesOld[i].area() != 0)
				{
				        // Standard image facedetector, take original image
				        // Take face from original data
					    allFaces = captureFrameBGR.operator()(facesOld[i]).clone();
					    resize(allFaces,allFaces,Size(faceSize,faceSize));
					    // LB processed skin segmented data
					    allFacesSkin = skinImage.operator()(facesOld[i]).clone();
					    resize(allFacesSkin,allFacesSkin,Size(faceSize,faceSize));
					    
					    // Take skinHSV (returned from skin detector) and uses face extracted rect.....
                        face_HSV = skinHSV.operator()(facesOld[i]).clone();
					    currentFaceRect = facesOld[0];
				    if( displayFaces )
				    {
					    imshow("faces",allFaces);
					    imshow("faces Skin",allFacesSkin);
				    }

                    // LB: Segment out just face....                    
                    Mat1b faceSegMask;
                    Mat faceSegmented=utilsObj->segmentFace(allFaces,allFacesSkin,displayFaces,&faceSegMask); 
                   
                    //LB Check face was found!
                    if (!faceSegmented.empty())
                    {
                        // Resize to standard
                        resize(faceSegmented,faceSegmented,Size(faceSize,faceSize));
                        // Send segmented face to yarp output... e.g. SAM face recog
                        utilsObj->convertCvToYarp(faceSegmented,faceImages);
                        imageOut.write();
                        std::vector<Mat> hsvPixels(3);
                        
                        // Little effect on frame rate here....
                        // Loop through rows
                        for (int i =0; i < face_HSV.rows; i++)
                        {
                        // Loop through cols
                            for (int j =0; j < face_HSV.cols; j++)
                            {
                                if (face_HSV(i,j)[0]!=0 || face_HSV(i,j)[1]!=0 || face_HSV(i,j)[2]!=0)
                                {
                                    // save HSV values from pixel in mask    
                                    hsvPixels[0].push_back(face_HSV(i,j)[0]); //H
                                    hsvPixels[1].push_back(face_HSV(i,j)[1]); //S
                                    hsvPixels[2].push_back(face_HSV(i,j)[2]); //V
                                }
                            }
                        }
                        hsvAdaptiveValues = utilsObj-> updateHSVAdaptiveSkin(hsvPixels, false);
                        
                        faceSegMaskInv = faceSegMask.clone();
                        faceSegFlag=true;
                    }
                    else
                    {
                        cout << " Face segmentation unsuccessful" << endl;
                    } 
                }                          
			}
			else
			{
			    if( storedFace == true )
			    {
					bodyPartLocations[0] = storedFacePositions[0];
					bodyPartLocations[1] = storedFacePositions[1];
					bodyPartLocations[2] = storedFacePositions[2];
			    }
			}
		    
            // BODY TRACK
		    if(noBodies != 0)
		    {
			    captureFrameBody=captureFrameBGR.clone();
			    noBodies = 1;

			    Mat allBodies;
                Mat allBodiesSkin;
                
			    objBufBodyGPU.colRange(0,noBodies).download(vectBodyArr);

				Rect* bodiesOld = vectBodyArr.ptr<Rect>();

                int i = 0;
                
                if (boxScaleFactor != 0)
                {
                    bodiesOld[i] = Rect(bodiesOld[i].x-boxScaleFactor,bodiesOld[i].y-boxScaleFactor,bodiesOld[i].width+(boxScaleFactor*2),bodiesOld[i].height+(boxScaleFactor*2));

                    bodiesOld[i]=utilsObj->checkRoiInImage(captureFrameRaw, bodiesOld[i]); // LB: seg fault (need to pass rect inside of vector...)
                }
                
				// Body split using centre of body region found...
				sagittalSplit = int(bodiesOld[i].x+(bodiesOld[i].width/2));
				
				if( displayBodies )
				{		
				    //Draw rectangle for body and line down centre
				    Point pt1(bodiesOld[i].x + bodiesOld[i].width, bodiesOld[i].y + bodiesOld[i].height);
				    Point pt2(bodiesOld[i].x, bodiesOld[i].y);
				    rectangle(captureFrameBody,pt1,pt2,Scalar(0,255,0),1,8,0);			
				    line(captureFrameBody,Point(sagittalSplit,0),Point(sagittalSplit,height),Scalar(0,0,255),1,8,0);						
                    imshow("Body seg",captureFrameBody);
                }				
				

				// if not reject segmentation....
				if (sagittalSplit > skinMaskDefault.cols*0.85 || sagittalSplit < skinMaskDefault.cols*0.15)
				{
				    cout << " Sagittal split line is too near edge -> rejecting body detection" << endl;
				    bodySegFlag=false;
				}
				else
				{
				    bodySegFlag=true;
				    // Add values to body part pos vector (right arm x(3),y(4),z(5))
                    bodyPartLocations[3]=int(bodiesOld[i].x+(bodiesOld[i].width/2));// Body  x
                    bodyPartLocations[4]=int(bodiesOld[i].y+(bodiesOld[i].height/2));// Body y
                    bodyPartLocations[5]=1.0;// Body z -> ++++++++++++++++++ SET AT DEFAULT 1 for NOW NEED TO UPDATE LATER...... STEREOVISION
                    bodyPosFound=true; // position found -> set flag to on		
                    
                    storedBodyPositions[0] = bodyPartLocations[3];
                    storedBodyPositions[1] = bodyPartLocations[4];
                    storedBodyPositions[2] = bodyPartLocations[5];
                    
                    storedBody = true;
				}
				
					if(bodiesOld[i].area() != 0)
					{
					    // Standard image facedetector, take original image
						allBodies = captureFrameBGR.operator()(bodiesOld[i]).clone();
						resize(allBodies,allBodies,Size(bodySize,bodySize));
						// LB processed skin segmented data
						allBodiesSkin = skinImage.operator()(bodiesOld[i]).clone();
						resize(allBodiesSkin,allBodiesSkin,Size(bodySize,bodySize));
						
					}
                        
			}
			else
			{
			    if( storedBody == true )
			    {
                    bodyPartLocations[3] = storedBodyPositions[0];
                    bodyPartLocations[4] = storedBodyPositions[1];
                    bodyPartLocations[5] = storedBodyPositions[2];
			    }
			}
			
        // LB: Body segmentation to find arms for action detection
        
            if (!faceSegMaskInv.empty() && faceSegFlag && bodySegFlag)
            {
                Mat skinImageTemp;
                Mat3b skinHSVtemp;
                Mat skinMask;

                skinMask = skinMaskDefault.clone();
                
			    Mat rectMaskFaceOnly = Mat::zeros( skinMask.size(), CV_8UC1 );
			    Mat skinMaskNoFace;
			    Mat faceSegTemp;
				resize(faceSegMaskInv,faceSegTemp,Size(currentFaceRect.width,currentFaceRect.height));
			    faceSegTemp.copyTo(rectMaskFaceOnly(currentFaceRect) );
			    bitwise_not(rectMaskFaceOnly,rectMaskFaceOnly);
		        bitwise_and(rectMaskFaceOnly,skinMask,skinMaskNoFace);
		        if( displayBodies )
		        {
			        imshow("Rectangle mask face",rectMaskFaceOnly);
                    imshow("skinmask no face :)",skinMaskNoFace);	
                }
                
			    Mat skelMat;
		        vector<Rect> boundingBox = utilsObj->segmentLineBoxFit(skinMaskNoFace, 1000, 2, &skelMat, &returnContours, &armRotatedRects, displayFaces);
		        		        
		        
			    //check atleast two bounding boxes found for left and right arms... > has to find both arms!
			    if (boundingBox.size()>1)
			    {
                    int leftArmInd=0;
                    int rightArmInd=0;
                    int testInXMost=0;
                    int testInXLeast=captureFrameFace.cols;
			        // Find boxes for left and right arms..... could be improved to cope with x-over the centre line of person!
			        
			        for (int i = 0; i<(int)boundingBox.size(); i++)
			        {
			            if (boundingBox[i].x<testInXLeast){
			            testInXLeast=boundingBox[i].x;
			            rightArmInd=i;
			            }
			            
			            if (boundingBox[i].x>testInXMost){
			            testInXMost=boundingBox[i].x;
			            leftArmInd=i;
			            }			        
                     }
                    // Check both boundingBoxes aren't the same for left and right 
                    if (rightArmInd!=leftArmInd)
                    {
                    
                        if (sagittalSplit!=0)
                        {
                            bodyCentre.x=sagittalSplit;
                            bodyCentre.y=currentFaceRect.y+currentFaceRect.height;
                            // Draw circle at centre below face
                            circle(captureFrameFace,bodyCentre,10,Scalar(0,255,255),3);
                        
                        }
                        
                        // LEFT ARM
                        Point2f leftArmPoints[4];                        
                        armRotatedRects[leftArmInd].points(leftArmPoints);
                        vector<Point2f> leftArmMiddlePoint;
                        
                        // Find middle point at base of left arm and track...
                        if( calibratedLeftPoints == false )
                        {                            
                            if( (abs(bodyCentre.x-armRotatedRects[leftArmInd].center.x) > 40.0) && (abs(bodyCentre.y-armRotatedRects[leftArmInd].center.y) > 40.0) )
                            {
                                int longestLeftIndex = utilsObj->updateArmPoints(bodyCentre, leftArmPoints,1);   //finds initial longest point
                                previousLeftArmPoints = leftArmPoints[longestLeftIndex];
                                leftArmMiddlePoint= utilsObj->updateArmMiddlePoint(previousLeftArmPoints, leftArmPoints,1);   //finds initial longest point
                                calibratedLeftPoints = true;                                
                            }                           
                        }
                        // Find current point which is closest to previous point
                        
                            leftArmMiddlePoint = utilsObj->updateArmMiddlePoint(previousLeftArmPoints, leftArmPoints,0);   //finds closest point
                            firstMiddlePointReady = true;

                        // Set left arm location
                        // Update previous point
                        left_hand_position = leftArmMiddlePoint.at(2);
                        previousLeftArmPoints=left_hand_position;
                        
             
                        circle(captureFrameFace, leftArmMiddlePoint.at(2), 10, Scalar(255,0,255), 3);    //middle point
				        Point pt1(boundingBox[leftArmInd].x + boundingBox[leftArmInd].width, boundingBox[leftArmInd].y + boundingBox[leftArmInd].height);
				        utilsObj->drawRotatedRect(captureFrameFace, armRotatedRects[leftArmInd], Scalar(255,0,0));
				        captureFrameFace=addText("Left arm", captureFrameFace, pt1, Scalar(0,0,255));
			            
			            // Right ARM
		            	//Draw right arm rectangles
				        Point pt3(boundingBox[rightArmInd].x + boundingBox[rightArmInd].width, boundingBox[rightArmInd].y + boundingBox[rightArmInd].height);
				        utilsObj->drawRotatedRect(captureFrameFace, armRotatedRects[rightArmInd], Scalar(255,0,0));
				        captureFrameFace=addText("Right arm", captureFrameFace, pt3, Scalar(0,0,255));			    
			            
                      
                        // @@@@@@@@@@@@@@@@@@@@@@ Are the hands moving @@@@@@@@@@@@@@@@@@@@@
                        //Set number of pixels to detect hand movement....
                        // IS the left hand moving
                        if( !firstLeftHandMovement )
                        {
                            previous_left_hand_position = left_hand_position;
                            firstLeftHandMovement = true;
                        }
                        
                        // Relative positions (take difference from start.....)
                        int relLeftXPosition = 0;
                        int relLeftYPosition = 0;

                            relLeftXPosition = left_hand_position.x;
                            relLeftYPosition = left_hand_position.y;

                        // Send out hand positions over yarp
                        // Add values to body part pos vector (left arm x(6),y(7),z(8))
                        bodyPartLocations[6]=relLeftXPosition;// left hand  x
                        bodyPartLocations[7]=relLeftYPosition;// left hand  y
                        bodyPartLocations[8]=1.0;// left hand  z -> ++++++++++++++++++ SET AT DEFAULT 1 for NOW NEED TO UPDATE LATER...... STEREOVISION
                        bodyPosFound=true; // position found -> set flag to on
                        previous_left_hand_position = left_hand_position;                          

                        // Right Arm
                        Point2f rightArmPoints[4];
                        armRotatedRects[rightArmInd].points(rightArmPoints);
                        vector<Point2f> rightArmMiddlePoint;

                        if( calibratedRightPoints == false )
                        {
                            if( (abs(bodyCentre.x-armRotatedRects[rightArmInd].center.x) > 40.0) && (abs(bodyCentre.y-armRotatedRects[rightArmInd].center.y) > 40.0) )
                            {
                                int longestRightIndex = utilsObj->updateArmPoints(bodyCentre, rightArmPoints, 1);   //finds initial longest point
                                previousRightArmPoints = rightArmPoints[longestRightIndex];
                                rightArmMiddlePoint= utilsObj->updateArmMiddlePoint(previousRightArmPoints, rightArmPoints,1);   //finds initial longest point

                                calibratedRightPoints = true;
                            }                           
                        }
                            rightArmMiddlePoint = utilsObj->updateArmMiddlePoint(previousRightArmPoints, rightArmPoints,0);   //finds initial nearest point
                        // Set right arm location
                        //right_hand_position=rightArmPoints[closestRightIndex];
                        right_hand_position = rightArmMiddlePoint.at(2);

                        // Update previous point
                        previousRightArmPoints=right_hand_position;
                                            
                        circle(captureFrameFace, rightArmMiddlePoint.at(2), 10, Scalar(255,0,255), 3);    //middle point
                        
                        if( !firstRightHandMovement )
                        {
                            previous_right_hand_position = right_hand_position;
                            firstRightHandMovement = true;
                        }

                        int relRightXPosition = 0;
                        int relRightYPosition = 0;

                            relRightXPosition = right_hand_position.x;
                            relRightYPosition = right_hand_position.y;

                        // Add values to body part pos vector (right arm x(9),y(10),z(11))
                        bodyPartLocations[9]=relRightXPosition;// Right hand  x
                        bodyPartLocations[10]=relRightYPosition;// Right hand  y
                        bodyPartLocations[11]=1.0;// Right hand  z -> ++++++++++++++++++ SET AT DEFAULT 1 for NOW NEED TO UPDATE LATER...... STEREOVISION
                        bodyPosFound=true; // position found -> set flag to on												
                                                 
                        previous_right_hand_position = right_hand_position;                          
                    }
                    else
		            {
    		            calibratedLeftPoints = false;
		                calibratedRightPoints = false;
		                firstMiddlePointReady = false;
			        }
			    }
			    else
		        {
		            calibratedLeftPoints = false;
		            calibratedRightPoints = false;
                    firstMiddlePointReady = false;
			    }

                // Main display here..... ALways on -> shows Face / Body and arms with hand locations detected
                if (addFrameRate)
                {
                    double frameRate = utilsObj->getFrameRate(startTime);
                    
                    std::stringstream ss(stringstream::in | stringstream::out);
                    ss << setprecision(2) << frameRate <<  " fps";
                    std::string str = ss.str();   
                    captureFrameFace=addText(str, captureFrameFace, Point2f(200,15), Scalar(0,0,255));		
                }
                
                imshow("Face / Body / Arms", captureFrameFace);

				// Send found body pos values out over YARP
				// If any body part position has been found -> face, body, left hand, right hand
				if (bodyPosFound)
				{
				    Bottle bodyPartPosOutput;
				    bodyPartPosOutput.clear();
			        for (int i = 0;i<12;i++)
				        bodyPartPosOutput.addDouble(bodyPartLocations[i]); 
				    bodyPartPosPort.write(bodyPartPosOutput);
				}
				
            }
					    
	    }
    }
    waitKey(1);
    return true;
}

bool visionDriver::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("visionDriver"), "module name (string)").asString();

    setName(moduleName.c_str());    

    Property config;
    config.fromConfigFile(rf.findFile("from").c_str());

    Bottle &bGeneral = config.findGroup("general");

    imageInPort = bGeneral.find("imageInPort").asString().c_str();
    imageOutPort = bGeneral.find("imageOutPort").asString().c_str();
    gazeOutPort = bGeneral.find("gazeOutPort").asString().c_str();
    
    string xmlFacePath = rf.findFileByName("haarcascade_frontalface_alt.xml");
    string xmlBodyPath = rf.findFileByName("haarcascade_mcs_upperbody.xml");
    
//    faceCascadeFile = bGeneral.find("faceCascadeFile").asString().c_str();
//    bodyCascadeFile = bGeneral.find("bodyCascadeFile").asString().c_str();

    faceCascadeFile = xmlFacePath;
    bodyCascadeFile = xmlBodyPath;

    cout << "------------------------" << endl;
    cout << imageInPort.c_str() << endl;
    cout << imageOutPort << endl;
    cout << gazeOutPort << endl;
    cout << faceCascadeFile << endl;
    cout << bodyCascadeFile << endl;
    bodyPartPosName="/visionDriver/bodyPartPosition:o";

    // Init bodyPartLocations vector if anything found...
    bodyPartLocations.clear();
    for (int i = 0; i < 12;i++)
        bodyPartLocations.push_back(0.0); // 0.0 is no position found....

    storedFacePositions.clear();
    for (int i = 0; i < 3;i++)
        storedFacePositions.push_back(0.0); // 0.0 is no position found....

    storedBodyPositions.clear();
    for (int i = 0; i < 3;i++)
        storedBodyPositions.push_back(0.0); // 0.0 is no position found....

    isGPUavailable = getCudaEnabledDeviceCount();

	if (isGPUavailable == 0)
	{
		cout << "No GPU found or the library is compiled without GPU support" << endl;
		cout << "Proceeding on CPU" << endl;
		cout << "Detecting largest face in view only for performance" << endl;
		hardware_int = 0;

        return false;
	}
	else
	{
		hardware_int = 1;
		cv::gpu::getDevice();
		cout << "Proceeding on GPU" << endl;
	}

	inOpen = faceTrack.open(imageInPort.c_str());
	imageOutOpen = imageOut.open(imageOutPort.c_str());

	gazeOut = gazePort.open(gazeOutPort.c_str());
	
	bodyPartPosPort.open(bodyPartPosName.c_str());

	if(!inOpen | !imageOutOpen | !gazeOut )
	{
		cout << "Could not open ports. Exiting" << endl;
		return false;
	}

	inCount = faceTrack.getInputCount();

    // Init dyn variables
    sagittalSplit = 0;  // split person in left and right 
    bodyCentre.x=0;
    bodyCentre.y=0; 
    
	step = 0;
    count = 0;
        
	inStatus = true;

	if( displayFaces )
	{
		namedWindow("faces",1);
		namedWindow("wholeImage",1);
		waitKey(1);
	}		
	
	face_cascade.load(faceCascadeFile.c_str());
	body_cascade.load(bodyCascadeFile.c_str());
	
	return true;
}

bool visionDriver::interruptModule()
{
    return true;
}

double visionDriver::getPeriod()
{
    return 0.01;
}


Mat visionDriver::addText(string textIn, Mat img, Point textLocation, Scalar colour)
{

int fontFace = FONT_HERSHEY_SIMPLEX;
double fontScale = 1;
int thickness = 2;

int baseline=0;
Size textSize = getTextSize(textIn, fontFace,
                            fontScale, thickness, &baseline);
baseline += thickness;
Point textOrg(textLocation.x - (textSize.width/2),textLocation.y + (textSize.height/2));

putText(img, textIn, textOrg, fontFace, fontScale, colour, thickness, 8);

return img;
}



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

#include "visionUtils.h"

visionUtils::visionUtils()
{
    useGPU = true;
}

visionUtils::~visionUtils()
{
}

void visionUtils::convertCvToYarp(cv::Mat MatImage, ImageOf<PixelRgb> & yarpImage)
{
    IplImage* IPLfromMat = new IplImage(MatImage);

    yarpImage.resize(IPLfromMat->width,IPLfromMat->height);

    IplImage * iplYarpImage = (IplImage*)yarpImage.getIplImage();

    if (IPL_ORIGIN_TL == IPLfromMat->origin){
            cvCopy(IPLfromMat, iplYarpImage, 0);
    }
    else{
            cvFlip(IPLfromMat, iplYarpImage, 0);
    }

    if (IPLfromMat->channelSeq[0]=='B') {
            cvCvtColor(iplYarpImage, iplYarpImage, CV_BGR2RGB);
    }
}

Rect visionUtils::checkRoiInImage(Mat src, Rect roi)
{
    // Get image sizes
    Size s = src.size();
    int height = s.height;
    int width = s.width;
    
    if (roi.x<0)
    {
        roi=Rect(0,roi.y,roi.width,roi.height);
    }
    if (roi.y<0)
    {
        roi=Rect(roi.x,0,roi.width,roi.height);        
    }
    if ((roi.width+roi.x)>width) 
    {
        //int temp=roi.width;
        roi=Rect(roi.x,roi.y,width-roi.x,roi.height);         
    }
    if ((roi.height+roi.y)>height)
    {
        //int temp=roi.height;
        roi=Rect(roi.x,roi.y,roi.width,height-roi.y); 
    }
    return roi;
}


Mat visionUtils::segmentFace(Mat srcImage, Mat maskImage, bool displayFaces, Mat *skinSegMaskInv)
{

    // Check mask and original image are the same size
    Size srcS = srcImage.size();
    int heightS = srcS.height;
    int widthS = srcS.width;

    Size maskS = maskImage.size();
    int heightM = maskS.height;
    int widthM = maskS.width;    

    if (heightS!=heightM || widthS!=widthM)
    {
        cout << "hS:" << heightS << " wS:" << widthS << " hM:" << heightM << " wM" << widthM << endl;  
        cout << "Source and mask images are not the same size... aborting" << endl;
        Mat ttt;
        return (ttt);
    }
  
    /// Convert image to gray and blur it
    cvtColor( maskImage, src_gray, CV_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    /// Find contours
    findContours( src_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // ########## Remove contour indents (defects), by finding the convex 
    /// Find the convex hull object for each contour
    vector<vector<Point> >hull( contours.size() );

    for( int i = 0; i < (int)contours.size(); i++ )
    {  
        convexHull( Mat(contours[i]), hull[i], false );
    }

    /// Draw contours + hull results
    Mat drawingHull = Mat::zeros( src_gray.size(), CV_8UC3 );
    
    //Check minimum contour size and find largest....
    int largest_area=-1;
    int largest_contour_index=0;
    for( int i = 0; i< (int)contours.size(); i++ )
    {
        if( (int)contours[i].size() > minContourSize )
        { 
            double a=contourArea( contours[i],false);  //  Find the area of contour
            if(a>largest_area)
            {
                largest_area=a;
                largest_contour_index=i;  
            }
        }
    }

    if (displayFaces)
    {
        RNG rng(12345); // for colour generation
        
        for( int i = 0; i< (int)contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawingHull, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            drawContours( drawingHull, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        }
        imshow( "Contour Convex Hull", drawingHull );
    }

  //// ############### Selected Hull contour to use -> ignoring ellipse etc
  // Check if hull found successfully... if not ABORT
    if (hull.empty() )
    {
        cout << "Hull region not found > returning...." << endl;
        Mat ttt;
        return (ttt);
    }

    // Check area of hull and abort if neded  
    vector<Point> approx;
    approxPolyDP(hull[largest_contour_index], approx, 5, true);
    double area1 = contourArea(approx);
    if  (area1<4000)
    {
        cout << "Hull area too small > returning...." << endl;
        Mat ttt;
        return (ttt);
    }

    // Cut down rect around convex contour hull 
    Rect boundRect;
    boundRect=boundingRect(Mat(hull[largest_contour_index]));
    // Check bounding box fits inside image.... resize if needed
    boundRect=checkRoiInImage(srcImage, boundRect);

    // Check bounding box has greater dimensions than 5x5pix
    if (boundRect.height<=5 || boundRect.width<=5)
    {
        cout << "Region selected too small... exiting" << endl;
        Mat ttt;
        return (ttt);
    }
    else
    {
        /// Repeat boxing but for masked skin data (Hull)
        // Make binary mask using hull largest contour
        Mat srcSegSkin = Mat::zeros( srcImage.size(), CV_8UC3 );
        Mat skinSegMask = Mat::zeros( srcImage.size(), CV_8UC1 );
        drawContours( skinSegMask, hull, largest_contour_index, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );
        srcImage.copyTo(srcSegSkin,skinSegMask);  // Copy using mask from skinSegMask
        srcSegSkin=srcSegSkin(boundRect);
        
        // Make face blocking mask (face pix = 0)
        Mat skinSegMaskInvTemp = Mat::zeros( srcImage.size(), CV_8UC1 );
        bitwise_not(skinSegMaskInvTemp,*skinSegMaskInv,skinSegMask);

        if (displayFaces)
        {   // Take boxed region of face from original image data
            // Copy inital image and section with bounding box
            Mat srcSegmented = srcImage.clone();
            srcSegmented=srcSegmented(boundRect);
            imshow("Rect region orig",srcSegmented);   
            Mat maskSegmented = maskImage.clone();
            maskSegmented=maskSegmented(boundRect);
            imshow("Rect region, with SkinSeg",maskSegmented);         
            imshow("Rect region, with hull region SkinSeg",srcSegSkin);      
        }      

        return(srcSegSkin);
    }
}


Mat visionUtils::skeletonDetect(Mat threshImage, int imgBlurPixels, bool displayFaces)
{
    // Finds skeleton of white objects in black image, using erode / dilate morph operations
    threshold(threshImage, threshImage, 127, 255, THRESH_BINARY);
    if (displayFaces) imshow("Skel in",threshImage);
    Mat skel(threshImage.size(), CV_8UC1, Scalar(0));
    Mat temp;
    Mat eroded;
 
    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
 
    bool done;      
    do
    {
        erode(threshImage, eroded, element);
        dilate(eroded, temp, element);
        subtract(threshImage, temp, temp);
        bitwise_or(skel, temp, skel);
        eroded.copyTo(threshImage);
        done = (countNonZero(threshImage) == 0);
    } while (!done);
    if (displayFaces) imshow("Skel raw",skel);
    // Blur to reduce noise
    GaussianBlur(skel, skel, Size(imgBlurPixels,imgBlurPixels), 1, 1);
    return skel;
}

bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 )
{
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}

vector<Rect> visionUtils::segmentLineBoxFit(Mat img0, int minPixelSize, int maxSegments, Mat *returnMask, std::vector<std::vector<cv::Point> > *returnContours, vector<RotatedRect> *rotatedBoundingBox, bool displayFaces)
{
    // Segments items in gray image (img0)
    // minPixelSize= pixels, threshold for removing smaller regions, with less than minPixelSize pixels
    // 0, returns all detected segments
    // maxSegments = max no segments to return, 0 = all
    RNG rng(12345);

    
    int padPixels=15;
    // Rect border added at start...
    Rect tempRect;
    tempRect.x=padPixels;
    tempRect.y=padPixels;
    tempRect.width=img0.cols;
    tempRect.height=img0.rows;
    Mat img1 = Mat::zeros(img0.rows+(padPixels*2), img0.cols+(padPixels*2), CV_8UC1);
    img0.copyTo(img1(tempRect));

    // find the contours
    std::vector<std::vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(img1, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Mask for segmented region
    Mat mask = Mat::zeros(img1.rows, img1.cols, CV_8UC3);

    vector<double> areas(contours.size());

    // Case for using minimum pixel size
    Vec4f lines;
    Scalar color;
    
    // sort contours
    std::sort(contours.begin(), contours.end(), compareContourAreas);

    // grab contours
    vector<Rect> boundingBox;
    
    // LB testing
    vector<RotatedRect> tempRotatedBoundingBox;
    
    std::vector<std::vector<cv::Point> > tempReturnContours;
    int maxIterations = 0;
    
    if( contours.size() > 0 )
    {
        if (maxSegments==0)// return all contours..
            maxIterations = contours.size();
        else if((int)contours.size() >= maxSegments)
            maxIterations = maxSegments;
        else
            maxIterations = 1;    // LB: need to check this is correct!
        int contourCount=0;
        
        for (int j = 1; j < maxIterations+1; j++)
        {
            int i = contours.size()-j;
            if (contourArea(Mat(contours[i]))>minPixelSize)
            {
                // Fit rotated rect to contour
                tempRotatedBoundingBox.push_back(minAreaRect( Mat(contours[i]) ));
                
                Point2f rectCentre=tempRotatedBoundingBox[contourCount].center;
                rectCentre.x=rectCentre.x-padPixels;
                rectCentre.y=rectCentre.y-padPixels;
                tempRotatedBoundingBox[contourCount].center=rectCentre;
                
                // Find line limits....
                boundingBox.push_back(boundingRect(Mat(contours[i])));
                
                // Remove edge padding effects....
                boundingBox[contourCount].x=boundingBox[contourCount].x-padPixels;
                boundingBox[contourCount].y=boundingBox[contourCount].y-padPixels;
                boundingBox[contourCount]=checkRoiInImage(img0, boundingBox[contourCount]);
                
                contourCount++;
                
                tempReturnContours.push_back(contours[i]);
            }
        }
        // Return contours
        returnContours->resize(tempReturnContours.size());
        *returnContours = tempReturnContours;
        // Return rotated rects
        rotatedBoundingBox->resize(tempRotatedBoundingBox.size());
        *rotatedBoundingBox = tempRotatedBoundingBox;        
        
        // normalize so imwrite(...)/imshow(...) shows the mask correctly!
        normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);
        // To Remove border added at start...    
        *returnMask=mask(tempRect);
        // show the images
        if (displayFaces)   imshow("Seg line utils: Img in", img0);
        if (displayFaces)   imshow("Seg line utils: Mask", *returnMask);
        if (displayFaces)   imshow("Seg line utils: Output", img1);
    }
    return boundingBox;
}

bool visionUtils::isHandMoving(Point handPoints, Point previousHandPoints, int limit)
{
    bool movement = false;
    if( ( handPoints.x < previousHandPoints.x - limit ) || ( handPoints.x > previousHandPoints.x + limit ) )
        movement = true;
    if( ( handPoints.y < previousHandPoints.y - limit ) || ( handPoints.y > previousHandPoints.y + limit ) )
        movement = true;

    return movement;
}

/** ###############################################################################
 * @Draw histogram
 * Takes in triple vector of values e.g. std::vector<Mat> bgrPixels(3);
 */
int visionUtils::drawHist(std::vector<Mat> pixelPlanes, int histBlurPixels)//( int, char** argv )
{
    /// Establish the number of bins
    int histSize = 256;

    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;

    Mat b_hist, g_hist, r_hist;

    /// Compute the histograms:
    calcHist( &pixelPlanes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &pixelPlanes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &pixelPlanes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

    // Draw the histograms for B, G and R or H S V
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );

    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    // Smooth hists if needed
    if (histBlurPixels>0)
    {
        GaussianBlur(b_hist, b_hist, Size(1,histBlurPixels), 1, 1);
        GaussianBlur(g_hist, g_hist, Size(1,histBlurPixels), 1, 1);
        GaussianBlur(r_hist, r_hist, Size(1,histBlurPixels), 1, 1);
    }

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    /// Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
    }

    /// Display
    namedWindow("calcHist Demo", WINDOW_AUTOSIZE );
    imshow("calcHist Demo", histImage );
    waitKey(1);
    return 0;
}


std::vector<int> visionUtils::updateHSVAdaptiveSkin(std::vector<Mat> pixelPlanes, bool displayFaces)//( int, char** argv )
{
    if (displayFaces)
    {
        drawHist(pixelPlanes, 35);
    }
    
    Scalar b_mean, b_stdDev;
    
    meanStdDev(pixelPlanes[0], b_mean, b_stdDev);
    int b_min=b_mean[0] - b_stdDev[0]*3;
    int b_max=b_mean[0] + b_stdDev[0]*3;

    Scalar g_mean,g_stdDev;
    meanStdDev(pixelPlanes[1], g_mean,g_stdDev);
    int g_min=g_mean[0] - g_stdDev[0]*3;
    int g_max=g_mean[0] + g_stdDev[0]*3;  

    Scalar r_mean, r_stdDev;
    meanStdDev(pixelPlanes[2], r_mean, r_stdDev);
    int r_min=r_mean[0] - r_stdDev[0]*3;
    int r_max=r_mean[0] + r_stdDev[0]*3;    

    std::vector<int> hsvAdaptiveUpdated;
    hsvAdaptiveUpdated.push_back(b_min);
    hsvAdaptiveUpdated.push_back(g_min);
    hsvAdaptiveUpdated.push_back(r_min);
    hsvAdaptiveUpdated.push_back(b_max);
    hsvAdaptiveUpdated.push_back(g_max);
    hsvAdaptiveUpdated.push_back(r_max);

  return hsvAdaptiveUpdated;

}

Mat visionUtils::skinDetect(Mat captureframe, Mat3b *skinDetectHSV, Mat *skinMask, std::vector<int> adaptiveHSV, int minPixelSize, int imgBlurPixels, int imgMorphPixels, int singleRegionChoice, bool displayFaces)
{

    if (adaptiveHSV.size()!=6 || adaptiveHSV.empty())
    {
        adaptiveHSV.clear();
        adaptiveHSV.push_back(5);
        adaptiveHSV.push_back(38);
        adaptiveHSV.push_back(51);
        adaptiveHSV.push_back(17);
        adaptiveHSV.push_back(250);
        adaptiveHSV.push_back(242);
    }


    //int step = 0;
    Mat3b frameTemp;
    Mat3b frame;
    // Forcing resize to 640x480 -> all thresholds / pixel filters configured for this size.....
    // Note returned to original size at end...
    Size s = captureframe.size();
    resize(captureframe,captureframe,Size(640,480));

    
    
    if (useGPU)
    {
    
        gpu::GpuMat imgGPU, imgGPUHSV;
        imgGPU.upload(captureframe);
        gpu::cvtColor(imgGPU, imgGPUHSV, CV_BGR2HSV);
        gpu::GaussianBlur(imgGPUHSV, imgGPUHSV, Size(imgBlurPixels,imgBlurPixels), 1, 1);
        imgGPUHSV.download(frameTemp);  
    }
    else
    {
        cvtColor(captureframe, frameTemp, CV_BGR2HSV);
        GaussianBlur(frameTemp, frameTemp, Size(imgBlurPixels,imgBlurPixels), 1, 1);
    }
    
    // Potential FASTER VERSION using inRange
    Mat frameThreshold = Mat::zeros(frameTemp.rows,frameTemp.cols, CV_8UC1);
    Mat hsvMin = (Mat_<int>(1,3) << adaptiveHSV[0], adaptiveHSV[1],adaptiveHSV[2] );
    Mat hsvMax = (Mat_<int>(1,3) << adaptiveHSV[3], adaptiveHSV[4],adaptiveHSV[5] );
    inRange(frameTemp,hsvMin ,hsvMax, frameThreshold);
    frameTemp.copyTo(frame,frameThreshold);
        
    /* BGR CONVERSION AND THRESHOLD */
    Mat1b frame_gray;
    
    // send HSV to skinDetectHSV for return
    *skinDetectHSV=frame.clone();
    
    cvtColor(frame, frame_gray, CV_BGR2GRAY);
                
                
    // Adaptive thresholding technique
    // 1. Threshold data to find main areas of skin
    adaptiveThreshold(frame_gray,frame_gray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,9,1);
    
    
    if (useGPU)
    {
        gpu::GpuMat imgGPU;
        imgGPU.upload(frame_gray);
        // 2. Fill in thresholded areas
        gpu::morphologyEx(imgGPU, imgGPU, CV_MOP_CLOSE, Mat1b(imgMorphPixels,imgMorphPixels,1), Point(-1, -1), 2);
        gpu::GaussianBlur(imgGPU, imgGPU, Size(imgBlurPixels,imgBlurPixels), 1, 1);
        imgGPU.download(frame_gray);
    
    }
    else
    {
        // 2. Fill in thresholded areas
        morphologyEx(frame_gray, frame_gray, CV_MOP_CLOSE, Mat1b(imgMorphPixels,imgMorphPixels,1), Point(-1, -1), 2);
        GaussianBlur(frame_gray, frame_gray, Size(imgBlurPixels,imgBlurPixels), 1, 1);
        // Select single largest region from image, if singleRegionChoice is selected (1)
    }
    

    if (singleRegionChoice)
    {
        *skinMask = cannySegmentation(frame_gray, -1, displayFaces);
    }
    else // Detect each separate block and remove blobs smaller than a few pixels
    {
        *skinMask = cannySegmentation(frame_gray, minPixelSize, displayFaces);
    }

    // Just return skin
    Mat frame_skin;
    captureframe.copyTo(frame_skin,*skinMask);  // Copy captureframe data to frame_skin, using mask from frame_ttt
    // Resize image to original before return
    resize(frame_skin,frame_skin,s);
    
    if (displayFaces)
    {   
    imshow("Skin HSV (B)",frame);
    imshow("Adaptive_threshold (D1)",frame_gray);
    imshow("Skin segmented",frame_skin);
    }
    
    return frame_skin;  
    waitKey(1);
}

Mat visionUtils::cannySegmentation(Mat img0, int minPixelSize, bool displayFaces)
{
    // Segments items in gray image (img0)
    // minPixelSize=
    // -1, returns largest region only
    // pixels, threshold for removing smaller regions, with less than minPixelSize pixels
    // 0, returns all detected segments
    
    
    // LB: Zero pad image to remove edge effects when getting regions....   
    int padPixels=20;
    // Rect border added at start...
    Rect tempRect;
    tempRect.x=padPixels;
    tempRect.y=padPixels;
    tempRect.width=img0.cols;
    tempRect.height=img0.rows;
    
    Mat img1 = Mat::zeros(img0.rows+(padPixels*2), img0.cols+(padPixels*2), CV_8UC1);
    img0.copyTo(img1(tempRect));
    
    
    if (useGPU)// converted to GPU -> NOT tested to speed up here!
    {
        gpu::GpuMat imgGPU;
        imgGPU.upload(img1);
        gpu::Canny(imgGPU, imgGPU, 100, 200, 3); //100, 200, 3);
        imgGPU.download(img1);
    }
    else
    {
        Canny(img1, img1, 100, 200, 3); //100, 200, 3);
    }


    // find the contours
    vector< vector<Point> > contours;
    findContours(img1, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Mask for segmented regiond
    Mat mask = Mat::zeros(img1.rows, img1.cols, CV_8UC1);

    vector<double> areas(contours.size());

    if (minPixelSize==-1)
    { // Case of taking largest region
        for(int i = 0; i < (int)contours.size(); i++)
            areas[i] = contourArea(Mat(contours[i]));
        double max;
        Point maxPosition;
        minMaxLoc(Mat(areas),0,&max,0,&maxPosition);
        drawContours(mask, contours, maxPosition.y, Scalar(1), CV_FILLED);
    }
    else
    { // Case for using minimum pixel size
        for (int i = 0; i < (int)contours.size(); i++)
        {
            if (contourArea(Mat(contours[i]))>minPixelSize)
            drawContours(mask, contours, i, Scalar(1), CV_FILLED);
        }
    }
    // normalize so imwrite(...)/imshow(...) shows the mask correctly!
    normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);
    
    Mat returnMask;
    returnMask=mask(tempRect);
    
    // show the images
    if (displayFaces)   imshow("Canny: Img in", img0);
    if (displayFaces)   imshow("Canny: Mask", returnMask);
    if (displayFaces)   imshow("Canny: Output", img1);
    
    return returnMask;
}

Mat visionUtils::drawRotatedRect(Mat image, RotatedRect rRect, Scalar colourIn)
{

Point2f vertices[4];
rRect.points(vertices);
for (int i = 0; i < 4; i++)
    line(image, vertices[i], vertices[(i+1)%4], colourIn, 3); //Scalar(0,255,0));

return image;
}

int visionUtils::updateArmPoints(Point2f previousPoint, Point2f *currentPoints, int mode)
{

    int final_mag = 0;
    // loop through current points
    double temp_mag;
    // Run for first point -> init sort_test (to find smallest value)
    double magnitude;

    magnitude = pow((currentPoints[0].x-previousPoint.x),2)+pow((currentPoints[0].y-previousPoint.y),2);
    temp_mag = magnitude;
    
    if( mode == 0 )
    {
        for( int i = 1; i < 4; i++ )
        {
            magnitude = pow((currentPoints[i].x-previousPoint.x),2)+pow((currentPoints[i].y-previousPoint.y),2);
            if (magnitude <= temp_mag)
            {
                temp_mag = magnitude;
                final_mag = i;
            }
        }
    }
    else if( mode == 1 )
    {
        for( int i = 1; i < 4; i++ )
        {
            magnitude = pow((currentPoints[i].x-previousPoint.x),2)+pow((currentPoints[i].y-previousPoint.y),2);
            if (magnitude > temp_mag)
            {
                temp_mag = magnitude;
                final_mag = i;
            }
        }
    }
    else
    {
        cout << "MODE not specified!" << endl;
        return 0;
    }

    return final_mag; // index of current point closest or longest to last previous point
}


vector<Point2f> visionUtils::updateArmMiddlePoint(Point2f previousPoint, Point2f *currentPoints, int mode)
{
    double global_mag;
    double temp_mag;
    double magnitude;
    vector<Point2f> tempCurrentPoints;
    vector<Point2f> smallestPoints;
    Point2f indices[2];
    Point2f middlePoint;
    int indexToRemove = 0;


    for( int i = 0; i < 4; i++ )
        tempCurrentPoints.push_back(currentPoints[i]);

    magnitude = 100000;
    global_mag = magnitude;

    if( mode == 0 )
    {
        for( int j = 0; j < 2; j++ )
        {
            temp_mag = global_mag;
            for( int i = 0; i < (int)tempCurrentPoints.size(); i++ )
            {
                magnitude = pow((tempCurrentPoints.at(i).x-previousPoint.x),2)+pow((tempCurrentPoints.at(i).y-previousPoint.y),2);
                if (magnitude <= temp_mag )
                {
                    temp_mag = magnitude;
                    indices[j].x = tempCurrentPoints.at(i).x;
                    indices[j].y = tempCurrentPoints.at(i).y;
            indexToRemove = i;
                }
            }
            tempCurrentPoints.erase(tempCurrentPoints.begin()+indexToRemove);
            previousPoint.x = indices[j].x;
            previousPoint.y = indices[j].y;
        }
    }
    else if( mode == 1 )
    {
        for( int j = 0; j < 2; j++ )
        {
            temp_mag = global_mag;
            for( int i = 0; i < (int)tempCurrentPoints.size(); i++ )
            {
                magnitude = pow((tempCurrentPoints.at(i).x-previousPoint.x),2)+pow((tempCurrentPoints.at(i).y-previousPoint.y),2);
                if (magnitude > temp_mag)
                {
                    temp_mag = magnitude;
                    indices[j].x = tempCurrentPoints.at(i).x;
                    indices[j].y = tempCurrentPoints.at(i).y;
                    indexToRemove = i;
                }
            }
            tempCurrentPoints.erase(tempCurrentPoints.begin()+indexToRemove);
            previousPoint.x = indices[j].x;
            previousPoint.y = indices[j].y;
        }
    }
    else
    {
        cout << "MODE not specified!" << endl;

        smallestPoints.push_back(previousPoint);    //first closest point
        smallestPoints.push_back(previousPoint);    //first closest point
        smallestPoints.push_back(previousPoint);    //first closest point

        return smallestPoints;
    }

    middlePoint = currentPoints[0];
    middlePoint.x = (indices[0].x+indices[1].x)/2.0;
    middlePoint.y = (indices[0].y+indices[1].y)/2.0;

    smallestPoints.push_back(indices[0]);    //first closest point
    smallestPoints.push_back(indices[1]);    //second closest point
    smallestPoints.push_back(middlePoint);    //middle point

    return smallestPoints;    //middle point from the two closest points 
}

double visionUtils::getFrameRate(clock_t start)
{
  // Fn to calc frame rate... must be initalised with start recording time using: 
  // clock_t start = clock();
  // then called with this clock time... (start)
  // uses std ctime lib
    
  // End - time in microseconds
  clock_t finish = clock();

  // Measure time elapsed
  double frameRate = 1.0/(double(finish - start) / CLOCKS_PER_SEC);
  return frameRate;
}

// get next hand position using Kalman Filter
vector<Point2f> visionUtils::getPredictedHandPosition(Point2f currentPoint, int mode)
{
    vector<Point2f> predictedPoint;

    // First predict, to update the internal statePre variable
    Mat prediction = KF.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
              
    measurement(0) = currentPoint.x;
    measurement(1) = currentPoint.y; 
  
    // The update phase 
    Mat estimated = KF.correct(measurement);
 
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    Point measPt(measurement(0),measurement(1));

    middlePointV = measPt;
    kalmanMiddlePointV = statePt;

    predictedPoint.push_back(middlePointV);    
    predictedPoint.push_back(kalmanMiddlePointV);

    return predictedPoint;
}

void visionUtils::initKalmanFilterParameters(Point2f previousPoint)
{
    KF.init(4, 2, 0);
    KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    
    measurement.create(2,1);
    measurement.setTo(Scalar(0));
    
    KF.statePost.at<float>(0) = previousPoint.x;
    KF.statePost.at<float>(1) = previousPoint.y;
    KF.statePost.at<float>(2) = 0;
    KF.statePost.at<float>(3) = 0;
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(10));
    setIdentity(KF.errorCovPost, Scalar::all(.1));

    middlePointV.x = 0;
    middlePointV.y = 0;
    kalmanMiddlePointV.x = 0;
    kalmanMiddlePointV.y = 0;
}


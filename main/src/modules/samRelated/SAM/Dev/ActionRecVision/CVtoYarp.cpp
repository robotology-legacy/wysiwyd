#include <cv.h>

using namespace yarp::os;
using namespace cv;
using namespace yarp::sig;

void CVtoYarp(cv::Mat MatImage, ImageOf<PixelRgb> & yarpImage);
void CVtoYarp(cv::Mat MatImage, ImageOf<PixelRgbFloat> & yarpImage, bool flip = true);
void CVtoYarp(cv::Mat MatImage, yarp::sig::ImageOf<PixelMono> & yarpImage);

void CVtoYarp(Mat MatImage, ImageOf<PixelRgb> & yarpImage)
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

void CVtoYarp(Mat MatImage, ImageOf<PixelRgbFloat> & yarpImage, bool flip)
{
	IplImage* IPLfromMat = new IplImage(MatImage);

	yarpImage.resize(IPLfromMat->width, IPLfromMat->height);

	IplImage * iplYarpImage = (IplImage*)yarpImage.getIplImage();

	if (IPL_ORIGIN_TL == IPLfromMat->origin){
		cvCopy(IPLfromMat, iplYarpImage, 0);
	}
	else{
		cvFlip(IPLfromMat, iplYarpImage, 0);
	}

	if (IPLfromMat->channelSeq[0] == 'B' & flip) {
		cvCvtColor(iplYarpImage, iplYarpImage, CV_BGR2RGB);
	}
}

void CVtoYarp(Mat MatImage, ImageOf<PixelMono> & yarpImage)
{
	IplImage* IPLfromMat = new IplImage(MatImage);

	yarpImage.resize(IPLfromMat->width, IPLfromMat->height);

	IplImage * iplYarpImage = (IplImage*)yarpImage.getIplImage();

	if (IPL_ORIGIN_TL == IPLfromMat->origin){
		cvCopy(IPLfromMat, iplYarpImage, 0);
	}
	else{
		cvFlip(IPLfromMat, iplYarpImage, 0);
	}
}

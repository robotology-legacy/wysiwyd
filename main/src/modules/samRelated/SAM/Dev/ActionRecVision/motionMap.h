#include <stdio.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/bioinspired.hpp>
#include <iostream>
#include <math.h>
#include <algorithm>

#define devDebug

#include <unistd.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace cv;
using namespace std;

void CVtoYarp(cv::Mat MatImage, yarp::sig::ImageOf<PixelRgb> & yarpImage);
void CVtoYarp(cv::Mat MatImage, yarp::sig::ImageOf<PixelRgbFloat> & yarpImage, bool flip = true);
void CVtoYarp(cv::Mat MatImage, yarp::sig::ImageOf<PixelMono> & yarpImage);
void pauseExec(int sleepms);

/*
* Copyright(C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
* Authors: Stephane Lallee
* email : stephane.lallee@gmail.com
* Permission is granted to copy, distribute, and / or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* wysiwyd / license / gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU General
* Public License for more details
*/

#include <iostream>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;

using namespace std;

class ImageSplitter : public yarp::os::RFModule
{
	BufferedPort<ImageOf<PixelRgb> > inPort;
	bool showImages;

public:
	vector< vector<BufferedPort<ImageOf<PixelRgb> >* > > ports;
	bool configure(ResourceFinder &rf)
	{
		string name = rf.check("name", Value("imageSplitter")).asString();
		setName(name.c_str());

		inPort.open(("/" + name + "/image:i").c_str());

		int splitW = rf.check("splitW", Value(3)).asInt();
		int splitH = rf.check("splitH", Value(3)).asInt();
		showImages = rf.check("showImages");

		ports.resize(splitW);
		for (int x = 0; x < splitW; x++)
		{
			ports[x].resize(splitH);
			for (int y = 0; y < splitH; y++)
			{
				stringstream ss;
				ss <<"/"<< getName().c_str() << "/split/" << x << "_" << y << ":o";
				ports[x][y] = new BufferedPort<ImageOf<PixelRgb> >();
				ports[x][y]->open(ss.str().c_str());
			}
		}


		if (showImages)
		{
			int xpos = 0;
			int ypos = 0;
			for (int x = 0; x < splitW; x++)
			{
				ports[x].resize(splitH);
				for (int y = 0; y < splitH; y++)
				{
					stringstream ss;
					ss << "/" << getName().c_str() << "/split/" << x << "_" << y << ":o";
					cvNamedWindow(ss.str().c_str(), CV_WINDOW_AUTOSIZE);
					cvMoveWindow(ss.str().c_str(), xpos + 300 * x, ypos + 300*y);
				}
			}
		}
		return true;
	}

	double getPeriod()
	{
		return 0.01;
	}

	bool updateModule()
	{
		ImageOf<PixelRgb> *img = inPort.read(false);
		if (img)
		{
			splitNsend((IplImage*)img->getIplImage());
		}
		return true;
	}

	void splitNsend(IplImage* img)
	{
		cvCvtColor(img, img, CV_BGR2RGB);
		int rectW = img->width / ports.size();
		int rectH = img->height / ports[0].size();
		IplImage *img2 = cvCreateImage(cvSize(rectW,rectH), img->depth, img->nChannels);
		for (int x = 0; x < ports.size(); x++)
		{
			for (int y = 0; y < ports[x].size(); y++)
			{
				cvSetImageROI(img, cvRect(x*rectW, y*rectH, rectW, rectH));
				cvCopy(img, img2, NULL);
				ImageOf<PixelRgb> &imgYarp = ports[x][y]->prepare();
				imgYarp.wrapIplImage(img2);
				ports[x][y]->write(true); 
				
				if (showImages)
				{
					stringstream ss;
					ss << "/" << getName().c_str() << "/split/" << x << "_" << y << ":o";
					cvShowImage(ss.str().c_str(), img2);
					cvWaitKey(1);
				}
				cvResetImageROI(img);
			}
		}
		cvReleaseImage(&img2);
	}

	bool close()
	{
		for (int x = 0; x < ports.size(); x++)
		{
			for (int y = 0; y < ports[x].size(); y++)
			{
				ports[x][y]->interrupt();
				ports[x][y]->close();
				delete ports[x][y];
			}
		}
		return true;
	}
};

int main(int argc, char *argv[]) 
{
	Network yarp;
	if (!yarp.checkNetwork())
	{
		printf("yarp network is not available!\n");
		return -1;
	}

	ResourceFinder rf;
	rf.setDefaultContext("imageSplitter");

	rf.configure(argc, argv);

	ImageSplitter mod;
	return mod.runModule(rf);
    return 0;
}

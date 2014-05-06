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

class Retina
{
	std::string name;
	vector< vector<BufferedPort<ImageOf<PixelRgb> >* > > ports;
	int reconstructWidth;
	int reconstructHeight;

public:
	Retina(std::string _name, int w, int h)
	{
		reconstructWidth = reconstructHeight = - 1;
		name = _name;
		ports.resize(w);
		for (int x = 0; x < w; x++)
		{
			ports[x].resize(h);
			for (int y = 0; y < h; y++)
			{
				stringstream ss;
				ss << "/" << name << "/split/" << x << "_" << y << ":i";
				ports[x][y] = new BufferedPort<ImageOf<PixelRgb> >();
				ports[x][y]->open(ss.str().c_str());
			}
		}
	}

	void connect(std::string splitterName)
	{
		for (int x = 0; x < ports.size(); x++)
		{
			for (int y = 0; y < ports[x].size(); y++)
			{
				stringstream ssSource;
				ssSource << "/" << splitterName << "/split/" << x << "_" << y << ":o";
				Network::connect(ssSource.str().c_str(), ports[x][y]->getName().c_str());
			}
		}
	}

	void createCvWindow(int xpos, int ypos)
	{
		cvNamedWindow( (name+"/Reconstruct").c_str(), CV_WINDOW_AUTOSIZE);
		cvMoveWindow((name + "/Reconstruct").c_str(), xpos, ypos);
	}

	void readAndShow(bool refreshGui)
	{
		if (reconstructWidth == -1 || reconstructHeight == -1)
		{
			//Sample the subdivision sizes
			std::cout << "Waiting for input..." << std::endl;
			ImageOf<PixelRgb>* testImg = ports[0][0]->read(true);
			reconstructWidth = testImg->width() * ports.size();
			reconstructHeight = testImg->height() * ports[0].size();
			std::cout << "Input detected with size : " << testImg->width() << "x" << testImg->height() << std::endl;
			std::cout << "Reconstruct size will: " << reconstructWidth << "x" << reconstructHeight << std::endl;
		}

		IplImage* reconstruct;
		reconstruct = cvCreateImage(cvSize(reconstructWidth, reconstructHeight), 8, 3);
		
		for (int x = 0; x < ports.size(); x++)
		{
			for (int y = 0; y < ports[x].size(); y++)
			{
				ImageOf<PixelRgb>* imgYarp = ports[x][y]->read(true);
				if (imgYarp)
				{
					IplImage* partImg = (IplImage*) imgYarp->getIplImage();
					cvCvtColor(partImg, partImg, CV_BGR2RGB);
					int rectW = reconstructWidth / ports.size();
					int rectH = reconstructHeight / ports[0].size();
					cvSetImageROI(reconstruct, cvRect(x*rectW, y*rectH, rectW, rectH));
					cvCopyImage(partImg, reconstruct);
					cvResetImageROI(reconstruct);
				}

			}
		}
		if (refreshGui)
			cvShowImage((name + "/Reconstruct").c_str(), reconstruct);

		cvWaitKey(1);
		cvReleaseImage(&reconstruct);
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

class ImageFuser : public yarp::os::RFModule
{
	double foveaRatio;
	bool showImages;

public:
	Retina* global;
	Retina* fovea;

	bool configure(ResourceFinder &rf)
	{
		string name = rf.check("name", Value("imageFuser")).asString();
		setName(name.c_str());

		string nameSource = rf.check("nameSplitter", Value("imageSplitter")).asString();
		showImages = true;// rf.check("showImages");

		int splitW = rf.check("splitW", Value(3)).asInt();
		int splitH = rf.check("splitH", Value(3)).asInt();
		int splitFovea = rf.check("splitFovea", Value(1)).asInt();
		foveaRatio = rf.check("foveaRatio", Value(0.3)).asDouble();

		//Create ports
		global = new Retina(name, splitW, splitH);

		fovea = new Retina(name+"/fovea", splitFovea, splitFovea);
		if (showImages)
		{
			global->createCvWindow(0, 0);
			fovea->createCvWindow(640, 0);
		}
		global->connect(nameSource);
		fovea->connect(nameSource+"/fovea");
		return true;
	}

	double getPeriod()
	{
		return 0.01;
	}

	bool updateModule()
	{
		global->readAndShow(showImages);
		fovea->readAndShow(showImages);
		return true;
	}

	bool close()
	{
		bool result = global->close();
		result &= fovea->close();
		return result;
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

	ImageFuser mod;
	return mod.runModule(rf);
    return 0;
}

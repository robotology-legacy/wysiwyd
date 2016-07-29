#include "motionMap.h"
//performance 16fps on GTX675MX / i7
int main(int argc, char** argv)
{
	std::string imageInPortName;
	std::string imageOutPortName;
	std::string magnoOutPortName;

	char mess[100];

	Network yarp;

	if (argc < 4)
	{
		cout << "Not enough arguments. Must provide port name to the input and output ports" << endl;
		cout << "Exiting ..." << endl;
		return -1;
	}
	else
	{
		imageInPortName = argv[1];
		imageOutPortName = argv[2];
		magnoOutPortName = argv[3];
	}


	BufferedPort<ImageOf<PixelRgb> > imageInPort;
	BufferedPort<ImageOf<PixelRgb> > imageOutPort;
	BufferedPort<ImageOf<PixelMono> > magnoOutPort;

	bool inOpen = imageInPort.open(imageInPortName.c_str());
	bool outOpen1 = imageOutPort.open(imageOutPortName.c_str());
	bool outOpen3 = magnoOutPort.open(magnoOutPortName.c_str());

	if (!(inOpen | outOpen1 | outOpen3))
	{
		cout << "Could not open ports. Exiting" << endl;
		return -1;
	}

	double t = 0, time = 0, time2 = 0;
	bool inCount1 = false, inCount2 = false;
	bool outCount1 = false, outCount2 = false, outCount3 = false;
	int step = 0, stereoStep = 0;
	int count = 0;
	cv::Ptr<cv::bioinspired::Retina> myRetina;

#ifdef devDebug
	{		
		yarp.connect("/grabber", "/image/in");
		yarp.connect("/image/out", "/imageOut");
		yarp.connect("/magno/out", "/magnoOut");
	}
#endif

	//acquire image to setup retina parameters
	bool setup = false;
	Size imageSize, inputSize;
	int border = 100;

	while (setup == false)
	{
		inCount1 = imageInPort.getInputCount() != 0;

		if (!inCount1 & !inCount2)
		{
			cout << "Awaiting input images" << endl;
			pauseExec(100);
		}
		else
		{
			ImageOf<PixelRgb> *imageIn = imageInPort.read();
			if (imageIn != NULL)
			{
				count = 0;
				step = imageIn->getRowSize() + imageIn->getPadding();
				Mat left_cpuRGB(imageIn->height(), imageIn->width(), CV_8UC3, imageIn->getRawImage(), step);
				imageSize = left_cpuRGB.size();

				//retina process to hdr input images currently too slow to implement fast enough
				myRetina = cv::bioinspired::createRetina(imageSize);
				myRetina->setupOPLandIPLParvoChannel(true, true, 0.89f, 0.5f, 0.53f, 0.3f, 1.0f, 7.0f, 0.89f);
				myRetina->clearBuffers();

				setup = true;
			}
			else
			{
				count++;
				if (count == 50)
				{
					cout << "No input image detected" << endl;
					return 0;
				}
			}
		}
	}

	Mat magnoOutMat, stereoMat;
	bool magno = false;
	bool parvo = false;
	bool imagesIn = false;
	bool imagesOut = false;

	while (true)
	{
		inCount1 = imageInPort.getInputCount() != 0;
		outCount1 = imageOutPort.getOutputCount() != 0;
		outCount3 = magnoOutPort.getOutputCount() != 0;

		if (!inCount1 & !outCount1 & !outCount3)
		{
			cout << "Awaiting input and output connections" << endl;
			pauseExec(100);
		}
		else
		{
			ImageOf<PixelRgb> *imageIn = imageInPort.read();
			if (imageIn != NULL)
			{
				t = (double)getTickCount();
				count = 0;
				step = imageIn->getRowSize() + imageIn->getPadding();
				Mat imin(imageIn->height(), imageIn->width(), CV_8UC3, imageIn->getRawImage(), step);

				myRetina->run(imin);
				myRetina->getMagno(magnoOutMat);

				ImageOf<PixelRgb>& imageOut = imageOutPort.prepare();
				ImageOf<PixelMono>&  magnoOut = magnoOutPort.prepare();

				imageOut = *imageIn;
				CVtoYarp(magnoOutMat, magnoOut);

				imageOutPort.write();
				magnoOutPort.write();

				time = ((((double)getTickCount() - t) / getTickFrequency())*0.5) + (time*0.5);
			}
		}
	}
}

void pauseExec(int sleepms)
{
	usleep(sleepms * 1000);   // usleep takes sleep time in us (1 millionth of a second)
}

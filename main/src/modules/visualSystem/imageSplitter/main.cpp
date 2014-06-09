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
    vector< vector<BufferedPort<ImageOf<PixelRgb> >* > > ports;
    vector< vector< ImageOf<PixelRgb> > > preparedImgs;
public:
    Retina(std::string name, int w, int h)
    {
        ports.resize(w);
        preparedImgs.resize(w);
        for (int x = 0; x < w; x++)
        {
            ports[x].resize(h);
            preparedImgs[x].resize(h);
            for (int y = 0; y < h; y++)
            {
                stringstream ss;
                ss << "/" << name << "/split/" << x << "_" << y << ":o";
                ports[x][y] = new BufferedPort<ImageOf<PixelRgb> >();
                ports[x][y]->open(ss.str().c_str());
                preparedImgs[x][y] = ports[x][y]->prepare();
            }
        }
    }

    void createCvWindows(int xpos, int ypos)
    {
        for (unsigned int x = 0; x < ports.size(); x++)
        {
			for (unsigned int y = 0; y < ports[x].size(); y++)
            {
                cvNamedWindow(ports[x][y]->getName().c_str(), CV_WINDOW_AUTOSIZE);
                cvMoveWindow(ports[x][y]->getName().c_str(), xpos + 300 * x, ypos + 300 * y);
            }
        }
    }

    void writeAndShow(IplImage* wholeImage, bool refreshGui)
    {
        IplImage* img;
        if (wholeImage->roi != NULL)
            img = cvCreateImage(cvSize(wholeImage->roi->width, wholeImage->roi->height), wholeImage->depth, wholeImage->nChannels);
        else
            img = cvCreateImage(cvSize(wholeImage->width, wholeImage->height), wholeImage->depth, wholeImage->nChannels);
        cvCopyImage(wholeImage, img);
        cvCvtColor(img, img, CV_BGR2RGB);
        int rectW = img->width / ports.size();
        int rectH = img->height / ports[0].size();
        
		for (unsigned int x = 0; x < ports.size(); x++)
        {
			for (unsigned int y = 0; y < ports[x].size(); y++)
            {
                cvSetImageROI(img, cvRect(x*rectW, y*rectH, rectW, rectH));
                ImageOf<PixelRgb> &imgYarp = ports[x][y]->prepare();
                imgYarp.resize(rectW, rectH);
                cvCopy(img, (IplImage*)imgYarp.getIplImage(), NULL);
                if (refreshGui)
                    cvShowImage(ports[x][y]->getName().c_str(), img);
            }
        }

        cvWaitKey(1);
        cvReleaseImage(&img);

        //write all at once to avoid diagonal delay
		for (unsigned int x = 0; x < ports.size(); x++)
        {
			for (unsigned int y = 0; y < ports[x].size(); y++)
            {
                ports[x][y]->write(true);
            }
        }
    }
    
    bool close()
    {
		for (unsigned int x = 0; x < ports.size(); x++)
        {
			for (unsigned int y = 0; y < ports[x].size(); y++)
            {
                ports[x][y]->interrupt();
                ports[x][y]->close();
                delete ports[x][y];
            }
        }
        return true;
    }
};

class ImageSplitter : public yarp::os::RFModule
{
    BufferedPort<ImageOf<PixelRgb> > inPort;
    double foveaRatio;
    bool showImages;

public:
    Retina* global;
    Retina* fovea;

    bool configure(ResourceFinder &rf)
    {
        string name = rf.check("name", Value("imageSplitter")).asString();
        setName(name.c_str());

        inPort.open(("/" + name + "/image:i").c_str());

        int splitW = rf.check("splitW", Value(3)).asInt();
        int splitH = rf.check("splitH", Value(3)).asInt();
        int splitFovea = rf.check("splitFovea", Value(1)).asInt();
        foveaRatio = rf.check("foveaRatio", Value(0.3)).asDouble();
        showImages = rf.check("showImages");

        //Create ports
        global = new Retina(name, splitW, splitH);
        fovea = new Retina(name+"/fovea", splitFovea, splitFovea);
        if (showImages)
        {
            global->createCvWindows(0, 0);
            fovea->createCvWindows(640, 0);
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
            IplImage* cvImg = (IplImage*)img->getIplImage();
            global->writeAndShow(cvImg, showImages);
            
            //Create the input for the fovea
            IplImage* foveaImg = cvCreateImage(cvSize(cvImg->width, cvImg->height), cvImg->depth, cvImg->nChannels);
            cvCopyImage(cvImg, foveaImg);
            cvSetImageROI(foveaImg, cvRect(
                foveaImg->width / 2 - (int)((foveaRatio*foveaImg->width)/2.0),
                foveaImg->height / 2 - (int)((foveaRatio*foveaImg->height)/2.0),
                (int)(foveaRatio*foveaImg->width), 
                (int)(foveaRatio*foveaImg->height)));
            fovea->writeAndShow(foveaImg, showImages);
            cvReleaseImage(&foveaImg);
        }
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

    ImageSplitter mod;
    return mod.runModule(rf);
    return 0;
}

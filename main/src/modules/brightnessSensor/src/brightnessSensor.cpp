#include "brightnessSensor.h"

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <yarp/sig/all.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <yarp/os/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

bool brightnessSensor::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("brightnessSensor")).asString().c_str();
    setName(moduleName.c_str());

    pBrightness = 0;
    // by default uses right cam
    boolRightCam        = rf.check("rightArm", Value((int)1)).asInt();
    boolLeftCam         = rf.check("leftArm", Value((int)0)).asInt();

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();



    bool    bEveryThingisGood = true;

    cout << "hi!";
    // create one input port from left cam
    // Open port2leftCam
    if (boolLeftCam) {
        Port2camLeftName = "/";
        Port2camLeftName += getName() + "/camLeft:i";
    }
    cout << "1";

    if (!Port2camLeft.open(Port2camLeftName.c_str())) {
        cout << getName() << ": Unable to open port " << Port2camLeftName << endl;
        bEveryThingisGood &= false;
    }
    cout << "1";
    // create one input port from left cam
    // Open port2rightCam
    if (boolRightCam) {
        Port2camRightName = "/";
        Port2camRightName += getName() + "/camRight:i";
    }
    cout << "1";
    if (!Port2camRight.open(Port2camRightName.c_str())) {
        cout << getName() << ": Unable to open port " << Port2camRightName << endl;
        bEveryThingisGood &= false;
    }

    // create one input port for brightness
    // Open port2outBrightness
    Port2outBrightnessName = "/";
    Port2outBrightnessName += getName() + "/brightness:o";


    cout << "yihaa";
    if (!Port2outBrightness.open(Port2outBrightnessName.c_str())) {
        cout << getName() << ": Unable to open port " << Port2outBrightnessName << endl;
        bEveryThingisGood &= false;
    }
    cout << "ehiiy!";

    while (!Network::connect("/icub/cam/right", Port2camRightName))
    {
        std::cout << "Trying to get input from RIGHTCAM..." << std::endl;
        yarp::os::Time::delay(1.0);
    }
    cout<<moduleName<<": finding configuration files..."<<endl;
    /*
    if (boolRightCam) {
        bEveryThingisGood &= Network::connect("/icub/cam/right", Port2camRightName);
    }   
    if (boolLeftCam) {
        bEveryThingisGood &= Network::connect("/icub/cam/left", Port2camLeftName);
    }
    */
    
    /*
    bEveryThingisGood &= Network::connect(port2abmName.c_str(), "/autobiographicalMemory/request:i");
    bEveryThingisGood &= Network::connect(port2SpeechRecogName.c_str(), "/speechRecognizer/rpc");
    bEveryThingisGood &= Network::connect(port2abmReasoningName.c_str(), "/abmReasoning/rpc");
    */

    rpc.open(("/" + moduleName + "/rpc").c_str());
    cout << "almost done";
    attach(rpc);

    cout<<"Configuration finished!";
    return true;
}


bool brightnessSensor::close() {
    //iCub->close();
    //delete iCub;
    Port2camRight.close();
    Port2outBrightness.close();
    rpc.close();

    return true;
}


bool brightnessSensor::respond(const Bottle& command, Bottle& reply) {

    return true;
}


/* Called periodically every getPeriod() seconds */
bool brightnessSensor::updateModule() {
    // get the input from camRight
    cout << "something else";
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* img = Port2camRight.read();
    cout << "something";
    if (img){
        IplImage* cvImg = (IplImage*)img->getIplImage();
        unsigned int w = cvImg->width;
        unsigned int h = cvImg->height;
        double pBlue = 0.0;
        double pRed = 0.0;
        double pGreen = 0.0;


        for (unsigned int y = 0; y < h; y++){
            uchar* ptr = (uchar*)(cvImg->imageData + cvImg->widthStep*y); //read line of image
            for (unsigned int x = 0; x < w; x++)
            {
                pBlue += ptr[3*x]/(double)255;
                pGreen += ptr[3*x+1]/(double)255;
                pRed += ptr[3*x+2]/(double)255;
            }
        }
        pBrightness = pBlue * 0.114 + pGreen * 0.587 + pRed * 0.299; //Brightenss formula for RGB. Data is in BGR :S
        pBrightness = pBrightness/(double)(cvImg->width*cvImg->height);


        yarp::os::Bottle &brightness = Port2outBrightness.prepare();
        brightness.clear();

        brightness.addDouble(pBrightness);
        Port2outBrightness.write();

        cvReleaseImage(&cvImg);
    }

    return true;
}





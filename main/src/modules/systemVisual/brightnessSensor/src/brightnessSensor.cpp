
#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdlib>
#include <vector>
#include <iostream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "brightnessSensor.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


bool brightnessSensor::connect(const std::string &targetPort = "/icub/face/emotions/in")
     {
         return yarp::os::Network::connect(toEmotionInterface.getName(),targetPort.c_str());
     }

bool brightnessSensor::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("brightnessSensor")).asString().c_str();
    setName(moduleName.c_str());

    pBrightness = 0;
    cout<<"If you see this printed is that the eyelid opening is still hardcoded"<<endl;
    minEyeOpening = 40;
    maxEyeOpening = 60;
    scaledValue = maxEyeOpening;

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();



    bool    bEveryThingisGood = true;


    // create one input port from right cam
    // Open port2rightCam
    Port2camRightName = "/";
    Port2camRightName += getName() + "/camRight:i";
    if (!Port2camRight.open(Port2camRightName.c_str())) {
        cout << getName() << ": Unable to open port " << Port2camRightName << endl;
        bEveryThingisGood = false;
    }

    // create one input port for brightness
    // Open port2outBrightness
    Port2outBrightnessName = "/";
    Port2outBrightnessName += getName() + "/brightness:o";


    if (!Port2outBrightness.open(Port2outBrightnessName.c_str())) {
        cout << getName() << ": Unable to open port " << Port2outBrightnessName << endl;
        bEveryThingisGood = false;
    }

    while (!Network::connect("/icub/cam/right", Port2camRightName))
    {
        std::cout << "Trying to get input from RIGHTCAM..." << std::endl;
        yarp::os::Time::delay(1.0);
    }
    cout<<moduleName<<": finding configuration files... 12"<<endl;


    std::string portName = "/";
    portName +=moduleName.c_str();
    portName +="/emotions:o";
    toEmotionInterface.open(portName.c_str());
    connect();

    if(!Network::exists(("/" + moduleName + "/rpc").c_str())){
        rpc.open(("/" + moduleName + "/rpc").c_str());
        attach(rpc);
    }

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
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* img = Port2camRight.read();
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
                pBlue += pow(ptr[3*x]/(double)255,2);
                pGreen += pow(ptr[3*x+1]/(double)255,2);
                pRed += pow(ptr[3*x+2]/(double)255,2);
            }
        }
        //pBrightness = pBlue * 0.114 + pGreen * 0.587 + pRed * 0.299; //Brightenss formula for RGB. Data is in BGR :S
        pBrightness = (pBlue + pGreen + pRed) / 3; //Brightenss as mean intensity of colors
        pBrightness = pBrightness/(double)(cvImg->width*cvImg->height);

        if (true){
        //Eyelids to be move in the new function
        if(pBrightness>0.35){
            scaledValue -= 1;
            scaledValue = max(minEyeOpening,min(maxEyeOpening,scaledValue));
        }else if(pBrightness<0.22){
            scaledValue += 1;
            scaledValue = max(minEyeOpening,min(maxEyeOpening,scaledValue));
        }
        std::stringstream strstr;
        strstr<<'S'<<scaledValue;
        std::string code = strstr.str();
        cmd.clear();
        cmd.addString("set");
        cmd.addString("raw");
        cmd.addString(code.c_str());
        toEmotionInterface.write(cmd);
        //cout<<"Sending..."<<cmd.toString().c_str()<<endl;
        }
        yarp::os::Bottle &brightness = Port2outBrightness.prepare();
        brightness.clear();


        brightness.addDouble(pBrightness);
        Port2outBrightness.write();
        cout<<"WARNING: The image is not being released!!"<<endl;


        //cvReleaseImage(&cvImg);

    }
    cout<<"or here"<<endl;

    return true;
}





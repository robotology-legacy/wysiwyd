/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Martina Zambelli, Tobias Fischer
 * email:   m.zambelli13@imperial.ac.uk, t.fischer@imperial.ac.uk
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "sensoryProcessor.h"

#include <vector>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

bool SensoryProcessor::configure(yarp::os::ResourceFinder &rf) {
    bool bEveryThingisGood = true;

    moduleName = rf.check("name",Value("sensoryProcessor"),"module name (string)").asString();

    part = rf.check("part",Value("left_arm")).asString();
    robot = rf.check("robot",Value("icubSim")).asString();
    fps = rf.check("fps",Value(30)).asInt(); //30;
    useSFM = rf.check("useSFM",Value(1)).asInt();

    // keep this part if you want to use both cameras
    // if you only want to use want, delete accordingly
    if (robot=="icub")
    {
        leftCameraPort = "/"+robot+"/camcalib/left/out";
        rightCameraPort = "/"+robot+"/camcalib/right/out";
    }
    else //if (robot=='icubSim')
    {
        leftCameraPort = "/"+robot+"/cam/left";
        rightCameraPort = "/"+robot+"/cam/right";
    }

    setName(moduleName.c_str());

    // Open handler port
    if (!handlerPort.open("/" + getName() + "/rpc")) {
        yError() << getName() << ": Unable to open port " << "/" << getName() << "/rpc";
        bEveryThingisGood = false;
    }

    if (!imgPortIn.open("/" + getName() + "/img:i")) {
            yError() << getName() << ": Unable to open port " << "/" << getName() << "/img:i";
    }

    while(!Network::isConnected(leftCameraPort, imgPortIn.getName())) {
        Network::connect(leftCameraPort, imgPortIn.getName());
        yInfo() << "Waiting for port " << leftCameraPort << " to connect to " << imgPortIn.getName();
        Time::delay(1.0);
    }

    if (!featureImgPortOut.open("/" + getName() + "/featureImg:o")) {
            yError() << getName() << ": Unable to open port " << "/" << getName() << "featureImg:o";
    }

    if (!portBodypartsPositionOut.open("/" + getName() + "/bodypartpos:o")) {
        yError() << getName() << ": Unable to open port " << "/" << getName() << "/bodypartpos:o";
        bEveryThingisGood = false;
    }

    if (!portSkinOut.open("/" + getName() + "/skin:o")) {
        yError() << getName() << ": Unable to open port " << "/" << getName() << "/skin:o";
        bEveryThingisGood = false;
    }

    if (!portArmEncodersOut.open("/" + getName() + "/armencoders:o")) {
        yError() << getName() << ": Unable to open port " << "/" << getName() << "/armencoders:o";
        bEveryThingisGood = false;
    }

    if (!portHeadEncodersOut.open("/" + getName() + "/headencoders:o")) {
        yError() << getName() << ": Unable to open port " << "/" << getName() << "/headencoders:o";
        bEveryThingisGood = false;
    }

    if (!portToABM.open("/" + getName() + "/toABM")) {
        yError() << getName() << ": Unable to open port " << "/" + getName() + "/toABM";
        bEveryThingisGood = false;
    }

    if (!portReadSkin.open("/" + getName() + "/portReadSkin:i")) {
        yError() << ": Unable to open port " << "/" << getName() << "portReadSkin:i";
        bEveryThingisGood = false;
    }

    if(useSFM)
    {
        if (!portToSFM.open("/toSFM")) {
            yError() << ": Unable to open port " << "/toSFM";
            bEveryThingisGood = false;
        }
    }

    if(part=="right_arm")
    {
        while(!Network::isConnected("/"+robot+"/skin/right_hand_comp", portReadSkin.getName())) {
            Network::connect("/"+robot+"/skin/right_hand_comp", portReadSkin.getName());
            yInfo() << "Waiting for port " << "/"+robot+"/skin/right_hand_comp" << " to connect to " << portReadSkin.getName();
            Time::delay(1.0);
        }
    }
    else
    {
        while(!Network::isConnected("/"+robot+"/skin/left_hand_comp", portReadSkin.getName())) {
            Network::connect("/"+robot+"/skin/left_hand_comp", portReadSkin.getName());
            yInfo() << "Waiting for port " << "/"+robot+"/skin/left_hand_comp" << " to connect to " << portReadSkin.getName();
            Time::delay(1.0);
        }
    }

    if(useSFM)
    {
        if (!portHandPositionOut.open("/" + getName() + "/handpos:o")) {
            yError() << getName() << ": Unable to open port " << "/" << getName() << "/handpos:o";
            bEveryThingisGood = false;
        }

        while(!Network::isConnected(portToSFM.getName(), "/SFM/rpc")) {
            Network::connect(portToSFM.getName(), "/SFM/rpc");
            yInfo() << "Waiting for port " << portToSFM.getName() << " to connect to " << "/SFM/rpc";
            Time::delay(1.0);
        }
    }

    yInfo() << "Connections ok...";

    Network::connect(portToABM.getName(), "/autobiographicalMemory/rpc");
    if(!Network::isConnected(portToABM.getName(), "/autobiographicalMemory/rpc")){
        yWarning() << "Cannot connect to ABM, storing data into it will not be possible unless manual connection";
    } else {
        yInfo() << "Connected to ABM : Data are coming!";
    }

    // Initialize iCub and Vision
    while (!init_iCub(part)) {
        yDebug() << getName() << ": initialising iCub... please wait... ";
        bEveryThingisGood = false;
    }

    yDebug() << "End configuration...";

    MAX_COUNT = 150;

    attach(handlerPort);

    return bEveryThingisGood;
}

bool SensoryProcessor::interruptModule() {

    imgPortIn.interrupt();
    featureImgPortOut.interrupt();
    portToSFM.interrupt();
    portToABM.interrupt();
    handlerPort.interrupt();
    portHandPositionOut.interrupt();
    portHeadEncodersOut.interrupt();
    portArmEncodersOut.interrupt();
    portBodypartsPositionOut.interrupt();
    portSkinOut.interrupt();
    portReadSkin.interrupt();

    yInfo() << "Bye!";

    return true;
}

bool SensoryProcessor::close() {
    yInfo() << "Closing module, please wait ... ";

    armDev->close();
    headDev->close();

    imgPortIn.interrupt();
    imgPortIn.close();

    featureImgPortOut.interrupt();
    featureImgPortOut.close();

    portHandPositionOut.interrupt();
    portHandPositionOut.close();

    portBodypartsPositionOut.interrupt();
    portBodypartsPositionOut.close();

    portHeadEncodersOut.interrupt();
    portHeadEncodersOut.close();

    portSkinOut.interrupt();
    portSkinOut.close();

    portArmEncodersOut.interrupt();
    portArmEncodersOut.close();

    portToABM.interrupt();
    portToABM.close();
    
    portToSFM.interrupt();
    portToSFM.close();

    portReadSkin.interrupt();
    portReadSkin.close();

    handlerPort.interrupt();
    handlerPort.close();

    yInfo() << "Bye!";

    return true;
}

bool SensoryProcessor::respond(const Bottle& command, Bottle& reply) {
    yInfo() << "Nothing happens here!";
    reply.addString("nack");

    return true;
}

bool SensoryProcessor::updateModule() {
    getMultimodalData();
    return true;
}

double SensoryProcessor::getPeriod() {
    return 0.1;
}

/*
 * Learning while babbling
 * This learns absolute positions
 */
bool SensoryProcessor::getMultimodalData()
{
    Point2f lost_indic(-1.0f, -1.0f);

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,50, 0.3);//75,0.0001);//20, 0.03);//
    Size subPixWinSize(10,10), winSize(31,31);

    ImageOf<PixelRgb> *yarpImage = imgPortIn.read();
    IplImage *cvImage = cvCreateImage(cvSize(yarpImage->width(), yarpImage->height()), IPL_DEPTH_8U, 3);
    cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage, CV_RGB2BGR);

    image = cvImage;
    if(!image.data) {
        yError() << "Error could not read image data!!!";
        return false;
    }

    cvtColor( image, gray, CV_BGR2GRAY );

    bool findFeat = findFeatures(termcrit, subPixWinSize, winSize);
    if(!findFeat) {
        yError() << "Error finding features";
    }

    std::swap(points[1], points[0]);
    cv::swap(prevGray, gray);

    yarp::sig::Vector handTarget, armTarget, fingerTarget;
    find_image(handTarget, armTarget, fingerTarget);

    /// get 3D position (SFM)
    if(useSFM)
    {
        Bottle bToSFM, bReplyFromSFM;
        bToSFM.clear(); bReplyFromSFM.clear();
        bToSFM.addString("Left");
        bToSFM.addDouble(handTarget[0]);
        bToSFM.addDouble(handTarget[1]);
        portToSFM.write(bToSFM,bReplyFromSFM);

        Bottle& bHandPosition = portHandPositionOut.prepare();
        bHandPosition.clear();
        bHandPosition.addDouble(bReplyFromSFM.get(0).asDouble());
        bHandPosition.addDouble(bReplyFromSFM.get(1).asDouble());
        bHandPosition.addDouble(bReplyFromSFM.get(2).asDouble());
        portHandPositionOut.write();
    }

    Bottle &bBodypartpositions = portBodypartsPositionOut.prepare();
    bBodypartpositions.clear();
    bBodypartpositions.addDouble(handTarget[0]);
    bBodypartpositions.addDouble(handTarget[1]);
    bBodypartpositions.addDouble(armTarget[0]);
    bBodypartpositions.addDouble(armTarget[1]);
    bBodypartpositions.addDouble(fingerTarget[0]);
    bBodypartpositions.addDouble(fingerTarget[1]);

    portBodypartsPositionOut.write();

    // get contact vector
    Bottle *skinContact;
    skinContact = portReadSkin.read(true);

    portSkinOut.write(skinContact);

    /// get proprioceptive info
    Bottle &bArm = portArmEncodersOut.prepare();
    Bottle &bHead = portHeadEncodersOut.prepare();
    bArm.clear(); bHead.clear();

    bool okEncArm = encsArm->getEncoders(encodersArm.data());
    bool okEncHead = encsHead->getEncoders(encodersHead.data());
    if(!okEncArm || !okEncHead) {
        cerr << "Error receiving encoders";
    } else {
        for (unsigned int kk=0; kk<16; kk++){
            bArm.addDouble(encodersArm[kk]);
        }
        for (unsigned int kk=0; kk<6; kk++){
            bHead.addDouble(encodersHead[kk]);
        }
        portArmEncodersOut.write();
        portHeadEncodersOut.write();
    }

    return true;
}

bool SensoryProcessor::findFeatures(TermCriteria &termcrit, Size &subPixWinSize, Size &winSize)
{
    if (points[0].empty())
    {
        Mat copy;
        copy = image.clone();

        // automatic initialization
        goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.003, 3, Mat(), 3, 0, 0.04);
        cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
        /// Draw corners detected
        int radius = 4;
        for(unsigned int i = 0; i < points[1].size(); i++ ) {
            circle( copy, points[1][i], radius, Scalar(0,255,0), -1, 8);
        }

        for(size_t i=0; i<points[1].size(); i++)
        {
            points_idx.push_back(i);
        }

        //cout << "Number of points " << num_init_points;
    } else {
        Mat copy;
        copy = image.clone();

        vector<uchar> status;
        vector<float> err;
        if(prevGray.empty())
            gray.copyTo(prevGray);
        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                3, termcrit, 0, 0.001);

        size_t i, k;
        for( i = k = 0; i < points[1].size(); i++ )
        {
            if( !status[i] ) {
                continue;
            }

            points_idx[k] = points_idx[i];
            points[1][k++] = points[1][i];

            circle( copy, points[1][i], 2, Scalar(0,255,0), -1, 8);
        }
        points[1].resize(k);
        points_idx.resize(k);

        // convert Mat to YARP image
        Mat toYarp(copy);
        cvtColor(toYarp, toYarp, CV_BGR2RGB);
        IplImage* imageIpl = new IplImage(toYarp);
        ImageOf<PixelRgb> &imageYarp = featureImgPortOut.prepare();
        imageYarp.resize(imageIpl->width, imageIpl->height);
        cvCopyImage(imageIpl, (IplImage *)imageYarp.getIplImage());

        // send YARP image to port
        featureImgPortOut.write();
    }

    return true;
}

bool SensoryProcessor::init_iCub(string &part)
{
    /* Create PolyDriver for left arm */
    Property option;

    string portnameArm = part;//"left_arm";
    option.put("robot", robot.c_str());
    option.put("device", "remote_controlboard");
    Value& robotnameArm = option.find("robot");

    string sA("/");
    sA += robotnameArm.asString();
    sA += "/";
    sA += portnameArm.c_str();
    sA += "/control";
    option.put("local", sA.c_str());

    sA.clear();
    sA += "/";
    sA += robotnameArm.asString();
    sA += "/";
    sA += portnameArm.c_str();
    option.put("remote", sA.c_str());

    armDev = new yarp::dev::PolyDriver(option);
    if (!armDev->isValid()) {
        yError() << "Device not available.  Here are the known devices:";
        yError() << yarp::dev::Drivers::factory().toString();
        Network::fini();
        return false;
    }

    armDev->view(encsArm);

    if (encsArm==NULL){
        yError() << "Cannot get interface to robot device";
        armDev->close();
    }

    int nj = 0;
    encsArm->getAxes(&nj);
    encodersArm.resize(nj);

    yInfo() << "Wait for arm encoders";
    while (!encsArm->getEncoders(encodersArm.data()))
    {
        Time::delay(0.1);
        yInfo() << "Wait for arm encoders";
    }

    yInfo() << "Arm initialized.";

    /* Init. head */
    string portnameHead = "head";
    option.put("robot", robot.c_str()); //"icub"); // typically from the command line.
    option.put("device", "remote_controlboard");
    Value& robotnameHead = option.find("robot");

    string sH("/");
    sH += robotnameHead.asString();
    sH += "/";
    sH += portnameHead.c_str();
    sH += "/control";
    option.put("local", sH.c_str());

    sH.clear();
    sH += "/";
    sH += robotnameHead.asString();
    sH += "/";
    sH += portnameHead.c_str();
    option.put("remote", sH.c_str());

    headDev = new yarp::dev::PolyDriver(option);
    if (!headDev->isValid()) {
        yError() << "Device not available.  Here are the known devices:";
        yError() << yarp::dev::Drivers::factory().toString();
        Network::fini();
        return false;
    }

    headDev->view(encsHead);
    if (encsHead==NULL){
        yError() << "Cannot get interface to robot head";
        headDev->close();
    }
    int jnts = 0;

    encsHead->getAxes(&jnts);
    encodersHead.resize(jnts);

    yInfo() << "Wait for encoders (HEAD)";
    while (!encsHead->getEncoders(encodersHead.data()))
    {
        Time::delay(0.1);
        yInfo() << "Wait for head encoders";
    }

    yInfo() << "Head initialized.";
    yInfo() << "> Initialisation done.";

    return true;
}

void SensoryProcessor::find_image(yarp::sig::Vector &handTarget, yarp::sig::Vector &armTarget, yarp::sig::Vector &fingerTarget)
{
    handTarget.resize(3);
    armTarget.resize(3);
    fingerTarget.resize(3);

    ImageOf<PixelRgb> *image = imgPortIn.read(true);
    if (image!=NULL)
    {
        double xMeanR = 0;
        double yMeanR = 0;
        int ctR = 0;
        double xMeanG = 0;
        double yMeanG = 0;
        int ctG = 0;
        double xMeanB = 0;
        double yMeanB = 0;
        int ctB = 0;

        for (int x=0; x<image->width(); x++)
        {
            for (int y=0; y<image->height(); y++)
            {
                PixelRgb& pixel = image->pixel(x,y);
                if (pixel.r>pixel.b*1.2+10 && pixel.r>pixel.g*1.2+10)
                {
                    xMeanR += x;
                    yMeanR += y;
                    ctR++;
                }
                if (pixel.g>pixel.b*1.2+10 && pixel.g>pixel.r*1.2+10)
                {
                    xMeanG += x;
                    yMeanG += y;
                    ctG++;
                }
                if (pixel.b>pixel.g*1.2+10 && pixel.b>pixel.r*1.2+10)
                {
                    xMeanB += x;
                    yMeanB += y;
                    ctB++;
                }
            }
        }
        if (ctR>0)
        {
            xMeanR /= ctR;
            yMeanR /= ctR;
        }
        else
        {
            xMeanR = 320/2;
            yMeanR = 240/2;
        }
        if (ctG>0)
        {
            xMeanG /= ctG;
            yMeanG /= ctG;
        }
        else
        {
            xMeanG = 320/2;
            yMeanG = 240/2;
        }
        if (ctB>0)
        {
            xMeanB /= ctB;
            yMeanB /= ctB;
        }
        else
        {
            xMeanB = 320/2;
            yMeanB = 240/2;
        }

        if (ctG>(image->width()/20)*(image->height()/20))
        {
            handTarget.resize(3);
            handTarget[0] = xMeanG;
            handTarget[1] = yMeanG;
            handTarget[2] = 1;

            xMeanPrevG=xMeanG; yMeanPrevG=yMeanG;
        }
        else
        {
            handTarget.resize(3);
            handTarget[0] = xMeanPrevG;
            handTarget[1] = yMeanPrevG;
            handTarget[2] = 0;
        }
        if (ctR>(image->width()/20)*(image->height()/20))
        {
            fingerTarget.resize(3);
            fingerTarget[0] = xMeanR;
            fingerTarget[1] = yMeanR;
            fingerTarget[2] = 1;

            xMeanPrevR=xMeanR; yMeanPrevR=yMeanR;
        }
        else
        {
            fingerTarget.resize(3);
            fingerTarget[0] = xMeanPrevR;
            fingerTarget[1] = yMeanPrevR;
            fingerTarget[2] = 0;

        }
        if (ctB>(image->width()/20)*(image->height()/20))
        {
            armTarget.resize(3);
            armTarget[0] = xMeanB;
            armTarget[1] = yMeanB;
            armTarget[2] = 1;

            xMeanPrevB=xMeanB; yMeanPrevB=yMeanB;

        }
        else
        {
            armTarget.resize(3);
            armTarget[0] = xMeanPrevB;
            armTarget[1] = yMeanPrevB;
            armTarget[2] = 0;
        }
    }
    else
    {
        printf(">>>>>>> Image NULL...\n");
        handTarget.resize(3);
        handTarget[0] = 320/2;
        handTarget[1] = 240/2;
        handTarget[2] = 0;
        fingerTarget.resize(3);
        fingerTarget[0] = 320/2;
        fingerTarget[1] = 240/2;
        fingerTarget[2] = 0;
        armTarget.resize(3);
        armTarget[0] = 320/2;
        armTarget[1] = 240/2;
        armTarget[2] = 0;
    }
}

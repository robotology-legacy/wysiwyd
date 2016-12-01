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
 *
 *
 *
 *
 * Note: if run in iCub_SIM, launch:
 * - simCartesianControl
 * - iKinCartesianSolver --context simCartesianControl --part right_arm
 *
 *
 *
*/

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


#include <vector>
#include <iostream>
#include <thread>

#include "sensoryProcessor.h"

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
    useSFM = rf.check("useSFM",Value(0)).asInt();


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

    if (!portHandPositionFromFeaturesOut.open("/" + getName() + "/handfeaturepos:o")) {
        yError() << getName() << ": Unable to open port " << "/" << getName() << "/handfeaturepos:o";
        bEveryThingisGood = false;
    }

    if (!portHandPositionFromTrackOut.open("/" + getName() + "/handtrackingpos:o")) {
        yError() << getName() << ": Unable to open port " << "/" << getName() << "/handtrackingpos:o";
        bEveryThingisGood = false;
    }
    yInfo() << "\n\n\n";

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

    if (!portReadSkinHand.open("/" + getName() + "/portReadSkinHand:i")) {
        yError() << ": Unable to open port " << "/" << getName() << "portReadSkinHand:i";
        bEveryThingisGood = false;
    }

    if (!portReadSkinForearm.open("/" + getName() + "/portReadSkinForearm:i")) {
        yError() << ": Unable to open port " << "/" << getName() << "portReadSkinForearm:i";
        bEveryThingisGood = false;
    }

    if (!portReadSkinArm.open("/" + getName() + "/portReadSkinArm:i")) {
        yError() << ": Unable to open port " << "/" << getName() << "portReadSkinArm:i";
        bEveryThingisGood = false;
    }

    if (!portMidiOut.open("/" + getName() + "/portMidiOut:o")) {
        yError() << ": Unable to open port " << "/" << getName() << "portMidiOut:o";
        bEveryThingisGood = false;
    }

    if (!portCartesianCtrlOut.open("/" + getName() + "/portCartesianCtrlOut:o")) {
        yError() << ": Unable to open port " << "/" << getName() << "portCartesianCtrlOut:o";
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
        while(!Network::isConnected("/"+robot+"/skin/right_hand_comp", portReadSkinHand.getName())) {
            Network::connect("/"+robot+"/skin/right_hand_comp", portReadSkinHand.getName());
            yInfo() << "Waiting for port " << "/"+robot+"/skin/right_hand_comp" << " to connect to " << portReadSkinHand.getName();
            Time::delay(1.0);
        }
        while(!Network::isConnected("/"+robot+"/skin/right_forearm_comp", portReadSkinForearm.getName())) {
            Network::connect("/"+robot+"/skin/right_forearm_comp", portReadSkinForearm.getName());
            yInfo() << "Waiting for port " << "/"+robot+"/skin/right_forearm_comp" << " to connect to " << portReadSkinForearm.getName();
            Time::delay(1.0);
        }
        while(!Network::isConnected("/"+robot+"/skin/right_arm_comp", portReadSkinArm.getName())) {
            Network::connect("/"+robot+"/skin/right_arm_comp", portReadSkinArm.getName());
            yInfo() << "Waiting for port " << "/"+robot+"/skin/right_arm_comp" << " to connect to " << portReadSkinArm.getName();
            Time::delay(1.0);
        }
    }
    else
    {
        while(!Network::isConnected("/"+robot+"/skin/left_hand_comp", portReadSkinHand.getName())) {
            Network::connect("/"+robot+"/skin/left_hand_comp", portReadSkinHand.getName());
            yInfo() << "Waiting for port " << "/"+robot+"/skin/left_hand_comp" << " to connect to " << portReadSkinHand.getName();
            Time::delay(1.0);
        }
        while(!Network::isConnected("/"+robot+"/skin/left_forearm_comp", portReadSkinForearm.getName())) {
            Network::connect("/"+robot+"/skin/left_forearm_comp", portReadSkinForearm.getName());
            yInfo() << "Waiting for port " << "/"+robot+"/skin/right_forearm_comp" << " to connect to " << portReadSkinForearm.getName();
            Time::delay(1.0);
        }
        while(!Network::isConnected("/"+robot+"/skin/left_arm_comp", portReadSkinArm.getName())) {
            Network::connect("/"+robot+"/skin/left_arm_comp", portReadSkinArm.getName());
            yInfo() << "Waiting for port " << "/"+robot+"/skin/left_arm_comp" << " to connect to " << portReadSkinArm.getName();
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

    // Initialize iCub
    while (!init_iCub(part)) {
        yDebug() << getName() << ": initialising iCub... please wait... ";
        bEveryThingisGood = false;
    }


    yDebug() << "End configuration...";

    MAX_COUNT = 20;

    tracker = Tracker::create( "KCF" );

    ImageOf<PixelRgb> *yarpImage = imgPortIn.read();
    ImageOf<PixelBgr> tmp; tmp.resize(*yarpImage);

    Mat cvImage1=cvarrToMat((IplImage*)yarpImage->getIplImage());
    Mat cvImage2=cvarrToMat((IplImage*)tmp.getIplImage());

    cvtColor(cvImage1,cvImage2,CV_RGB2BGR);

    image = cvImage2;
    //roi=selectROI("tracker",image);
    roi = Rect(180,130,50,50); //this for piano
    roi = Rect(165,100,50,50); //this for ELIMB
    tracker->init(image,roi);

    //tracking

    tracker->update(image,roi);
    rectangle( image, roi, Scalar( 255, 0, 0 ), 2, 1 );
    imshow("tracker",image);
    waitKey(25);


    readMidi = thread(&SensoryProcessor::readMidiKeyboard, this);
    cout << "readMidi thread running..." << endl;

    //readMidiKeyboard();

//    midiin = new RtMidiIn();
//    midiin->openPort( 1 );

    attach(handlerPort);

    return bEveryThingisGood;
}

bool SensoryProcessor::interruptModule() {


    imgPortIn.interrupt();
    featureImgPortOut.interrupt();
    portToSFM.interrupt();
    handlerPort.interrupt();
    portHandPositionFromFeaturesOut.interrupt();
    portHandPositionFromTrackOut.interrupt();
    portHandPositionOut.interrupt();
    portHeadEncodersOut.interrupt();
    portArmEncodersOut.interrupt();
    portBodypartsPositionOut.interrupt();
    portSkinOut.interrupt();
    portReadSkinHand.interrupt();
    portReadSkinForearm.interrupt();
    portReadSkinArm.interrupt();
    portMidiOut.interrupt();
    portCartesianCtrlOut.interrupt();

    yInfo() << "Bye!";

    return true;
}

bool SensoryProcessor::close() {
    yInfo() << "Closing module, please wait ... ";

    readMidi.join();

    armDev->close();
    headDev->close();

    imgPortIn.interrupt();
    imgPortIn.close();

    featureImgPortOut.interrupt();
    featureImgPortOut.close();

    portHandPositionFromFeaturesOut.interrupt();
    portHandPositionFromFeaturesOut.close();

    portHandPositionFromTrackOut.interrupt();
    portHandPositionFromTrackOut.close();

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

    portToSFM.interrupt();
    portToSFM.close();

    portReadSkinHand.interrupt();
    portReadSkinHand.close();

    portReadSkinForearm.interrupt();
    portReadSkinForearm.close();

    portReadSkinArm.interrupt();
    portReadSkinArm.close();

    portMidiOut.interrupt();
    portMidiOut.close();

    portCartesianCtrlOut.interrupt();
    portCartesianCtrlOut.close();

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
    ImageOf<PixelBgr> tmp; tmp.resize(*yarpImage);

    Mat cvImage1=cvarrToMat((IplImage*)yarpImage->getIplImage());
    Mat cvImage2=cvarrToMat((IplImage*)tmp.getIplImage());

    cvtColor(cvImage1,cvImage2,CV_RGB2BGR);

    image = cvImage2;
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

    //tracking
    tracker->update(image,roi);

    double mean_y = roi.y+roi.height/2;
    double mean_x = roi.x+roi.width/2;

    Point2f mean_point(mean_x, mean_y);
    circle( image, mean_point, 5, Scalar(255,255,0), -1, 8);
    rectangle( image, roi, Scalar( 255, 0, 0 ), 2, 1 );
    imshow("tracker",image);
    waitKey(25);

    Bottle &bHandTrackedPositions = portHandPositionFromTrackOut.prepare();
    bHandTrackedPositions.clear();
    bHandTrackedPositions.addDouble(mean_point.x);
    bHandTrackedPositions.addDouble(mean_point.y);

    portHandPositionFromTrackOut.write();

    // get contact vector
    Bottle *skinContactHand;
    Bottle *skinContactForearm;
    Bottle *skinContactArm;
    skinContactHand = portReadSkinHand.read(true);
    skinContactForearm = portReadSkinForearm.read(true);
    skinContactArm = portReadSkinArm.read(true);

    Bottle &bSkin = portSkinOut.prepare();
    bSkin.clear();
    for (int i=0; i<skinContactHand->size(); i++)
        bSkin.addDouble(skinContactHand->get(i).asDouble());
    for (int i=0; i<skinContactForearm->size(); i++)
        bSkin.addDouble(skinContactForearm->get(i).asDouble());
    for (int i=0; i<skinContactArm->size(); i++)
        bSkin.addDouble(skinContactArm->get(i).asDouble());
    portSkinOut.write();



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



    Bottle &bCartCtrlArm = portCartesianCtrlOut.prepare();
    bCartCtrlArm.clear();
    yarp::sig::Vector x,o;
    bool okCartCtrlArm = icart->getPose(x,o);
    if(!okCartCtrlArm) {
        cerr << "Error receiving cartesian coordinates";
    } else {
        for (unsigned int kk=0; kk<3; kk++){
            bCartCtrlArm.addDouble(x[kk]);
        }
        portCartesianCtrlOut.write();
    }






//    cout << "MIDI" << endl;
//    Bottle &bMidiByte = portMidiOut.prepare();
//    bMidiByte.clear();

//    cout << "MIDI1" << endl;
//    std::vector<unsigned char> *message = NULL;
//    midiin->getMessage( message );
//    unsigned int nBytes = message->size();
//    for ( unsigned int i=0; i<nBytes; i++ )
//    {
//        std::cout << "Byte " << i << " = " << (int)message->at(i) << ", ";
//    }
//    if((int)message->at(0)==144)
//    {
//        cout << (int)message->at(1) << " "  << ", ";
//    }
//    if((int)message->at(0)==128)
//    {
//        cout << (int)message->at(1) << " " << ", ";
//    }
//    bMidiByte.addInt((int)message->at(0));
//    bMidiByte.addInt((int)message->at(1));
//    portMidiOut.write();
//    // Don't ignore sysex, timing, or active sensing messages.
//    midiin->ignoreTypes( false, false, false );


//    readMidi.join();



    return true;
}

bool SensoryProcessor::findFeatures(TermCriteria &termcrit, Size &subPixWinSize, Size &winSize)
{
    if (points[0].empty())
    {
        Mat copy;
        copy = image.clone();

        // automatic initialization
        //goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.0003, 5, Mat(), 3, 0, 0.0004);
        goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.0003, 0.001, Mat(), 2, 0, 0.004);
        cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
        /// Draw corners detected
        int radius = 2;
        for(unsigned int i = 0; i < points[1].size(); i++ ) {
            circle( copy, points[1][i], radius, Scalar(0,255,0), -1, 8);
        }
        cv::Point2f zero(0.0f, 0.0f);
        cv::Point2f sum  = std::accumulate(points[1].begin(), points[1].end(), zero);
        Point2f mean_point(sum.x / points[1].size(), sum.y / points[1].size());
        circle( copy, mean_point, 5, Scalar(255,255,0), -1, 8);

        namedWindow( "source_window", WINDOW_NORMAL );
        imshow( "source_window", copy );
        waitKey(25);

        for(size_t i=0; i<points[1].size(); i++)
        {
            points_idx.push_back(i);
        }



        Bottle &bHandFeatPositions = portHandPositionFromFeaturesOut.prepare();
        bHandFeatPositions.clear();
        bHandFeatPositions.addDouble(mean_point.x);
        bHandFeatPositions.addDouble(mean_point.y);

        portHandPositionFromFeaturesOut.write();

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

            //cout << points[1][i] << endl;
            //sum_x=sum_x+points[1][i];

            circle( copy, points[1][i], 2, Scalar(0,255,0), -1, 8);
        }
        cv::Point2f zero(0.0f, 0.0f);
        cv::Point2f sum  = std::accumulate(points[1].begin(), points[1].end(), zero);
        Point2f mean_point(sum.x / points[1].size(), sum.y / points[1].size());
        circle( copy, mean_point, 5, Scalar(255,255,0), -1, 8);

        Bottle &bHandFeatPositions = portHandPositionFromFeaturesOut.prepare();
        bHandFeatPositions.clear();
        bHandFeatPositions.addDouble(mean_point.x);
        bHandFeatPositions.addDouble(mean_point.y);

        portHandPositionFromFeaturesOut.write();

        points[1].resize(k);
        points_idx.resize(k);

        imshow( "source_window", copy );
        waitKey(25);

        // convert Mat to YARP image
        ImageOf<PixelRgb> &imageYarp=featureImgPortOut.prepare();
        imageYarp.resize(copy.cols,copy.rows);
        Mat toYarp=cvarrToMat((IplImage*)imageYarp.getIplImage());
        cvtColor(copy,toYarp,CV_BGR2RGB);

        // send YARP image to port
        featureImgPortOut.write();
    }


    return true;
}

void SensoryProcessor::midiCallback( double deltatime, std::vector< unsigned char > *message, void * ptr )
{

    yarp::os::BufferedPort<yarp::os::Bottle> *portMidiOut_local = (yarp::os::BufferedPort<yarp::os::Bottle> *) ptr;

    Bottle &bMidiByte = portMidiOut_local->prepare();
    bMidiByte.clear();

    unsigned int nBytes = message->size();
    for ( unsigned int i=0; i<nBytes; i++ )
    {
        std::cout << "Byte " << i << " = " << (int)message->at(i) << ", ";
    }
    if((int)message->at(0)==144)
    {
        cout << (int)message->at(1) << " "  << ", ";
    }
    if((int)message->at(0)==128)
    {
        cout << (int)message->at(1) << " " << ", ";
    }
    bMidiByte.addInt((int)message->at(0));
    bMidiByte.addInt((int)message->at(1));
    portMidiOut_local->write();

    if ( nBytes > 0 )
        std::cout << "stamp = " << deltatime << std::endl;


}


void SensoryProcessor::readMidiKeyboard()
{

    RtMidiIn *midiin = 0;
    try {

        // RtMidiIn constructor
        midiin = new RtMidiIn();
        midiin->openPort( 1 );
        midiin->setCallback( &SensoryProcessor::midiCallback, &portMidiOut );
        // Don't ignore sysex, timing, or active sensing messages.
        midiin->ignoreTypes( false, false, false );
      } catch ( RtMidiError &error ) {
        error.printMessage();
      }
}




bool SensoryProcessor::init_iCub(string &part)
{
    /* Create PolyDriver for left arm */
    Property option;

    string portnameArm = part;//"left_arm";
    option.put("robot", robot.c_str());
    option.put("device", "remote_controlboard");
    Value& robotnameArm = option.find("robot");

    string sA("/sensoryProcessor/");
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


    /* Cartesian control  */
//    option.put("robot", robot.c_str());
//    option.put("device", "remote_controlboard");

//    string sC("/sensoryProcessor/");
//    sC += "cartesian_client";
//    sC += "/";
//    sC += portnameArm.c_str();
//    option.put("local", sC.c_str());

//    sC.clear();
//    sC += "/";
//    sC += robotnameArm.asString();
//    sC += "/";
//    sC += "cartesianController";
//    sC += "/";
//    sC += portnameArm.c_str();
//    option.put("remote", sC.c_str());

    Property option2("(device cartesiancontrollerclient)");
    option2.put("remote","/icubSim/cartesianController/right_arm");
    option2.put("local","/cartesian_client/right_arm");
    icartClient = new yarp::dev::PolyDriver(option2);

    if (!icartClient->isValid()) {
        yError() << "Error with Cartesian interface";
        Time::delay(50);
        Network::fini();
        return false;
    }

    yDebug() << "***************\n";

    // open the view
    icartClient->view(icart);
    if (icart==NULL){
        yError() << "Cannot get interface to robot device (cartesian control problem)";
        icart->stopControl();
    }
    else
    {
        // print out some info about the controller
        Bottle info;
        icart->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());
    }



    /* Init. head */
    string portnameHead = "head";
    option.put("robot", robot.c_str()); //"icub"); // typically from the command line.
    option.put("device", "remote_controlboard");
    Value& robotnameHead = option.find("robot");

    string sH("/sensoryProcessor/");
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
                if (pixel.r>pixel.b*1.2+5 && pixel.r>pixel.g*1.2+5)
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
        if (ctR>(image->width()/50)*(image->height()/50))
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

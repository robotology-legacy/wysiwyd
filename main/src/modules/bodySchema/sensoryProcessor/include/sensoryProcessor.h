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

#ifndef _SENSORYPROCESSOR_H_
#define _SENSORYPROCESSOR_H_

#include <opencv2/opencv.hpp>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <thread>

#include <wrdac/clients/icubClient.h>
#include "RtMidi.h"

class SensoryProcessor : public yarp::os::RFModule {
private:
    std::string moduleName;

    yarp::os::Port handlerPort;

    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgPortIn;

    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelRgb> > featureImgPortOut;
    yarp::os::BufferedPort<yarp::os::Bottle> portHandPositionOut;
    yarp::os::BufferedPort<yarp::os::Bottle> portHeadEncodersOut;
    yarp::os::BufferedPort<yarp::os::Bottle> portArmEncodersOut;
    yarp::os::BufferedPort<yarp::os::Bottle> portSkinOut;
    yarp::os::BufferedPort<yarp::os::Bottle> portBodypartsPositionOut;
    yarp::os::BufferedPort<yarp::os::Bottle> portReadSkinHand;
    yarp::os::BufferedPort<yarp::os::Bottle> portReadSkinForearm;
    yarp::os::BufferedPort<yarp::os::Bottle> portReadSkinArm;
    yarp::os::BufferedPort<yarp::os::Bottle> portHandPositionFromFeaturesOut;
    yarp::os::BufferedPort<yarp::os::Bottle> portMidiOut;

    yarp::os::RpcClient portToSFM;

    yarp::dev::IEncoders* encsArm;
    yarp::dev::IEncoders* encsHead;

    yarp::dev::PolyDriver* armDev;
    yarp::dev::PolyDriver* headDev;

    yarp::sig::Vector encodersArm;
    yarp::sig::Vector encodersHead;

    std::string leftCameraPort, rightCameraPort;

    RtMidiIn *midiin;

    int MAX_COUNT;
    int fps;

    std::string part;
    std::string robot;

    bool useSFM;

    double xMeanPrevR, yMeanPrevR, xMeanPrevG, yMeanPrevG, xMeanPrevB, yMeanPrevB;

    std::vector<int> points_idx;
    cv::Mat gray, prevGray, image;
    std::vector<cv::Point2f> points[2];

    std::thread readMidi;


public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();

private:


    bool init_iCub(std::string &part);
    bool getMultimodalData();
    bool findFeatures(cv::TermCriteria &termcrit, cv::Size &subPixWinSize, cv::Size &winSize);
    void readMidiKeyboard();
    void midiCallback( double deltatime, std::vector< unsigned char > *message, void */*userData*/ );
    void find_image(yarp::sig::Vector &handTarget, yarp::sig::Vector &armTarget, yarp::sig::Vector &fingerTarget);
};

#endif // __BODYSCHEMA_H__

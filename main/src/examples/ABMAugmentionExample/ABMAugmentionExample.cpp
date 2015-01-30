/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
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

#include <stdio.h>

#include <cv.h>
#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "ABMAugmentionExample.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cv;

bool ABMAugmentionExample::configure(yarp::os::ResourceFinder &rf) {
    setName(rf.check("name", Value("ABMAugmentionExample"), "module name (string)").asString().c_str());

    // connect to ABM
    string abmName = rf.check("abm",Value("autobiographicalMemory")).asString().c_str();
    string abmLocal = "/"+getName()+"/abm:o";
    abm.open(abmLocal.c_str());
    string abmRemote = "/"+abmName+"/rpc";

    while (!Network::connect(abmLocal.c_str(),abmRemote.c_str())) {
        cout << "Waiting for connection to ABM..." << endl;
        Time::delay(1.0);
    }

    // attach to rpc port
    string handlerPortName = "/" + getName() + "/rpc";

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    attach(handlerPort);

    return true;
}

bool ABMAugmentionExample::respond(const Bottle& bCommand, Bottle& bReply) {
    if (bCommand.get(0).asString() == "augmentImages" )
    {
        if(bCommand.size() == 2 && bCommand.get(1).isInt()) {
            int instance = (atoi((bCommand.get(1)).toString().c_str()));
            if(augmentImages(instance)) {
                bReply.addString("[augmentImages]: Successful");
            }
        } else {
            bReply.addString("[augmentImages]: wrong function signature: augmentImages instance");
        }
        bReply.addString("ack");
    }
    else
    {
        bReply.addString("nack");
    }
    return true;
}

bool ABMAugmentionExample::augmentImages(int instance) {
    // find out how many images are saved for a specific instance
    Bottle bCmdImagesInfo, bRespImagesInfo;
    bCmdImagesInfo.addString("getImagesInfo");
    bCmdImagesInfo.addInt(instance);
    bCmdImagesInfo.addInt(0); // do not include already augmented images
    abm.write(bCmdImagesInfo, bRespImagesInfo);

    int numberOfImages = atoi(bRespImagesInfo.get(0).toString().c_str());
    //cout << "Num images: " << numberOfImages << endl;

    if(numberOfImages>0) {
        // now, get images frame by frame
        for(int frame_number=0; frame_number<numberOfImages; frame_number++) {
            Bottle bCmdGetFrame;
            Bottle bRespGetFrame;

            bCmdGetFrame.addString("provideImagesByFrame");
            bCmdGetFrame.addInt(instance);
            bCmdGetFrame.addInt(frame_number);
            bCmdGetFrame.addInt(0); // do not include already augmented images

            // bRespGetFrame contains the images + meta information
            abm.write(bCmdGetFrame, bRespGetFrame);

            // in augmentFrame(...), we do the actual processing
            Bottle bCmdSaveAugmentedImages = augmentFrame(bRespGetFrame);
            Bottle bRespSaveAugmentedImages;

            // now, call the saveAugmentedImages function of the ABM
            abm.write(bCmdSaveAugmentedImages, bRespSaveAugmentedImages);
            cout << bRespSaveAugmentedImages.toString() << endl;
        }
        return true;
    } else {
        return false;
    }
}

Bottle ABMAugmentionExample::augmentFrame(Bottle bInput) {
    Bottle toSend;
    toSend.addString("saveAugmentedImages"); // method name to call in ABM

    // bInput: ack ( ((imageMeta1.1) (image1.1)) ((imageMeta1.2) (image1.2)) ((imageMeta1.3) (image1.3)) )
    if(bInput.get(0).asString()!="ack") {
        Bottle bError;
        bError.addString("no ack in input bottle");
        return bError;
    }

    Bottle* bImagesWithMeta = bInput.get(1).asList(); // ((imageMeta1.1) (image1.1)) ((imageMeta1.2) (image1.2)) ((imageMeta1.3) (image1.3))

    //go through each of the sub-bottles, one for each imageProvider
    for(int i = 0; i < bImagesWithMeta->size(); i++) {
        Bottle* bSingleImageWithMeta = bImagesWithMeta->get(i).asList(); // ((imageMeta1.1) (image1.1))

        Bottle* bImageMeta = bSingleImageWithMeta->get(0).asList(); // (imageMeta1.1)
        Bottle* bRawImage = bSingleImageWithMeta->get(1).asList(); //image1.1

        // get yarp image from bottle
        ImageOf<PixelRgb> rawImageYarp;
        yarp::os::Portable::copyPortable(*bRawImage, rawImageYarp);

        // get cv::Mat from yarp image
        IplImage *rawImageIpl = cvCreateImage(cvSize(rawImageYarp.width(), rawImageYarp.height()), IPL_DEPTH_8U, 3);
        cvCvtColor((IplImage*)rawImageYarp.getIplImage(), rawImageIpl, CV_RGB2BGR);
        Mat myImage(rawImageIpl);

        // you do your stuff here
        // alternatively, you can just save the image here
        // and do batch processing after
        // however, make sure you keep bImageMeta somewhere around!
        string augmentedLabel = "Canny"; // identifier what we have done with the image

        // here, do Canny edge detection as example
        Mat myImage_gray;
        cvtColor( myImage, myImage_gray, CV_BGR2GRAY );
        int lowThreshold = 20;
        int ratio = 3;
        int kernel_size = 3;
        Canny( myImage_gray, myImage_gray, lowThreshold, lowThreshold*ratio, kernel_size );
        Mat augmented;
        augmented.create( myImage.size(), myImage.type() );
        augmented = Scalar::all(0);
        myImage.copyTo( augmented, myImage_gray);

        //imshow("Canny", augmented);
        //waitKey(0);

        // convert back to IplImage
        IplImage augmentedImageIpl = augmented;
        // end your stuff
        // from IplImage to yarp image
        cvCvtColor(&augmentedImageIpl, &augmentedImageIpl, CV_BGR2RGB);
        ImageOf<PixelRgb> augmentedImageYarp;
        augmentedImageYarp.resize(augmentedImageIpl.width, augmentedImageIpl.height);
        cvCopyImage(&augmentedImageIpl, (IplImage *)augmentedImageYarp.getIplImage());

        cvReleaseImage(&rawImageIpl);

        // from yarp image to bottle
        Bottle bAugmentedImage;
        yarp::os::Portable::copyPortable(augmentedImageYarp, bAugmentedImage);

        // add the augmented image with the meta information and the augmented label
        Bottle bAugmentedImageWithMeta;
        bAugmentedImageWithMeta.addList() = *bImageMeta;
        bAugmentedImageWithMeta.addString(augmentedLabel);
        bAugmentedImageWithMeta.addList() = bAugmentedImage;

        toSend.addList() = bAugmentedImageWithMeta;
   }

    return toSend;
}

double ABMAugmentionExample::getPeriod() {
    return 0.1;
}


bool ABMAugmentionExample::updateModule() {
    return true;
}

bool ABMAugmentionExample::interruptModule() {
    abm.interrupt();
    handlerPort.interrupt();

    return true;
}

bool ABMAugmentionExample::close() {
    abm.interrupt();
    abm.close();
    handlerPort.interrupt();
    handlerPort.close();

    return true;
}

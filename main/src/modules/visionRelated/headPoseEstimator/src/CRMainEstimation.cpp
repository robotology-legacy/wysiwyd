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

#include <opencv2/opencv.hpp>
#include "CRMainEstimation.h"

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

bool CRMainEstimation::configure(yarp::os::ResourceFinder &rf) {
    setName(rf.check("name", Value("headPoseEstimator"), "module name (string)").asString().c_str());

    g_ntrees = rf.check("g_ntrees",Value(10)).asInt();
    g_maxv = rf.check("g_maxv",Value(800)).asInt();
    g_larger_radius_ratio = rf.check("g_larger_radius_ratio",Value(1.6)).asDouble();
    g_smaller_radius_ratio = rf.check("g_smaller_radius_ratio",Value(5.0)).asDouble();
    g_stride = rf.check("g_stride",Value(5)).asInt();
    g_max_z = rf.check("g_max_z",Value(1300)).asInt();
    g_th = rf.check("g_th",Value(500)).asInt();
    string carrier=rf.check("carrier",Value("udp")).asString().c_str();
    string remote=rf.check("remote",Value("kinectServer")).asString().c_str();

    g_prob_th = 1.0f;
    g_Estimate = new CRForestEstimator();
    buf=new unsigned short[320*240];

    string fullpath = rf.findFileByName("tree000.bin");
    string relativepath = fullpath.substr(0, fullpath.find("000.bin"));

    if( !g_Estimate->loadForest(relativepath.c_str(), g_ntrees) ){
        cerr << "Could not read forest!" << endl;
        return false;
    }

    cout << endl << "------------------------------------" << endl << endl;
    cout << "Estimation:       " << endl;
    cout << "Trees:            " << g_ntrees << " " << relativepath << endl;
    cout << "Stride:           " << g_stride << endl;
    cout << "Max Variance:     " << g_maxv << endl;
    cout << "Max Distance:     " << g_max_z << endl;
    cout << "Head Threshold:   " << g_th << endl;

    cout << endl << "------------------------------------" << endl << endl;

    depthPort.open(("/"+ getName() + "/depth:i").c_str());
    while(!Network::connect(("/"+remote+"/depth:o").c_str(),depthPort.getName().c_str(),carrier.c_str())) {
        cout << "Waiting for connection to kinectServer" << endl;
        Time::delay(1.0);
    }

    if (!handlerPort.open(("/" + getName() + "/rpc").c_str())) {
        cout << getName() << ": Unable to open port " << handlerPort.getName() << endl;
        return false;
    }

    attach(handlerPort);

    return true;
}

bool CRMainEstimation::respond(const Bottle& command, Bottle& reply) {
    reply.clear();

    if (command.get(0).asString()=="getPoses") {
        reply.addString("[ack]");
        reply.addInt(g_means.size());
        for(unsigned int i=0; i<g_means.size(); i++) {
            for(unsigned int j=0; j<6; j++) { // 6DOF
                reply.addDouble(g_means[i][j]);
            }
        }
    }
    else {
        reply.addString("[nack]");
    }

    return true;
}

double CRMainEstimation::getPeriod() {
    return 0.1;
}

bool CRMainEstimation::updateModule() {
    estimate();

    return true;
}

void CRMainEstimation::estimate() {
    // get depth image
    int depth_width=320;
    int depth_height=240;
    ImageOf<PixelMono16>* img;
    IplImage* depthCV;
    depthCV=cvCreateImageHeader(cvSize(depth_width,depth_height),IPL_DEPTH_16U,1);

    if ((img=(ImageOf<PixelMono16>*)depthPort.read(false)))
    {
        unsigned short* pBuff=(unsigned short*)img->getRawImage();
        for (int i=0; i<img->width()*img->height(); i++)
        {
            //We take only the first 13 bits, that contain the depth value in mm
            unsigned short realDepth = (pBuff[i]&0xFFF8)>>3;
            buf[i]=realDepth;
        }
        cvSetData(depthCV,buf,depth_width*2);
    }
    Mat depth(depthCV, true);
    if(depth.cols != depth_width || depth.rows != depth_height) {
        cerr << "Wrong image format : " << depth.cols << " x " << depth.rows << endl;
        return;
    }

    bool isInMM = depth.type() == CV_16UC1;

    float f = 575.0;
    //float f = client.getFocalLength(_depthFocal);
    if(depth.cols == 320) { // upscale to 640x480
        cv::resize(depth, depth, Size(), 2.0, 2.0, INTER_NEAREST);
    }

    float cx = float(depth.cols/2) - 0.5f;
    float cy = float(depth.rows/2) - 0.5f;

    //input 3D image
    Mat g_im3D;
    g_im3D.create(depth.rows, depth.cols, CV_32FC3);

    //generate 3D image
    int valid_pixels = 0;

    for(int y = 0; y < g_im3D.rows; y++) {
        Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
        for(int x = 0; x < g_im3D.cols; x++) {
            float d = isInMM?(float)depth.at<uint16_t>(y,x):depth.at<float>(y,x)*1000.0;

            if ( d < g_max_z && d > 0 ) {
                valid_pixels++;

                Mi[x][0] = ( float(d * (x - cx)) / f );
                Mi[x][1] = ( float(d * (y - cy)) / f );
                Mi[x][2] = d;
            } else {
                Mi[x] = 0;
            }
        }
    }

    g_means.clear();
    g_votes.clear();
    g_clusters.clear();

    //do the actual estimation
    g_Estimate->estimate( 	g_im3D,
                            g_means,
                            g_clusters,
                            g_votes,
                            g_stride,
                            g_maxv,
                            g_prob_th,
                            g_larger_radius_ratio,
                            g_smaller_radius_ratio,
                            false,
                            g_th
                            );

    cout << "Heads found : " << g_means.size() << endl;
    cout << "Valid pixels: " << valid_pixels << endl;

    //assuming there's only one head in the image!
    if(g_means.size()>0) {
        cout << "Estimated: " << g_means[0][0] << " " << g_means[0][1] << " " << g_means[0][2] << " " << g_means[0][3] << " " << g_means[0][4] << " " << g_means[0][5] <<endl;
    }
}

bool CRMainEstimation::interruptModule() {
    cout << "Interrupting your module, for port cleanup" << endl;
    depthPort.interrupt();
    handlerPort.interrupt();
    return true;
}

bool CRMainEstimation::close() {
    cout << "Calling close function" << endl;
    depthPort.interrupt();
    depthPort.close();
    handlerPort.interrupt();
    handlerPort.close();

    delete g_Estimate;
    delete[] buf;

    return true;
}

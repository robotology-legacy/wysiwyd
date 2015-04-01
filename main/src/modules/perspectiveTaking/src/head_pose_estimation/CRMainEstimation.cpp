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
#include <yarp/os/all.h>
#include "PerspectiveTaking.h"
#include "head_pose_estimation/CRMainEstimation.h"

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

CRMainEstimation::CRMainEstimation(yarp::os::ResourceFinder &rf):
    g_max_z(0),
    g_th(400),
    g_prob_th(1.0f),
    g_maxv(1000.0f),
    g_stride(5),
    g_larger_radius_ratio(1.0f),
    g_smaller_radius_ratio(6.0f),
    g_Estimate(new CRForestEstimator())
{
    Bottle &bHeadPoseProperties = rf.findGroup("head_pose_estimator");
    g_ntrees = bHeadPoseProperties.check("g_ntrees",Value(10)).asInt();
    g_maxv = bHeadPoseProperties.check("g_maxv",Value(800)).asInt();
    g_larger_radius_ratio = bHeadPoseProperties.check("g_larger_radius_ratio",Value(1.6)).asDouble();
    g_smaller_radius_ratio = bHeadPoseProperties.check("g_smaller_radius_ratio",Value(5.0)).asDouble();
    g_stride = bHeadPoseProperties.check("g_stride",Value(5)).asInt();
    g_max_z = bHeadPoseProperties.check("g_max_z",Value(1300)).asInt();
    g_th = bHeadPoseProperties.check("g_th",Value(500)).asInt();

    string fullpath = rf.findFileByName("tree000.bin");
    string relativepath = fullpath.substr(0, fullpath.find("000.bin"));

    if( !g_Estimate->loadForest(relativepath.c_str(), g_ntrees) ){
        cerr << "Could not read forest!" << endl;
        std::exit(-1);
    }

    cout << endl << "------------------------------------" << endl << endl;
    cout << "Estimation:       " << endl;
    cout << "Trees:            " << g_ntrees << " " << relativepath << endl;
    cout << "Stride:           " << g_stride << endl;
    cout << "Max Variance:     " << g_maxv << endl;
    cout << "Max Distance:     " << g_max_z << endl;
    cout << "Head Threshold:   " << g_th << endl;

    cout << endl << "------------------------------------" << endl << endl;
}

CRMainEstimation::~CRMainEstimation() {
    delete g_Estimate;
}

void CRMainEstimation::estimate() {
    if(head_pose_mutex.try_lock()) {
        // get depth image
        Mat depth = perspectiveTaking::lastDepth;

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

        head_pose_mutex.unlock();
    }
}

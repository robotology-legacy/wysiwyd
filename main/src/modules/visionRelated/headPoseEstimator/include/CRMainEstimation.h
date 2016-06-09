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

#ifndef VPT_CRMAINESTIMATION
#define VPT_CRMAINESTIMATION

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "CRForestEstimator.h"

class CRMainEstimation : public yarp::os::RFModule {
public:
    //outputs
    std::vector< cv::Vec<float,POSE_SIZE> > g_means;
    std::vector< cv::Vec<float,POSE_SIZE> > g_means_last;

    bool configure(yarp::os::ResourceFinder &rf);
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
    bool interruptModule();
    bool updateModule();
    bool close();
    double getPeriod();

private:
    yarp::os::Port handlerPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > depthPort;
    unsigned short* buf;

    //full clusters of votes
    std::vector< std::vector< Vote > > g_clusters;
    //all votes returned by the forest
    std::vector< Vote > g_votes;

    // Number of trees
    int g_ntrees;
    // Patch width
    //int g_p_width;
    // Patch height
    //int g_p_height;
    //maximum distance form the sensor - used to segment the person
    int g_max_z;
    //head threshold - to classify a cluster of votes as a head
    int g_th;
    //threshold for the probability of a patch to belong to a head
    float g_prob_th;
    //threshold on the variance of the leaves
    float g_maxv;
    //stride (how densely to sample test patches - increase for higher speed)
    int g_stride;
    //radius used for clustering votes into possible heads
    float g_larger_radius_ratio;
    //radius used for mean shift
    float g_smaller_radius_ratio;

    void estimate();

protected:
    //pointer to the actual estimator
    CRForestEstimator* g_Estimate;
};

#endif

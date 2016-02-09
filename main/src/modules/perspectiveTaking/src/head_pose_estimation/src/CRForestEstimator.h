/*
// Authors: Gabriele Fanelli, Thibaut Weise, Juergen Gall, BIWI, ETH Zurich
// Email: fanelli@vision.ee.ethz.ch

// You may use, copy, reproduce, and distribute this Software for any
// non-commercial purpose, subject to the restrictions of the
// Microsoft Research Shared Source license agreement ("MSR-SSLA").
// Some purposes which can be non-commercial are teaching, academic
// research, public demonstrations and personal experimentation. You
// may also distribute this Software with books or other teaching
// materials, or publish the Software on websites, that are intended
// to teach the use of the Software for academic or other
// non-commercial purposes.
// You may not use or distribute this Software or any derivative works
// in any form for commercial purposes. Examples of commercial
// purposes would be running business operations, licensing, leasing,
// or selling the Software, distributing the Software for use with
// commercial products, using the Software in the creation or use of
// commercial products or any other activity which purpose is to
// procure a commercial gain to you or others.
// If the Software includes source code or data, you may create
// derivative works of such portions of the Software and distribute
// the modified Software for non-commercial purposes, as provided
// herein.

// THE SOFTWARE COMES "AS IS", WITH NO WARRANTIES. THIS MEANS NO
// EXPRESS, IMPLIED OR STATUTORY WARRANTY, INCLUDING WITHOUT
// LIMITATION, WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A
// PARTICULAR PURPOSE, ANY WARRANTY AGAINST INTERFERENCE WITH YOUR
// ENJOYMENT OF THE SOFTWARE OR ANY WARRANTY OF TITLE OR
// NON-INFRINGEMENT. THERE IS NO WARRANTY THAT THIS SOFTWARE WILL
// FULFILL ANY OF YOUR PARTICULAR PURPOSES OR NEEDS. ALSO, YOU MUST
// PASS THIS DISCLAIMER ON WHENEVER YOU DISTRIBUTE THE SOFTWARE OR
// DERIVATIVE WORKS.

// NEITHER MICROSOFT NOR ANY CONTRIBUTOR TO THE SOFTWARE WILL BE
// LIABLE FOR ANY DAMAGES RELATED TO THE SOFTWARE OR THIS MSR-SSLA,
// INCLUDING DIRECT, INDIRECT, SPECIAL, CONSEQUENTIAL OR INCIDENTAL
// DAMAGES, TO THE MAXIMUM EXTENT THE LAW PERMITS, NO MATTER WHAT
// LEGAL THEORY IT IS BASED ON. ALSO, YOU MUST PASS THIS LIMITATION OF
// LIABILITY ON WHENEVER YOU DISTRIBUTE THE SOFTWARE OR DERIVATIVE
// WORKS.

// When using this software, please acknowledge the effort that
// went into development by referencing the paper:
//
// Fanelli G., Weise T., Gall J., Van Gool L.,
// Real Time Head Pose Estimation from Consumer Depth Cameras
// 33rd Annual Symposium of the German Association for Pattern Recognition
// (DAGM'11), 2011

*/


#ifndef CRFORSTESTIMATOR_H
#define CRFORSTESTIMATOR_H

#include "CRForest.h"
#include <fstream>
#include <opencv2/core/core.hpp>

struct Vote {

    cv::Vec<float,POSE_SIZE> vote;
    const float* trace;
    const float* conf;
    bool operator<(const Vote& a) const { return trace<a.trace; }

};

class CRForestEstimator {

public:

    CRForestEstimator(){ crForest = 0; };

    ~CRForestEstimator(){ if(crForest) delete crForest; };

    bool loadForest(const char* treespath, int ntrees = 0);

    void estimate( const cv::Mat & im3D, //input: 3d image (x,y,z coordinates for each pixel)
                   std::vector< cv::Vec<float,POSE_SIZE> >& means, //output: heads' centers and orientations (x,y,z,pitch,yaw,roll)
                   std::vector< std::vector< Vote > >& clusters, //all clusters
                   std::vector< Vote >& votes, //all votes
                   int stride = 5, //stride
                   float max_variance = 1000, //max leaf variance
                   float prob_th = 1.0, //threshold on the leaf's probability of belonging to a head
                   float larger_radius_ratio = 1.0, //for clustering heads
                   float smaller_radius_ratio = 6.0, //for mean shift
                   bool verbose = false, //print out more info
                   int threshold = 400 //head threshold
            );


private:

    cv::Rect getBoundingBox(const cv::Mat& im3D);

    CRForest* crForest;

};


#endif // CRFORSTESTIMATOR_H

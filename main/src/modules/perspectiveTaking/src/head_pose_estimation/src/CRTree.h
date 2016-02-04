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


#ifndef CRTREE_H
#define CRTREE_H

#include <opencv2/core/core.hpp>

const int TEST_DIM = 11;
const int POSE_SIZE = 6;
const float AVG_FACE_DIAMETER = 236.4f;
const float AVG_FACE_DIAMETER2 = 55884.96f;

// Structure for the leafs
class LeafNode {

public:

    LeafNode() { }
    ~LeafNode(){ }

    // Probability of belonging to a head
    float pfg;
    // mean vector
    cv::Mat mean;
    // trace of the covariance matrix
    float trace;

};

class CRTree {

public:

    CRTree() {};

    ~CRTree() { delete [] leaf; delete[] treetable; }

    bool loadTree(const char* filename);

    int getDepth() const {return max_depth;}
    int getPatchWidth() const {return m_pwidth;}
    int getPatchHeight() const {return m_pheight;}
    int getNoChannels() const {return m_no_chans;}

    const LeafNode* regressionIntegral(const std::vector< cv::Mat >&, const cv::Mat& nonZeros, const cv::Rect& roi);


private:

    int m_pwidth, m_pheight, m_no_chans;

    // Data structure
    // tree table
    int* treetable;

    // number of nodes: 2^(max_depth+1)-1
    int num_nodes;

    // number of leafs
    int num_leaf;

    int max_depth;

    //leafs as vector
    LeafNode* leaf;

};

inline const LeafNode* CRTree::regressionIntegral(const std::vector< cv::Mat >& patch, const cv::Mat& nonZeros, const cv::Rect& roi) {

    // pointer to current node
    const int* pnode = &treetable[0];
    int node = 0;

    // Go through tree until one arrives at a leaf, i.e. pnode[0]>=0)
    while(pnode[0]==-1) {

        const cv::Mat ptC = patch[ pnode[9] ];

        int xa1 = roi.x + pnode[1];		int xa2 = xa1 + pnode[5];
        int ya1 = roi.y + pnode[2];		int ya2 = ya1 + pnode[6];
        int xb1 = roi.x + pnode[3];		int xb2 = xb1 + pnode[7];
        int yb1 = roi.y + pnode[4];		int yb2 = yb1 + pnode[8];

        double mz1 = ( ptC.at<double>(ya1,xa1) +
                       ptC.at<double>(ya2,xa2) -
                       ptC.at<double>(ya2,xa1) -
                       ptC.at<double>(ya1,xa2) )/
                (double)MAX(1, nonZeros.at<double>(ya1,xa1) +
                            nonZeros.at<double>(ya2,xa2) -
                            nonZeros.at<double>(ya2,xa1) -
                            nonZeros.at<double>(ya1,xa2));

        double mz2 = ( ptC.at<double>(yb1,xb1) +
                       ptC.at<double>(yb2,xb2) -
                       ptC.at<double>(yb2,xb1) -
                       ptC.at<double>(yb1,xb2) )/
                (double)MAX(1, nonZeros.at<double>(yb1,xb1) +
                            nonZeros.at<double>(yb2,xb2) -
                            nonZeros.at<double>(yb2,xb1) -
                            nonZeros.at<double>(yb1,xb2));

        //check test
        int test = ( (mz1 - mz2) >= (double)pnode[10] );

        //the test result sends the patch to one of the children nodes
        int incr = node+1+test;
        node += incr;
        pnode += incr*TEST_DIM;

    }

    return &leaf[pnode[0]];

}

#endif //CRTREE_H

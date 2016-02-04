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


#ifndef CRFOREST_H
#define CRFOREST_H

#include "CRTree.h"
#include <vector>
#include <iomanip>

class CRForest {
public:
    // Constructor
    CRForest(int trees = 0) {
        vTrees.resize(trees);
    }

    // Destructor
    ~CRForest() {
        for(std::vector<CRTree*>::iterator it = vTrees.begin(); it != vTrees.end(); ++it)
            delete *it; // delete pointers
        vTrees.clear(); // specialized routine for clearing trees
    }

    // Set/Get functions
    int getSize() const {return vTrees.size();} // get size of trees
    int getDepth() const {return vTrees[0]->getDepth();}
    int getPatchWidth(){ return vTrees[0]->getPatchWidth(); }
    int getPatchHeight(){ return vTrees[0]->getPatchHeight(); }
    int getNoChans(){ return vTrees[0]->getNoChannels(); }

    std::vector< const LeafNode* > regressionIntegral( const std::vector< cv::Mat >& patch, const cv::Mat& nonZeros, const cv::Rect& roi ) const;

    bool loadForest(const char* filename);

    // Trees
    std::vector<CRTree*> vTrees;


};

// Regression
inline std::vector<const LeafNode*> CRForest::regressionIntegral( const std::vector< cv::Mat >& patch, const cv::Mat& nonZeros, const cv::Rect& roi ) const {

    std::vector<const LeafNode*> res;
    for(int i=0; i<(int)vTrees.size(); ++i)
        res.push_back( vTrees[i]->regressionIntegral(patch,nonZeros,roi) );
    return res;
}


inline bool CRForest::loadForest(const char* filename) {

    bool success = true;
    for(unsigned int i=0; i<vTrees.size(); ++i) {
        std::stringstream buffer;
        buffer << filename  << std::setfill('0') << std::setw(3) << i << ".bin";
        vTrees[i] = new CRTree();
        success &= vTrees[i]->loadTree(buffer.str().c_str());
    }
    return success;
}


#endif // CRFOREST_H

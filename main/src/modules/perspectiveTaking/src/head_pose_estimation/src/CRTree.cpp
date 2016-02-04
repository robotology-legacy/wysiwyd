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


#include <iostream>
#include <fstream>
#include <vector>
#include "CRTree.h"

using namespace std;

bool CRTree::loadTree(const char* filename) {

    cout << "Load Tree (BIN) " << filename << " " << flush;
    int dummy;
    bool success = true;

    FILE* fp = fopen(filename,"rb");

    if(!fp){
        cout << "failed" << endl;
        return false;
    }

    success &= ( fread( &max_depth, sizeof(int), 1, fp) == 1);
    success &= ( fread( &num_leaf, 	sizeof(int), 1, fp) == 1);
    success &= ( fread( &m_pwidth,  sizeof(int), 1, fp) == 1);
    success &= ( fread( &m_pheight, sizeof(int), 1, fp) == 1);
    success &= ( fread( &m_no_chans,sizeof(int), 1, fp) == 1);

    num_nodes = (int)pow(2.0,int(max_depth+1))-1;   // compute number of existing nodes

    treetable = new int[num_nodes * TEST_DIM];   // num_nodes x test size: [index, x1,y1,x2,y,2,w1,h1,w2,h2,channel,threshold]
    int* ptT = &treetable[0];                    // get pointer to the tree table

    // get number of leaves from text file
    leaf = new LeafNode[num_leaf];

    // read tree nodes
    for(unsigned int n=0; n<num_nodes; ++n) {

        success &= ( fread( &dummy,   sizeof(int), 1, fp) == 1);
        success &= ( fread( &dummy,   sizeof(int), 1, fp) == 1);

        //read in the test parameters
        for(unsigned int i=0; i<TEST_DIM; ++i, ++ptT){
            success &= ( fread( ptT, sizeof(int), 1, fp) == 1);
        }

    }

    // read tree leafs
    LeafNode* ptLN = &leaf[0];
    for(unsigned int l=0; l<num_leaf; ++l, ++ptLN) {

        ptLN->mean.create(POSE_SIZE, 1, CV_32FC1); ptLN->mean.setTo(0);

        success &= ( fread( &dummy,			sizeof(int), 1, fp) == 1);
        success &= ( fread( &(ptLN->pfg),    sizeof(float), 1, fp) == 1);
        success &= ( fread( ptLN->mean.data, sizeof(float), POSE_SIZE, fp) == POSE_SIZE );
        success &= ( fread( &(ptLN->trace),    sizeof(float), 1, fp) == 1);

    }

    fclose(fp);
    std::cout << " done " << endl;

    return success;

}


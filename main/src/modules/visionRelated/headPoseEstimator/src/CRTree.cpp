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
    success &= ( fread( &num_leaf,  sizeof(int), 1, fp) == 1);
    success &= ( fread( &m_pwidth,  sizeof(int), 1, fp) == 1);
    success &= ( fread( &m_pheight, sizeof(int), 1, fp) == 1);
    success &= ( fread( &m_no_chans,sizeof(int), 1, fp) == 1);

    num_nodes = (int)pow(2.0,int(max_depth+1))-1;   // compute number of existing nodes

    treetable = new int[num_nodes * TEST_DIM];   // num_nodes x test size: [index, x1,y1,x2,y,2,w1,h1,w2,h2,channel,threshold]
    int* ptT = &treetable[0];                    // get pointer to the tree table

    // get number of leaves from text file
    leaf = new LeafNode[num_leaf];

    // read tree nodes
    for(int n=0; n<num_nodes; ++n) {

        success &= ( fread( &dummy,   sizeof(int), 1, fp) == 1);
        success &= ( fread( &dummy,   sizeof(int), 1, fp) == 1);

        //read in the test parameters
        for(unsigned int i=0; i<TEST_DIM; ++i, ++ptT){
            success &= ( fread( ptT, sizeof(int), 1, fp) == 1);
        }

    }

    // read tree leafs
    LeafNode* ptLN = &leaf[0];
    for(int l=0; l<num_leaf; ++l, ++ptLN) {

        ptLN->mean.create(POSE_SIZE, 1, CV_32FC1); ptLN->mean.setTo(0);

        success &= ( fread( &dummy,         sizeof(int), 1, fp) == 1);
        success &= ( fread( &(ptLN->pfg),    sizeof(float), 1, fp) == 1);
        success &= ( fread( ptLN->mean.data, sizeof(float), POSE_SIZE, fp) == POSE_SIZE );
        success &= ( fread( &(ptLN->trace),    sizeof(float), 1, fp) == 1);

    }

    fclose(fp);
    std::cout << " done " << endl;

    return success;

}


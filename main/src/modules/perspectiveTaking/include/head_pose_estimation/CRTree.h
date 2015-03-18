#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#define TEST_DIM 11
#define POSE_SIZE 6
#define AVG_FACE_DIAMETER 236.4f
#define AVG_FACE_DIAMETER2 55884.96f

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



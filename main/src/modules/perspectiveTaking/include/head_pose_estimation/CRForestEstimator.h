#pragma once

#include "CRForest.h"
#include <fstream>
#include "opencv2/core/core.hpp"

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



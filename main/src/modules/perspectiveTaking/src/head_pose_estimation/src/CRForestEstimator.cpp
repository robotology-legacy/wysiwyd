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

#include "CRForestEstimator.h"
#include <vector>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

bool CRForestEstimator::loadForest(const char* treespath, int ntrees){

    // Init forest with number of trees
    crForest = new CRForest( ntrees );
    // Load forest
    if( !crForest->loadForest( treespath ) )
        return false;

    return true;
}

Rect CRForestEstimator::getBoundingBox(const Mat& im3D){

    Rect bbox;
    int min_x = im3D.cols;
    int min_y = im3D.rows;
    int max_x = 0;
    int max_y = 0;
    int p_width = crForest->getPatchWidth();
    int p_height = crForest->getPatchHeight();

    for(int y = 0; y < im3D.rows; y++)
    {
        const Vec3f* Mi = im3D.ptr<Vec3f>(y);
        for(int x = 0; x < im3D.cols; x++){

            if( Mi[x][2] > 0) {

                min_x = MIN(min_x,x); min_y = MIN(min_y,y);
                max_x = MAX(max_x,x); max_y = MAX(max_y,y);
            }

        }
    }

    int new_w = max_x - min_x + p_width;
    int new_h = max_y - min_y + p_height;

    bbox.x = MIN( im3D.cols-1, MAX( 0 , min_x - p_width/2 ) );
    bbox.y = MIN( im3D.rows-1, MAX( 0 , min_y - p_height/2) );

    bbox.width  = MAX(0, MIN( new_w, im3D.cols-bbox.x));
    bbox.height = MAX(0, MIN( new_h, im3D.rows-bbox.y));

    return bbox;

}

void CRForestEstimator::estimate( const Mat & im3D,
                                  std::vector< cv::Vec<float,6> >& means, //output
                                  std::vector< std::vector< Vote > >& clusters, //all clusters
                                  std::vector< Vote >& votes, //all votes
                                  int stride,
                                  float max_variance,
                                  float prob_th,
                                  float larger_radius_ratio,
                                  float smaller_radius_ratio,
                                  bool verbose , int threshold ){

    unsigned int max_clusters = 20;
    int max_ms_iterations = 10;

    int p_width = crForest->getPatchWidth();
    int p_height = crForest->getPatchHeight();

    //get bounding box around the data in the image
    Rect bbox = getBoundingBox(im3D);

    Mat* channels = new Mat[3];
    split(im3D, channels);

    //vector<Mat> channels;
    //split(im3D, channels);

    int rows = im3D.rows;
    int cols = im3D.cols;

    //integral image of the depth channel
    Mat depthInt( rows+1, cols+1, CV_64FC1);
    integral( channels[2], depthInt );

    //feature channels vector, in our case it contains only the depth integral image, but it could have other channels (e.g., greyscale)
    std::vector< cv::Mat > featureChans;
    featureChans.push_back(depthInt);

    //mask
    Mat mask( rows, cols, CV_32FC1); mask.setTo(0);
    cv::threshold( channels[2], mask, 10, 1, THRESH_BINARY);

    //integral image of the mask
    Mat maskIntegral( rows+1, cols+1, CV_64FC1); maskIntegral.setTo(0);
    integral( mask, maskIntegral );

    //defines the test patch
    Rect roi = Rect(0,0,p_width,p_height);

    int min_no_pixels = p_width*p_height/10;
    int half_w = roi.width/2;
    int half_h = roi.height/2;

    //process each patch
    for(roi.y=bbox.y; roi.y<bbox.y+bbox.height-p_height; roi.y+=stride) {

        float* rowX = channels[0].ptr<float>(roi.y + half_h);
        float* rowY = channels[1].ptr<float>(roi.y + half_h);
        float* rowZ = channels[2].ptr<float>(roi.y + half_h);

        double* maskIntY1 = maskIntegral.ptr<double>(roi.y);
        double* maskIntY2 = maskIntegral.ptr<double>(roi.y + roi.height);

        for(roi.x=bbox.x; roi.x<bbox.x+bbox.width-p_width; roi.x+=stride) {

            //discard if the middle of the patch does not have depth data
            if( rowZ[roi.x + half_w] <= 0.f )
                continue;

            //discard if the patch is filled with data for less than 10%
            if( (maskIntY1[roi.x] + maskIntY2[roi.x + roi.width] - maskIntY1[roi.x + roi.width] - maskIntY2[roi.x]) <= min_no_pixels )
                continue;

            //send the patch down the trees and retrieve leaves
            std::vector< const LeafNode* > leaves = crForest->regressionIntegral( featureChans, maskIntegral, roi );

            //go through the results
            for(unsigned int t=0;t<leaves.size();++t){

                //discard bad votes
                if ( leaves[t]->pfg < prob_th || leaves[t]->trace > max_variance )
                    continue;

                Vote v;

                //add the 3D location under the patch center to the vote for the head center
                v.vote[0] = leaves[t]->mean.at<float>(0) + rowX[roi.x + half_w];
                v.vote[1] = leaves[t]->mean.at<float>(1) + rowY[roi.x + half_w];
                v.vote[2] = leaves[t]->mean.at<float>(2) + rowZ[roi.x + half_w];

                //angles, leave as in the leaf
                v.vote[3] = leaves[t]->mean.at<float>(3);
                v.vote[4] = leaves[t]->mean.at<float>(4);
                v.vote[5] = leaves[t]->mean.at<float>(5);

                v.trace = &(leaves[t]->trace);
                v.conf = &(leaves[t]->pfg);

                votes.push_back(v);
            }

        } // end for x

    } // end for y

    if(verbose)
        cout << endl << "votes : " << votes.size() << endl;

    delete [] channels;


    Vec<float,POSE_SIZE> temp_mean;
    vector< vector< Vote* > > temp_clusters;
    vector< Vec<float,POSE_SIZE> > cluster_means;

    //radius for clustering votes
    float large_radius = AVG_FACE_DIAMETER2/(larger_radius_ratio*larger_radius_ratio);

    //cluster using the head centers
    for(unsigned int l=0;l<votes.size();++l){

        bool found = false;
        float best_dist = FLT_MAX;
        unsigned int best_cluster = 0;

        //for each cluster
        for(unsigned int c=0; ( c<cluster_means.size() && found==false ); ++c){

            float norm = 0;
            for(int n=0;n<3;++n)
                norm += (votes[l].vote[n]-cluster_means[c][n])*(votes[l].vote[n]-cluster_means[c][n]);

            //is the offset smaller than the radius?
            if( norm < large_radius ){

                best_cluster = c;
                found = true;

                //add (pointer to) vote to the closest cluster (well, actually, the first cluster found within the distance)
                temp_clusters[best_cluster].push_back( &(votes[l]) );

                //update cluster's mean
                ((cluster_means[best_cluster])) = 0;
                if ( temp_clusters[best_cluster].size() > 0 ){

                    for(unsigned int i=0;i < temp_clusters[best_cluster].size(); ++i)
                        ((cluster_means[best_cluster])) = ((cluster_means[best_cluster])) + temp_clusters[best_cluster][i]->vote;

                    float div = float(MAX(1,temp_clusters[best_cluster].size()));
                    for(int n=0;n<POSE_SIZE;++n)
                        cluster_means[best_cluster][n] /= div;
                }

            }

        }

        //create a new cluster
        if( !found && temp_clusters.size() < max_clusters ){

            vector< Vote* > new_cluster;
            new_cluster.push_back( &(votes[l]) );
            temp_clusters.push_back( new_cluster );

            Vec<float,POSE_SIZE> vote( votes[l].vote );
            cluster_means.push_back( vote );

        }

    }

    if(verbose){
        cout << cluster_means.size() << " CLUSTERS ";
        for(unsigned int c = 0 ; c<cluster_means.size(); ++c)
            cout << temp_clusters[c].size() << " ";
        cout << endl;
    }

    std::vector< std::vector< Vote* > > new_clusters; //after MS
    vector< Vec<float,POSE_SIZE> > new_means;

    int count = 0;
    float ms_radius2 = AVG_FACE_DIAMETER*AVG_FACE_DIAMETER/(smaller_radius_ratio*smaller_radius_ratio);


    //threshold defining if the cluster belongs to a head: it depends on the stride and on the number of trees
    int th = cvRound((double)threshold*crForest->getSize()/(double)(stride*stride));

    //do MS for each cluster
    for(unsigned int c=0;c<cluster_means.size();++c){

        if(verbose){
            cout << endl << "MS cluster " << c << " : ";
            for(int n=0;n<3;++n)
                cout << cluster_means[c][n] << " ";
            cout << endl;
        }

        if ( temp_clusters[c].size() <= th ){
            if(verbose)
                cout << "skipping cluster " << endl;
            continue;
        }


        vector< Vote* > new_cluster;

        for(unsigned int it=0; it<max_ms_iterations; ++it){

            count = 0;
            temp_mean = 0;
            new_cluster.clear();

            //for each vote in the cluster
            for(unsigned int idx=0; idx < temp_clusters[c].size() ;++idx){

                float norm = 0;
                for(int n=0;n<3;++n)
                    norm += (temp_clusters[c][idx]->vote[n]-cluster_means[c][n])*(temp_clusters[c][idx]->vote[n]-cluster_means[c][n]);

                if( norm < ms_radius2 ){

                    temp_mean = temp_mean + temp_clusters[c][idx]->vote;
                    new_cluster.push_back( temp_clusters[c][idx] );
                    count++;

                }

            }

            for(int n=0;n<POSE_SIZE;++n)
                temp_mean[n] /= (float)MAX(1,count);

            float distance_to_previous_mean2 = 0;
            for(int n=0;n<3;++n){
                distance_to_previous_mean2 += (temp_mean[n]-cluster_means[c][n])*(temp_mean[n]-cluster_means[c][n]);
            }

            //cout << it << " " << distance_to_previous_mean2 << endl;

            //update the mean of the cluster
            cluster_means[c] = temp_mean;

            if( distance_to_previous_mean2 < 1 )
                break;

        }

        new_clusters.push_back( new_cluster );
        new_means.push_back( cluster_means[c] );

    }

    for(unsigned int c=0; c < new_clusters.size() ;++c){

        if( new_clusters[c].size() < th ) //discard clusters with not enough votes
            continue;

        vector< Vote > cluster;
        cluster_means[c] = 0;

        //for each vote in the cluster
        for(unsigned int k=0; k < new_clusters[c].size(); k++ ){

            cluster_means[c] = cluster_means[c] + new_clusters[c][k]->vote;
            cluster.push_back( *(new_clusters[c][k]) );

        }

        float div = (float)MAX(1,new_clusters[c].size());
        for(int n=0;n<POSE_SIZE;++n)
            cluster_means[c][n] /= div;

        means.push_back( cluster_means[c] );
        clusters.push_back( cluster );

    }

}

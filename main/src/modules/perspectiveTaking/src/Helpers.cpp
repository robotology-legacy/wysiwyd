#include "Helpers.h"

void yarp2pclKinectMatrix(const yarp::sig::Matrix& kinect2icubYarp,
                          Eigen::Matrix4f& kinect2icubPCL) {
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            kinect2icubPCL(i,j)=kinect2icubYarp(i,j);
        }
    }
}

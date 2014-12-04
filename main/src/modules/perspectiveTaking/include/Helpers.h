#include "yarp/math/Math.h"
#include <pcl/pcl_base.h>

void yarp2pclKinectMatrix(const yarp::sig::Matrix& kinect2icubYarp,
                          Eigen::Matrix4f& kinect2icubPCL);

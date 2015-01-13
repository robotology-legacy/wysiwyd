/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef VPT_HELPERS
#define VPT_HELPERS

#include <yarp/math/Math.h>
#include <pcl/pcl_base.h>

void yarp2pclKinectMatrix(const yarp::sig::Matrix& kinect2icubYarp,
                          Eigen::Matrix4f& kinect2icubPCL);
Eigen::Vector4f yarp2EigenV(yarp::sig::Vector);

#endif

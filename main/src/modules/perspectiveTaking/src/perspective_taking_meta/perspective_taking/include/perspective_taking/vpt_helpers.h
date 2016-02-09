#ifndef VPT_HELPERS_H
#define VPT_HELPERS_H

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <yarp/sig/Vector.h>

namespace vpt {
    pcl::PointXYZ eigen_to_pcl(Eigen::Vector4f in) {
        return pcl::PointXYZ(in[0], in[1], in[2]);
    }

    Eigen::Vector4f pcl_to_eigen(pcl::PointXYZ in) {
        return Eigen::Vector4f(in.x, in.y, in.z, 1.0);
    }

    pcl::PointXYZ ros_to_pcl(geometry_msgs::PointStamped in) {
        return pcl::PointXYZ(in.point.x, in.point.y, in.point.z);
    }

    pcl::PointXYZ ros_to_pcl(geometry_msgs::Point in) {
        return pcl::PointXYZ(in.x, in.y, in.z);
    }

    Eigen::Vector4f ros_to_eigen(geometry_msgs::PointStamped in) {
        return Eigen::Vector4f(in.point.x, in.point.y, in.point.z, 1.0);
    }

    Eigen::Vector4f ros_to_eigen(geometry_msgs::Point in) {
        return Eigen::Vector4f(in.x, in.y, in.z, 1.0);
    }

    tf::Vector3 yarp_to_tf_converter(yarp::sig::Vector in) {
        return tf::Vector3(-1.0*in[0], -1.0*in[1], in[2]);
    }

    void printTransform() {
        // kinect to iCub head frame
        double camOffsetX = 0.4, camOffsetY = 0.05, camOffsetZ = -0.3;
        // kinect to iCub root frame
        //double camOffsetX = 0.65, camOffsetY = 0.05, camOffsetZ = -0.6;
        double camAngle = -22.5;

        Eigen::Affine3d rot_trans = Eigen::Affine3d::Identity();
        rot_trans.translation() << camOffsetX, camOffsetY, camOffsetZ;
        float theta = camAngle/180.0*M_PI;
        rot_trans.rotate (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()));

        tf::Transform t;
        tf::poseEigenToTF(rot_trans, t);
        ROS_INFO_STREAM("Origin: " << t.getOrigin().x() << " "
                                   << t.getOrigin().y() << " "
                                   << t.getOrigin().z());
        Eigen::Quaterniond q_eigen;
        tf::quaternionTFToEigen(t.getRotation(), q_eigen);
        ROS_INFO_STREAM("Quaternion: " << q_eigen.x() << " "
                                       << q_eigen.y() << " "
                                       << q_eigen.z() << " "
                                       << q_eigen.w());
    }
}

#endif // VPT_HELPERS_H

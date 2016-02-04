/*
 * This is an abstract class which is used by many other classes to visualise
 * point clouds. It is basically a slim wrapper around PCLVisualizer
 */

#include <pcl_ros/point_cloud.h>

#include "point_cloud_visualizer.h"

PointCloudVisualizer::PointCloudVisualizer(ros::NodeHandle &nh, const ros::NodeHandle &privNh, const std::string &pt_cloud_topic) :
    viewer(new pcl::visualization::PCLVisualizer(ros::this_node::getName())),
    pointCloudSubscriber(nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(
                         pt_cloud_topic,
                         1,
                         &PointCloudVisualizer::callbackPointCloud,
                         this) ),
    updateVisualizerTimer(nh.createTimer(ros::Duration(0.01), &PointCloudVisualizer::callbackVisualizeUpdater, this))
{
    auto fovy=58.0;
    auto size_x=640, size_y=480;
    auto pos_x=1300, pos_y=0;
    auto bg_color=0.2;

    privNh.getParam("fovy", fovy);
    privNh.getParam("size_x", size_x);
    privNh.getParam("size_y", size_y);
    privNh.getParam("pos_x", pos_x);
    privNh.getParam("pos_y", pos_y);
    privNh.getParam("bg_color", bg_color);

    ROS_INFO_STREAM("Start visualizer with fovy " << fovy);

    viewer->setBackgroundColor(bg_color, bg_color, bg_color);
    viewer->initCameraParameters();

    viewer->setSize(size_x, size_y);
    viewer->setPosition(pos_x, pos_y);
    viewer->setCameraFieldOfView(fovy/180.0*M_PI);
    // this is pos, view, up in camera coordinates!
    viewer->setCameraPosition(0.0, 0.0, 0.0,
                              0.0, 0.0, 1.0,
                              0.0, -1.0, 0.0);
}

void PointCloudVisualizer::callbackPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
    updateVis(msg);
}

void PointCloudVisualizer::callbackVisualizeUpdater(const ros::TimerEvent&) {
    viewer->spinOnce();
}

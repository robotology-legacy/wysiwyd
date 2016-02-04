#ifndef POINT_CLOUD_VISUALIZER_H
#define POINT_CLOUD_VISUALIZER_H

#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>

class PointCloudVisualizer {
public:
    PointCloudVisualizer(ros::NodeHandle &nh, const ros::NodeHandle& privNh, const std::string& pt_cloud_topic);
    void callbackPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    virtual void updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) = 0;

private:
    ros::Subscriber pointCloudSubscriber;
    ros::Timer updateVisualizerTimer;
    void callbackVisualizeUpdater(const ros::TimerEvent&);
};

#endif // POINT_CLOUD_VISUALIZER_H

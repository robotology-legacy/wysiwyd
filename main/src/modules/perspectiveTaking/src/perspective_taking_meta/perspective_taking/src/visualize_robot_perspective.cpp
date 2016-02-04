/*
 * Uses a virtual camera position which equals that of robot_head to visualise the scene
 * from the robot perspective (rather than the camera perspective)
 */

// General includes
#include <iostream>

// ROS includes
#include <tf/transform_listener.h>

// Project includes
#include "point_cloud_visualizer.h"
#include "vpt_helpers.h"

class VisualizeRobotPerspective : public PointCloudVisualizer {
public:
    VisualizeRobotPerspective(ros::NodeHandle &nh, ros::NodeHandle& privNh)
            : PointCloudVisualizer(nh, privNh, privNh.param<std::string>("point_cloud_topic", "/camera/depth_registered/points"))
    {
        ROS_DEBUG("Initialize VisualizeRobotPerspective");
        source_frame = "headpose_root";
    }

protected:
    void updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
    const std::string cloud_name="robot_view_cloud";
    std::string source_frame;
    const int visualizer_point_size=3;
};

void VisualizeRobotPerspective::updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->removePointCloud(cloud_name);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cloud_name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualizer_point_size, cloud_name);

    const std::string target_frame = cloud->header.frame_id;
    static tf::TransformListener listener;

    try {
        geometry_msgs::PointStamped pos_robot_head_frame, view_robot_head_frame, up_robot_head_frame;
        geometry_msgs::PointStamped pos_cloud_frame,      view_cloud_frame,      up_cloud_frame;
        pos_robot_head_frame.header.frame_id  = source_frame;
        view_robot_head_frame.header.frame_id = source_frame;
        up_robot_head_frame.header.frame_id   = source_frame;
        pos_robot_head_frame.point.x  = 0.0; pos_robot_head_frame.point.y  = 0.0; pos_robot_head_frame.point.z  = 0.0;
        view_robot_head_frame.point.x = 1.0; view_robot_head_frame.point.y = 0.0; view_robot_head_frame.point.z = 0.0;
        up_robot_head_frame.point.x   = 0.0; up_robot_head_frame.point.y   = 0.0; up_robot_head_frame.point.z   = 1.0;

        listener.waitForTransform(source_frame, target_frame, ros::Time::now(), ros::Duration(1.0));
        listener.transformPoint(target_frame, pos_robot_head_frame,  pos_cloud_frame);
        listener.transformPoint(target_frame, view_robot_head_frame, view_cloud_frame);
        listener.transformPoint(target_frame, up_robot_head_frame,   up_cloud_frame);

        viewer->setCameraPosition(pos_cloud_frame.point.x,  pos_cloud_frame.point.y,  pos_cloud_frame.point.z,
                                  view_cloud_frame.point.x, view_cloud_frame.point.y, view_cloud_frame.point.z,
                                  up_cloud_frame.point.x,   up_cloud_frame.point.y,   up_cloud_frame.point.z);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "visualize_robot_perspective");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    if(!nhPriv.hasParam("fovy")) {
        nhPriv.setParam("fovy", 90.0);
    }

    VisualizeRobotPerspective node(nh, nhPriv);
    (void) node;

    ros::spin();

    return 0;
}

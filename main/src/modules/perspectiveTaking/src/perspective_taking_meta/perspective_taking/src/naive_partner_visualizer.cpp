/*
 * This is the most simple form to transform the point cloud captured by the camera
 * to the coordinate frame of the human head. It takes an offset into consideration,
 * which takes into account that the head pose estimation returns a point in the
 * center of the head rather than the nose tip (what we want). This is not an actual
 * transformation, but an offset / rotation of the camera around the scene!
 */

// ROS includes
#include <tf/transform_listener.h>

// Project includes
#include "point_cloud_visualizer.h"
#include "vpt_helpers.h"

class NaivePartnerVisualizer : public PointCloudVisualizer {
public:
    NaivePartnerVisualizer(ros::NodeHandle &nh, ros::NodeHandle& privNh)
        : PointCloudVisualizer(nh, privNh, privNh.param<std::string>("point_cloud_topic", "/rtabmap/assembled_clouds")),
          head_offset(tf::Vector3(0.15, 0.0, 0.0)) {
        ROS_DEBUG("Initialize NaivePartnerVisualizer");
    }

protected:
    void updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->removePointCloud(cloud_name);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cloud_name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualizer_point_size, cloud_name);

        const std::string source_frame = "head_origin1";
        const std::string target_frame = cloud->header.frame_id;
        static tf::TransformListener listener;

        try {
            geometry_msgs::PointStamped pos_head_frame,  view_head_frame,  up_head_frame;
            geometry_msgs::PointStamped pos_cloud_frame, view_cloud_frame, up_cloud_frame;
            pos_head_frame.header.frame_id  = source_frame;
            view_head_frame.header.frame_id = source_frame;
            up_head_frame.header.frame_id   = source_frame;
            tf::pointTFToMsg(head_offset, pos_head_frame.point);
            view_head_frame.point.x = 1.0; view_head_frame.point.y = 0.0; view_head_frame.point.z = -0.4;
            up_head_frame.point.x   = 0.0; up_head_frame.point.y   = 0.0; up_head_frame.point.z   = 1.0;

            listener.waitForTransform(source_frame, target_frame,  ros::Time::now(), ros::Duration(1.0));
            listener.transformPoint(target_frame, pos_head_frame,  pos_cloud_frame);
            listener.transformPoint(target_frame, view_head_frame, view_cloud_frame);
            listener.transformPoint(target_frame, up_head_frame,   up_cloud_frame);

            viewer->setCameraPosition(pos_cloud_frame.point.x,
                                      pos_cloud_frame.point.y,
                                      pos_cloud_frame.point.z,
                                      view_cloud_frame.point.x,
                                      view_cloud_frame.point.y,
                                      view_cloud_frame.point.z,
                                      up_cloud_frame.point.x,
                                      up_cloud_frame.point.y,
                                      up_cloud_frame.point.z);
        } catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
    }

private:
    const std::string cloud_name="partner_view_cloud";
    const int visualizer_point_size=3;
    tf::Vector3 head_offset;
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "naive_partner_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    ROS_INFO("Start naive");

    NaivePartnerVisualizer node(nh, nhPriv);
    (void) node;

    ros::spin();

    return 0;
}

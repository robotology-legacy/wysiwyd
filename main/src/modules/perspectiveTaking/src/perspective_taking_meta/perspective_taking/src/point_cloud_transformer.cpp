/*
 * This transforms a pointcloud from a given topic to a newly created topic,
 * with a target frame which also has to be provided
 *
 */

#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class PointCloudTransformer {
public:
    PointCloudTransformer(ros::NodeHandle &nh, ros::NodeHandle &privNh);
    void callbackPointcloud(const sensor_msgs::PointCloud2::ConstPtr&);

protected:

private:
    std::string     target_frame;
    ros::Subscriber pointcloudSubscriber;
    ros::Publisher  transformedPointcloudPublisher;
};

PointCloudTransformer::PointCloudTransformer(ros::NodeHandle &nh, ros::NodeHandle &privNh) :
    pointcloudSubscriber(nh.subscribe<sensor_msgs::PointCloud2>(
                         privNh.param<std::string>("pc_from", "/camera/depth_registered/points"),
                         1,
                         &PointCloudTransformer::callbackPointcloud,
                         this) ),
    transformedPointcloudPublisher(nh.advertise<sensor_msgs::PointCloud2>(
                         privNh.param<std::string>("pc_to", "/transformed_cloud"),
                         1)),
    target_frame(std::move(privNh.param<std::string>("target_frame", "robot_root")))
{
}

void PointCloudTransformer::callbackPointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sensor_msgs::PointCloud2 cloud_transformed;
    static tf::TransformListener listener;

    listener.waitForTransform(target_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
    pcl_ros::transformPointCloud(target_frame, *msg, cloud_transformed, listener);
    transformedPointcloudPublisher.publish(cloud_transformed);

    /*for(int i=0; i<cloud_transformed.fields.size(); i++) {
        ROS_DEBUG_STREAM("field i: " << i << " name:" << cloud_transformed.fields[i].name);

    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    sensor_msgs::PointCloud2ConstIterator<float> iter_x_trans(cloud_transformed, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y_trans(cloud_transformed, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z_trans(cloud_transformed, "z");

    for(int i=0; i<10 && iter_x!=iter_x.end() && iter_y!=iter_y.end() && iter_z!=iter_z.end() &&
                         iter_x_trans!=iter_x_trans.end() && iter_y_trans!=iter_y_trans.end() && iter_z_trans!=iter_z_trans.end();
                         ++iter_x, ++iter_y, ++iter_z, ++iter_x_trans, ++iter_y_trans, ++iter_z_trans) {
        ROS_DEBUG_STREAM("i: " << i << " x org:" << *iter_x);
        ROS_DEBUG_STREAM("i: " << i << " x tra:" << *iter_x_trans);
    }*/
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_transformer");
    ros::NodeHandle nh;
    ros::NodeHandle privNh("~");

    PointCloudTransformer pct(nh, privNh);
    (void) pct;

    ros::spin();

    return 0;
}

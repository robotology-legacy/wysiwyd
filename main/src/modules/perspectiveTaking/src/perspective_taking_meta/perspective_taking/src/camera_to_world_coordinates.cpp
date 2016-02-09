/*
 * This is a test showing how to get the 3D world coordinates from 2D pixel coordinates
 */


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

class CameraToWorldCoordinates {
public:
    CameraToWorldCoordinates(ros::NodeHandle &nh);
    void callbackPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&);

protected:

private:
    ros::Subscriber pointcloudSubscriber;
};

CameraToWorldCoordinates::CameraToWorldCoordinates(ros::NodeHandle &nh) :
    pointcloudSubscriber(nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(
                         "/camera/depth_registered/points",
                         1,
                         &CameraToWorldCoordinates::callbackPointcloud,
                         this) )
{

}

void CameraToWorldCoordinates::callbackPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
    ROS_INFO_STREAM("coordinates:" << msg->at(50,50).x << " " << msg->at(50,50).y << " " << msg->at(50,50).z );
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "camera_to_world_coordinates");
    ros::NodeHandle nh;

    CameraToWorldCoordinates c(nh);
    (void) c;

    ros::spin();

    return 0;
}

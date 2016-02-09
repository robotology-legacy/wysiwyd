/*
 * Based on the location of objects relative to the human head,
 * this outputs whether an object is to the left, right or in front
 * of the human (+-5 degrees for center area)
 */

// C++ includes
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <tf/transform_listener.h>

// YARP includes
#include <wrdac/clients/opcClient.h>

class SpatialPerspectiveLevel2 {
public:
    SpatialPerspectiveLevel2(ros::NodeHandle &nh, const ros::NodeHandle& privNh);
    ~SpatialPerspectiveLevel2();

protected:

private:
    ros::Subscriber pointCloudSubscriber;
    void callbackTimer(const ros::TimerEvent&);
    ros::Timer updateVisualizerTimer;
    std::unique_ptr<wysiwyd::wrdac::OPCClient> opc;
};

SpatialPerspectiveLevel2::SpatialPerspectiveLevel2(ros::NodeHandle &nh, const ros::NodeHandle &privNh) :
    updateVisualizerTimer(nh.createTimer(ros::Duration(0.5), &SpatialPerspectiveLevel2::callbackTimer, this)),
    opc(new wysiwyd::wrdac::OPCClient(ros::this_node::getName().substr(1)))
{
    if (!yarp::os::Network::checkNetwork()) {
        ROS_ERROR("YARP could not be initialized");
    } else {
        int itry=0;
        while (!opc->connect("OPC") && itry<3) {
            ROS_INFO("Waiting for connection to OPC...");
            ros::Duration(0.5).sleep();
            itry++;
        }
    }
}

SpatialPerspectiveLevel2::~SpatialPerspectiveLevel2() {
    if (yarp::os::Network::checkNetwork()) {
        opc->interrupt();
        opc->close();
    }
}

void SpatialPerspectiveLevel2::callbackTimer(const ros::TimerEvent&) {
    static tf::TransformListener listener;

    std::string target_frame = "head_origin1";
    std::vector<std::string> frames;
    listener.getFrameStrings(frames);

    for(size_t i=0; i<frames.size(); i++) {
        std::string obj_name = frames.at(i);
        if(obj_name.find("obj_")!=std::string::npos) {
            geometry_msgs::PointStamped object_head_frame;
            try {
                geometry_msgs::PointStamped object_object_frame;
                // get object coordinates in target_frame, needed because of offset
                object_object_frame.header.frame_id = obj_name;
                object_object_frame.point.x = 0;
                object_object_frame.point.y = 0;
                object_object_frame.point.z = 0;

                listener.waitForTransform(obj_name, target_frame, ros::Time::now(), ros::Duration(1.0));
                listener.transformPoint(target_frame, object_object_frame, object_head_frame);

                double angle = atan(object_head_frame.point.y / object_head_frame.point.x) * 180 / M_PI;

                if(angle > 5.0) {
                    ROS_INFO_STREAM("Object " << obj_name << " is left of human.");
                } else if(angle < -5.0){
                    ROS_INFO_STREAM("Object " << obj_name << " is right of human.");
                } else {
                    ROS_INFO_STREAM("Object " << obj_name << " is central.");
                }
            } catch (tf::TransformException ex) {
                ROS_WARN_STREAM("Skip object " << obj_name << " as point could not be transformed!");
                continue;
            }
        }
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "spatial_perspective_2");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    yarp::os::Network yarp;
    (void) yarp;

    SpatialPerspectiveLevel2 node(nh, nhPriv);
    (void) node;

    ros::spin();

    return 0;
}

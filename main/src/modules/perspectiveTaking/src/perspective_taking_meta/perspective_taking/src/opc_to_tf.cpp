/*
 * Here we retrieve a list of objects from the OPC, and
 * publish TF transformations from the robot_root to those
 * objects.
 */

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// YARP includes
#include <yarp/os/all.h>
#include <wrdac/clients/opcClient.h>

// Eigen includes
#include <Eigen/Geometry>

// Project includes
#include "vpt_helpers.h"

std::unique_ptr<wysiwyd::wrdac::OPCClient> opc;

void updateObjectsCallback(const ros::TimerEvent&) {
    static tf::TransformBroadcaster br;

    opc->checkout();
    auto entities = opc->EntitiesCache();

    for(const auto& entity:entities) {
        if(entity->isType("object")) {
            if (auto obj=dynamic_cast<wysiwyd::wrdac::Object*>(entity)) {
                if((obj->m_present==1.0) && obj->name()!="partner") {
                    // draw object in visualizer
                    auto obj_name = obj->name();
                    auto obj_pos_yarp = obj->m_ego_position;

                    tf::Vector3 obj_pos = vpt::yarp_to_tf_converter(obj_pos_yarp);

                    tf::Transform transform;
                    transform.setOrigin(obj_pos);
                    transform.setRotation(tf::Quaternion(0, 0, 0, 1));

                    ROS_DEBUG_STREAM("Send transform for " << obj_name << ": " << std::endl << obj_pos);

                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_root", "obj_"+obj_name));
                }
            } else {
                ROS_WARN("Could not cast entity to object!");
            }
        }
    }
}

int main (int argc, char** argv) {
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        ROS_ERROR("YARP could not be initialized");
        return -1;
    }

    ros::init(argc, argv, "opc_to_tf");
    ros::NodeHandle nh;

    opc = std::unique_ptr<wysiwyd::wrdac::OPCClient>(new wysiwyd::wrdac::OPCClient(ros::this_node::getName().substr(1)));
    while (!opc->connect("OPC")) {
        ROS_INFO("Waiting for connection to OPC...");
        ros::Duration(0.5).sleep();
    }

    ros::Timer update_object_timer = nh.createTimer(ros::Duration(0.3), updateObjectsCallback);
    (void) update_object_timer;

    ros::spin();

    opc->interrupt();
    opc->close();

    return 0;
}

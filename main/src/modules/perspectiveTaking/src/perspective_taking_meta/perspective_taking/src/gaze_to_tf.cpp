/*
 * This code subscribes to the iKinGazeCtrl to publish transforms from
 * robot_root to robot_head, robot_left_eye and robot_right_eye
 *
 * It is also a nice showcase for C++11 features, e.g. a map containing a string as key
 * and a function as value, then iterating over this map and calling the function stored
 * as value
 */

// General includes
#include <vector>
#include <functional>

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// YARP includes
#include <yarp/os/all.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>

// Eigen includes
#include <Eigen/Geometry>

// Project includes
#include "vpt_helpers.h"

yarp::dev::IGazeControl *igaze=NULL;

void updateReferenceFramesCallback(const ros::TimerEvent&) {
    static tf::TransformBroadcaster br;

    tf::Quaternion yarp_to_ros; yarp_to_ros.setRPY(M_PI, -M_PI/2.0, M_PI*2.0);
    tf::Transform transform_yarp_to_ros(yarp_to_ros);

    auto function_map = std::map<std::string, std::function<bool (yarp::sig::Vector&, yarp::sig::Vector&)> > {
        {"robot_head",      std::bind(&yarp::dev::IGazeControl::getHeadPose,     igaze, std::placeholders::_1, std::placeholders::_2, nullptr)},
        {"robot_left_eye",  std::bind(&yarp::dev::IGazeControl::getLeftEyePose,  igaze, std::placeholders::_1, std::placeholders::_2, nullptr)},
        {"robot_right_eye", std::bind(&yarp::dev::IGazeControl::getRightEyePose, igaze, std::placeholders::_1, std::placeholders::_2, nullptr)}
    };

    for (auto& f : function_map) {
        yarp::sig::Vector origin; origin.resize(3);
        yarp::sig::Vector orientation; orientation.resize(4);
        if(f.second(origin, orientation)) {
            tf::Quaternion q(vpt::yarp_to_tf_converter(orientation), orientation[3]);
            tf::Transform transform_yarp(q, vpt::yarp_to_tf_converter(origin));

            tf::Transform transform = transform_yarp*transform_yarp_to_ros;
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_root", f.first));
        } else {
            ROS_ERROR_STREAM("Could not retrieve transformation for " << f.first);
        }
    }
}

int main (int argc, char** argv) {
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        ROS_ERROR("YARP could not be initialized");
        return -1;
    }

    ros::init(argc, argv, "gaze_to_tf");
    ros::NodeHandle nh;

    yarp::os::Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local", "/gaze_to_tf/gaze");

    yarp::dev::PolyDriver clientGazeCtrl(option);
    if (clientGazeCtrl.isValid()) {
       clientGazeCtrl.view(igaze);
    } else {
        ROS_ERROR("Could not initialize clientGazeCtrl!");
        return -1;
    }

    ros::Timer update_frames_timer = nh.createTimer(ros::Duration(0.3), updateReferenceFramesCallback);
    (void) update_frames_timer;

    ros::spin();

    igaze->stopControl();
    clientGazeCtrl.close();

    return 0;
}

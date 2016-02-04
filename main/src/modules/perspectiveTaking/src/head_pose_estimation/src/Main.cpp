/*
// Copyright (c) 2012 Miguel Sarabia del Castillo
// Imperial College London

// You may use, copy, reproduce, and distribute this Software for any
// non-commercial purpose, subject to the restrictions of the
// Microsoft Research Shared Source license agreement ("MSR-SSLA").
// Some purposes which can be non-commercial are teaching, academic
// research, public demonstrations and personal experimentation. You
// may also distribute this Software with books or other teaching
// materials, or publish the Software on websites, that are intended
// to teach the use of the Software for academic or other
// non-commercial purposes.
// You may not use or distribute this Software or any derivative works
// in any form for commercial purposes. Examples of commercial
// purposes would be running business operations, licensing, leasing,
// or selling the Software, distributing the Software for use with
// commercial products, using the Software in the creation or use of
// commercial products or any other activity which purpose is to
// procure a commercial gain to you or others.
// If the Software includes source code or data, you may create
// derivative works of such portions of the Software and distribute
// the modified Software for non-commercial purposes, as provided
// herein.

// THE SOFTWARE COMES "AS IS", WITH NO WARRANTIES. THIS MEANS NO
// EXPRESS, IMPLIED OR STATUTORY WARRANTY, INCLUDING WITHOUT
// LIMITATION, WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A
// PARTICULAR PURPOSE, ANY WARRANTY AGAINST INTERFERENCE WITH YOUR
// ENJOYMENT OF THE SOFTWARE OR ANY WARRANTY OF TITLE OR
// NON-INFRINGEMENT. THERE IS NO WARRANTY THAT THIS SOFTWARE WILL
// FULFILL ANY OF YOUR PARTICULAR PURPOSES OR NEEDS. ALSO, YOU MUST
// PASS THIS DISCLAIMER ON WHENEVER YOU DISTRIBUTE THE SOFTWARE OR
// DERIVATIVE WORKS.

// NEITHER MICROSOFT NOR ANY CONTRIBUTOR TO THE SOFTWARE WILL BE
// LIABLE FOR ANY DAMAGES RELATED TO THE SOFTWARE OR THIS MSR-SSLA,
// INCLUDING DIRECT, INDIRECT, SPECIAL, CONSEQUENTIAL OR INCIDENTAL
// DAMAGES, TO THE MAXIMUM EXTENT THE LAW PERMITS, NO MATTER WHAT
// LEGAL THEORY IT IS BASED ON. ALSO, YOU MUST PASS THIS LIMITATION OF
// LIABILITY ON WHENEVER YOU DISTRIBUTE THE SOFTWARE OR DERIVATIVE
// WORKS.

*/

//==============================================================================
// INCLUDES
//==============================================================================
//Project includes
#include "CRForestEstimator.h"

//Standard includes
#include <string>
#include <vector>

//ROS includes
#include <ros/ros.h>
#include <rospack/rospack.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//Boost includes
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

//Eigen includes
#include <Eigen/Geometry>

//==============================================================================
// CONSTANTS
//==============================================================================
namespace constants
{
const std::string pkgName = "head_pose_estimation";
const std::string initError = "Cannot initialise HeadPoseEstimator";
const std::string expectedEnconding = "16UC1";
const std::string encodingError =
        "Bad image encoding received, expected " +
        expectedEnconding;

}//end of constants namespace

//==============================================================================
// EULER TO QUATERNION
//==============================================================================
namespace
{

inline double milliToMetres(double milli)
{
    return milli / 1000.0;
}

inline double deg2rad(double deg)
{
    double rad = deg*M_PI/180.0;

    while (rad <= -M_PI)
    {
        rad += 2*M_PI;
    }
    while (rad > M_PI )
    {
        rad -= 2*M_PI;
    }
    return rad;
}

inline void rpyToQuaternion(
        double roll,
        double pitch,
        double yaw,
        geometry_msgs::Quaternion& out)
{
    Eigen::AngleAxis<double> eigenRoll(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxis<double> eigenPitch(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxis<double> eigenYaw(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = eigenRoll * eigenPitch * eigenYaw;

    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
}
} //end of anonymous namespace

//==============================================================================
// DEFINITION OF HeadPoseEstimator
//==============================================================================
class HeadPoseEstimator
{
public:
    HeadPoseEstimator(ros::NodeHandle& nh, const ros::NodeHandle& privNh);

    void callback(const sensor_msgs::ImageConstPtr &);

    void getCameraInfo(const sensor_msgs::CameraInfoConstPtr& );

private:
    //Private typedefs
    typedef cv::Vec<float, 6> HeadPose;
    typedef std::vector<HeadPose> HeadPoses;
    typedef std::vector<Vote> Votes;
    typedef std::vector<Votes> Clusters;
    typedef cv::Mat Image;

    //Forest Estimator variables
    CRForestEstimator estimator_;

    struct Parameters
    {
        Parameters();
        void update(const ros::NodeHandle& privNh);

        int maxDistance;
        int headThreshold;
        int width;
        int height;
        double probabilityThreshold;
        double maxVariance;
        double stride;
        double clusteringRadius;
        double meanShiftRadius;
        double focalLengthX;
        double focalLengthY;
        std::string frameTo;
    } params_;

    Image image_;
    bool cameraInfoReceived_;

    //ROS related variables
    ros::Publisher posePublisher_;
    ros::Subscriber imageSubscriber_;
    ros::Subscriber cameraInfoSubscriber_;

    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;

    //private functions
    void updateImage(const sensor_msgs::ImageConstPtr &);
};


//==============================================================================
// IMPLEMENTATION of HeadPoseEstimator
//==============================================================================

HeadPoseEstimator::HeadPoseEstimator(
        ros::NodeHandle &nh,
        const ros::NodeHandle &privNh)
    : estimator_()
    , params_()
    , cameraInfoReceived_(false)
    , posePublisher_(nh.advertise<geometry_msgs::PoseArray>("head_pose", 1000))
    , imageSubscriber_(nh.subscribe<sensor_msgs::Image>(
                           "depth/image_raw",
                           1,
                           &HeadPoseEstimator::callback,
                           this) )
    , cameraInfoSubscriber_(nh.subscribe<sensor_msgs::CameraInfo>(
                                "depth/camera_info",
                                1,
                                &HeadPoseEstimator::getCameraInfo,
                                this))
{
    //Find path to head_pose_estimation package
    std::string pkgPath;

    rospack::Rospack rospack;
    std::vector<std::string> searchPath;
    rospack.getSearchPathFromEnv(searchPath);
    rospack.crawl(searchPath, true);

    if (!rospack.find( constants::pkgName, pkgPath) )
    {
        ROS_FATAL_STREAM("Cannot find package " << constants::pkgName);
        throw std::runtime_error( constants::initError );
    }
    pkgPath += "/data/tree";

    //Load trees from that location
    if (! estimator_.loadForest( pkgPath.c_str() , 10) )
    {
        ROS_FATAL("Could not load HeadPoseEstimator trees");
        throw std::runtime_error( constants::initError );
    }

    //Update parameters if needed
    params_.update(privNh);

    std::cout << "Use max_distance of " << params_.maxDistance << std::endl;
}

void HeadPoseEstimator::callback(const sensor_msgs::ImageConstPtr &rosImage)
{
    //Cannot do anything till we receive info about the focal length
    if (!cameraInfoReceived_)
    {
        ROS_WARN("Discarding depth image as CameraInfo hasn't been received");
        return;
    }

    //Get at member image
    updateImage(rosImage);

    //Prepare outputs of estimator
    HeadPoses output;
    Votes votes;
    Clusters clusters;

    estimator_.estimate(
                image_,// 3d-image
                output, //xyz-rpy position of images
                clusters,
                votes,
                params_.stride,
                params_.maxVariance,
                params_.probabilityThreshold,
                params_.clusteringRadius,
                params_.meanShiftRadius,
                false,
                params_.headThreshold);

    ROS_INFO("Heads found: %lu", output.size());

    // if no poses were found return early
    if ( output.empty() )
    {
        return;
    }

    // Prepare output message
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = rosImage->header.frame_id;
    msg.header.stamp = ros::Time::now();

    for(HeadPoses::iterator it = output.begin(); it != output.end(); ++it)
    {
        const HeadPose& headPose = *it;
        geometry_msgs::PoseStamped rosPoseStamped;
        rosPoseStamped.header.stamp = ros::Time();
        rosPoseStamped.header.frame_id = "headpose_root";

        rosPoseStamped.pose.position.x = ::milliToMetres(+headPose[2]);
        rosPoseStamped.pose.position.y = ::milliToMetres(-headPose[0]);
        rosPoseStamped.pose.position.z = ::milliToMetres(-headPose[1]);

        //Convert rpy
        ::rpyToQuaternion(
                    ::deg2rad(headPose[5]), // Roll
                    ::deg2rad(-headPose[3]), // Pitch
                    ::deg2rad(180.0-headPose[4]), // Yaw
                    rosPoseStamped.pose.orientation);

        try {
            geometry_msgs::PoseStamped transformedPose;
            listener_.waitForTransform(rosImage->header.frame_id, rosPoseStamped.header.frame_id, rosPoseStamped.header.stamp, ros::Duration(1.0));
            listener_.transformPose(rosImage->header.frame_id, rosPoseStamped, transformedPose);

            msg.poses.push_back(transformedPose.pose);

            tf::Transform transform;
            tf::poseMsgToTF(transformedPose.pose, transform);
            /*ROS_INFO_STREAM("TEST x:" << transformedPose.pose.position.x << " " << rosPoseStamped.pose.position.x);
            ROS_INFO_STREAM("TEST y:" << transformedPose.pose.position.y << " " << rosPoseStamped.pose.position.y);
            ROS_INFO_STREAM("TEST z:" << transformedPose.pose.position.z << " " << rosPoseStamped.pose.position.z);*/

            std::string final_frame_name = params_.frameTo + boost::lexical_cast<std::string>(msg.poses.size());
            broadcaster_.sendTransform(tf::StampedTransform(transform, msg.header.stamp, rosImage->header.frame_id, final_frame_name));
        } catch(tf::TransformException& ex) {
            ROS_ERROR("Received an exception trying to transform a point: %s", ex.what());
        }
    }

    //And publish message
    posePublisher_.publish(msg);
}


void HeadPoseEstimator::getCameraInfo(
        const sensor_msgs::CameraInfoConstPtr& info)
{
    params_.width = info->width;
    params_.height = info->height;
    params_.focalLengthX = info->K[0];
    params_.focalLengthY = info->K[4];

    image_.create(params_.width, params_.height, CV_32FC3);

    cameraInfoReceived_ = true;

    cameraInfoSubscriber_.shutdown();
}


void HeadPoseEstimator::updateImage(
        const sensor_msgs::ImageConstPtr& rosImage)
{
    //Define type of depth pixel
    typedef ushort DepthPixel;

    //Convert ROS image to something we can traverse with OpenCV
    cv_bridge::CvImageConstPtr bridgeImage = cv_bridge::toCvShare(rosImage);

    //Check bridgeImage comes in the right format
    if ( bridgeImage->encoding != constants::expectedEnconding)
    {
        ROS_FATAL_STREAM( "Bad image encoding: " << bridgeImage->encoding );
        throw std::runtime_error(constants::encodingError);
    }

    int halfWidth = params_.width / 2;
    int halfHeight = params_.height / 2;

    //I have no idea of what I'm doing now, I got it from demo code :S
    for(int y = 0; y < params_.height; ++y)
    {
        //Get pointers to columns
        cv::Vec3f* row = image_.ptr<cv::Vec3f>(y);
        const DepthPixel* bridgeRow = bridgeImage->image.ptr<DepthPixel>(y);

        for(int x = 0; x < params_.width; ++x)
        {
            DepthPixel depth = bridgeRow[x];

            if ( depth < params_.maxDistance && depth > 0 )
            {
                geometry_msgs::PointStamped camera_point;
                camera_point.header.frame_id = rosImage->header.frame_id;
                camera_point.header.stamp = ros::Time();

                camera_point.point.x = ::milliToMetres(depth * (x - halfWidth) / params_.focalLengthX);
                camera_point.point.y = ::milliToMetres(depth * (y - halfHeight) / params_.focalLengthY);
                camera_point.point.z = ::milliToMetres(depth);

                try {
                    geometry_msgs::PointStamped base_point;
                    listener_.waitForTransform(camera_point.header.frame_id, "headpose_root_optical", ros::Time(0), ros::Duration(1.0));
                    listener_.transformPoint("headpose_root_optical", camera_point, base_point);

                    row[x][0] = base_point.point.x * 1000.0;
                    row[x][1] = base_point.point.y * 1000.0;
                    row[x][2] = base_point.point.z * 1000.0;

                    /*if(x == halfWidth && y == halfWidth) {
                        ROS_INFO_STREAM("x org:" << camera_point.point.x << " x new: " << base_point.point.x);
                        ROS_INFO_STREAM("y org:" << camera_point.point.y << " y new: " << base_point.point.y);
                        ROS_INFO_STREAM("z org:" << camera_point.point.z << " z new: " << base_point.point.z);
                    }*/
                }
                catch(tf::TransformException& ex) {
                    row[x] = 0;
                    ROS_ERROR("Received an exception trying to transform a point: %s", ex.what());
                }
            }
            else
            {
                row[x] = 0;
            }
        }
    }
}

//==============================================================================
// IMPLEMENTATION OF Parameters
//==============================================================================

HeadPoseEstimator::Parameters::Parameters()
    : maxDistance(1300)
    , headThreshold(500)
    , width(0)
    , height(0)
    , probabilityThreshold(1.0)
    , maxVariance(800)
    , stride(5)
    , clusteringRadius(1.6)
    , meanShiftRadius(5)
    , focalLengthX(0)
    , focalLengthY(0)
    , frameTo("head_origin")
{
}

void HeadPoseEstimator::Parameters::update(const ros::NodeHandle &privNh)
{
    //Update parameters if user has changed them in parameter server
    privNh.getParam("max_distance", maxDistance);
    privNh.getParam("head_threshold", headThreshold);
    privNh.getParam("probability_threshold", probabilityThreshold);
    privNh.getParam("max_variance", maxVariance);
    privNh.getParam("stride", stride);
    privNh.getParam("clustering_radius", clusteringRadius);
    privNh.getParam("mean_shift_radius", meanShiftRadius);
    privNh.getParam("frame_to", frameTo);
}


//==============================================================================
// MAIN()
//==============================================================================

int main(int argc, char* argv[])
{
    ros::init(argc, argv, constants::pkgName);
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");


    HeadPoseEstimator node(nh, nhPriv);
    (void) node;

    ros::spin();

    return 0;
}

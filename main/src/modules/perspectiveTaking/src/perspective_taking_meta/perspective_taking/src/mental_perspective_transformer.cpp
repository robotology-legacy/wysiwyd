/*
 * Here we perform successive mental rotations starting from
 * the robot head towards the human head. See the ESA for
 * details on the math behind the transforms.
 *
 * Using these transformations, one achieves similar reaction
 * times to those of humans.
 */

//Standard includes
#include <string>
#include <vector>

//ROS includes
#include <ros/ros.h>
#include <perspective_taking_python/CircleEquation.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

//Eigen includes
#include <Eigen/Geometry>

// Project includes
#include "point_cloud_visualizer.h"

// This class visualises the human_perspective cloud published by MentalPerspectiveTransformer
class TransformedPerspectiveVisualizer : public PointCloudVisualizer {
public:
    TransformedPerspectiveVisualizer(ros::NodeHandle &nh, const ros::NodeHandle& privNh, const std::string &pt_cloud_topic)
        : PointCloudVisualizer(nh, privNh, pt_cloud_topic) {
        ROS_DEBUG("Initialize VisualizeTransformedPerspective");

        viewer->setCameraPosition(0.1, 0.0, 0.0,
                                  1.0, 0.0, 0.0,
                                  0.0, 0.0, 1.0);
    }

protected:
    void updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->removePointCloud(cloud_name);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cloud_name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualizer_point_size, cloud_name);
    }

private:
    const std::string cloud_name="partner_view_cloud";
    const int visualizer_point_size=3;
};

class MentalPerspectiveTransformer
{
public:
    MentalPerspectiveTransformer(ros::NodeHandle& nh, ros::NodeHandle &privNh);

    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_in);
private:
    ros::Publisher transformedCloudPublisher_;
    ros::Subscriber cloudSubscriber_;
    ros::ServiceClient equationSolverClient_;
};

MentalPerspectiveTransformer::MentalPerspectiveTransformer(
        ros::NodeHandle& nh,
        ros::NodeHandle& privNh):
    transformedCloudPublisher_(nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("human_perspective", 1)),
    cloudSubscriber_(nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(
                         privNh.param<std::string>("point_cloud_topic", "/rtabmap/assembled_clouds"),
                         1,
                         &MentalPerspectiveTransformer::callback,
                         this) ),
    equationSolverClient_(nh.serviceClient<perspective_taking_python::CircleEquation>("solve_circle_equation"))
{
    equationSolverClient_.waitForExistence(); // writes INFO message automatically
    //privNh.getParam("max_distance", maxDistance);
}

void MentalPerspectiveTransformer::callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_in) {
    static tf::TransformListener listener;
    const std::string source_frame = "robot_head";
    const std::string target_frame = "head_origin1";

    // first get cloud in the source frame (it is usually in camera frame)
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in_source_frame;
    pcl_ros::transformPointCloud(source_frame, *cloud_in, cloud_in_source_frame, listener);

    static tf::TransformBroadcaster br;
    try {
        // get transform from source to target frame
        tf::StampedTransform transform_source_target;
        listener.waitForTransform(target_frame, source_frame, pcl_conversions::fromPCL(cloud_in_source_frame.header.stamp), ros::Duration(1.0));
        listener.lookupTransform (target_frame, source_frame, ros::Time(0), transform_source_target);

        ROS_INFO_STREAM("Origin: " << transform_source_target.getOrigin().getX() << " "
                                   << transform_source_target.getOrigin().getY() << " "
                                   << transform_source_target.getOrigin().getZ());
        ROS_INFO_STREAM("Angle: "  << transform_source_target.getRotation().getAngle());
        ROS_INFO_STREAM("Axis: "   << transform_source_target.getRotation().getAxis().getX() << " "
                                   << transform_source_target.getRotation().getAxis().getY() << " "
                                   << transform_source_target.getRotation().getAxis().getZ());

        tf::Quaternion q_origin       = tf::Quaternion::getIdentity();
        tf::Quaternion q_dest         = transform_source_target.getRotation();
        tf::Vector3    v_dest         = transform_source_target.getOrigin();

        //ROS_INFO_STREAM("q_dest: " << q_dest.getX() << " " << q_dest.getY() << " " << q_dest.getZ() << " " << q_dest.getW());

        /*tf::Vector3 v_r               = v_dest/2.0;
        ROS_INFO_STREAM("v_r: " << v_r.getX() << " " << v_r.getY() << " " << v_r.getZ());
        tf::Vector3 c                 = tf::Vector3(-1.0*v_r[1], v_r[0], 0);
        tf::Vector3 p_circle          = v_r + c;
        ROS_INFO_STREAM("p_circle " << p_circle.getX() << " " << p_circle.getY() << " " << p_circle.getZ());
        double radius                 = v_r.length();
        ROS_INFO_STREAM("radius " << radius);
        tf::Vector3 n = p_circle.cross(v_r).normalize();
        if(transform.getRotation().getAngle()>M_PI) {
            n = -1.0*n;
        }

        ROS_INFO_STREAM("n " << n.getX() << " " << n.getY() << " " << n.getZ());
        tf::Vector3 u                 = v_dest.normalized();
        ROS_INFO_STREAM("u " << u.getX() << " " << u.getY() << " " << u.getZ());*/

        // compute circle equation
        double r1x = (pow(v_dest.getX(), 2.0) + pow(v_dest.getY(), 2.0) + pow(v_dest.getZ(), 2.0)) / (2.0*(v_dest.getX()+pow(v_dest.getZ(), 2.0)/v_dest.getX()));
        double r1y = 0.0;
        double r1z = r1x * v_dest.getZ()/v_dest.getX();

        tf::Vector3 r1(r1x, r1y, r1z);
        tf::Vector3 r2(v_dest.getX() - r1x, v_dest.getY() - r1y, v_dest.getZ() - r1z);

        double radius = r1.length();

        tf::Vector3 n = v_dest.cross(2*r1).normalize();
        tf::Vector3 u = r1.normalized();
        tf::Vector3 v_r = r1;

        ROS_INFO_STREAM("u " << u.getX() << " " << u.getY() << " " << u.getZ());
        ROS_INFO_STREAM("n " << n.getX() << " " << n.getY() << " " << n.getZ());
        ROS_INFO_STREAM("r1 " << r1x << " " << r1y << " " << r1z);
        ROS_INFO_STREAM("r2 " << r2.getX() << " " << r2.getY() << " " << r2.getZ());
        ROS_INFO_STREAM("radius " << radius);

        // get theta for head coordinates
        perspective_taking_python::CircleEquation ce;
        ce.request.radius = radius;
        tf::vector3TFToMsg(v_dest, ce.request.h);
        tf::vector3TFToMsg(n, ce.request.n);
        tf::vector3TFToMsg(u, ce.request.u);
        tf::vector3TFToMsg(v_r, ce.request.c);
        if(equationSolverClient_.call(ce)) {
            ROS_INFO_STREAM("response: " << ce.response.theta);
        } else {
            ROS_ERROR("Did not get response from equationSolverClient!");
            return;
        }

        double delta = 0.01; // approximately 10 steps at 1.3 meters distance to human (straight ahead)
        double epsilon = M_PI * delta / radius;
        double resp_theta = ce.response.theta;
        ROS_INFO_STREAM("epsilon: " << epsilon);

        for(double theta = -1.0, step=0; theta<=resp_theta; theta+=epsilon, step+=1) {
            double b = 1.0/(resp_theta+1.0);
            if(transform_source_target.getRotation().getAngle()>M_PI) {
                b = -1.0*b;
            }
            double theta_for_q = b * theta + b;
            tf::Quaternion q_intermediate = q_origin.slerp(q_dest, theta_for_q);
            //tf::Quaternion q_intermediate = q_origin.slerp(q_dest, 1.0-theta);
            //tf::Vector3 v_intermediate    = v_origin.lerp(v_dest, ration);
            tf::Vector3 v_intermediate    = radius*cos(M_PI*theta)*u + radius*sin(M_PI*theta)*n.cross(u)+v_r;

            ROS_INFO_STREAM("theta_for_q: " << theta_for_q);
            ROS_INFO_STREAM("theta: " << theta << ": " << v_intermediate.getX() << " " << v_intermediate.getY() << " " << v_intermediate.getZ());

            tf::Transform intermediate_transform;
            intermediate_transform.setOrigin(v_intermediate);
            intermediate_transform.setRotation(q_intermediate);
            tf::Transform intermediate_transform_inv = intermediate_transform.inverse();

            std::stringstream ss; ss << std::fixed << setprecision(2) << step;
            br.sendTransform(tf::StampedTransform(intermediate_transform_inv, ros::Time::now(), source_frame, "rot"+ss.str()));

            ROS_INFO_STREAM("step: " << step);
        }

        pcl::PointCloud<pcl::PointXYZRGB> output;
        pcl_ros::transformPointCloud(target_frame, cloud_in_source_frame, output, listener);
        transformedCloudPublisher_.publish(output);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mental_perspective_transformer");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    if(!nhPriv.hasParam("fovy")) {
        nhPriv.setParam("fovy", 90.0);
    }

    MentalPerspectiveTransformer transform_node(nh, nhPriv);
    (void) transform_node;

    TransformedPerspectiveVisualizer vis_node(nh, nhPriv, "human_perspective");
    (void) vis_node;

    ros::spin();

    return 0;
}

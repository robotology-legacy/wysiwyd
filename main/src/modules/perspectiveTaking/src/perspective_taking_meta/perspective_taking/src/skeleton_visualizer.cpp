/*
 * This is a visualizer of the skeleton data acquired by the Kinect
 * using openni_tracker. It shows the skeleton overlaid on the point cloud.
 */

// ROS includes
#include <string>
#include <fstream>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>

// Project includes
#include "point_cloud_visualizer.h"
#include "vpt_helpers.h"

class SkeletonVisualizer : public PointCloudVisualizer {
public:
    SkeletonVisualizer(ros::NodeHandle &nh, ros::NodeHandle& privNh)
        : PointCloudVisualizer(nh, privNh, privNh.param<std::string>("point_cloud_topic", "/camera/depth_registered/points")),
          info_depth_sub(nh.subscribe<sensor_msgs::CameraInfo >(
                                                 "/camera/depth_registered/camera_info",
                                                 1,
                                                 &SkeletonVisualizer::callbackCameraInfo,
                                                 this) ),
          bodyParts {
              { "head_1", "left_elbow_1",  "left_foot_1",  "left_hand_1",  "left_hip_1",  "left_knee_1",  "left_shoulder_1",
                "neck_1", "right_elbow_1", "right_foot_1", "right_hand_1", "right_hip_1", "right_knee_1", "right_shoulder_1", "torso_1" }
          }, connections {
              { "head_1", "neck_1" },
              { "neck_1", "left_shoulder_1" }, { "neck_1", "right_shoulder_1" },
              { "left_shoulder_1", "left_elbow_1" }, { "right_shoulder_1", "right_elbow_1" },
              { "left_elbow_1", "left_hand_1" }, { "right_elbow_1", "right_hand_1" },
              //{ "left_shoulder_1", "torso_1" }, { "right_shoulder_1", "torso_1" },
              { "neck_1", "torso_1" },
              { "torso_1", "left_hip_1" }, { "torso_1", "right_hip_1" },
              { "left_hip_1", "left_knee_1" }, { "right_hip_1", "right_knee_1" },
              //{ "left_hip_1", "right_hip_1" },
              { "left_knee_1", "left_foot_1" }, { "right_knee_1", "right_foot_1" }
          }, nodes {
            { "head_1", "head_1" }, { "neck_1", "neck_1" }, { "torso_1", "torso_1" },
            { "left_foot_1",  "left_foot_1"},  { "left_hand_1",  "left_hand_1" },  { "left_shoulder_1",  "left_elbow_1" },  { "left_hip_1",  "left_knee_1" },
            { "right_foot_1", "right_foot_1"}, { "right_hand_1", "right_hand_1" }, { "right_shoulder_1", "right_elbow_1" }, { "right_hip_1", "right_knee_1" }
          }, joints {
            { "head_1", "neck_1" }, { "neck_1", "torso_1" },
            { "left_elbow_1",  "left_elbow_1" },  { "left_hip_1",  "left_hip_1" },  {"left_knee_1",  "left_knee_1"},  {"left_shoulder_1",  "left_shoulder_1" },
            { "right_elbow_1", "right_elbow_1" }, { "right_hip_1", "right_hip_1" }, {"right_knee_1", "right_knee_1"}, {"right_shoulder_1", "right_shoulder_1" }
          },
          retrieved_info(false),
          outfile("trial/joints.txt", ios::out | ios::binary),
          frame_nb(0),
          P(3, 4)
    {
        ROS_DEBUG("Initialize SkeletonVisualizer");
        if(!privNh.param<bool>("reset", false)) {
            viewer->loadCameraParameters("cameraParameters.cam");
        }
    }

    ~SkeletonVisualizer() {
        viewer->saveCameraParameters("cameraParameters.cam");
        outfile.close();
    }

protected:
    void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& info_depth_msg) {
        for(int i=0; i<=2; i++) {
            for(int j=0; j<=3; j++) {
                P(i, j) = info_depth_msg->P[i*4+j];
            }
        }

        retrieved_info = true;
        info_depth_sub.shutdown();
    }

    void updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
        if(!retrieved_info) {
            ROS_WARN("Camera info not yet retrieved, waiting");
            ros::Duration(1.0).sleep();
            return;
        }
        frame_nb++;
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->removePointCloud(cloud_name);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cloud_name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualizer_point_size, cloud_name);

        static tf::TransformListener listener;

        std::map<std::string, geometry_msgs::PointStamped> bodyPartsTransformed;

        for(auto bodyPart : bodyParts) {
            try {
                geometry_msgs::PointStamped origin; origin.header.frame_id = "/"+bodyPart; origin.point.x = 0.0; origin.point.y = 0.0; origin.point.z = 0.0;
                bodyPartsTransformed[bodyPart] = geometry_msgs::PointStamped();
                bodyPartsTransformed[bodyPart].header.frame_id = cloud->header.frame_id;
                //ROS_INFO_STREAM("Transform from " << origin.header.frame_id << " to " << bodyPartsTransformed[bodyPart].header.frame_id);
                listener.waitForTransform(origin.header.frame_id, bodyPartsTransformed[bodyPart].header.frame_id, ros::Time(0), ros::Duration(1.0));
                listener.transformPoint(bodyPartsTransformed[bodyPart].header.frame_id, origin, bodyPartsTransformed[bodyPart]);
                bodyPartsTransformed[bodyPart].point.z -= 0.15;
            } catch (tf::TransformException ex) {
              ROS_ERROR("%s",ex.what());
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr poly (new pcl::PointCloud<pcl::PointXYZ>);
        poly->points.resize(3);

        // draw connections
        for(auto connection : connections) {
            auto connectionname = connection.first+"2"+connection.second;
            //ROS_INFO_STREAM("Add arrow " << connectionname);
            viewer->removeShape(connectionname);
            auto from = vpt::ros_to_pcl(bodyPartsTransformed[connection.first]);
            auto to = vpt::ros_to_pcl(bodyPartsTransformed[connection.second]);
            viewer->addLine(from, to, 0.0, 1.0, 0.0, connectionname);
            viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, connectionname);
        }

        // draw nodes
        for(auto node : nodes) {
            auto nodename = node.first+node.second;
            viewer->removeShape(nodename);
            auto p = vpt::eigen_to_pcl((vpt::ros_to_eigen(bodyPartsTransformed[node.first]) + vpt::ros_to_eigen(bodyPartsTransformed[node.second])) / 2.0);
            auto d = 0.015;
            viewer->addCube(p.x-d, p.x+d, p.y-d, p.y+d, p.z-d, p.z+d, 1.0, 0.0, 0.0, nodename);
            viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, nodename);
        }

        // draw joints
        for(auto joint : joints) {
            auto jointname = joint.first+joint.second;
            viewer->removeShape(jointname);
            auto p = vpt::eigen_to_pcl((vpt::ros_to_eigen(bodyPartsTransformed[joint.first]) + vpt::ros_to_eigen(bodyPartsTransformed[joint.second])) / 2.0);
            auto d = 0.025;

            viewer->addSphere (p, d, 1.0, 1.0, 0.0, jointname);
        }

        // write screenshot
        viewer->saveScreenshot("trial/"+std::to_string(frame_nb)+".png");

        // write joints.txt
        unsigned int bodyPart_nr = 1;
        for(auto bodyPart : bodyParts) {
            std::vector<pcl::visualization::Camera> cam; viewer->getCameras(cam);
            Eigen::Vector4d bp_window;
            cam.at(0).cvtWindowCoordinates(vpt::ros_to_pcl(bodyPartsTransformed[bodyPart]), bp_window);
            auto x_2d_win = (int)bp_window[0];;
            auto y_2d_win = 480-(int)bp_window[1];

            auto uvw = P * vpt::ros_to_eigen(bodyPartsTransformed[bodyPart]);
            auto x_2d_org = (int)(uvw[0]/uvw[2]);
            auto y_2d_org = (int)(uvw[1]/uvw[2]);

            ROS_INFO_STREAM(bodyPart << " " << x_2d_org << " " << y_2d_org << " Kinect coordinates");
            ROS_INFO_STREAM(bodyPart << " " << x_2d_win << " " << y_2d_win << " Window Coordinates");

            auto p = vpt::ros_to_pcl(bodyPartsTransformed[bodyPart]);

            outfile << frame_nb << "\t" << bodyPart_nr << "\t";
            outfile << p.x << "\t" << p.y << "\t" << p.z << "\t";
            outfile << x_2d_org << "\t" << y_2d_org << "\t";
            outfile << x_2d_win << "\t" << y_2d_win << "\n";

            bodyPart_nr++;
        }
    }

private:
    std::vector<std::pair<std::string, std::string>> connections;
    std::vector<std::pair<std::string, std::string>> nodes;
    std::vector<std::pair<std::string, std::string>> joints;

    std::vector<std::string> bodyParts;
    const std::string cloud_name="skeleton_cloud";
    const int visualizer_point_size=5;
    ros::Subscriber info_depth_sub;
    bool retrieved_info;
    std::ofstream outfile;
    unsigned int frame_nb;
    Eigen::MatrixXf P;
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "skeleton_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    ROS_INFO("Start skeleton visualizer");

    SkeletonVisualizer node(nh, nhPriv);
    (void) node;

    ros::spin();

    return 0;
}

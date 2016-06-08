// General includes
#include <iostream>

// ROS includes
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>

// YARP includes
#include <wrdac/clients/opcClient.h>

// Project includes
#include "perspective_taking/point_cloud_visualizer.h"
#include "perspective_taking/vpt_helpers.h"
#include "raytracing.h"

class Raytracer : public PointCloudVisualizer {
public:
    Raytracer(ros::NodeHandle &nh, const ros::NodeHandle& privNh, const std::string& pt_cloud_topic) :
            PointCloudVisualizer(nh, privNh, pt_cloud_topic),
            head_offset(tf::Vector3(0.10, 0.0, -0.05)),
            object_offset(tf::Vector3(0.0, 0.0, 0.0)),
            voxel_leaf_size(0.03),
            opc(new wysiwyd::wrdac::OPCClient(ros::this_node::getName().substr(1)))
    {
        ROS_DEBUG("Initialize VisualizePartnerPerspective");
        privNh.getParam("voxel_leaf_size", voxel_leaf_size);

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

    ~Raytracer() {
        if (yarp::os::Network::checkNetwork()) {
            opc->interrupt();
            opc->close();
        }
    }

    void updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

private:
    void doRaytrace(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

    tf::Vector3 head_offset;
    tf::Vector3 object_offset;
    double voxel_leaf_size;

    boost::shared_ptr<wysiwyd::wrdac::OPCClient> opc;

    std::vector<std::string> shape_list;

    static const std::string cloud_name;
    static const int visualizer_point_size=3;
#if __cplusplus > 199711L
    constexpr static const double obj_size=0.05;
    constexpr static const double traversal_sphere_size=0.02;
    constexpr static const float theta=4.0;
#else
    static const double obj_size=0.05;
    static const double traversal_sphere_size=0.02;
    static const float theta=4.0;
#endif
};

const std::string Raytracer::cloud_name="partner_view_cloud";

void Raytracer::updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
    viewer->spinOnce();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->removePointCloud(cloud_name);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cloud_name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualizer_point_size, cloud_name);

    viewer->spinOnce();
    doRaytrace(cloud);
    viewer->spinOnce();
}

void Raytracer::doRaytrace(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
    static tf::TransformListener listener;

    RayTracing<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    sor.initializeVoxelGrid();
    ROS_INFO_STREAM("Cloud size: " << sor.getFilteredPointCloud().size());
    if(sor.getFilteredPointCloud().empty()) {
        return;
    }
    viewer->spinOnce();

    std::string target_frame_cloud=cloud->header.frame_id;

    geometry_msgs::PointStamped head_cloud_frame;
    try {
        // get head position in target_frame, needed because of nose tip
        std::string head_frame = "head_origin1";
        geometry_msgs::PointStamped head_head_frame;
        head_head_frame.header.frame_id = head_frame;
        tf::pointTFToMsg(head_offset, head_head_frame.point);
        listener.waitForTransform(head_frame, target_frame_cloud, ros::Time::now(), ros::Duration(0.1));
        listener.transformPoint(target_frame_cloud, head_head_frame, head_cloud_frame);
    } catch (tf::TransformException ex) {
        ROS_WARN_STREAM("Head coordinate transformation could not be obtained, return!");
        return;
    }

    // delete old shapes
    for (size_t i=0; i<shape_list.size(); i++) {
        viewer->removeShape(shape_list.at(i));
    }
    shape_list.clear();

    // check all frames which are known to tf
    std::vector<std::string> frames;
    listener.getFrameStrings(frames);

    for(size_t i=0; i<frames.size(); i++) {
        std::string obj_name = frames.at(i);
        if(obj_name.find("obj_")!=std::string::npos) {
            geometry_msgs::PointStamped object_cloud_frame;
            try {
                geometry_msgs::PointStamped object_object_frame;
                // get object coordinates in target_frame, needed because of offset
                object_object_frame.header.frame_id = obj_name;
                tf::pointTFToMsg(object_offset, object_object_frame.point);

                listener.waitForTransform(obj_name, target_frame_cloud, ros::Time::now(), ros::Duration(0.1));
                listener.transformPoint(target_frame_cloud, object_object_frame, object_cloud_frame);
            } catch (tf::TransformException ex) {
                ROS_WARN_STREAM("Skip object " << obj_name << " as point could not be transformed!");
                continue;
            }

            // draw object in visualizer
            // get random color, but consistent for one object
            boost::hash<std::string> string_hash;
            srand(string_hash(obj_name));
            double obj_r = double(rand()) / double(RAND_MAX);
            double obj_g = double(rand()) / double(RAND_MAX);
            double obj_b = double(rand()) / double(RAND_MAX);

            viewer->addSphere(vpt::ros_to_pcl(object_cloud_frame.point),
                              obj_size,
                              obj_r, obj_g, obj_b,
                              obj_name);
            shape_list.push_back(obj_name);

            // do level 1 perspective taking
            int is_occluded=-1;
            std::vector <Eigen::Vector3i> out_ray;
            sor.rayTraceWrapper(is_occluded,
                                vpt::ros_to_eigen(object_cloud_frame.point),
                                vpt::ros_to_eigen(head_cloud_frame.point),
                                theta,
                                out_ray);

            wysiwyd::wrdac::Relation r_occluded(obj_name, "occluded");
            wysiwyd::wrdac::Relation r_visible(obj_name, "visible");

            if(is_occluded) {
                ROS_INFO_STREAM("Object " << obj_name << " is occluded from partner.");
                if(opc->isConnected()) {
                    wysiwyd::wrdac::Agent* partner = opc->addOrRetrieveEntity<wysiwyd::wrdac::Agent>("partner");
                    partner->addBelief(r_occluded);
                    partner->removeBelief(r_visible);
                    opc->commit(partner);
                }
            } else {
                ROS_INFO_STREAM("Object " << obj_name << " is visible to partner.");
                if(opc->isConnected()) {
                    wysiwyd::wrdac::Agent* partner = opc->addOrRetrieveEntity<wysiwyd::wrdac::Agent>("partner");

                    partner->addBelief(r_visible);
                    partner->removeBelief(r_occluded);

                    opc->commit(partner);
                }
            }

            double dist=(vpt::ros_to_eigen(object_cloud_frame.point)-vpt::ros_to_eigen(head_cloud_frame.point)).norm();

            ROS_INFO_STREAM("Raytrace counter: " << out_ray.size());
            ROS_INFO_STREAM("Distance to object: " << dist);
            for(int i=0; i<out_ray.size(); i++) {
                pcl::PointXYZ traversed_pcl = vpt::eigen_to_pcl(sor.getCentroidCoordinate(out_ray.at(i)));
                std::string traversal_sphere_name = obj_name+boost::lexical_cast<std::string>(i);

                viewer->addSphere(traversed_pcl,
                                  traversal_sphere_size,
                                  obj_r, obj_g, obj_b,
                                  traversal_sphere_name);
                shape_list.push_back(traversal_sphere_name);
            }
        }
    }
}

int main (int argc, char** argv) {
    if (argc < 2) {
        ROS_ERROR("Not enough arguments: raytracer <pointcloud_topic>");
        return -1;
    }

    ros::init(argc, argv, "raytracer");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    if(!nhPriv.hasParam("fovy")) {
        nhPriv.setParam("fovy", 50.0);
    }

    yarp::os::Network yarp;
    (void) yarp;

    std::string pointcloud_topic = argv[1];
    Raytracer node(nh, nhPriv, pointcloud_topic);
    (void) node;

    ros::spin();

    return 0;
}

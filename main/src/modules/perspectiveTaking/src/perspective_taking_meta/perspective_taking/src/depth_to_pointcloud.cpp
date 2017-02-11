/*
 * This creates a PCL point cloud from the kinectWrapperClient
 */

#include <yarp/os/Network.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Image.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <kinectWrapper/kinectWrapper_client.h>
#include <pcl/common/eigen.h>
#include <tf/transform_broadcaster.h>
#include <wrdac/clients/opcClient.h>
#include "vpt_helpers.h"

class DepthToPointCloud {
public:
    DepthToPointCloud(ros::NodeHandle &nh, ros::NodeHandle& privNh);
    virtual ~DepthToPointCloud();
protected:

private:
    yarp::sig::Matrix icub2kinect;
    ros::Publisher pointcloudPublisher;
    kinectWrapper::KinectWrapperClient client;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> rgb;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> depth;
    ros::Timer updateVisualizerTimer;
    wysiwyd::wrdac::OPCClient opc;
    double focallength;
    tf::TransformBroadcaster br;

    void callbackTimer(const ros::TimerEvent&);
    void connectToWrapper();
    void connectToOPC();
    void connectToRFH();
    void sendHeadTransform();
    void sendCameraTransform();
    void sendRobotTransform();
    void publishCloud();
};

void DepthToPointCloud::connectToWrapper() {
    std::string clientName = ros::this_node::getName().substr(1) + "/kinect";
    int verbosity = 0;

    yarp::os::Property options;
    options.put("carrier","tcp");
    options.put("remote","kinectServer");
    options.put("local",clientName.c_str());
    options.put("verbosity",verbosity);

    if (!client.open(options)) {
        ROS_ERROR("Could not connect to Kinect Wrapper");
        ros::shutdown();
    }

    yarp::os::Property opt;
    client.getInfo(opt);

    int img_width=opt.find("img_width").asInt();
    int img_height=opt.find("img_height").asInt();
    int depth_width=opt.find("depth_width").asInt();
    int depth_height=opt.find("depth_height").asInt();

    rgb.resize(img_width, img_height);
    depth.resize(depth_width,depth_height);

    if (!client.getFocalLength(focallength)) {
        ROS_ERROR("Could not get focal length from Kinect Wrapper");
        ros::shutdown();
    }
}

void DepthToPointCloud::connectToOPC() {
    while (!opc.connect("OPC")) {
        ROS_INFO("Waiting for connection to OPC");
        ros::Duration(1.0).sleep();
    }
}

void DepthToPointCloud::connectToRFH() {
    std::string rfhLocal = ros::this_node::getName() + "/rfh:o";
    std::string rfhRemote = "/referenceFrameHandler/rpc";

    yarp::os::RpcClient rfh;
    rfh.open(rfhLocal.c_str());

    while (!yarp::os::Network::connect(rfhLocal.c_str(),rfhRemote.c_str())) {
        ROS_INFO("Waiting connection to RFH...");
        ros::Duration(1.0).sleep();
    }

    if (rfh.getOutputCount()>0)
    {
        yarp::os::Bottle bCmd,reply;
        bCmd.addString("mat");
        bCmd.addString("icub");
        bCmd.addString("kinect");
        rfh.write(bCmd,reply);

        if (reply.get(0)!="nack")
        {
            if (yarp::os::Bottle *bMat=reply.get(1).asList())
            {
                icub2kinect.resize(4,4);
                for (int i=0; i<4; i++)
                    for (int j=0; j<4; j++)
                        icub2kinect(i,j)=bMat->get(4*i+j).asDouble();
            }
        }
    }

    rfh.close();
}

DepthToPointCloud::DepthToPointCloud(ros::NodeHandle &nh, ros::NodeHandle& privNh) :
    updateVisualizerTimer(nh.createTimer(ros::Duration(0.03), &DepthToPointCloud::callbackTimer, this)),
    pointcloudPublisher(nh.advertise<sensor_msgs::PointCloud2>(
                        privNh.param<std::string>("pt_cloud_topic", "/pointcloud_from_yarp"),
                        1)),
    opc(ros::this_node::getName().substr(1))
{
    connectToWrapper();
    connectToOPC();
    connectToRFH();
}

DepthToPointCloud::~DepthToPointCloud() {
    client.close();
    opc.close();
}

void DepthToPointCloud::sendCameraTransform() {
    // transform world->camera_link
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_link"));
}

void DepthToPointCloud::sendRobotTransform() {
    // transform camera_link->robot_root
    yarp::sig::Vector kOrigin = yarp::sig::Vector(3, 0.0);
    kOrigin.push_back(1.0);
    yarp::sig::Vector v = yarp::math::operator*(icub2kinect, kOrigin);

    tf::Transform transform_robot;
    transform_robot.setOrigin( tf::Vector3(v[2], v[0], v[1]) );
    transform_robot.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform_robot, ros::Time::now(), "camera_link", "robot_root"));
}

void DepthToPointCloud::sendHeadTransform() {
    opc.checkout();
    std::list<wysiwyd::wrdac::Entity*> lEntities = opc.EntitiesCache();
    for (auto& entity : lEntities) {
        if (entity->entity_type() == "agent") {
            wysiwyd::wrdac::Agent* a = dynamic_cast<wysiwyd::wrdac::Agent*>(entity);
            if(a && a->m_present==1.0) {
                auto agent_pos_yarp = a->m_ego_position;
                tf::Vector3 agent_pos = vpt::yarp_to_tf_converter(agent_pos_yarp);
                agent_pos[0] = agent_pos[0]-0.3;

                tf::Transform transform;
                transform.setOrigin(agent_pos);
                transform.setRotation(tf::Quaternion(0, 0, 1, 0));

                ROS_DEBUG_STREAM("Send transform for head_origin1" << ": " << std::endl << agent_pos);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_root", "head_origin1"));
            }
        }
    }
}

void DepthToPointCloud::publishCloud() {
    bool newDepthFetched = client.getDepth(depth);
    bool newRGBFetched = client.getRgb(rgb);
    if(newDepthFetched && newRGBFetched) {
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
        pointcloud.header.frame_id = "camera_link";
        pointcloud.width  = depth.width();
        pointcloud.height = depth.height();
        pointcloud.points.resize (pointcloud.height * pointcloud.width);
        int depth_idx = 0;
        int factor = rgb.width() / depth.width();
        int cx = pointcloud.width/2.0;
        int cy = pointcloud.height/2.0;
        for(int i=0; i<depth.width(); i++) {
            for(int j=0; j<depth.height(); j++) {
                yarp::sig::PixelRgb& pixel_color    = rgb.pixel(i*factor,j*factor);
                yarp::sig::PixelMono16& pixel_depth = depth.pixel(i,j);
                pcl::PointXYZRGB& pt = pointcloud.points[depth_idx++];
                double depth_scaled = pixel_depth / 1000.f;
                pt.x = depth_scaled;
                pt.y = (i - cx) * depth_scaled / focallength;
                pt.z = -1.0*(j - cy) * depth_scaled / focallength;
                pt.r = pixel_color.r;
                pt.g = pixel_color.g;
                pt.b = pixel_color.b;
            }
        }
        pointcloudPublisher.publish(pointcloud);
    }
}

void DepthToPointCloud::callbackTimer(const ros::TimerEvent&) {
    sendCameraTransform();
    sendRobotTransform();
    publishCloud();
    sendHeadTransform();
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "depth_to_pointcloud");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout, "Yarp network not available\n");
        return -1;
    }

    DepthToPointCloud c(nh, nhPriv);
    (void) c;

    ros::spin();

    return 0;
}

/*
 * This publishes images from a point cloud topic to a chosen image topic
 * This is used to publish images to the ABM.
 */

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "point_cloud_visualizer.h"

class ScreenshotPublisher : public PointCloudVisualizer {
public:
    ScreenshotPublisher(ros::NodeHandle &nh,
                        ros::NodeHandle& privNh)
            : PointCloudVisualizer(nh, privNh, privNh.param<std::string>("point_cloud_topic", "/camera/depth_registered/points")),
              it(nh),
              pub(it.advertise(privNh.param<std::string>("screenshottopic", "partnerperspective2d"), 100)),
              schedule_screenshot(false),
              screenshot_timer(nh.createTimer(ros::Duration(0.1), &ScreenshotPublisher::screenshotCallback, this))
    {
        ROS_DEBUG("Initialize ScreenshotPublisher");
    }

protected:
    void updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
    const std::string cloud_name="screenshot_cloud";
    const int visualizer_point_size=3;

private:
    image_transport::ImageTransport it;
    image_transport::Publisher pub;

    bool schedule_screenshot;
    ros::Timer screenshot_timer;

    void screenshotCallback(const ros::TimerEvent&) {
        schedule_screenshot = true;
    }
};

void ScreenshotPublisher::updateVis(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->removePointCloud(cloud_name);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, cloud_name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualizer_point_size, cloud_name);

    if(schedule_screenshot) {
        viewer->saveScreenshot("/tmp/" + ros::this_node::getName() + ".png");
        cv::Mat screen = cv::imread("/tmp/" + ros::this_node::getName() + ".png", CV_LOAD_IMAGE_COLOR);
        if(!screen.data) {
            ROS_ERROR_STREAM("Could not load " << "/tmp/" << ros::this_node::getName() << ".png");
        } else {
            //cv::imshow("Partner Perspective", screen);
            //cv::waitKey(50);
            sensor_msgs::ImagePtr partner_message =
                    cv_bridge::CvImage(std_msgs::Header(), "bgr8", screen).toImageMsg();
            pub.publish(partner_message);
        }
        schedule_screenshot = false;
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "publish_screenshot");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    if(!nhPriv.hasParam("fovy")) {
        nhPriv.setParam("fovy", 90.0);
    }

    ScreenshotPublisher node(nh, nhPriv);
    (void) node;

    ros::spin();

    return 0;
}

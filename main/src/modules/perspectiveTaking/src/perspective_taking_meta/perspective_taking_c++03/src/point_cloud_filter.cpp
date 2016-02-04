#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/publisher.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

class PointCloudFilter {
public:
    PointCloudFilter(ros::NodeHandle &nh, const ros::NodeHandle& privNh, const std::string& input_cloud, const std::string& output_cloud) :
    pointCloudSubscriber(nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(
                         input_cloud,
                         1,
                         &PointCloudFilter::callbackPointCloud,
                         this) ),
    pointCloudPublisher(nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > (output_cloud, 100)){
    }

    void callbackPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud_z(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud_xz(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Filter object.
        /*pcl::PassThrough<pcl::PointXYZRGB> filter_z;
        filter_z.setInputCloud(cloud);
        // Filter out all points with Z values not in the [0-2] range.
        filter_z.setFilterFieldName("z");
        filter_z.setFilterLimits(0.0, 2.7);
        filter_z.filter(*filteredCloud_z);

        pcl::PassThrough<pcl::PointXYZRGB> filter_x;
        filter_x.setInputCloud(filteredCloud_z);
        filter_z.setFilterFieldName("x");
        filter_z.setFilterLimits(-0.5, 0.5);
        filter_z.filter(*filteredCloud_xz);*/

        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZRGB>);
        condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0.0)));
        condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 3.0)));
        condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, -0.75)));
        condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, 0.75)));
        condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, 0.75)));

        pcl::ConditionalRemoval<pcl::PointXYZRGB> filter;
        filter.setCondition(condition);
        filter.setInputCloud(cloud);
        filter.setKeepOrganized(true);
        filter.setUserFilterValue(0.0);
        filter.filter(*filteredCloud_xz);

        filteredCloud_xz->header.frame_id = cloud->header.frame_id;
        pointCloudPublisher.publish(*filteredCloud_xz);
    }

protected:

private:
    ros::Subscriber pointCloudSubscriber;
    ros::Publisher pointCloudPublisher;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_filter");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    ROS_INFO("Start point cloud filter");

    PointCloudFilter node(nh, nhPriv,
                          nhPriv.param<std::string>("input_cloud", "/camera/depth_registered/points"),
                          nhPriv.param<std::string>("output_cloud", "/depth_filtered_cloud"));
    (void) node;

    ros::spin();

    return 0;
}



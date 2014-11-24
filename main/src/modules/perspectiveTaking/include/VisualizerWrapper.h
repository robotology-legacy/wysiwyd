#ifndef VISUALIZER_WRAPPER_H
#define VISUALIZER_WRAPPER_H

#include <string>
#include <memory>
#include <map>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <rtabmap/core/Transform.h>

using namespace rtabmap;

class VColor {
public:
    VColor();
    VColor(unsigned char red, unsigned char green, unsigned char blue) :
        r(red),
        g(green),
        b(blue)
    {
    }
    virtual ~VColor() {}
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

class VisualizerWrapper
{
public:
    VisualizerWrapper();
    virtual ~VisualizerWrapper();

    const std::map<std::string, Transform> & getAddedClouds() {return _addedClouds;}

    void setBackgroundColor(const VColor &color);

    bool getPose(const std::string & id, Transform & pose); //including meshes

    bool updateCloudPose(
            const std::string & id,
            const Transform & pose); //including mesh

    void setCloudVisibility(const std::string & id, bool isVisible);

    void removeAllClouds(); //including meshes

    bool removeCloud(const std::string & id); //including mesh

    bool updateCloud(
            const std::string & id,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const Transform & pose = Transform::getIdentity());

    bool updateCloud(
            const std::string & id,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
            const Transform & pose = Transform::getIdentity());

    bool addOrUpdateCloud(
            const std::string & id,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const Transform & pose = Transform::getIdentity(),
            const VColor & color = _vgrey);

    bool addOrUpdateCloud(
            const std::string & id,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
            const Transform & pose = Transform::getIdentity(),
            const VColor & color = _vgrey);

    bool addCloud(
            const std::string & id,
            const pcl::PCLPointCloud2Ptr & binaryCloud,
            const Transform & pose,
            bool rgb,
            const VColor & color = _vgrey);

    bool addCloud(
            const std::string & id,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const Transform & pose = Transform::getIdentity(),
            const VColor & color = _vgrey);

    bool addCloud(
            const std::string & id,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
            const Transform & pose = Transform::getIdentity(),
            const VColor & color = _vgrey);

    void updateCameraPosition(
            const Transform & pose);

    pcl::visualization::PCLVisualizer& getVisualizer() {
        return *_visualizer;
    }

private:
    std::map<std::string, Transform> _addedClouds;
    unsigned int _maxTrajectorySize;
    Transform _lastPose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _trajectory;
    static const VColor _vgrey;
    pcl::visualization::PCLVisualizer* _visualizer;
};

#endif

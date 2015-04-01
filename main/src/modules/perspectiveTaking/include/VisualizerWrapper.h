/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef VPT_VISUALIZER_WRAPPER
#define VPT_VISUALIZER_WRAPPER

#include <QVTKWidget.h>
#include <QtCore/qnamespace.h>
#include <map>
#include <boost/shared_ptr.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <rtabmap/core/Transform.h>

class cloudWithPose {
public:
    cloudWithPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c, rtabmap::Transform p) :
        cloud(c),
        pose(p) {}

    virtual ~cloudWithPose() {}

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    rtabmap::Transform pose;
};

class VisualizerWrapper : public QVTKWidget {
    Q_OBJECT

public:
    VisualizerWrapper(QWidget *parent = 0, Eigen::Matrix4f cloudTransform = Eigen::Matrix4f::Identity());
    virtual ~VisualizerWrapper();

    const std::map<std::string, boost::shared_ptr<cloudWithPose> > & getAddedClouds() const {return _addedClouds;}

    void setBackgroundColor(const double r, const double g, const double b) {
        _visualizer->setBackgroundColor(r, g, b);
    }

    static rtabmap::Transform transformFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    bool getPose(const std::string & id, rtabmap::Transform & pose); //including meshes

    bool updateCloudPose(
            const std::string & id,
            const rtabmap::Transform & pose); //including mesh

    void setCloudVisibility(const std::string & id, bool isVisible);

    void removeAllClouds() { //including meshes
        _addedClouds.clear();
        _visualizer->removeAllPointClouds();
    }

    bool removeCloud(const std::string & id); //including mesh

    bool updateCloud(
            const std::string & id,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const rtabmap::Transform & pose = rtabmap::Transform::getIdentity());

    bool addOrUpdateCloud(
            const std::string & id,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const rtabmap::Transform & pose = rtabmap::Transform::getIdentity());

    bool addCloud(
            const std::string & id,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const rtabmap::Transform & pose = rtabmap::Transform::getIdentity());

    void updateCameraPosition(
            const rtabmap::Transform & pose);

    pcl::visualization::PCLVisualizer& getVisualizer() {
        return *_visualizer;
    }

private:
    std::map<std::string, boost::shared_ptr<cloudWithPose> > _addedClouds;
    unsigned int _maxTrajectorySize;
    rtabmap::Transform _lastPose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _trajectory;
    pcl::visualization::PCLVisualizer* _visualizer;
    Eigen::Matrix4f _viewportTransform;
};

#endif

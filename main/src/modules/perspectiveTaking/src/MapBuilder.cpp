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
 *
 * Massively inspired by https://github.com/introlab/rtabmap/blob/master/examples/RGBDMapping/MapBuilder.h!
*/

#include <string>
#include <map>
#include <boost/thread/mutex.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

#include <rtabmap/core/util3d.h>

#include <rtabmap/utilite/UConversion.h>

#include "PerspectiveTaking.h"
#include "VisualizerWrapper.h"
#include "MapBuilder.h"

using namespace rtabmap;

// This class receives RtabmapEvent and construct/update a 3D Map
MapBuilder::MapBuilder(unsigned int decOdo,
                       unsigned int decVis,
                       Eigen::Matrix4f robotTransform,
                       Eigen::Matrix4f partnerTransform) :
    doProcessStats(true),
    _vWrapper_robot(new VisualizerWrapper(this, robotTransform)),
    _vWrapper_partner(new VisualizerWrapper(this, partnerTransform)),
    decimationOdometry_(decOdo),
    decimationStatistics_(decVis),
    odometryCorrection_(Transform::getIdentity())
{
    this->setWindowFlags(Qt::Dialog);
    this->setWindowTitle(tr("Perspective Taking"));
    this->setMinimumWidth(1600);
    this->setMinimumHeight(600);

    QHBoxLayout *layout = new QHBoxLayout();
    layout->addWidget(_vWrapper_robot);
    layout->addWidget(_vWrapper_partner);
    this->setLayout(layout);

    qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");
    qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");
}

MapBuilder::~MapBuilder() {
    this->unregisterFromEventsManager();
    delete _vWrapper_robot;
    delete _vWrapper_partner;
}

void MapBuilder::setCameraPosition(double pos_x, double pos_y, double pos_z,
                                   double view_x, double view_y, double view_z,
                                   double up_x, double up_y, double up_z,
                                   const std::string &visualizerName) {
    vis_mutex.lock();
    VisualizerWrapper* vis = getVisualizerByName(visualizerName);
    if(vis) {
        vis->getVisualizer().setCameraPosition(pos_x, pos_y, pos_z,
                                               view_x, view_y, view_z,
                                               up_x, up_y, up_z);
    }
    vis_mutex.unlock();
}

cv::Mat MapBuilder::getScreen() {
    vis_mutex.lock();

    // create a pixmap which contains this (a QWidget)
    QPixmap pixmap(this->size());
    this->render(&pixmap);

    vis_mutex.unlock();

    // convert pixmap to a QImage, which can then be converted to a Mat
    QImage image = pixmap.toImage().convertToFormat(QImage::Format_RGB888).rgbSwapped();

    return cv::Mat( image.height(), image.width(), CV_8UC3, const_cast<uchar*>(image.bits()), image.bytesPerLine() ).clone();
}

void MapBuilder::setCameraFieldOfView(double fovy, const std::string &visualizerName) {
    vis_mutex.lock();
    VisualizerWrapper* vis = getVisualizerByName(visualizerName);
    if(vis) {
        vis->getVisualizer().setCameraFieldOfView(fovy);
    }
    vis_mutex.unlock();
}

void MapBuilder::processOdometry(const rtabmap::SensorData & data) {
    if(!data.depth().empty()) {
        perspectiveTaking::lastDepth = data.depth();
    }

    if(!this->isVisible())
    {
        return;
    }

    Transform pose = data.pose();

    if(pose.isNull()) {
        //Odometry lost
        _vWrapper_robot->setBackgroundColor(1.0, 0, 0);
        _vWrapper_partner->setBackgroundColor(1.0, 0, 0);
        pose = lastOdomPose_;
    } else {
        _vWrapper_robot->setBackgroundColor(0.1, 0.1, 0.1);
        _vWrapper_partner->setBackgroundColor(0.1, 0.1, 0.1);
    }
    if(!pose.isNull()) {
        lastOdomPose_ = pose;

        // 3d cloud
        if(data.depth().cols == data.image().cols &&
                data.depth().rows == data.image().rows &&
                !data.depth().empty() &&
                data.fx() > 0.0f &&
                data.fy() > 0.0f) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
                        data.image(),
                        data.depth(),
                        data.cx(),
                        data.cy(),
                        data.fx(),
                        data.fy(),
                        decimationOdometry_); // decimation // high definition
            if(cloud->size()) {
                cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);
                if(cloud->size()) {
                    cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, data.localTransform());
                }
            }

            if(!_vWrapper_robot->addOrUpdateCloud("cloudOdom", cloud, odometryCorrection_*pose)) {
                yError() << "Adding cloudOdom to viewer failed!";
            }
            if(!_vWrapper_partner->addOrUpdateCloud("cloudOdom", cloud, odometryCorrection_*pose)) {
                yError() << "Adding cloudOdom to viewer failed!";
            }
        }
        if(!data.pose().isNull()) { // NOT: pose.isNull()!!!
            _vWrapper_robot->updateCameraPosition(odometryCorrection_*data.pose());
            _vWrapper_partner->updateCameraPosition(odometryCorrection_*data.pose());
        }
    }

    _vWrapper_robot->update();
    _vWrapper_partner->update();
}

void MapBuilder::processStatistics(const rtabmap::Statistics & stats) {
    const std::map<int, Transform> & poses = stats.poses();
    for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter) {
        if(!iter->second.isNull()) {
            std::string cloudName = uFormat("cloud%d", iter->first);

            // 3d point cloud
            if(_vWrapper_robot->getAddedClouds().count(cloudName) && _vWrapper_partner->getAddedClouds().count(cloudName)) {
                // Update only if the pose has changed
                Transform tCloud_robot, tCloud_partner;
                _vWrapper_robot -> getPose(cloudName, tCloud_robot);
                _vWrapper_partner->getPose(cloudName, tCloud_partner);
                if(tCloud_robot.isNull() || iter->second != tCloud_robot) {
                    if(!_vWrapper_robot->updateCloudPose(cloudName, iter->second)) {
                        yError() << "Updating pose cloud " << iter->first << " failed!";
                    }
                }
                if(tCloud_robot.isNull() || iter->second != tCloud_partner) {
                    if(!_vWrapper_partner->updateCloudPose(cloudName, iter->second)) {
                        yError() << "Updating pose cloud " << iter->first << " failed!";
                    }
                }
                _vWrapper_robot->setCloudVisibility(cloudName, true);
                _vWrapper_partner->setCloudVisibility(cloudName, true);
            }
            else if(iter->first == stats.refImageId() &&
                    stats.getSignature().id() == iter->first) {
                // Add the new cloud
                Signature s = stats.getSignature();
                s.uncompressData();
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
                            s.getImageRaw(),
                            s.getDepthRaw(),
                            s.getCx(),
                            s.getCy(),
                            s.getFx(),
                            s.getFy(),
                            decimationStatistics_); // decimation

                if(cloud->size()) {
                    cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);
                    if(cloud->size()) {
                        cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, stats.getSignature().getLocalTransform());
                    }
                }
                if(!_vWrapper_robot->addOrUpdateCloud(cloudName, cloud, iter->second)) {
                    yError() << "Adding cloud " << iter->first << " to viewer failed!";
                }
                if(!_vWrapper_partner->addOrUpdateCloud(cloudName, cloud, iter->second)) {
                    yError() << "Adding cloud " << iter->first << " to viewer failed!";
                }
            }
        }
    }

    odometryCorrection_ = stats.mapCorrection();

    _vWrapper_robot->update();
    _vWrapper_partner->update();
}

void MapBuilder::handleEvent(UEvent * event) {
    //yInfo() << "Event: " << event->getClassName();
    if(event->getClassName().compare("RtabmapEvent") == 0 && doProcessStats) {
        RtabmapEvent * rtabmapEvent = dynamic_cast<RtabmapEvent *>(event);
        const Statistics & stats = rtabmapEvent->getStats();
        // Statistics must be processed in the Qt thread
        //yDebug() << "Process statistics";
        if(this->isVisible() && vis_mutex.try_lock()) {
            QMetaObject::invokeMethod(this, "processStatistics", Q_ARG(rtabmap::Statistics, stats));
            vis_mutex.unlock();
        }
    }
    else if(event->getClassName().compare("OdometryEvent") == 0) {
        OdometryEvent * odomEvent = dynamic_cast<OdometryEvent *>(event);
        //yDebug() << "Quality (#inliers): " << odomEvent->info().inliers;
        if(this->isVisible() && vis_mutex.try_lock()) {
            QMetaObject::invokeMethod(this, "processOdometry", Q_ARG(rtabmap::SensorData, odomEvent->data()));
            vis_mutex.unlock();
        }
    }
}

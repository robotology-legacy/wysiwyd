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

#include <string>
#include <map>
#include <boost/thread/mutex.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

#include <rtabmap/core/util3d.h>

#include <rtabmap/utilite/UConversion.h>

#include "VisualizerWrapper.h"
#include "MapBuilder.h"

using namespace rtabmap;

// This class receives RtabmapEvent and construct/update a 3D Map
MapBuilder::MapBuilder(unsigned int decOdo, unsigned int decVis) :
    _vWrapper(new VisualizerWrapper(this)),
    decimationOdometry_(decOdo),
    decimationStatistics_(decVis)
{
    this->setWindowFlags(Qt::Dialog);
    this->setWindowTitle(tr("3D Map"));
    this->setMinimumWidth(1600);
    this->setMinimumHeight(600);

    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(_vWrapper);
    this->setLayout(layout);

    qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");
    qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");
}

MapBuilder::~MapBuilder() {
    this->unregisterFromEventsManager();
    delete _vWrapper;
}

void MapBuilder::setCameraPosition( double pos_x, double pos_y, double pos_z,
                                    double view_x, double view_y, double view_z,
                                    double up_x, double up_y, double up_z,
                                    int viewport) {
    vis_mutex.lock();
    _vWrapper->getVisualizer().setCameraPosition(pos_x, pos_y, pos_z,
                                                 view_x, view_y, view_z,
                                                 up_x, up_y, up_z,
                                                 viewport);
    vis_mutex.unlock();
}

void MapBuilder::setCameraFieldOfView(double fovy, int viewport ) {
    vis_mutex.lock();
    _vWrapper->getVisualizer().setCameraFieldOfView(fovy, viewport);
    vis_mutex.unlock();
}

void MapBuilder::processOdometry(const rtabmap::SensorData & data) {
    if(!this->isVisible())
    {
        return;
    }

    Transform pose = data.pose();

    if(pose.isNull()) {
        //Odometry lost
        _vWrapper->setBackgroundColor(1.0, 0, 0);
        pose = lastOdomPose_;
    } else {
        _vWrapper->setBackgroundColor(0.1, 0.1, 0.1);
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

            if(!_vWrapper->addOrUpdateCloud("cloudOdom", cloud, pose)) {
                cerr << "Adding cloudOdom to viewer failed!" << endl;
            }
        }
        if(!data.pose().isNull()) { // NOT: pose.isNull()!!!
            _vWrapper->updateCameraPosition(data.pose());
        }
    }

    _vWrapper->update();
}

void MapBuilder::processStatistics(const rtabmap::Statistics & stats) {
    const std::map<int, Transform> & poses = stats.poses();
    for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter) {
        if(!iter->second.isNull()) {
            std::string cloudName = uFormat("cloud%d", iter->first);

            // 3d point cloud
            if(_vWrapper->getAddedClouds().count(cloudName)) {
                // Update only if the pose has changed
                Transform tCloud;
                _vWrapper->getPose(cloudName, tCloud);
                if(tCloud.isNull() || iter->second != tCloud) {
                    if(!_vWrapper->updateCloudPose(cloudName, iter->second)) {
                        cerr << "Updating pose cloud " << iter->first << " failed!" << endl;
                    }
                }
                _vWrapper->setCloudVisibility(cloudName, true);
            }
            else if(iter->first == stats.refImageId() &&
                    stats.getSignature().id() == iter->first) {
                // Add the new cloud
                Signature s = stats.getSignature();
                s.uncompressData();
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
                            s.getImageRaw(),
                            s.getDepthRaw(),
                            s.getDepthCx(),
                            s.getDepthCy(),
                            s.getDepthFx(),
                            s.getDepthFy(),
                            decimationStatistics_); // decimation

                if(cloud->size()) {
                    cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);
                    if(cloud->size()) {
                        cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, s.getLocalTransform());
                    }
                }
                if(!_vWrapper->addOrUpdateCloud(cloudName, cloud, iter->second)) {
                    cerr << "Adding cloud " << iter->first << " to viewer failed!" << endl;
                }
            }
        }
    }

    _vWrapper->update();
}

void MapBuilder::handleEvent(UEvent * event) {
    std::cout << "Event: " << event->getClassName() << std::endl;
    if(event->getClassName().compare("RtabmapEvent") == 0) {
        RtabmapEvent * rtabmapEvent = dynamic_cast<RtabmapEvent *>(event);
        const Statistics & stats = rtabmapEvent->getStats();
        // Statistics must be processed in the Qt thread
        cout << "Process statistics" << endl;
        if(this->isVisible() && vis_mutex.try_lock()) {
            QMetaObject::invokeMethod(this, "processStatistics", Q_ARG(rtabmap::Statistics, stats));
            vis_mutex.unlock();
        }
    }
    else if(event->getClassName().compare("OdometryEvent") == 0) {
        OdometryEvent * odomEvent = dynamic_cast<OdometryEvent *>(event);
        cout << "Quality (#inliers): " << odomEvent->info().inliers << endl;
        if(this->isVisible() && vis_mutex.try_lock()) {
            QMetaObject::invokeMethod(this, "processOdometry", Q_ARG(rtabmap::SensorData, odomEvent->data()));
            vis_mutex.unlock();
        }
    }
}

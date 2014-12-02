#include <string>
#include <map>
#include <boost/thread/mutex.hpp>

#include <opencv/cv.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/RtabmapEvent.h>

#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UEventsHandler.h>

#include "VisualizerWrapper.h"
#include "MapBuilder.h"

using namespace rtabmap;

// This class receives RtabmapEvent and construct/update a 3D Map
MapBuilder::MapBuilder(unsigned int decOdo, unsigned int decVis) :
    _vWrapper(new VisualizerWrapper),
    decimationOdometry_(decOdo),
    decimationVisualization_(decVis),
    _processingStatistics(false),
    _processingOdometry(false),
    _lastPoseNull(false)
{
}

MapBuilder::~MapBuilder()
{
    this->unregisterFromEventsManager();
    delete _vWrapper;
}

void MapBuilder::spinOnce(int time, bool force_redraw) {
    vis_mutex.lock();
    _vWrapper->getVisualizer().spinOnce(time, force_redraw);
    vis_mutex.unlock();
}

bool MapBuilder::wasStopped() {
    return _vWrapper->getVisualizer().wasStopped();
}

void MapBuilder::processOdometry(const rtabmap::SensorData & data)
{
    _processingOdometry = true;
    Transform pose = data.pose();
    if(pose.isNull())
    {
        //Odometry lost
        _vWrapper->setBackgroundColor(VColor(255, 0, 0));
        pose = lastOdomPose_;
    }
    else
    {
        _vWrapper->setBackgroundColor(VColor(0, 0, 0));
    }
    if(!pose.isNull())
    {
        lastOdomPose_ = pose;

        // 3d cloud
        if(data.depth().cols == data.image().cols &&
                data.depth().rows == data.image().rows &&
                !data.depth().empty() &&
                data.fx() > 0.0f &&
                data.fy() > 0.0f)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
                        data.image(),
                        data.depth(),
                        data.cx(),
                        data.cy(),
                        data.fx(),
                        data.fy(),
                        decimationOdometry_); // decimation // high definition
            if(cloud->size())
            {
                cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);
                if(cloud->size())
                {
                    cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, data.localTransform());
                }
            }
            if(!_vWrapper->addOrUpdateCloud("cloudOdom", cloud, pose))
            {
                UERROR("Adding cloudOdom to viewer failed!");
            }
        }
        if(!pose.isNull())
        {
            _vWrapper->updateCameraPosition(pose);
        }
    }
    _processingOdometry = false;
}


void MapBuilder::processStatistics(const rtabmap::Statistics & stats)
{
    _processingStatistics = true;
    const std::map<int, Transform> & poses = stats.poses();
    std::map<std::string, Transform> clouds = _vWrapper->getAddedClouds();
    for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
    {
        if(!iter->second.isNull())
        {
            std::string cloudName = uFormat("cloud%d", iter->first);

            // 3d point cloud
            if(clouds.count(cloudName))
            {
                // Update only if the pose has changed
                Transform tCloud;
                _vWrapper->getPose(cloudName, tCloud);
                if(tCloud.isNull() || iter->second != tCloud)
                {
                    if(!_vWrapper->updateCloudPose(cloudName, iter->second))
                    {
                        UERROR("Updating pose cloud %d failed!", iter->first);
                    }
                }
                _vWrapper->setCloudVisibility(cloudName, true);
            }
            else if(iter->first == stats.refImageId() &&
                    stats.getSignature().id() == iter->first)
            {
                // Add the new cloud
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
                            stats.getSignature().getImageRaw(),
                            stats.getSignature().getDepthRaw(),
                            stats.getSignature().getDepthCx(),
                            stats.getSignature().getDepthCy(),
                            stats.getSignature().getDepthFx(),
                            stats.getSignature().getDepthFy(),
                            decimationVisualization_); // decimation

                if(cloud->size())
                {
                    cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);
                    if(cloud->size())
                    {
                        cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, stats.getSignature().getLocalTransform());
                    }
                }
                if(!_vWrapper->addOrUpdateCloud(cloudName, cloud, iter->second))
                {
                    UERROR("Adding cloud %d to viewer failed!", iter->first);
                }
            }
        }
    }
    _processingStatistics = false;
}

void MapBuilder::handleEvent(UEvent * event)
{
    std::cout << "Event: " << event->getClassName() << std::endl;
    if(event->getClassName().compare("RtabmapEvent") == 0)
    {
        RtabmapEvent * rtabmapEvent = (RtabmapEvent *)event;
        const Statistics & stats = rtabmapEvent->getStats();
        // Statistics must be processed in the Qt thread
        std::cout << "Process statistics\n";
        vis_mutex.lock();
        processStatistics(stats);
        vis_mutex.unlock();
    }
    else if(event->getClassName().compare("OdometryEvent") == 0)
    {
        OdometryEvent * odomEvent = (OdometryEvent *)event;
        std::cout << "Quality: " << odomEvent->quality() << std::endl;
        if(!_processingOdometry && !_processingStatistics)
        {
            vis_mutex.lock();
            processOdometry(odomEvent->data());
            vis_mutex.unlock();
        }
    }
}


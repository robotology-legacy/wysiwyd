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

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/util3d.h>

#include "VisualizerWrapper.h"

VisualizerWrapper::VisualizerWrapper() :
    _maxTrajectorySize(100),
    _trajectory(new pcl::PointCloud<pcl::PointXYZ>),
    _visualizer(new pcl::visualization::PCLVisualizer("PCLVisualizer", true)) {
    std::cout << "Created visualizer in thread " << boost::this_thread::get_id() << std::endl;
    // Create the two view ports
    int viewiCub(0), viewPartner(0);

    _visualizer->createViewPort (0.0, 0.0, 0.5, 1.0, viewiCub);
    _visualizer->addText ("View from the iCub", 10, 10, "iCub text", viewiCub);
    _viewports["icub"]=viewiCub;

    _visualizer->createViewPort (0.5, 0.0, 1.0, 1.0, viewPartner);
    _visualizer->addText ("View from the partner", 10, 10, "partner text", viewPartner);
    _visualizer->createViewPortCamera(viewPartner);
    _viewports["partner"]=viewPartner;

    _visualizer->setCameraPosition(
        -1, 0, 0,
        0, 0, 0,
        0, 0, 1);
}

VisualizerWrapper::~VisualizerWrapper() {
    this->removeAllClouds();
    delete _visualizer;
}

void VisualizerWrapper::removeAllClouds() {
    _addedClouds.clear();
    _visualizer->removeAllPointClouds();
}

void VisualizerWrapper::setBackgroundColor(const VColor &color) {
    _visualizer->setBackgroundColor(color.r, color.g, color.b);
}

bool VisualizerWrapper::getPose(const std::string & id, Transform & pose) {
    if(_addedClouds.count(id)) {
        pose = _addedClouds.at(id);
        return true;
    } else {
        return false;
    }
}

bool VisualizerWrapper::updateCloudPose(
        const std::string & id,
        const Transform & pose) {
    if(_addedClouds.count(id)) {
        UDEBUG("Updating pose %s to %s", id.c_str(), pose.prettyPrint().c_str());
        if(_addedClouds.at(id) == pose ||
                _visualizer->updatePointCloudPose(id, util3d::transformToEigen3f(pose))) {
            _addedClouds.at(id) = pose;
            return true;
        }
    }
    return false;
}

void VisualizerWrapper::setCloudVisibility(const std::string & id, bool isVisible) {
    pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
    pcl::visualization::CloudActorMap::iterator iter = cloudActorMap->find(id);
    if(iter != cloudActorMap->end()) {
        iter->second.actor->SetVisibility(isVisible?1:0);
    } else {
        UERROR("Cannot find actor named \"%s\".", id.c_str());
    }
}

bool VisualizerWrapper::removeCloud(const std::string & id) {
    _addedClouds.erase(id);
    bool success = _visualizer->removePointCloud(id);
    return success;
}

bool VisualizerWrapper::updateCloud(
        const std::string & id,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        const Transform & pose) {
    if(_addedClouds.count(id)) {
        UDEBUG("Updating %s with %d points", id.c_str(), (int)cloud->size());
        int index = _visualizer->getColorHandlerIndex(id);
        removeCloud(id);
        if(addCloud(id, cloud, pose)) {
            _visualizer->updateColorHandlerIndex(id, index);
            return true;
        }
    }
    return false;
}

bool VisualizerWrapper::addOrUpdateCloud(
        const std::string & id,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        const Transform & pose,
        const VColor & color) {
    if(!updateCloud(id, cloud, pose)) {
        return addCloud(id, cloud, pose, color);
    }
    return true;
}

bool VisualizerWrapper::addCloud(
        const std::string & id,
        const pcl::PCLPointCloud2Ptr & binaryCloud,
        const Transform & pose,
        bool rgb,
        const VColor & color) {
    if(!_addedClouds.count(id)) {
        Eigen::Vector4f origin(pose.x(), pose.y(), pose.z(), 0.0f);
        Eigen::Quaternionf orientation = Eigen::Quaternionf(util3d::transformToEigen3f(pose).rotation());

        // add random color channel
        pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr colorHandler;
        colorHandler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (binaryCloud));
        if(_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id))
        {
            // white
            colorHandler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2> (binaryCloud, color.r, color.g, color.b));
            _visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);

            // x,y,z
            colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "x"));
            _visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);
            colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "y"));
            _visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);
            colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "z"));
            _visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);

            if(rgb)
            {
                //rgb
                colorHandler.reset(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(binaryCloud));
                _visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);

                _visualizer->updateColorHandlerIndex(id, 5);
            }

            _addedClouds[id]=pose;
            return true;
        }
    }
    return false;
}

bool VisualizerWrapper::addCloud(
        const std::string & id,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        const Transform & pose,
        const VColor & color) {
    if(!_addedClouds.count(id)) {
        UDEBUG("Adding %s with %d points", id.c_str(), (int)cloud->size());

        pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cloud, *binaryCloud);
        return addCloud(id, binaryCloud, pose, true, color);
    }
    return false;
}


void VisualizerWrapper::updateCameraPosition(const Transform & pose)
{
    if(!pose.isNull()) {
        Eigen::Affine3f m = util3d::transformToEigen3f(pose);
        Eigen::Vector3f pos = m.translation();
        Eigen::Vector3f lastPos(0,0,0);
        if(_trajectory->size())
        {
            lastPos[0]=_trajectory->back().x;
            lastPos[1]=_trajectory->back().y;
            lastPos[2]=_trajectory->back().z;
        }
        _trajectory->push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
        if(_maxTrajectorySize>0) {
            while(_trajectory->size() > _maxTrajectorySize) {
                _trajectory->erase(_trajectory->begin());
            }
        }
        /*if(_aShowTrajectory->isChecked())
        {
            _visualizer->removeShape("trajectory");
            pcl::PolygonMesh mesh;
            pcl::Vertices vertices;
            vertices.vertices.resize(_trajectory->size());
            for(unsigned int i=0; i<vertices.vertices.size(); ++i)
            {
                vertices.vertices[i] = i;
            }
            pcl::toPCLPointCloud2(*_trajectory, mesh.cloud);
            mesh.polygons.push_back(vertices);
            _visualizer->addPolylineFromPolygonMesh(mesh, "trajectory");
        }*/

        if(pose != _lastPose || _lastPose.isNull()) {
            if(_lastPose.isNull()) {
                _lastPose.setIdentity();
            }

            std::vector<pcl::visualization::Camera> cameras;
            _visualizer->getCameras(cameras);

            /*if(_aLockCamera->isChecked()) {
                //update camera position
                Eigen::Vector3f diff = pos - Eigen::Vector3f(_lastPose.x(), _lastPose.y(), _lastPose.z());
                cameras.front().pos[0] += diff[0];
                cameras.front().pos[1] += diff[1];
                cameras.front().pos[2] += diff[2];
                cameras.front().focal[0] += diff[0];
                cameras.front().focal[1] += diff[1];
                cameras.front().focal[2] += diff[2];
            }*/
            //else if(_aFollowCamera->isChecked()) {
            Eigen::Vector3f vPosToFocal = Eigen::Vector3f(cameras.front().focal[0] - cameras.front().pos[0],
                    cameras.front().focal[1] - cameras.front().pos[1],
                    cameras.front().focal[2] - cameras.front().pos[2]).normalized();
            Eigen::Vector3f zAxis(cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);
            Eigen::Vector3f yAxis = zAxis.cross(vPosToFocal);
            Eigen::Vector3f xAxis = yAxis.cross(zAxis);
            Transform PR(xAxis[0], xAxis[1], xAxis[2],0,
                    yAxis[0], yAxis[1], yAxis[2],0,
                    zAxis[0], zAxis[1], zAxis[2],0);

            Transform P(PR[0], PR[1], PR[2], cameras.front().pos[0],
                    PR[4], PR[5], PR[6], cameras.front().pos[1],
                    PR[8], PR[9], PR[10], cameras.front().pos[2]);
            Transform F(PR[0], PR[1], PR[2], cameras.front().focal[0],
                    PR[4], PR[5], PR[6], cameras.front().focal[1],
                    PR[8], PR[9], PR[10], cameras.front().focal[2]);
            Transform N = pose;
            Transform O = _lastPose;
            Transform O2N = O.inverse()*N;
            Transform F2O = F.inverse()*O;
            Transform T = F2O * O2N * F2O.inverse();
            Transform Fp = F * T;
            Transform P2F = P.inverse()*F;
            Transform Pp = P * P2F * T * P2F.inverse();

            cameras.front().pos[0] = Pp.x();
            cameras.front().pos[1] = Pp.y();
            cameras.front().pos[2] = Pp.z();
            cameras.front().focal[0] = Fp.x();
            cameras.front().focal[1] = Fp.y();
            cameras.front().focal[2] = Fp.z();
            //FIXME: the view up is not set properly...
            cameras.front().view[0] = Fp[8];
            cameras.front().view[1] = Fp[9];
            cameras.front().view[2] = Fp[10];
            //}*/

#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
            _visualizer->removeCoordinateSystem("reference", 0);
            _visualizer->addCoordinateSystem(0.2, m, "reference", 0);
#else
            _visualizer->removeCoordinateSystem(0);
            _visualizer->addCoordinateSystem(0.2, m, 0);
#endif
            _visualizer->setCameraPosition(
                cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
                cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
                cameras.front().view[0], cameras.front().view[1], cameras.front().view[2], _viewports["icub"]);
        }
    }

    _lastPose = pose;
}

const VColor VisualizerWrapper::_vgrey = VColor(50, 50, 50);

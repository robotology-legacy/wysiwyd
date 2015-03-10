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

//#include <vtkExtractVOI.h>
//#include <vtkRenderWindow.h>
//#include <vtkWindowToImageFilter.h>
//#include <vtkImageExport.h>

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
    this->setMinimumWidth(800);
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
        if(!pose.isNull()) {
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
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(
                            stats.getSignature().getImageRaw(),
                            stats.getSignature().getDepthRaw(),
                            stats.getSignature().getDepthCx(),
                            stats.getSignature().getDepthCy(),
                            stats.getSignature().getDepthFx(),
                            stats.getSignature().getDepthFy(),
                            decimationStatistics_); // decimation

                if(cloud->size()) {
                    cloud = util3d::passThrough<pcl::PointXYZRGB>(cloud, "z", 0, 4.0f);
                    if(cloud->size()) {
                        cloud = util3d::transformPointCloud<pcl::PointXYZRGB>(cloud, stats.getSignature().getLocalTransform());
                    }
                }
                if(!_vWrapper->addOrUpdateCloud(cloudName, cloud, iter->second)) {
                    cerr << "Adding cloud " << iter->first << " to viewer failed!" << endl;
                }
            }
        }
    }

    _vWrapper->update();

    /*vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New ();
    windowToImageFilter->SetInput(_vWrapper->getVisualizer().getRenderWindow());
    vtkSmartPointer<vtkImageExport>exporter_ = vtkImageExport::New();
    exporter_->SetInput(windowToImageFilter->GetOutput());
    exporter_->ImageLowerLeftOff();

    // create a OpenCV IplImage
    IplImage* img_;
    img_ = cvCreateImage(cvSize(1, 1), IPL_DEPTH_8U, 1);
    cvZero( img_);

    windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer
    windowToImageFilter->Update();
    windowToImageFilter->Modified();

    int memsize_;
    int *dimensions_;
    int csize_;

    // get render window info from exporter
    memsize_ = exporter_->GetDataMemorySize();
    dimensions_ = exporter_->GetDataDimensions();
    csize_ = exporter_->GetDataNumberOfScalarComponents();
    cvReleaseImage(&img_);
    img_ = cvCreateImage(cvSize(dimensions_[0], dimensions_[1]), IPL_DEPTH_8U,
            csize_);

    // export the image data to OpenCV image data
    exporter_->Export(img_->imageData);
    img_->widthStep = dimensions_[0] * csize_;

    // swap R and B channel
    cvCvtColor(img_, img_, CV_BGR2RGB);

    cvShowImage("Test", img_);
    cvWaitKey(10);
    cvReleaseImage(&img_);*/

    /*vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New ();
    vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
    image = windowToImageFilter->GetOutput();

    windowToImageFilter->SetInput(_vWrapper->getVisualizer().getRenderWindow());
    windowToImageFilter->ReadFrontBufferOff();
    windowToImageFilter->Update();
    image->Modified();

    // Construct the OpenCv Mat
    int dims[3];
    image->GetDimensions(dims);
    cout << "DIMS: " << dims[0] << " " << dims[1] << " " << dims[2] << endl;
    cout << "SCALAR TYPE: " << image->GetScalarTypeAsString() << endl;
    cv::Mat openCVImage(dims[1], dims[0], CV_8UC3);

    for (int y = 0; y < dims[1]; y++) {
        for (int x = 0; x < dims[0]; x++) {
            unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x,y,0));
            openCVImage.at<cv::Vec3b>(y,x)[0] = pixel[0];
            openCVImage.at<cv::Vec3b>(y,x)[1] = pixel[1];
            openCVImage.at<cv::Vec3b>(y,x)[2] = pixel[2];

            if(pixel[0]!=0) cout << y <<" " << x << " " << (int)pixel[0] << endl;
        }
    }

    cv::imshow("Test", openCVImage);
    cv::waitKey(0);

    cv::Mat oClone = openCVImage.clone();

    cvtColor(oClone, oClone, CV_BGR2RGB);

    // Flip because of different origins between vtk and OpenCV
    cv::flip(oClone,oClone, 0);
    cv::imwrite( "Test.jpg", oClone );*/

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
        cout << "Quality: " << odomEvent->quality() << endl;
        if(this->isVisible() && vis_mutex.try_lock()) {
            QMetaObject::invokeMethod(this, "processOdometry", Q_ARG(rtabmap::SensorData, odomEvent->data()));
            vis_mutex.unlock();
        }
    }
}

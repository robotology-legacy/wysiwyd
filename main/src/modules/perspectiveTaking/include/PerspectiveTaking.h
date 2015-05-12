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

#ifndef VPT_PERSPECTIVETAKING
#define VPT_PERSPECTIVETAKING

#include <QTimer>

#include <yarp/os/RFModule.h>
#include <yarp/math/Math.h>

#include <kinectWrapper/kinectWrapper_client.h>

#include <wrdac/helpers.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryThread.h>

#include "CameraKinectWrapper.h"
#include "MapBuilder.h"
#include "perspectiveTaking_IDL.h"

enum partnerCameraMode_t {staticPos, agentDetector, headPose};

class perspectiveTaking: public QObject, public yarp::os::RFModule, public perspectiveTaking_IDL {
    Q_OBJECT
protected:
    // Kinect related
    yarp::os::BufferedPort<yarp::os::Bottle> getClickPortKinect;
    void connectToKinectServer(int verbosity);
    KinectWrapperClient client;

    // RTabmap related
    CameraKinectWrapper* camera;
    CameraThread* cameraThread;
    Rtabmap* rtabmap;
    MapBuilder* mapBuilder;
    OdometryThread* odomThread;
    RtabmapThread* rtabmapThread;

    // OPC related
    void connectToOPC(const std::string& opcName);
    wysiwyd::wrdac::OPCClient* opc;
    wysiwyd::wrdac::Agent* partner;

    // ABM related
    void connectToABM(const std::string& abmName);
    bool addABMImgProvider(const std::string& portName, bool addProvider);
    bool sendImagesToPorts();
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > selfPerspImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > partnerPerspImgPort;
    yarp::os::Port abm;
    bool isConnectedToABM;

    // RFH related; deprecated
    void connectToRFH(const std::string& rfhName);
    Eigen::Matrix4f getRFHTransMat();
    yarp::os::RpcClient rfh;

    // agentDetector related; deprecated
    void connectToAgentDetector(const std::string& agentDetectorName);
    yarp::os::RpcClient agentdetector;
    bool isConnectedToAgentDetector;

    // SFM related
    void connectToSFM(const std::string& SFMName);
    yarp::os::BufferedPort<yarp::os::Bottle> getClickPortStereo;
    yarp::os::RpcClient sfm;

    // headPoseEstimator related
    void connectToHeadPoseEstimator(const std::string& headPoseEstimatorName);
    yarp::os::RpcClient headPoseEstimator;
    bool isConnectedToHeadPoseEstimator;

    // actual perspective Taking
    double voxelLeafSize;
    void updateObjects();

    void setViewRobotReference(const Eigen::Vector4f& p_pos, const Eigen::Vector4f& p_view, const Eigen::Vector4f& p_up, const std::string& viewport);
    void setViewRobotReference(const yarp::sig::Vector& pos, const yarp::sig::Vector& view, const yarp::sig::Vector& up, const std::string& viewport);
    void setViewCameraReference(const Eigen::Vector4f &p_pos, const Eigen::Vector4f &p_view, const Eigen::Vector4f &p_up, const std::string& viewport);
    void setViewCameraReference(const yarp::sig::Vector& pos, const yarp::sig::Vector& view, const yarp::sig::Vector& up, const std::string& viewport);

    Eigen::Matrix4f kin2head;
    Eigen::Affine3f kin2root;
    Eigen::Matrix4f yarp2pcl;
    partnerCameraMode_t partnerCameraMode;

    Eigen::Matrix4f yarp2EigenM(const yarp::sig::Matrix& yarpMatrix);
    Eigen::Vector4f yarp2EigenV(const Vector& yarpVector);
    pcl::PointXYZ eigen2pclV(const Eigen::Vector4f& eigenVector);

    // actual perspective taking, deprecated
    double lookDown;

    // QT related
    QTimer *setCamPosTimer;
    QThread* setCamPosThread;

    // misc
    unsigned long loopCounter;
    yarp::os::ResourceFinder resfind;
    bool openHandlerPort();
    bool setupThreads();
    bool attach(yarp::os::RpcServer &source);
    yarp::os::RpcServer handlerPort;

private slots:
    void setPartnerCamera();

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool queryRFHTransMat(const std::string& from, const std::string& to, yarp::sig::Matrix& m);
    bool updateModule() { return false; }
    bool close();

    static Eigen::Matrix4f getManualTransMat(float camOffsetX, float camOffsetY, float camOffsetZ, float camAngle);
    static cv::Mat lastDepth;

    bool kinectStereoCalibrate();
    bool setUpdateTimer(const int32_t interval);
    bool setDecimationOdometry(const int32_t decimation);
    bool setDecimationStatistics(const int32_t decimation);
    bool processStats(const bool enable);
};

#endif

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

#include <yarp/os/RFModule.h>
#include <yarp/math/Math.h>

#include <kinectWrapper/kinectWrapper_client.h>

#include <wrdac/helpers.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/Odometry.h>

#include "head_pose_estimation/CRMainEstimation.h"
#include "CameraKinectWrapper.h"
#include "MapBuilder.h"

enum partnerCameraMode_t {staticPos, agentDetector, headPose};

class perspectiveTaking: public QObject, public yarp::os::RFModule {
    Q_OBJECT
protected:
    // Kinect related
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

    // RFH related
    Eigen::Matrix4f getRFHTransMat(const std::string& rfhName);
    yarp::os::Port rfh;

    // agentDetector related; deprecated
    void connectToAgentDetector(const std::string& agentDetectorName);
    yarp::os::Port agentdetector;
    bool isConnectedToAgentDetector;

    // actual perspective Taking
    void setViewRobotReference(const Eigen::Vector4f& p_pos, const Eigen::Vector4f& p_view, const Eigen::Vector4f& p_up, const std::string& viewport);
    void setViewRobotReference(const yarp::sig::Vector& pos, const yarp::sig::Vector& view, const yarp::sig::Vector& up, const std::string& viewport);
    void setViewCameraReference(const Eigen::Vector4f &p_pos, const Eigen::Vector4f &p_view, const Eigen::Vector4f &p_up, const std::string& viewport);
    void setViewCameraReference(const yarp::sig::Vector& pos, const yarp::sig::Vector& view, const yarp::sig::Vector& up, const std::string& viewport);
    Eigen::Matrix4f kinect2robot_pcl;
    Eigen::Matrix4f yarp2pcl;
    partnerCameraMode_t partnerCameraMode;

    Eigen::Matrix4f yarp2pclKinectMatrix(const yarp::sig::Matrix& kinect2robotYarp);
    Eigen::Vector4f yarp2EigenV(yarp::sig::Vector);

    // actual perspective taking, deprecated
    double distanceMultiplier;
    double lookDown;

    // QT related
    QTimer *setCamPosTimer;
    QThread* setCamPosThread;
    QTimer *headPoseTimer;
    QThread* headPoseThread;

    // misc
    CRMainEstimation* head_estimator;
    unsigned long loopCounter;
    yarp::os::ResourceFinder resfind;
    bool openHandlerPort();
    bool setupThreads();
    yarp::os::Port handlerPort;

    cv::Mat MatFromDepth(const cv::Mat & imageDepth,
                         float cx, float cy, float fx, float fy);

private slots:
    void setPartnerCamera();

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
    bool queryRFHTransMat(const std::string& from, const std::string& to, yarp::sig::Matrix& m);
    bool updateModule() { return false; }
    bool close();

    static Eigen::Matrix4f getManualTransMat(float camOffsetX, float camOffsetZ, float camAngle);
    static cv::Mat lastDepth;
};

#endif

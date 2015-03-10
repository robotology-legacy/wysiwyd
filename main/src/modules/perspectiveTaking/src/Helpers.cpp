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

#include <rtabmap/utilite/UEventsManager.h>
#include "PerspectiveTaking.h"
#include "CameraKinectWrapper.h"
#include "Helpers.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

bool perspectiveTaking::setupThreads() {
    // Read parameters for rtabmap from rtabmap_config.ini
    ParametersMap parameters;
    Rtabmap::readParameters(resfind.findFileByName("rtabmap_config.ini"), parameters);

    // Here is the pipeline that we will use:
    // CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

    // Create the OpenNI camera, it will send a CameraEvent at the rate specified.
    // Set transform to camera so z is up, y is left and x going forward (as in PCL)
    camera = new CameraKinectWrapper(client, 20, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0));

    cameraThread = new CameraThread(camera);

    if(!cameraThread->init()) {
        cout << getName() << ":Camera init failed!";
        return false;
    }

    // Create an odometry thread to process camera events, it will send OdometryEvent
    odomThread = new OdometryThread(new OdometryBOW(parameters));

    // Create RTAB-Map to process OdometryEvent
    rtabmap = new Rtabmap();
    rtabmap->init(parameters, resfind.findFileByName("rtabmap.db"));
    rtabmapThread = new RtabmapThread(rtabmap);

    boost::this_thread::sleep_for (boost::chrono::milliseconds (100));

    // Setup handlers
    odomThread->registerToEventsManager();
    rtabmapThread->registerToEventsManager();
    mapBuilder->registerToEventsManager();

    // The RTAB-Map is subscribed by default to CameraEvent, but we want
    // RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
    // We can do that by creating a "pipe" between the camera and odometry, then
    // only the odometry will receive CameraEvent from that camera. RTAB-Map is
    // also subscribed to OdometryEvent by default, so no need to create a pipe
    // between odometry and RTAB-Map.
    UEventsManager::createPipe(cameraThread, odomThread, "CameraEvent");

    // Let's start the threads
    rtabmapThread->start();
    odomThread->start();
    cameraThread->start();

    return true;
}

void perspectiveTaking::getManualTransMat(float camOffsetZ, float camAngle) {
    kinect2icub_pcl = Eigen::Matrix4f::Zero();

    // Estimate rotation+translation from kinect to icub head
    Eigen::Affine3f rot_trans = Eigen::Affine3f::Identity();

    // iCub head is ~40cm below kinect
    rot_trans.translation() << 0.0, 0.0, camOffsetZ;
    // iCub head is tilted ~30 degrees down
    float theta = camAngle/180*M_PI;
    rot_trans.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));

    kinect2icub_pcl = rot_trans.matrix();
}

// TODO: Not working, needs to be fixed
void perspectiveTaking::getRFHTransMat(const string &rfhName) {
    string rfhLocal = "/"+getName()+"/rfh:o";
    rfh.open(rfhLocal.c_str());
    string rfhRemote = "/"+rfhName+"/rpc";

    while (!Network::connect(rfhLocal.c_str(),rfhRemote.c_str())) {
        cout << "Waiting for connection to RFH..." << endl;
        Time::delay(1.0);
    }

    yarp::sig::Matrix kinect2icub;
    yarp::sig::Matrix icub2kinect;

    while(!queryRFHTransMat("kinect", "icub", kinect2icub)) {
        cout << "Kinect2iCub matrix not calibrated, please do so in agentDetector" << endl;
        Time::delay(1.0);
    }
    while(!queryRFHTransMat("icub", "kinect", icub2kinect)) {
        cout << "iCub2Kinect matrix not calibrated, please do so in agentDetector" << endl;
        Time::delay(1.0);
    }

    kinect2icub_pcl = Eigen::Matrix4f::Zero();
    yarp2pclKinectMatrix(kinect2icub, kinect2icub_pcl);
}

bool perspectiveTaking::queryRFHTransMat(const string& from, const string& to, Matrix& m)
{
    if (rfh.getOutputCount() > 0) {
        //Get the kinect2icub
        Bottle bCmd;
        bCmd.clear();
        bCmd.addString("mat");
        bCmd.addString(from);
        bCmd.addString(to);

        Bottle reply;
        reply.clear();
        rfh.write(bCmd, reply);
        if (reply.get(0) == "nack") {
            return false;
        } else {
            Bottle* bMat = reply.get(1).asList();
            m.resize(4,4);
            for(int i=0; i<4; i++) {
                for(int j=0; j<4; j++) {
                    m(i,j)=bMat->get(4*i+j).asDouble();
                }
            }
            cout << "Transformation matrix from " << from
                 << " to " << to << " retrieved:" << endl;
            cout << m.toString(3,3).c_str() << endl;
            return true;
        }
    }
    return false;
}

void perspectiveTaking::connectToABM(const string &abmName) {
    string abmLocal = "/"+getName()+"/abm:o";
    abm.open(abmLocal.c_str());
    string abmRemote = "/"+abmName+"/rpc";

    while (!Network::connect(abmLocal.c_str(),abmRemote.c_str())) {
        cout << "Waiting for connection to ABM..." << endl;
        Time::delay(1.0);
    }
}

void perspectiveTaking::connectToAgentDetector(const string &agentDetectorName) {
    string agentDetectorLocal = "/"+getName()+"/agentdetector:o";
    agentdetector.open(agentDetectorLocal.c_str());
    string agentDetectorRemote = "/"+agentDetectorName+"/rpc";

    while (!Network::connect(agentDetectorLocal.c_str(),agentDetectorRemote.c_str())) {
        cout << "Waiting for connection to Agent Detector..." << endl;
        Time::delay(1.0);
    }
}

void perspectiveTaking::connectToKinectServer(int verbosity) {
    string clientName = getName()+"/kinect";

    Property options;
    options.put("carrier","tcp");
    options.put("remote","kinectServer");
    options.put("local",clientName.c_str());
    options.put("verbosity",verbosity);

    while (!client.open(options)) {
        cout<<"Waiting for connection to KinectServer..."<<endl;
        Time::delay(1.0);
    }
}

void perspectiveTaking::connectToOPC(const string &opcName) {
    opc = new OPCClient(getName());
    while (!opc->connect(opcName)) {
        cout<<"Waiting for connection to OPC..."<<endl;
        Time::delay(1.0);
    }
}

bool perspectiveTaking::openHandlerPort() {
    string handlerPortName = "/" + getName() + "/rpc";

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    // attach to rpc port
    attach(handlerPort);

    return true;
}

bool perspectiveTaking::addABMImgProvider(const string &portName) {
    Bottle bCmd, bReply;
    bCmd.addString("addImgStreamProvider");
    bCmd.addString(portName);

    abm.write(bCmd, bReply);

    if(bReply.toString()=="ack") {
        return true;
    } else {
        return false;
    }
}

bool perspectiveTaking::removeABMImgProvider(const string &portName) {
    Bottle bCmd, bReply;
    bCmd.addString("removeImgStreamProvider");
    bCmd.addString(portName);

    abm.write(bCmd, bReply);

    if(bReply.toString()=="ack") {
        return true;
    } else {
        return false;
    }
}

void perspectiveTaking::setCamera(const Vector &p_pos, const Vector &p_view, const Vector &p_up, const string &cameraName) {
    setCamera(yarp2EigenV(p_pos), yarp2EigenV(p_view), yarp2EigenV(p_up), cameraName);
}

void perspectiveTaking::setCamera(const Eigen::Vector4f &p_pos, const Eigen::Vector4f &p_view, const Eigen::Vector4f &p_up, const string &cameraName) {
    Eigen::Vector4f pos = kinect2icub_pcl * yarp2pcl * p_pos;
    //Eigen::Vector4f view = kinect2icub_pcl * yarp2pcl * Eigen::Vector4f(p_headPos[0]+1.0,p_headPos[1],p_headPos[2],1);
    Eigen::Vector4f view = kinect2icub_pcl * yarp2pcl * p_view;
    Eigen::Vector4f up = kinect2icub_pcl * yarp2pcl * p_up;

    pos/=pos[3]; view/=view[3], up/=up[3];

    Eigen::Vector4f up_diff = up-pos;

    mapBuilder->setCameraPosition(
        pos[0],     pos[1],     pos[2],
        view[0],    view[1],    view[2],
        up_diff[0], up_diff[1], up_diff[2],
        mapBuilder->getViewports()[cameraName]);
}

void yarp2pclKinectMatrix(const yarp::sig::Matrix& kinect2icubYarp,
                          Eigen::Matrix4f& kinect2icubPCL) {
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            kinect2icubPCL(i,j)=kinect2icubYarp(i,j);
        }
    }
}

Eigen::Vector4f yarp2EigenV(Vector yVec) {
    return Eigen::Vector4f(yVec[0], yVec[1], yVec[2], 1);
}

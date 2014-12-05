/*
 *
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
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

#include <stdio.h>

#include <boost/chrono/chrono.hpp>

#include <yarp/math/Math.h>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>

#include "CameraKinectWrapper.h"
#include "MapBuilder.h"
#include "PerspectiveTaking.h"
#include "Helpers.h"

using namespace std;
using namespace yarp::os;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the
 * equivalent of the "open" method.
 */

bool perspectiveTaking::configure(yarp::os::ResourceFinder &rf) {
    resfind = rf;

    int verbosity=rf.check("verbosity",Value(0)).asInt();
    string moduleName = rf.check("name", Value("perspectiveTaking"), "module name (string)").asString();
    setName(moduleName.c_str());

    loopCounter = 0;

    // Read parameters for rtabmap from rtabmap_config.ini
    ParametersMap parameters;
    Rtabmap::readParameters(rf.findFileByName("rtabmap_config.ini"), parameters);

    //Open the OPC Client
    string opcName=rf.check("opc",Value("OPC")).asString().c_str();
    opc = new OPCClient(moduleName);
    while (!opc->connect(opcName))
    {
        cout<<"Waiting connection to OPC..."<<endl;
        Time::delay(1.0);
    }

    // Connect to kinectServer
    string clientName = getName()+"/kinect";

    Property options;
    options.put("carrier","tcp");
    options.put("remote","kinectServer");
    options.put("local",clientName.c_str());
    options.put("verbosity",verbosity);

    while (!client.open(options))
    {
        cout<<"Waiting connection to KinectServer..."<<endl;
        Time::delay(1.0);
    }

    //Retrieve the calibration matrix from RFH
    //WARNING: THIS IS CURRENTLY NOT USED!
    string rfhLocal = "/"+moduleName+"/rfh:o";
    rfh.open(rfhLocal.c_str());

    string rfhName=rf.check("rfh",Value("referenceFrameHandler")).asString().c_str();
    string rfhRemote = "/"+rfhName+"/rpc";

    while (!Network::connect(rfhLocal.c_str(),rfhRemote.c_str()))
    {
        cout << "Waiting for connection to RFH..." << endl;
        Time::delay(1.0);
    }

    while(!getRFHMatrix("kinect", "icub", kinect2icub))
    {
        cout << "Kinect2iCub matrix not calibrated, please do so in agentDetector" << endl;
        Time::delay(1.0);
    }
    while(!getRFHMatrix("icub", "kinect", icub2kinect))
    {
        cout << "iCub2Kinect matrix not calibrated, please do so in agentDetector" << endl;
        Time::delay(1.0);
    }

    kinect2icub_pcl = Eigen::Matrix4f::Zero();
    //TODO: Get kinect2icub_pcl from referenceFrameHandler
    //yarp2pclKinectMatrix(kinect2icub, kinect2icub_pcl);
    //cout << kinect2icub_pcl << endl;

    // Estimate rotation+translation from kinect to icub head
    Eigen::Affine3f rot_trans = Eigen::Affine3f::Identity();

    // iCub head is ~60cm below kinect
    rot_trans.translation() << 0.0, 0.0, -0.6;
    // iCub head is tilted ~30 degrees down
    float theta_degrees=-30;
    float theta = theta_degrees/180*M_PI;
    rot_trans.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));

    kinect2icub_pcl = rot_trans.matrix();
    cout << "Kinect 2 iCub PCL: " << kinect2icub_pcl << endl;

    // we need to convert from yarp reference frame to pcl reference frame
    yarp2pcl = Eigen::Matrix4f::Identity();
    yarp2pcl(0, 0) = -1; // x is back on the icub, forward in pcl
    yarp2pcl(1, 1) = -1; // y is right on the icub, left in pcl

    Eigen::Vector4f pos = kinect2icub_pcl * yarp2pcl * Eigen::Vector4f(0,0,0,1);
    pos /= pos[3];

    Eigen::Vector4f view = kinect2icub_pcl * yarp2pcl * Eigen::Vector4f(-1,0,0,1);
    view /= view[3];

    Eigen::Vector4f up = kinect2icub_pcl * yarp2pcl * Eigen::Vector4f(0,0,1,1);
    up /= up[3];

    Eigen::Vector4f up_diff = up-pos;

    // GUI stuff, the handler will receive RtabmapEvent and construct the map
    int decimationOdometry = 2;     // decimation to show points clouds
                                    // the higher, the lower the resolution
                                    // odometry: most recent cloud
    int decimationStatistics = 2;   // statistics: past point cloud decimation

    mapBuilder = new MapBuilder(decimationOdometry, decimationStatistics,
                                pos, view, up_diff);

    // Here is the pipeline that we will use:
    // CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

    // Create the OpenNI camera, it will send a CameraEvent at the rate specified.
    // Set transform to camera so z is up, y is left and x going forward (as in PCL)
    camera = new CameraKinectWrapper(client, 20, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0));

    cameraThread = new CameraThread(camera);

    if(!cameraThread->init())
    {
        cout << getName() << ":Camera init failed!";
        return false;
    }

    // Create an odometry thread to process camera events, it will send OdometryEvent
    odomThread = new OdometryThread(new OdometryBOW(parameters));

    // Create RTAB-Map to process OdometryEvent
    rtabmap = new Rtabmap();
    rtabmap->init(parameters, rf.findFileByName("rtabmap.db"));
    rtabmapThread = new RtabmapThread(rtabmap);

    boost::this_thread::sleep_for (boost::chrono::milliseconds (1000));

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

    // Open handler port
    string handlerPortName = "/" + getName() + "/rpc";

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    // attach to rpc port
    attach(handlerPort);

    // Let's start the threads
    rtabmapThread->start();
    odomThread->start();
    cameraThread->start();

    return true;
}

bool perspectiveTaking::respond(const Bottle& cmd, Bottle& reply) {
    // TODO: Not yet implemented
    if (cmd.get(0).asString() == "learnEnvironment" )
    {
        //cout << getName() << ": Going to learn the environment" << endl;
        //parameter = cmd.get(1).asString();
        reply.addString("ack");
    }
    else
    {
        reply.addString("nack");
    }
    return true;
}

bool perspectiveTaking::getRFHMatrix(const string& from, const string& to, Matrix& m)
{
    if (rfh.getOutputCount()>0)
    {
        //Get the kinect2icub
        Bottle bCmd;
        bCmd.clear();
        bCmd.addString("mat");
        bCmd.addString(from);
        bCmd.addString(to);

        Bottle reply;
        reply.clear();
        rfh.write(bCmd, reply);
        if (reply.get(0) == "nack")
        {
            return false;
        }
        else
        {
            Bottle* bMat = reply.get(1).asList();
            m.resize(4,4);
            for(int i=0; i<4; i++)
            {
                for(int j=0; j<4; j++)
                {
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

double perspectiveTaking::getPeriod() {
    return 0.1;
}

/* Called periodically every getPeriod() seconds */
bool perspectiveTaking::updateModule() {
    if(!mapBuilder->wasStopped()) {
        ++loopCounter;
        partner = (Agent*)opc->getEntity("partner", true);
        if(partner) {
            if(loopCounter%5==0) {
                //TODO: This transformation does not work yet!
                Vector p_headPos = partner->m_ego_position;
                Vector p_shoulderLeft = partner->m_body.m_parts["shoulderLeft"];
                Vector p_shoulderRight = partner->m_body.m_parts["shoulderRight"];

                Vector p_view = yarp::math::cross(p_shoulderRight-p_headPos, p_shoulderLeft-p_headPos);
                Vector p_up = p_headPos; p_up[2]+=1;
                Vector p_up_diff = p_up-p_headPos;

                /*cout << "Pos : " << p_headPos.toString() << endl;
                cout << "View: " << p_view.toString()    << endl;
                cout << "Up  : " << p_up_diff.toString() << endl;*/

                Eigen::Vector4f pos = kinect2icub_pcl * yarp2pcl * Eigen::Vector4f(p_headPos[0],p_headPos[1],p_headPos[2],1);
                pos /= pos[3];

                Eigen::Vector4f view = kinect2icub_pcl * yarp2pcl * Eigen::Vector4f(p_view[0],p_view[1],p_view[2],1);
                view /= view[3];

                Eigen::Vector4f up = kinect2icub_pcl * yarp2pcl * Eigen::Vector4f(0,0,1,1);
                up /= up[3];

                Eigen::Vector4f up_diff = up-pos;

                mapBuilder->setCameraPosition(
                    p_headPos[0], p_headPos[1], p_headPos[2],
                    p_view[0], p_view[1], p_view[2],
                    p_up_diff[0], p_up_diff[1], p_up_diff[2],
                    mapBuilder->getViewPartner());

                /*mapBuilder->setCameraPosition(
                    pos[0], pos[1], pos[2],
                    view[0], view[1], view[2],
                    up_diff[0], up_diff[1], up_diff[2],
                    mapBuilder->getViewPartner());*/
            }
        } else {
            cout << "No partner found in OPC!" << endl;
        }

        // update GUI
        mapBuilder->spinOnce(40);

        // print some statistics
        if(rtabmap->getLoopClosureId())
        {
            printf(" #%d ptime(%fs) STM(%ld) WM(%ld) hyp(%d) value(%.2f) *LOOP %d->%d*\n",
                   loopCounter,
                   rtabmap->getLastProcessTime(),
                   rtabmap->getSTM().size(), // short-term memory
                   rtabmap->getWM().size(), // working memory
                   rtabmap->getLoopClosureId(),
                   rtabmap->getLcHypValue(),
                   rtabmap->getLastLocationId(),
                   rtabmap->getLoopClosureId());
        }
        else
        {
            printf(" #%d ptime(%fs) STM(%ld) WM(%ld) hyp(%d) value(%.2f)\n",
                   loopCounter,
                   rtabmap->getLastProcessTime(),
                   rtabmap->getSTM().size(), // short-term memory
                   rtabmap->getWM().size(), // working memory
                   rtabmap->getRetrievedId(), // highest loop closure hypothesis
                   rtabmap->getLcHypValue());
        }
        return true;
    }
    else {
        return false;
    }

}

bool perspectiveTaking::interruptModule() {
    handlerPort.interrupt();
    opc->interrupt();
    return true;
}

bool perspectiveTaking::close() {
    // remove handlers
    mapBuilder->unregisterFromEventsManager();
    rtabmapThread->unregisterFromEventsManager();
    odomThread->unregisterFromEventsManager();

    boost::this_thread::sleep_for (boost::chrono::milliseconds (100));

    // Kill all threads
    cameraThread->kill();
    delete cameraThread;

    odomThread->join(true);
    rtabmapThread->join(true);

    // generate graph and save Long-Term Memory
    rtabmap->generateDOTGraph(resfind.findFileByName("Graph.dot"));
    printf("Generated graph \"Graph.dot\", viewable with Graphiz using \"neato -Tpdf Graph.dot -o out.pdf\"\n");

    // Cleanup... save database and logs
    printf("Saving Long-Term Memory to \"rtabmap.db\"...\n");
    rtabmap->close();

    // Delete pointers
    delete mapBuilder;
    delete rtabmapThread;
    delete odomThread;

    // Close ports
    opc->interrupt();
    opc->close();
    handlerPort.interrupt();
    handlerPort.close();
    client.close();

    return true;
}

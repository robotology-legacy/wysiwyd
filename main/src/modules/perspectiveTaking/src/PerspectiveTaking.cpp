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

#include <stdio.h>

#include <boost/chrono/chrono.hpp>

#include <yarp/math/Math.h>

#include <rtabmap/core/Parameters.h>

#include "CameraKinectWrapper.h"
#include "MapBuilder.h"
#include "PerspectiveTaking.h"
#include "Helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace kinectWrapper;
using namespace wysiwyd::wrdac;

bool perspectiveTaking::configure(yarp::os::ResourceFinder &rf) {
    resfind = rf;
    setName(rf.check("name", Value("perspectiveTaking"), "module name (string)").asString().c_str());
    loopCounter = 0;

    connectToOPC(rf.check("opc",Value("OPC")).asString().c_str());
    connectToKinectServer(rf.check("kinClientVerbosity",Value(0)).asInt());
    connectToABM(rf.check("abm",Value("autobiographicalMemory")).asString().c_str());
    openHandlerPort();

    //getRFHTransMat(resfind.check("rfh",Value("referenceFrameHandler")).asString().c_str());
    getManualTransMat();
    cout << "Kinect 2 iCub PCL: " << kinect2icub_pcl << endl;

    //port for images
    selfPerspImgPort.open("/"+getName()+"/images/self:o");
    partnerPerspImgPort.open("/"+getName()+"/images/partner:o");

    addABMImgProvider("VPTSelf", selfPerspImgPort.getName());
    addABMImgProvider("VPTPartner", partnerPerspImgPort.getName());

    mapBuilder = new MapBuilder(rf.check("decimationOdometry",Value(2)).asInt(),
                                rf.check("decimationStatistics",Value(2)).asInt());

    setupThreads();

    // we need to convert from yarp reference frame to pcl reference frame
    yarp2pcl = Eigen::Matrix4f::Identity();
    yarp2pcl(0, 0) = -1; // x is back on the icub, forward in pcl
    yarp2pcl(1, 1) = -1; // y is right on the icub, left in pcl

    // Set camera position for iCub viewpoint
    Eigen::Vector4f icub_pos =  Eigen::Vector4f( 0,0,0,1);
    Eigen::Vector4f icub_view = Eigen::Vector4f(-1,0,0,1);
    Eigen::Vector4f icub_up =   Eigen::Vector4f( 0,0,1,1);
    setCamera(icub_pos, icub_view, icub_up, "icub");

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

double perspectiveTaking::getPeriod() {
    return 0.1;
}

bool perspectiveTaking::addABMImgProvider(string label, string portName) {
    Bottle bCmd, bReply;
    bCmd.addString("addImgProvider");
    Bottle imgProvider;
    imgProvider.addString(label);
    imgProvider.addString(portName);
    bCmd.addList() = imgProvider;
    cout << bCmd.toString() << endl;

    abm.write(bCmd, bReply);

    if(bReply.toString()=="ack") {
        return true;
    } else {
        return false;
    }
}

bool perspectiveTaking::sendImages() {
    mapBuilder->saveScreenshot("temp.png");
    cv::Mat screen;
    screen = cv::imread("temp.png", CV_LOAD_IMAGE_COLOR);
    if(!screen.data ) // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl;
        return false;
    }

    cv::Mat selfPersp, partnerPersp;
    // self perspective = left side of screenshot
    // partner perspective = right side of screenshot
    selfPersp=screen(cv::Rect(0,0,screen.cols/2,screen.rows));
    partnerPersp=screen(cv::Rect(screen.cols/2,0,screen.cols/2,screen.rows));
    cv::imshow("Self Perspective", selfPersp);
    cv::imshow("Partner Perspective", partnerPersp);
    cv::waitKey(50);

    IplImage* partnerPersp_ipl = new IplImage(partnerPersp);
    ImageOf<PixelRgb> &partnerPers_yarp = partnerPerspImgPort.prepare();
    partnerPers_yarp.resize(partnerPersp_ipl->width, partnerPersp_ipl->height);
    cvCopyImage(partnerPersp_ipl, (IplImage *)partnerPers_yarp.getIplImage());

    IplImage* selfPersp_ipl = new IplImage(selfPersp);
    ImageOf<PixelRgb> &selfPers_yarp = selfPerspImgPort.prepare();
    selfPers_yarp.resize(selfPersp_ipl->width, selfPersp_ipl->height);
    cvCopyImage(selfPersp_ipl, (IplImage *)selfPers_yarp.getIplImage());

    //send the images
    selfPerspImgPort.write();
    partnerPerspImgPort.write();

    return true;
}

void perspectiveTaking::setCamera(Vector p_pos, Vector p_view, Vector p_up, string cameraName) {
    setCamera(yarp2EigenV(p_pos), yarp2EigenV(p_view), yarp2EigenV(p_up), cameraName);
}

void perspectiveTaking::setCamera(Eigen::Vector4f p_pos, Eigen::Vector4f p_view, Eigen::Vector4f p_up, string cameraName) {
    Eigen::Vector4f pos = kinect2icub_pcl * yarp2pcl * p_pos;
    //Eigen::Vector4f view = kinect2icub_pcl * yarp2pcl * Eigen::Vector4f(p_headPos[0]+1.0,p_headPos[1],p_headPos[2],1);
    Eigen::Vector4f view = kinect2icub_pcl * yarp2pcl * p_view;
    Eigen::Vector4f up = kinect2icub_pcl * yarp2pcl * p_up;

    pos/=pos[3]; view/=view[3], up/=up[3];

    Eigen::Vector4f up_diff = up-pos;

    mapBuilder->setCameraPosition(
        pos[0], pos[1], pos[2],
        view[0], view[1], view[2],
        up_diff[0], up_diff[1], up_diff[2],
        mapBuilder->getViewports()[cameraName]);
}

/* Called periodically every getPeriod() seconds */
bool perspectiveTaking::updateModule() {
    if(!mapBuilder->wasStopped()) {
        ++loopCounter;
        if(loopCounter%25==0) { // only update camera every now and then
            sendImages();

            partner = (Agent*)opc->getEntity("partner", true);
            if(partner) { // TODO:  && partner->m_present
                Vector p_headPos = partner->m_ego_position;
                double p_up_double[3] = {p_headPos[0], p_headPos[1], p_headPos[2]+1.0};
                Vector p_up = Vector(3, p_up_double);
                Vector p_shoulderLeft = partner->m_body.m_parts["shoulderLeft"];
                Vector p_shoulderRight = partner->m_body.m_parts["shoulderRight"];

                // For now, the partner is thought to look towards the icub
                // This is achieved by laying a plane between left shoulder,
                // right shoulder and head and using the normal vector of
                // the plane as viewing vector
                Vector p_view = yarp::math::cross(p_shoulderRight-p_headPos, p_shoulderLeft-p_headPos);

                setCamera(p_headPos, p_view, p_up, "partner");
            } else {
                cout << "No partner found in OPC!" << endl;
            }
        }
        // update GUI
        mapBuilder->spinOnce(40);

        // print some statistics
        if(rtabmap->getLoopClosureId()) {
            printf(" #%ld ptime(%fs) STM(%ld) WM(%ld) hyp(%d) value(%.2f) *LOOP %d->%d*\n",
                   loopCounter,
                   rtabmap->getLastProcessTime(),
                   rtabmap->getSTM().size(), // short-term memory
                   rtabmap->getWM().size(), // working memory
                   rtabmap->getLoopClosureId(),
                   rtabmap->getLcHypValue(),
                   rtabmap->getLastLocationId(),
                   rtabmap->getLoopClosureId());
        }
        else {
            printf(" #%ld ptime(%fs) STM(%ld) WM(%ld) hyp(%d) value(%.2f)\n",
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
    selfPerspImgPort.interrupt();
    partnerPerspImgPort.interrupt();
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
    selfPerspImgPort.interrupt();
    selfPerspImgPort.close();
    partnerPerspImgPort.interrupt();
    partnerPerspImgPort.close();
    opc->interrupt();
    opc->close();
    handlerPort.interrupt();
    handlerPort.close();
    client.close();

    return true;
}

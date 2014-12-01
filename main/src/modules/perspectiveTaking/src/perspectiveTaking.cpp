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

#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>

#include "CameraKinectWrapper.h"
#include "MapBuilder.h"
#include "perspectiveTaking.h"

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

    // Connect to kinectServer
    string clientName = getName()+"/kinect:i";

    Property options;
    options.put("carrier","tcp");
    options.put("remote","kinectServer");
    options.put("local",clientName.c_str());
    options.put("verbosity",verbosity);

    if (!client.open(options)) {
        cout << getName() << ": Unable to connect to kinectServer" << endl;
        return false;
    }

    //Retrieve the calibration matrix from RFH
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

    // Read parameters from rtabmap_config.ini
    ParametersMap parameters;
    Rtabmap::readParameters(rf.findFileByName("rtabmap_config.ini"), parameters);

    // GUI stuff, the handler will receive RtabmapEvent and construct the map
    mapBuilder = new MapBuilder(2, 2);

    // Here is the pipeline that we will use:
    // CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

    // Create the OpenNI camera, it will send a CameraEvent at the rate specified.
    // Set transform to camera so z is up, y is left and x going forward

    camera = new CameraKinectWrapper(client, 30, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0));

    cameraThread = new CameraThread(camera);

    if(!cameraThread->init())
    {
        cout << getName() << ":Camera init failed!";
        return false;
    }

    // Create an odometry thread to process camera events, it will send OdometryEvent.
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
    // also subscribed to OdometryEvent by default, so no need to create a pipe between
    // odometry and RTAB-Map.
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
            cout << "Transformation matrix from " << from << " to " << to << " retrieved" << endl;
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
        mapBuilder->spinOnce(40);
        ++loopCounter;
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
    }
    return true;
}

bool perspectiveTaking::interruptModule() {
    handlerPort.interrupt();

    return true;
}

bool perspectiveTaking::close() {
    // remove handlers
    mapBuilder->unregisterFromEventsManager();
    rtabmapThread->unregisterFromEventsManager();
    odomThread->unregisterFromEventsManager();

    boost::this_thread::sleep_for (boost::chrono::milliseconds (100));

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

    // Kill all threads
    cameraThread->kill();
    delete cameraThread;

    odomThread->join(true);
    rtabmapThread->join(true);

    // Close ports
    handlerPort.interrupt();
    handlerPort.close();
    client.close();

    return true;
}

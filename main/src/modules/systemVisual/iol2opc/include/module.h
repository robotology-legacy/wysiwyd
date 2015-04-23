/* 
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Ugo Pattacini, Tobias Fischer
 * email:   ugo.pattacini@iit.it, t.fischer@imperial.ac.uk
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

#ifndef __MODULE_H__
#define __MODULE_H__

#include <string>
#include <deque>
#include <map>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/ctrl/filters.h>

#include <cv.h>

#include "utils.h"
#include "classifierHandling.h"
#include "iol2opc_IDL.h"

#define RET_INVALID     -1

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;


/**********************************************************/
class IOL2OPCBridge : public RFModule, public iol2opc_IDL
{
protected:
    RpcServer rpcPort;
    RpcClient rpcClassifier;
    RpcClient rpcGet3D;
    RpcClient rpcMemory;

    BufferedPort<Bottle>             blobExtractor;
    BufferedPort<Bottle>             histObjLocPort;
    BufferedPort<Property>           recogTriggerPort;
    BufferedPort<ImageOf<PixelBgr> > imgIn;
    BufferedPort<ImageOf<PixelBgr> > imgOut;
    BufferedPort<ImageOf<PixelBgr> > imgRtLocOut;
    BufferedPort<ImageOf<PixelBgr> > imgHistogram;
    Port imgClassifier;

    RtLocalization rtLocalization;
    MemoryUpdater memoryUpdater;
    ClassifiersDataBase db;
    map<string,int> memoryIds;

    ImageOf<PixelBgr> img;
    ImageOf<PixelBgr> imgRtLoc;
    Semaphore mutexResources;
    Semaphore mutexResourcesMemory;
    Semaphore mutexAttention;
    Semaphore mutexMemoryUpdate;
    
    string name;
    bool scheduleLoadMemory;
    bool enableInterrupt;
    bool actionInterrupted;
    bool skipGazeHoming;
    bool doAttention;
    bool trainOnFlipped;
    bool trainBurst;    
    double improve_train_period;
    double classification_threshold;

    map<string,Filter*> histFiltersPool;
    int histFilterLength;
    deque<CvScalar> histColorsCode;

    Bottle lastBlobs;
    Bottle memoryBlobs;
    Bottle memoryScores;
    
    Vector skim_blobs_x_bounds;
    Vector skim_blobs_y_bounds;
    Vector histObjLocation;

    friend class RtLocalization;
    friend class MemoryUpdater;

    Bottle  skimBlobs(const Bottle &blobs);
    Bottle  getBlobs();
    CvPoint getBlobCOG(const Bottle &blobs, const int i);
    bool    get3DPosition(const CvPoint &point, Vector &x);
    void    acquireImage(const bool rtlocalization=false);
    void    drawBlobs(const Bottle &blobs, const int i, Bottle *scores=NULL);
    void    rotate(cv::Mat &src, const double angle, cv::Mat &dst);
    void    drawScoresHistogram(const Bottle &blobs, const Bottle &scores, const int i);
    void    updateClassifierInMemory(Classifier *pClassifier);
    void    updateObjCartPosInMemory(const string &object, const Bottle &blobs, const int i);
    int     findClosestBlob(const Bottle &blobs, const CvPoint &loc);
    int     findClosestBlob(const Bottle &blobs, const Vector &loc);
    Bottle  classify(const Bottle &blobs, const bool rtlocalization=false);
    void    train(const string &object, const Bottle &blobs, const int i);
    void    execForget(const string &object);
    void    doLocalization();
    bool    get3DPositionFromMemory(const string &object, Vector &position);
    bool    doExploration(const string &object, const Vector &position);
    void    updateMemory();

    bool    configure(ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    updateModule();
    bool    attach(RpcServer &source);
    double  getPeriod();

public:
    bool    add_name(const string &name);
    bool    remove_name(const string &name);
};

#endif


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

#include <cv.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/ctrl/filters.h>
#include <wrdac/clients/opcClient.h>

#include "utils.h"
#include "iol2opc_IDL.h"

#define RET_INVALID     -1
#define OBJECT_UNKNOWN  "?"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace wysiwyd::wrdac;


/**********************************************************/
class IOL2OPCBridge : public RFModule, public iol2opc_IDL
{
protected:
    RpcServer  rpcPort;
    RpcClient  rpcClassifier;
    RpcClient  rpcGet3D;
    OPCClient *opc;

    BufferedPort<Bottle>             blobExtractor;
    BufferedPort<Bottle>             histObjLocPort;
    BufferedPort<Bottle>             getClickPort;
    BufferedPort<ImageOf<PixelBgr> > imgIn;
    BufferedPort<ImageOf<PixelBgr> > imgRtLocOut;
    BufferedPort<ImageOf<PixelBgr> > imgHistogram;
    Port imgClassifier;

    RtLocalization rtLocalization;
    OpcUpdater opcUpdater;

    ImageOf<PixelBgr> imgRtLoc;
    Mutex mutexResources;
    Mutex mutexResourcesOpc;
    
    string name;

    map<string,Filter*> histFiltersPool;
    int histFilterLength;
    deque<CvScalar> histColorsCode;

    Bottle lastBlobs;
    Bottle opcBlobs;
    Bottle opcScores;
    
    Vector skim_blobs_x_bounds;
    Vector skim_blobs_y_bounds;
    Vector histObjLocation;

    friend class RtLocalization;
    friend class OpcUpdater;

    string  findName(const Bottle &scores, const string &tag);
    Bottle  skimBlobs(const Bottle &blobs);
    Bottle  getBlobs();
    CvPoint getBlobCOG(const Bottle &blobs, const int i);
    bool    get3DPosition(const CvPoint &point, Vector &x);
    bool    getClickPosition(CvPoint &pos);
    void    acquireImage();
    void    drawBlobs(const Bottle &blobs, const int i, const Bottle &scores);
    void    rotate(cv::Mat &src, const double angle, cv::Mat &dst);
    void    drawScoresHistogram(const Bottle &blobs, const Bottle &scores, const int i);
    int     findClosestBlob(const Bottle &blobs, const CvPoint &loc);
    int     findClosestBlob(const Bottle &blobs, const Vector &loc);
    Bottle  classify(const Bottle &blobs);
    void    train(const string &object, const Bottle &blobs, const int i);
    void    doLocalization();
    void    updateOPC();

    bool    configure(ResourceFinder &rf);
    bool    interruptModule();
    bool    close();    
    bool    attach(RpcServer &source);
    double  getPeriod();
    bool    updateModule();

public:
    bool    add_name(const string &name);
    bool    remove_name(const string &name);
};

#endif


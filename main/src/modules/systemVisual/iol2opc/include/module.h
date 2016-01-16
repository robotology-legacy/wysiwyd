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
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <iCub/ctrl/filters.h>
#include <wrdac/clients/opcClient.h>

#include <opencv2/opencv.hpp>
#ifdef IOL2OPC_TRACKING
    #include <opencv2/tracking.hpp>
#endif

#include "utils.h"
#include "iol2opc_IDL.h"

#define RET_INVALID     -1
#define OBJECT_UNKNOWN  "?"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace wysiwyd::wrdac;


/**********************************************************/
namespace Bridge {
    typedef enum { idle, load_database, localization } State;
}


/**********************************************************/
class IOLObject
{
protected:
    MedianFilter filter;
    bool init_filter;
    double presenceTmo;
    double presenceTimer;

    bool tracking;
    double trackerTmo;
    double trackerTimer;    
    cv::Rect2d trackerResult;

#ifdef IOL2OPC_TRACKING
    cv::Ptr<cv::Tracker> tracker;
#endif

public:
    int opc_id;

    /**********************************************************/
    IOLObject(const int filter_order=1, const double presenceTmo_=0.0,
              const double trackerTmo_=0.0) :
              filter(filter_order), init_filter(true),
              presenceTmo(presenceTmo_), tracking(false), trackerTmo(trackerTmo_),
              trackerTimer(0.0), trackerResult(cv::Rect2d(0,0,0,0))
    {
        heartBeat();
    }

    /**********************************************************/
    void heartBeat()
    {
        presenceTimer=Time::now();
    }

    /**********************************************************/
    bool isDead() const
    {
        return (Time::now()-presenceTimer>=presenceTmo);
    }

    /**********************************************************/
    Vector filt(const Vector &x)
    {
        if (init_filter)
        {
            filter.init(x);
            init_filter=false;
            return x;
        }
        else
            return filter.filt(x);
    }

    /**********************************************************/
    void tracker_init(const Image& img, const CvRect& bbox)
    {
        trackerResult=cv::Rect2d(bbox.x,bbox.y,bbox.width,bbox.height);

    #ifdef IOL2OPC_TRACKING
        cv::Mat frame=cv::cvarrToMat((IplImage*)img.getIplImage());

        tracker=cv::Tracker::create("BOOSTING");
        tracker->init(frame,trackerResult);

        trackerTimer=Time::now();
    #endif

        tracking=true;
    }

    /**********************************************************/
    void tracker_update(const Image& img)
    {
    #ifdef IOL2OPC_TRACKING
        tracking=(Time::now()-trackerTimer<trackerTmo);
        if (tracking)
        {
            cv::Mat frame=cv::cvarrToMat((IplImage*)img.getIplImage());
            tracker->update(frame,trackerResult);
        }
    #endif
    }

    /**********************************************************/
    bool is_tracking(CvRect& bbox) const
    {
        bbox=cvRect((int)trackerResult.x,(int)trackerResult.y,
                    (int)trackerResult.width,(int)trackerResult.height);
        return tracking;
    }
};


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
    BufferedPort<ImageOf<PixelBgr> > imgTrackOut;
    BufferedPort<ImageOf<PixelBgr> > imgSelBlobOut;
    BufferedPort<ImageOf<PixelBgr> > imgHistogram;
    Port imgClassifier;

    RtLocalization rtLocalization;
    OpcUpdater opcUpdater;
    int opcMedianFilterOrder;
    ClassifierReporter classifierReporter;

    ImageOf<PixelBgr> imgRtLoc;
    Mutex mutexResources;
    Mutex mutexResourcesOpc;
    Mutex mutexResourcesSFM;

    bool empty;
    double period;
    double presence_timeout;
    double tracker_timeout;
    map<string,IOLObject> db;
    Bridge::State state;
    IOLObject onlyKnownObjects;

    map<string,Filter*> histFiltersPool;
    int histFilterLength;
    deque<CvScalar> histColorsCode;

    Bottle lastBlobs;
    Bottle opcBlobs;
    Bottle opcScores;

    Vector skim_blobs_x_bounds;
    Vector skim_blobs_y_bounds;
    Vector histObjLocation;

    CvPoint clickLocation;

    friend class RtLocalization;
    friend class OpcUpdater;
    friend class ClassifierReporter;

    string  findName(const Bottle &scores, const string &tag);
    Bottle  skimBlobs(const Bottle &blobs);
    Bottle  getBlobs();
    CvPoint getBlobCOG(const Bottle &blobs, const int i);
    bool    get3DPosition(const CvPoint &point, Vector &x);
    bool    get3DPositionAndDimensions(const CvRect &bbox, Vector &x, Vector &dim);
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
    bool    train_object(const string &name);
    bool    remove_object(const string &name);
    bool    remove_all();
    bool    change_name(const string &old_name, const string &new_name);
    void    pause();
    void    resume();
};

#endif


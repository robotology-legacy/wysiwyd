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
#include <opencv2/tracking.hpp>

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
    MedianFilter filterPos;
    MedianFilter filterDim;
    bool init_filters;
    double presenceTmo;
    double presenceTimer;

    enum { idle, init, no_need, tracking };

    string trackerType;
    int trackerState;
    double trackerTmo;
    double trackerTimer;    

    cv::Rect2d trackerResult;
    cv::Ptr<cv::Tracker> tracker;

public:
    /**********************************************************/
    IOLObject(const int filter_order=1, const double presenceTmo_=0.0,
              const string &trackerType_="BOOSTING", const double trackerTmo_=0.0) :
              filterPos(filter_order), filterDim(10*filter_order),
              init_filters(true), presenceTmo(presenceTmo_),
              trackerType(trackerType_), trackerState(idle),
              trackerTmo(trackerTmo_), trackerTimer(0.0)              
    {
        trackerResult.x=trackerResult.y=0;
        trackerResult.width=trackerResult.height=0;
        heartBeat();
    }

    /**********************************************************/
    void heartBeat()
    {
        presenceTimer=Time::now();
    }

    /**********************************************************/
    bool isDead()
    {
        bool dead=(Time::now()-presenceTimer>=presenceTmo);
        if (dead)
            trackerState=idle;
        return dead;
    }

    /**********************************************************/
    void filt(const Vector &x, Vector &xFilt,
              const Vector &d, Vector &dFilt)
    {
        if (init_filters)
        {
            filterPos.init(x); xFilt=x;
            filterDim.init(d); dFilt=d;
            init_filters=false;
        }
        else
        {
            xFilt=filterPos.filt(x);
            dFilt=filterDim.filt(d);
        }
    }

    /**********************************************************/
    void prepare()
    {
        if (trackerState==no_need)
            trackerState=init;
    }

    /**********************************************************/
    void latchBBox(const CvRect& bbox)
    {
        trackerResult.x=bbox.x;
        trackerResult.y=bbox.y;
        trackerResult.width=bbox.width;
        trackerResult.height=bbox.height;
        trackerState=no_need;
    }

    /**********************************************************/
    void track(const Image& img)
    {
        cv::Mat frame=cv::cvarrToMat((IplImage*)img.getIplImage());
        if (trackerState==init)
        {
            tracker=cv::Tracker::create(trackerType);
            tracker->init(frame,trackerResult);
            trackerTimer=Time::now();
            trackerState=tracking;
        }
        else if (trackerState==tracking)
        {
            if (Time::now()-trackerTimer<trackerTmo)                
            {
                tracker->update(frame,trackerResult);
                CvPoint tl=cvPoint((int)trackerResult.x,(int)trackerResult.y);
                CvPoint br=cvPoint(tl.x+(int)trackerResult.width,
                                   tl.y+(int)trackerResult.height);

                if ((tl.x<5) || (br.x>frame.cols-5) ||
                    (tl.y<5) || (br.y>frame.rows-5))
                    trackerState=idle;                    
                else
                    heartBeat();                    
            }
            else
                trackerState=idle;
        }
    }

    /**********************************************************/
    bool is_tracking(CvRect& bbox) const
    {
        bbox=cvRect((int)trackerResult.x,(int)trackerResult.y,
                    (int)trackerResult.width,(int)trackerResult.height);
        return (trackerState!=idle);
    }
};


/**********************************************************/
class IOL2OPCBridge : public RFModule, public iol2opc_IDL
{
protected:
    RpcServer  rpcPort;
    RpcClient  rpcClassifier;
    RpcClient  rpcGet3D;
    RpcClient  rpcCalib;
    OPCClient *opc;

    BufferedPort<Bottle>             blobExtractor;
    BufferedPort<Bottle>             histObjLocPort;
    BufferedPort<Bottle>             getClickPort;
    BufferedPort<Bottle>             objLocOut;
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

    double period;
    bool empty;
    bool object_persistence;
    string calib_entry;    

    double presence_timeout;
    string tracker_type;
    double tracker_timeout;
    VectorOf<int> tracker_min_blob_size;
    map<string,IOLObject> db;
    Bridge::State state;
    IOLObject onlyKnownObjects;

    map<string,Filter*> histFiltersPool;
    int histFilterLength;
    deque<CvScalar> histColorsCode;

    double blobs_detection_timeout;
    double lastBlobsArrivalTime;
    Bottle lastBlobs;
    Bottle opcBlobs;
    Bottle opcScores;

    Vector skim_blobs_x_bounds;
    Vector skim_blobs_y_bounds;
    Vector histObjLocation;

    Vector human_area_x_bounds;
    Vector human_area_y_bounds;
    Vector robot_area_x_bounds;
    Vector robot_area_y_bounds;
    Vector shared_area_x_bounds;
    Vector shared_area_y_bounds;

    CvPoint clickLocation;

    friend class RtLocalization;
    friend class OpcUpdater;
    friend class ClassifierReporter;

    string  findName(const Bottle &scores, const string &tag);
    Bottle  skimBlobs(const Bottle &blobs);
    bool    thresBBox(CvRect &bbox, const Image &img);
    Bottle  getBlobs();
    CvPoint getBlobCOG(const Bottle &blobs, const int i);
    bool    get3DPosition(const CvPoint &point, Vector &x);
    bool    get3DPositionAndDimensions(const CvRect &bbox, Vector &x, Vector &dim);
    Vector  calibPosition(const Vector &x);
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
    void    setBounds(ResourceFinder &rf, Vector &bounds, string configName, double std_lower, double std_upper);
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
    bool    set_object_persistence(const string &sw);
    string  get_object_persistence();
    void    pause();
    void    resume();
};

#endif


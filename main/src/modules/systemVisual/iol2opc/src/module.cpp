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

#include <cmath>
#include <limits>
#include <sstream>
#include <cstdio>
#include <algorithm>
#include <set>

#include "module.h"


/**********************************************************/
string IOL2OPCBridge::findName(const Bottle &scores,
                               const string &tag)
{
    string retName=OBJECT_UNKNOWN;
    double maxScore=0.0;

    Bottle *blobScores=scores.find(tag.c_str()).asList();
    if (blobScores==NULL)
        return retName;

    // first find the most likely object for the given blob
    for (int i=0; i<blobScores->size(); i++)
    {
        Bottle *item=blobScores->get(i).asList();
        if (item==NULL)
            continue;

        string name=item->get(0).asString().c_str();
        double score=item->get(1).asDouble();
        if (score>maxScore)
        {
            maxScore=score;
            retName=name;
        }
    }

    // then double-check that the found object remains the best
    // prediction over the remaining blobs
    if (retName!=OBJECT_UNKNOWN)
    {
        for (int i=0; i<scores.size(); i++)
        {
            if (Bottle *blob=scores.get(i).asList())
            {
                // skip the blob under examination
                string name=blob->get(0).asString().c_str();
                Bottle *blobScores=blob->get(1).asList();
                if ((name==tag) || (blobScores==NULL))
                    continue;

                if (blobScores->find(retName.c_str()).asDouble()>=maxScore)
                    return OBJECT_UNKNOWN;
            }
        }
    }

    return retName;
}


/**********************************************************/
Bottle IOL2OPCBridge::skimBlobs(const Bottle &blobs)
{
    Bottle skimmedBlobs;
    for (int i=0; i<blobs.size(); i++)
    {
        CvPoint cog=getBlobCOG(blobs,i);
        if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
            continue;

        // skim out blobs that are too far in the cartesian space
        Vector x;
        if (get3DPosition(cog,x))
        {
            if ((x[0]>skim_blobs_x_bounds[0]) && (x[0]<skim_blobs_x_bounds[1]) &&
                (x[1]>skim_blobs_y_bounds[0]) && (x[1]<skim_blobs_y_bounds[1]))
                skimmedBlobs.add(blobs.get(i));
        }
    }

    return skimmedBlobs;
}


/**********************************************************/
bool IOL2OPCBridge::thresBBox(CvRect &bbox, const Image &img)
{
    CvPoint tl=cvPoint(bbox.x,bbox.y);
    CvPoint br=cvPoint(tl.x+bbox.width,tl.y+bbox.height);
    tl.x=std::min(img.width(),std::max(tl.x,0));
    tl.y=std::min(img.height(),std::max(tl.y,0));
    br.x=std::min(img.width(),std::max(br.x,0));
    br.y=std::min(img.height(),std::max(br.y,0));

    bbox=cvRect(tl.x,tl.y,br.x-tl.x,br.y-tl.y);
    if ((bbox.width>tracker_min_blob_size[0]) &&
        (bbox.height>tracker_min_blob_size[1]))
        return true;
    else
        return false;
}


/**********************************************************/
Bottle IOL2OPCBridge::getBlobs()
{
    // grab resources
    mutexResources.lock();

    if (Bottle *pBlobs=blobExtractor.read(false))
    {
        lastBlobsArrivalTime=Time::now();
        lastBlobs=skimBlobs(*pBlobs);
        yInfo("Received blobs list: %s",lastBlobs.toString().c_str());

        if (lastBlobs.size()==1)
        {
            if (lastBlobs.get(0).asVocab()==Vocab::encode("empty"))
                lastBlobs.clear();
        }        
    }
    else if (Time::now()-lastBlobsArrivalTime>blobs_detection_timeout)
        lastBlobs.clear();

    // release resources
    mutexResources.unlock();

    return lastBlobs;
}


/**********************************************************/
CvPoint IOL2OPCBridge::getBlobCOG(const Bottle &blobs, const int i)
{
    CvPoint cog=cvPoint(RET_INVALID,RET_INVALID);
    if ((i>=0) && (i<blobs.size()))
    {
        CvPoint tl,br;
        Bottle *item=blobs.get(i).asList();
        if (item==NULL)
            return cog;

        tl.x=(int)item->get(0).asDouble();
        tl.y=(int)item->get(1).asDouble();
        br.x=(int)item->get(2).asDouble();
        br.y=(int)item->get(3).asDouble();

        cog.x=(tl.x+br.x)>>1;
        cog.y=(tl.y+br.y)>>1;
    }

    return cog;
}


/**********************************************************/
bool IOL2OPCBridge::getClickPosition(CvPoint &pos)
{    
    if (Bottle *bPos=getClickPort.read(false))
    {
        if (bPos->size()>=2)
        {
            clickLocation.x=bPos->get(0).asInt();
            clickLocation.y=bPos->get(1).asInt();
            yInfo("Received new click location: (%d,%d)",
                  clickLocation.x,clickLocation.y);
        }
        else
            yError("Mis-sized bottle for click location");
    }

    pos=clickLocation;
    return ((clickLocation.x!=RET_INVALID) &&
            (clickLocation.y!=RET_INVALID));
}


/**********************************************************/
bool IOL2OPCBridge::get3DPosition(const CvPoint &point, Vector &x)
{
    if (rpcGet3D.getOutputCount()>0)
    {
        Bottle cmd,reply;
        cmd.addString("Root");
        cmd.addInt(point.x);
        cmd.addInt(point.y);
        mutexResourcesSFM.lock();
        rpcGet3D.write(cmd,reply);
        mutexResourcesSFM.unlock();

        if (reply.size()>=3)
        {
            x.resize(3);
            x[0]=reply.get(0).asDouble();
            x[1]=reply.get(1).asDouble();
            x[2]=reply.get(2).asDouble();
            return (norm(x)>0.0);
        }
    }

    return false;
}


/**********************************************************/
bool IOL2OPCBridge::get3DPositionAndDimensions(const CvRect &bbox,
                                               Vector &x,
                                               Vector &dim)
{
    if (rpcGet3D.getOutputCount()>0)
    {
        Bottle cmd,reply;
        cmd.addString("Rect");
        cmd.addInt(bbox.x);
        cmd.addInt(bbox.y);
        cmd.addInt(bbox.width);
        cmd.addInt(bbox.height);
        cmd.addInt(2);
        mutexResourcesSFM.lock();
        rpcGet3D.write(cmd,reply);
        mutexResourcesSFM.unlock();

        x.resize(3);
        dim.resize(3);

        // find mean and standard deviation
        Vector tmp(3);
        for (int i=0; i<reply.size(); i+=3)
        {
            tmp[0]=reply.get(i+0).asDouble();
            tmp[1]=reply.get(i+1).asDouble();
            tmp[2]=reply.get(i+2).asDouble();

            x+=tmp;
            dim+=tmp*tmp;
        }
        
        double N=reply.size()/3.0;
        x/=N; dim=dim/N-x*x;
        dim[0]=2.0*sqrt(dim[0]);
        dim[1]=2.0*sqrt(dim[1]);
        dim[2]=2.0*sqrt(dim[2]);

        return true;
    }

    return false;
}


/**********************************************************/
Vector IOL2OPCBridge::calibPosition(const Vector &x)
{
    Vector y=x;

    // apply 3D correction
    if (rpcCalib.getOutputCount()>0)
    {
        yarp::os::Bottle cmd,reply;
        cmd.addString("get_location_nolook");
        cmd.addString(calib_entry);
        cmd.addDouble(y[0]);
        cmd.addDouble(y[1]);
        cmd.addDouble(y[2]);
        rpcCalib.write(cmd,reply);
        y[0]=reply.get(1).asDouble();
        y[1]=reply.get(2).asDouble();
        y[2]=reply.get(3).asDouble();
    }
    else
        yError("Unable to connect to calibrator");

    return y;
}


/**********************************************************/
void IOL2OPCBridge::acquireImage()
{
    // grab resources
    mutexResources.lock();

    // wait for incoming image
    if (ImageOf<PixelBgr> *tmp=imgIn.read())
        imgRtLoc=*tmp;

    // release resources
    mutexResources.unlock();
}


/**********************************************************/
void IOL2OPCBridge::drawBlobs(const Bottle &blobs, const int i,
                              const Bottle &scores)
{
    // grab resources
    mutexResources.lock();

    if (imgRtLocOut.getOutputCount()>0)
    {
        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1);

        // latch image
        ImageOf<PixelBgr> imgLatch=this->imgRtLoc;
        for (int j=0; j<blobs.size(); j++)
        {
            CvPoint tl,br,txtLoc;
            Bottle *item=blobs.get(j).asList();
            tl.x=(int)item->get(0).asDouble();
            tl.y=(int)item->get(1).asDouble();
            br.x=(int)item->get(2).asDouble();
            br.y=(int)item->get(3).asDouble();
            txtLoc.x=tl.x;
            txtLoc.y=tl.y-5;

            ostringstream tag;
            tag<<"blob_"<<j;

            // find the blob name (or unknown)
            string object=findName(scores,tag.str());
            tag.str("");
            tag.clear();
            tag<<object;

            CvScalar highlight=cvScalar(0,255,0);
            CvScalar lowlight=cvScalar(150,125,125);
            cvRectangle(imgLatch.getIplImage(),tl,br,(j==i)?highlight:lowlight,2);
            cvPutText(imgLatch.getIplImage(),tag.str().c_str(),txtLoc,&font,(j==i)?highlight:lowlight);
        }

        imgRtLocOut.prepare()=imgLatch;
        imgRtLocOut.write();
    }

    // release resources
    mutexResources.unlock();
}


/**********************************************************/
void IOL2OPCBridge::rotate(cv::Mat &src, const double angle,
                           cv::Mat &dst)
{
    int len=std::max(src.cols,src.rows);
    cv::Point2f pt(len/2.0f,len/2.0f);
    cv::Mat r=cv::getRotationMatrix2D(pt,angle,1.0);
    cv::warpAffine(src,dst,r,cv::Size(len,len));
}


/**********************************************************/
void IOL2OPCBridge::drawScoresHistogram(const Bottle &blobs,
                                        const Bottle &scores,
                                        const int i)
{
    if (imgHistogram.getOutputCount()>0)
    {
        // grab resources
        mutexResources.lock();

        // create image containing histogram
        ImageOf<PixelBgr> imgConf;
        imgConf.resize(600,600);
        imgConf.zero();

        ostringstream tag;
        tag<<"blob_"<<i;

        // process scores on the given blob
        if (Bottle *blobScores=scores.find(tag.str().c_str()).asList())
        {
            // set up some variables and constraints
            int maxHeight=(int)(imgConf.height()*0.8);
            int minHeight=imgConf.height()-20;
            int widthStep=(blobScores->size()>0)?(int)(imgConf.width()/blobScores->size()):0;
            set<string> gcFilters;

            // cycle over classes
            for (int j=0; j<blobScores->size(); j++)
            {
                Bottle *item=blobScores->get(j).asList();
                if (item==NULL)
                    continue;

                string name=item->get(0).asString().c_str();
                double score=std::max(std::min(item->get(1).asDouble(),1.0),0.0);

                // smooth out quickly varying scores
                map<string,Filter*>::iterator it=histFiltersPool.find(name);

                // create filter if not available
                if (it==histFiltersPool.end())
                {
                    Vector num(histFilterLength,1.0);
                    Vector den(histFilterLength,0.0); den[0]=histFilterLength;
                    histFiltersPool[name]=new Filter(num,den,Vector(1,score));
                }
                else
                {
                    Vector scoreFilt=it->second->filt(Vector(1,score));
                    score=scoreFilt[0];
                }

                // put the class name in a convenient set for garbage collection
                gcFilters.insert(name);

                int classHeight=std::min(minHeight,imgConf.height()-(int)(maxHeight*score));

                cvRectangle(imgConf.getIplImage(),cvPoint(j*widthStep,classHeight),cvPoint((j+1)*widthStep,minHeight),
                            histColorsCode[j%(int)histColorsCode.size()],CV_FILLED);

                cv::Mat textImg=cv::Mat::zeros(imgConf.height(),imgConf.width(),CV_8UC3);
                cv::putText(textImg,name.c_str(),cvPoint(imgConf.width()-580,(j+1)*widthStep-10),
                            cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(255,255,255),2);
                rotate(textImg,90.0,textImg);

                cv::Mat orig=cv::cvarrToMat((IplImage*)imgConf.getIplImage());
                orig=orig+textImg;
            }

            // draw the blob snapshot
            CvPoint tl,br,sz;
            Bottle *item=blobs.get(i).asList();
            tl.x=(int)item->get(0).asDouble();
            tl.y=(int)item->get(1).asDouble();
            br.x=(int)item->get(2).asDouble();
            br.y=(int)item->get(3).asDouble();
            sz.x=br.x-tl.x;
            sz.y=br.y-tl.y;

            // copy the blob
            ImageOf<PixelBgr> imgTmp1;
            imgTmp1.resize(sz.x,sz.y);
            cvSetImageROI((IplImage*)imgRtLoc.getIplImage(),cvRect(tl.x,tl.y,sz.x,sz.y));
            cvCopy(imgRtLoc.getIplImage(),imgTmp1.getIplImage());
            cvResetImageROI((IplImage*)imgRtLoc.getIplImage());

            // resize the blob
            ImageOf<PixelBgr> imgTmp2;
            int f=2;    // magnifying factor
            imgTmp2.resize(f*imgTmp1.width(),f*imgTmp1.height());
            cvResize(imgTmp1.getIplImage(),imgTmp2.getIplImage());

            // superimpose the blob on the histogram
            cvSetImageROI((IplImage*)imgConf.getIplImage(),cvRect(0,0,imgTmp2.width(),imgTmp2.height()));
            cvCopy(imgTmp2.getIplImage(),imgConf.getIplImage());
            cvRectangle(imgConf.getIplImage(),cvPoint(0,0),cvPoint(imgTmp2.width(),imgTmp2.height()),cvScalar(255,255,255),3);

            // give chance for disposing filters that are no longer used (one at time)
            if ((int)histFiltersPool.size()>blobScores->size())
            {
                for (auto& it : histFiltersPool)
                {
                    if (gcFilters.find(it.first)==gcFilters.end())
                    {
                        delete it.second;
                        histFiltersPool.erase(it.first);
                        break;
                    }
                }
            }

        }

        imgHistogram.prepare()=imgConf;
        imgHistogram.write();

        // release resources
        mutexResources.unlock();
    }
}


/**********************************************************/
int IOL2OPCBridge::findClosestBlob(const Bottle &blobs,
                                   const CvPoint &loc)
{
    int ret=RET_INVALID;
    double min_d2=std::numeric_limits<double>::max();

    for (int i=0; i<blobs.size(); i++)
    {
        CvPoint cog=getBlobCOG(blobs,i);
        if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
            continue;

        double dx=loc.x-cog.x;
        double dy=loc.y-cog.y;
        double d2=dx*dx+dy*dy;

        if (d2<min_d2)
        {
            min_d2=d2;
            ret=i;
        }
    }

    return ret;
}


/**********************************************************/
int IOL2OPCBridge::findClosestBlob(const Bottle &blobs,
                                   const Vector &loc)
{
    int ret=RET_INVALID;
    double curMinDist=std::numeric_limits<double>::max();
    
    for (int i=0; i<blobs.size(); i++)
    {
        CvPoint cog=getBlobCOG(blobs,i);
        if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
            continue;

        Vector x;
        if (get3DPosition(cog,x))
        {
            double dist=norm(loc-x);
            if (dist<curMinDist)
            {
                ret=i;
                curMinDist=dist;
            }
        }
    }

    return ret;
}


/**********************************************************/
Bottle IOL2OPCBridge::classify(const Bottle &blobs)
{
    // grab resources
    mutexResources.lock();

    imgClassifier.write(imgRtLoc);

    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("classify"));
    Bottle &options=cmd.addList();
    for (int i=0; i<blobs.size(); i++)
    {
        ostringstream tag;
        tag<<"blob_"<<i;
        Bottle &item=options.addList();
        item.addString(tag.str().c_str());
        item.addList()=*blobs.get(i).asList();
    }
    yInfo("Sending classification request: %s",cmd.toString().c_str());
    rpcClassifier.write(cmd,reply);
    yInfo("Received reply: %s",reply.toString().c_str());

    // release resources
    mutexResources.unlock();

    return reply;
}


/**********************************************************/
void IOL2OPCBridge::train(const string &object, const Bottle &blobs,
                          const int i)
{
    // grab resources
    mutexResources.lock();

    imgClassifier.write(imgRtLoc);

    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("train"));
    Bottle &options=cmd.addList().addList();
    options.addString(object.c_str());

    if (i<0)
    {
        Bottle &noBlob=options.addList();
        noBlob.addDouble(0.0);
        noBlob.addDouble(0.0);
        noBlob.addDouble(0.0);
        noBlob.addDouble(0.0);
    }
    else
        options.add(blobs.get(i));

    yInfo("Sending training request: %s",cmd.toString().c_str());
    rpcClassifier.write(cmd,reply);
    yInfo("Received reply: %s",reply.toString().c_str());

    // release resources
    mutexResources.unlock();
}


/**********************************************************/
void IOL2OPCBridge::doLocalization()
{
    if (state==Bridge::localization)
    {
        // acquire image for classification/training
        acquireImage();
        // grab the blobs
        Bottle blobs=getBlobs();
        // get the scores from the learning machine
        Bottle scores=classify(blobs);
        // update location of histogram display
        if (Bottle *loc=histObjLocPort.read(false))
        {
            if (loc->size()>=2)
            {
                Vector x;
                if (get3DPosition(cvPoint(loc->get(0).asInt(),loc->get(1).asInt()),x))
                    histObjLocation=x;
            }
        }
        // find the closest blob to the location of histogram display
        int closestBlob=findClosestBlob(blobs,histObjLocation);
        // draw the blobs
        drawBlobs(blobs,closestBlob,scores);
        // draw scores histogram
        drawScoresHistogram(blobs,scores,closestBlob);

        // data for opc update
        mutexResourcesOpc.lock();
        opcBlobs=blobs;
        opcScores=scores;
        mutexResourcesOpc.unlock();
    }
}


/**********************************************************/
void IOL2OPCBridge::updateOPC()
{
    if ((state==Bridge::localization) && opc->isConnected())
    {
        // grab resources
        LockGuard lg(mutexResources);
        opc->checkout();

        // latch image
        ImageOf<PixelBgr> &imgLatch=imgTrackOut.prepare();
        imgLatch=imgRtLoc;

        mutexResourcesOpc.lock();
        Bottle blobs=opcBlobs;
        Bottle scores=opcScores;
        mutexResourcesOpc.unlock();

        // reset internal tracking state
        for (auto& it : db)
            it.second.prepare();

        // check detected objects
        bool unknownObjectInScene=false;
        for (int j=0; j<blobs.size(); j++)
        {
            Bottle *item=blobs.get(j).asList();
            if (item==NULL)
                continue;

            ostringstream tag;
            tag<<"blob_"<<j;

            // find the blob name (or unknown)
            string object=findName(scores,tag.str());
            if (object!=OBJECT_UNKNOWN)
            {
                auto it=db.find(object);
                if (it!=db.end())
                {
                    CvPoint cog=getBlobCOG(blobs,j);
                    if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
                        continue;

                    // compute the bounding box
                    CvPoint tl,br;
                    tl.x=(int)item->get(0).asDouble();
                    tl.y=(int)item->get(1).asDouble();
                    br.x=(int)item->get(2).asDouble();
                    br.y=(int)item->get(3).asDouble();

                    CvRect bbox=cvRect(tl.x,tl.y,br.x-tl.x,br.y-tl.y);
                    if (thresBBox(bbox,imgLatch))
                    {
                        it->second.latchBBox(bbox);
                        it->second.heartBeat();
                    }
                }
            }
            else
                unknownObjectInScene=true;
        }

        // cycle over objects to handle tracking
        for (auto& it : db)
            it.second.track(imgLatch);

        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1);                

        // perform operations
        for (auto& it : db)
        {
            string object=it.first;
            Object *obj=opc->addOrRetrieveEntity<Object>(object);

            // garbage collection
            if (it.second.isDead())
            {
                if (object_persistence)
                {
                    if (obj->m_present!=0.0)
                        obj->m_present=0.5; 
                }
                else
                    obj->m_present=0.0; 
                continue;
            }

            CvRect bbox;
            if (it.second.is_tracking(bbox))
            {
                // find 3d position
                Vector x,dim;
                if (get3DPositionAndDimensions(bbox,x,dim))
                {
                    Vector x_filtered,dim_filtered;
                    it.second.filt(x,x_filtered,dim,dim_filtered);

                    obj->m_ego_position=calibPosition(x_filtered);
                    obj->m_dimensions=dim_filtered;
                    obj->m_present=1.0;

                    // threshold bbox
                    if (thresBBox(bbox,imgLatch))
                    {
                        // Extract color information from blob
                        // create temporary image, and copy blob in there
                        ImageOf<PixelBgr> imgTmp;
                        imgTmp.resize(bbox.width,bbox.height);
                        cvSetImageROI((IplImage*)imgLatch.getIplImage(),bbox);
                        cvCopy(imgLatch.getIplImage(),imgTmp.getIplImage());
                        cvResetImageROI((IplImage*)imgLatch.getIplImage());

                        // now get mean color of blob, and fill the OPC object with the information
                        // be careful: computation done in BGR => save in RGB for displaying purpose
                        CvScalar meanColor=cvAvg(imgTmp.getIplImage());
                        obj->m_color[0]=(int)meanColor.val[2];
                        obj->m_color[1]=(int)meanColor.val[1];
                        obj->m_color[2]=(int)meanColor.val[0];

                        cvRectangle(imgLatch.getIplImage(),cvPoint(bbox.x,bbox.y),
                                    cvPoint(bbox.x+bbox.width,bbox.y+bbox.height),cvScalar(255,0,0),2);

                        cvPutText(imgLatch.getIplImage(),object.c_str(),
                                  cvPoint(bbox.x,bbox.y-5),&font,cvScalar(255,0,0));
                    }
                }
            }
        }

        if (!unknownObjectInScene)
            onlyKnownObjects.heartBeat();

        opc->commit();

        if (imgTrackOut.getOutputCount()>0)
            imgTrackOut.write();
        else
            imgTrackOut.unprepare();
    }
}


/**********************************************************/
bool IOL2OPCBridge::configure(ResourceFinder &rf)
{
    string name=rf.check("name",Value("iol2opc")).asString().c_str();
    period=rf.check("period",Value(0.1)).asDouble();
    empty=rf.check("empty");
    object_persistence=(rf.check("object_persistence",Value("off")).asString()=="on");
    calib_entry=rf.check("calib_entry",Value("iol-right")).asString();

    opc=new OPCClient(name);
    if (!opc->connect(rf.check("opcName",Value("OPC")).asString().c_str()))
    {
        yError("OPC doesn't seem to be running!");
        return false;
    }
    opc->checkout();

    imgIn.open(("/"+name+"/img:i").c_str());
    blobExtractor.open(("/"+name+"/blobs:i").c_str());
    imgRtLocOut.open(("/"+name+"/imgLoc:o").c_str());
    imgTrackOut.open(("/"+name+"/imgTrack:o").c_str());
    imgSelBlobOut.open(("/"+name+"/imgSel:o").c_str());
    imgClassifier.open(("/"+name+"/imgClassifier:o").c_str());
    imgHistogram.open(("/"+name+"/imgHistogram:o").c_str());
    histObjLocPort.open(("/"+name+"/histObjLocation:i").c_str());

    rpcPort.open(("/"+name+"/rpc").c_str());
    rpcClassifier.open(("/"+name+"/classify:rpc").c_str());
    rpcGet3D.open(("/"+name+"/get3d:rpc").c_str());
    rpcCalib.open(("/"+name+"/calib:rpc").c_str());
    getClickPort.open(("/"+name+"/getClick:i").c_str());

    skim_blobs_x_bounds.resize(2);
    skim_blobs_x_bounds[0]=-0.50;
    skim_blobs_x_bounds[1]=-0.10;
    if (rf.check("skim_blobs_x_bounds"))
    {
        if (Bottle *bounds=rf.find("skim_blobs_x_bounds").asList())
        {
            if (bounds->size()>=2)
            {
                skim_blobs_x_bounds[0]=bounds->get(0).asDouble();
                skim_blobs_x_bounds[1]=bounds->get(1).asDouble();
            }
        }
    }

    skim_blobs_y_bounds.resize(2);
    skim_blobs_y_bounds[0]=-0.30;
    skim_blobs_y_bounds[1]=+0.30;
    if (rf.check("skim_blobs_y_bounds"))
    {
        if (Bottle *bounds=rf.find("skim_blobs_y_bounds").asList())
        {
            if (bounds->size()>=2)
            {
                skim_blobs_y_bounds[0]=bounds->get(0).asDouble();
                skim_blobs_y_bounds[1]=bounds->get(1).asDouble();
            }
        }
    }

    // location used to display the
    // histograms upon the closest blob
    histObjLocation.resize(3);
    histObjLocation[0]=-0.3;
    histObjLocation[1]=0.0;
    histObjLocation[2]=-0.1;

    rtLocalization.setBridge(this);
    rtLocalization.setRate(rf.check("rt_localization_period",Value(30)).asInt());

    opcUpdater.setBridge(this);
    opcUpdater.setRate(rf.check("opc_update_period",Value(60)).asInt());
    opcMedianFilterOrder=rf.check("opc_median_window",Value(5)).asInt();

    classifierReporter.setBridge(this);

    blobs_detection_timeout=rf.check("blobs_detection_timeout",Value(0.2)).asDouble();
    histFilterLength=std::max(1,rf.check("hist_filter_length",Value(10)).asInt());
    presence_timeout=std::max(0.0,rf.check("presence_timeout",Value(1.0)).asDouble());
    tracker_type=rf.check("tracker_type",Value("BOOSTING")).asString().c_str();
    tracker_timeout=std::max(0.0,rf.check("tracker_timeout",Value(5.0)).asDouble());

    tracker_min_blob_size.resize(2,0);
    if (rf.check("tracker_min_blob_size"))
    {
        if (Bottle *size=rf.find("tracker_min_blob_size").asList())
        {
            if (size->size()>=2)
            {
                tracker_min_blob_size[0]=size->get(0).asInt();
                tracker_min_blob_size[1]=size->get(1).asInt();
            }
        }
    }

    imgRtLoc.resize(320,240);
    imgRtLoc.zero();

    histColorsCode.push_back(cvScalar( 65, 47,213));
    histColorsCode.push_back(cvScalar(122, 79, 58));
    histColorsCode.push_back(cvScalar(154,208, 72));
    histColorsCode.push_back(cvScalar( 71,196,249));
    histColorsCode.push_back(cvScalar(224,176, 96));
    histColorsCode.push_back(cvScalar( 22,118,238));

    clickLocation=cvPoint(RET_INVALID,RET_INVALID);

    attach(rpcPort);

    lastBlobsArrivalTime=0.0;
    rtLocalization.start();
    opcUpdater.start();

    state=Bridge::idle;
    rpcClassifier.setReporter(classifierReporter);
    return true;
}


/**********************************************************/
bool IOL2OPCBridge::interruptModule()
{
    yDebug() << "Interrupt iol2opc";
    LockGuard lg(mutexResources);

    rtLocalization.stop();
    opcUpdater.stop();
    yDebug() << "iol2opc threads stopped";

    imgIn.interrupt();
    imgRtLocOut.interrupt();
    imgTrackOut.interrupt();
    imgSelBlobOut.interrupt();
    imgClassifier.interrupt();
    imgHistogram.interrupt();
    histObjLocPort.interrupt();
    rpcPort.interrupt();

    yDebug() << "iol2opc half way through interrupt";

    blobExtractor.interrupt();
    rpcClassifier.interrupt();
    getClickPort.interrupt();
    rpcGet3D.interrupt();
    rpcCalib.interrupt();
    opc->interrupt();
    yDebug() << "Interrupt finished";

    return true;
}


/**********************************************************/
bool IOL2OPCBridge::close()
{
    yDebug() << "Close iol2opc";
    LockGuard lg(mutexResources);

    rtLocalization.stop();
    opcUpdater.stop();

    rpcPort.interrupt();
    rpcPort.close();
    imgIn.interrupt();
    imgIn.close();
    imgRtLocOut.interrupt();
    imgRtLocOut.close();
    imgTrackOut.interrupt();
    imgTrackOut.close();
    imgSelBlobOut.interrupt();
    imgSelBlobOut.close();
    imgClassifier.interrupt();
    imgClassifier.close();
    imgHistogram.interrupt();
    imgHistogram.close();
    histObjLocPort.interrupt();
    histObjLocPort.close();
    blobExtractor.interrupt();
    blobExtractor.close();
    rpcClassifier.interrupt();
    rpcClassifier.close();
    getClickPort.interrupt();
    getClickPort.close();
    rpcGet3D.interrupt();
    rpcGet3D.close();
    rpcCalib.interrupt();
    rpcCalib.close();
    opc->interrupt();
    opc->close();

    delete opc;

    // dispose filters used for scores histogram
    for (auto& it : histFiltersPool)
        delete it.second;

    return true;
}


/**********************************************************/
double IOL2OPCBridge::getPeriod()
{
    return period;
}


/**********************************************************/
bool IOL2OPCBridge::updateModule()
{
    if (state==Bridge::load_database)
    {
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("list"));
        yInfo("Sending list request: %s",cmd.toString().c_str());
        rpcClassifier.write(cmd,reply);
        yInfo("Received reply: %s",reply.toString().c_str());

        if (reply.get(0).asString()=="ack")
        {
            if (Bottle *names=reply.get(1).asList())
                for (int i=0; i<names->size(); i++)
                    db[names->get(i).asString().c_str()]=IOLObject(opcMedianFilterOrder,presence_timeout,
                                                                   tracker_type,tracker_timeout);

            if (empty)
                remove_all();

            yInfo("Turning localization on");
            state=Bridge::localization;
            onlyKnownObjects=IOLObject(opcMedianFilterOrder,10.0);
        }
    }
    // highlight selected blob
    else if (state==Bridge::localization)
    {
        CvPoint loc;
        if (imgSelBlobOut.getOutputCount()>0)
        {
            mutexResourcesOpc.lock();
            Bottle blobs=opcBlobs;
            mutexResourcesOpc.unlock();

            mutexResources.lock();
            ImageOf<PixelBgr> imgLatch=this->imgRtLoc;
            mutexResources.unlock();

            if (getClickPosition(loc))
            {
                int i=findClosestBlob(blobs,loc);
                if (i!=RET_INVALID)
                {
                    // latch image
                    CvPoint tl,br;
                    Bottle *item=blobs.get(i).asList();
                    tl.x=(int)item->get(0).asDouble();
                    tl.y=(int)item->get(1).asDouble();
                    br.x=(int)item->get(2).asDouble();
                    br.y=(int)item->get(3).asDouble();

                    cvRectangle(imgLatch.getIplImage(),tl,br,cvScalar(0,255,0),2);
                }
            }

            imgSelBlobOut.prepare()=imgLatch;
            imgSelBlobOut.write();
        }

        if (onlyKnownObjects.isDead())
        {
            // grab resources
            mutexResourcesOpc.lock();
            Bottle blobs=opcBlobs;
            Bottle scores=opcScores;
            mutexResourcesOpc.unlock();

            for (int j=0; j<blobs.size(); j++)
            {
                ostringstream tag;
                tag<<"blob_"<<j;

                // find the blob name (or unknown)
                string object=findName(scores,tag.str());
                if (object==OBJECT_UNKNOWN)
                {
                    mutexResources.lock();
                    opc->checkout();
                    Object* obj=opc->addEntity<Object>("unknown_object");
                    opc->commit(obj);
                    mutexResources.unlock();

                    db[obj->name()]=IOLObject(opcMedianFilterOrder,presence_timeout,
                                              tracker_type,tracker_timeout);
                    train(obj->name(),blobs,j);                    
                    break;
                }
            }

            onlyKnownObjects.heartBeat();
        }
    }

    return true;
}


/**********************************************************/
bool IOL2OPCBridge::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}


/**********************************************************/
bool IOL2OPCBridge::train_object(const string &name)
{
    if (!opc->isConnected())
    {
        yError("No connection to OPC");
        return false;
    }

    CvPoint loc;
    if (getClickPosition(loc))
    {
        mutexResourcesOpc.lock();
        Bottle blobs=opcBlobs;
        mutexResourcesOpc.unlock();

        int i=findClosestBlob(blobs,loc);
        if (i==RET_INVALID)
            return false;

        train(name,blobs,i);

        // grab resources
        LockGuard lg(mutexResources);

        // add a new object in the database
        // if not already existing
        if (db.find(name)==db.end())
            db[name]=IOLObject(opcMedianFilterOrder,presence_timeout,
                               tracker_type,tracker_timeout);

        return true;
    }
    else
    {
        yError("Could not retrieve click location");
        return false;
    }
}


/**********************************************************/
bool IOL2OPCBridge::remove_object(const string &name)
{
    if (!opc->isConnected())
    {
        yError("No connection to OPC");
        return false;
    }

    // grab resources
    LockGuard lg(mutexResources);

    Bottle cmdClassifier,replyClassifier;
    cmdClassifier.addVocab(Vocab::encode("forget"));
    cmdClassifier.addString(name.c_str());
    yInfo("Sending clearing request: %s",cmdClassifier.toString().c_str());
    rpcClassifier.write(cmdClassifier,replyClassifier);
    yInfo("Received reply: %s",replyClassifier.toString().c_str());

    opc->checkout();

    bool success = false;
    auto it=db.find(name);
    if (it!=db.end()) {
        success = opc->removeEntity(it->first);
        db.erase(it);
    }

    return success;
}


/**********************************************************/
bool IOL2OPCBridge::remove_all()
{
    if (!opc->isConnected())
    {
        yError("No connection to OPC");
        return false;
    }

    // grab resources
    LockGuard lg(mutexResources);

    Bottle cmdClassifier,replyClassifier;
    cmdClassifier.addVocab(Vocab::encode("forget"));
    cmdClassifier.addString("all");
    yInfo("Sending clearing request: %s",cmdClassifier.toString().c_str());
    rpcClassifier.write(cmdClassifier,replyClassifier);
    yInfo("Received reply: %s",replyClassifier.toString().c_str());

    opc->checkout();

    bool success = true;
    for (auto& it : db)
        success &= opc->removeEntity(it.first);

    db.clear();
    return success;
}


/**********************************************************/
bool IOL2OPCBridge::change_name(const string &old_name,
                                const string &new_name)
{
    if (!opc->isConnected())
    {
        yError("No connection to OPC");
        return false;
    }

    // grab resources
    LockGuard lg(mutexResources);

    Bottle cmdClassifier,replyClassifier;
    cmdClassifier.addVocab(VOCAB4('c','h','n','a'));
    cmdClassifier.addString(old_name);
    cmdClassifier.addString(new_name);
    yInfo("Sending change name request: %s",cmdClassifier.toString().c_str());
    rpcClassifier.write(cmdClassifier,replyClassifier);
    yInfo("Received reply: %s",replyClassifier.toString().c_str());

    if (replyClassifier.get(0).asString()=="nack")
    {
        yError("Classifier did not allow name change.");
        yError("Is there already an object with this name in the classifier database?");
        return false;
    }
    else if (new_name!=old_name)
    {   
        const auto it_new=db.find(new_name);
        if (it_new!=db.end())
        {
            yError("\"%s\" is already existing in the database",new_name.c_str());
            return false;
        }

        const auto it_old=db.find(old_name);
        if (it_old!=db.end())
        {
            db[new_name]=IOLObject(opcMedianFilterOrder,presence_timeout,
                                   tracker_type,tracker_timeout);
            opc->checkout();
            opc->removeEntity(it_old->first);
            db.erase(it_old);
            yInfo("Name change successful: reloading local cache");
        }
        else
        {
            yError("\"%s\" not present in the database",old_name.c_str());
            return false;
        }
    }
    else
    {
        yWarning("matching names: no operations performed");
        return true;
    }

    return true;
}


/**********************************************************/
bool IOL2OPCBridge::set_object_persistence(const string &sw)
{
    if (sw=="on")
        object_persistence=true;
    else if (sw=="off")
        object_persistence=false;
    else
        return false;

    return true;
}


/**********************************************************/
string IOL2OPCBridge::get_object_persistence()
{
    return (object_persistence?"on":"off");
}


/**********************************************************/
void IOL2OPCBridge::pause()
{
    mutexResources.lock();
}


/**********************************************************/
void IOL2OPCBridge::resume()
{
    mutexResources.unlock();
}



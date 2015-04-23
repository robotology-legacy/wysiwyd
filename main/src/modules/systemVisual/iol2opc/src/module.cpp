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

#include <sstream>
#include <cstdio>
#include <algorithm>
#include <set>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include "module.h"

using namespace std;
using namespace yarp::math;


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
            if ((x[0]>skim_blobs_x_bounds[0])&&(x[0]<skim_blobs_x_bounds[1])&&
                (x[1]>skim_blobs_y_bounds[0])&&(x[1]<skim_blobs_y_bounds[1]))
                skimmedBlobs.add(blobs.get(i));
        }
    }

    return skimmedBlobs;
}


/**********************************************************/
Bottle IOL2OPCBridge::getBlobs()
{
    // grab resources
    mutexResources.wait();

    if (Bottle *pBlobs=blobExtractor.read(false))
    {
        lastBlobs=skimBlobs(*pBlobs);
        printf("Received blobs list: %s\n",lastBlobs.toString().c_str());
        
        if (lastBlobs.size()==1)
        {
            if (lastBlobs.get(0).asVocab()==Vocab::encode("empty"))
                lastBlobs.clear();
        }
    }

    // release resources
    mutexResources.post();
    
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

bool IOL2OPCBridge::getClickPosition(Vector &pos)
{
    if (Bottle *bPos=getClickPort.read(false))
    {
        if (bPos->size()>=2)
        {
            pos[0] = bPos->get(0).asInt();
            pos[1] = bPos->get(1).asInt();
            return true;
        }
    }
    return false;
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
        printf("Sending get3D query: %s\n",cmd.toString().c_str());
        rpcGet3D.write(cmd,reply);
        printf("Received blob cartesian coordinates: %s\n",reply.toString().c_str());

        if (reply.size()>0)
        {
            if (Bottle *pInfo=reply.get(0).asList())
            {
                if (pInfo->size()>=3)
                {
                    x.resize(3);
                    x[0]=pInfo->get(0).asDouble();
                    x[1]=pInfo->get(1).asDouble();
                    x[2]=pInfo->get(2).asDouble();
                    return true;
                }
            }
        }
    }

    return false;
}


/**********************************************************/
void IOL2OPCBridge::acquireImage(const bool rtlocalization)
{
    // grab resources
    mutexResources.wait();

    // wait for incoming image
    if (ImageOf<PixelBgr> *tmp=imgIn.read())
    {
        if (rtlocalization)
            imgRtLoc=*tmp;
        else
            img=*tmp;
    }
    
    // release resources
    mutexResources.post();
}


/**********************************************************/
void IOL2OPCBridge::drawBlobs(const Bottle &blobs, const int i,
                              Bottle *scores)
{
    // grab resources
    mutexResources.wait();

    BufferedPort<ImageOf<PixelBgr> > *port=(scores==NULL)?&imgOut:&imgRtLocOut;
    if (port->getOutputCount()>0)
    {
        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1);

        // latch image
        ImageOf<PixelBgr> img=(scores==NULL)?this->img:this->imgRtLoc;
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

            if (scores!=NULL)
            {
                // find the blob name (or unknown)
                string object=findName(*scores,tag.str());
                tag.str("");
                tag.clear();
                tag<<object;
            }

            CvScalar highlight=cvScalar(0,255,0);
            CvScalar lowlight=cvScalar(150,125,125);
            cvRectangle(img.getIplImage(),tl,br,(j==i)?highlight:lowlight,2);
            cvPutText(img.getIplImage(),tag.str().c_str(),txtLoc,&font,(j==i)?highlight:lowlight);
        }

        port->prepare()=img;
        port->write();
    }

    // release resources
    mutexResources.post();
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
        mutexResources.wait();

        // create image containing histogram
        ImageOf<PixelBgr> imgConf;
        imgConf.resize(600,600);
        imgConf.zero();

        ostringstream tag;
        tag<<"blob_"<<i;

        // process scores on the given blob
        if (Bottle *blobScores=scores.find(tag.str().c_str()).asList())
        {
            CvFont font;
            cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.8,0.8,0,2);

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
                cv::putText(textImg,name.c_str(),cvPoint(imgConf.width()-580,(j+1)*widthStep-10),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(255,255,255),2);
                rotate(textImg,90.0,textImg);

                cv::Mat orig=(IplImage*)imgConf.getIplImage();
                orig=orig+textImg;
                orig.copyTo(cv::Mat((IplImage*)imgConf.getIplImage()));                
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
                for (map<string,Filter*>::iterator it=histFiltersPool.begin();
                     it!=histFiltersPool.end(); it++)
                {
                    if (gcFilters.find(it->first)==gcFilters.end())
                    {
                        delete it->second;
                        histFiltersPool.erase(it);
                        break;
                    }
                }
            }
            
        }

        imgHistogram.prepare()=imgConf;
        imgHistogram.write();

        // release resources
        mutexResources.post();
    }
}


/**********************************************************/
int IOL2OPCBridge::findClosestBlob(const Bottle &blobs, const CvPoint &loc)
{
    int ret=RET_INVALID;
    double min_d2=1e9;

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
int IOL2OPCBridge::findClosestBlob(const Bottle &blobs, const Vector &loc)
{
    int ret=RET_INVALID;
    double curMinDist=1e9;
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
Bottle IOL2OPCBridge::classify(const Bottle &blobs,
                               const bool rtlocalization)
{
    // grab resources
    mutexResources.wait();

    if (rtlocalization)
        imgClassifier.write(imgRtLoc);
    else
        imgClassifier.write(img);

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
    printf("Sending classification request: %s\n",cmd.toString().c_str());
    rpcClassifier.write(cmd,reply);
    printf("Received reply: %s\n",reply.toString().c_str());

    // release resources
    mutexResources.post();

    return reply;
}


/**********************************************************/
void IOL2OPCBridge::train(const string &object, const Bottle &blobs,
                          const int i)
{
    // grab resources
    mutexResources.wait();

    imgClassifier.write(img);

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

    printf("Sending training request: %s\n",cmd.toString().c_str());
    rpcClassifier.write(cmd,reply);
    printf("Received reply: %s\n",reply.toString().c_str());

    // release resources
    mutexResources.post();
}


/**********************************************************/
void IOL2OPCBridge::execForget(const string &object)
{
    Bottle cmdClassifier,replyClassifier,replyHuman;

    // grab resources
    mutexResources.wait();

    // forget the whole memory
    if (object=="all")
    {
        cmdClassifier.addVocab(Vocab::encode("forget"));
        cmdClassifier.addString("all");
        printf("Sending clearing request: %s\n",cmdClassifier.toString().c_str());
        rpcClassifier.write(cmdClassifier,replyClassifier);
        printf("Received reply: %s\n",replyClassifier.toString().c_str());
    }
    else    // forget specific object
    {
        ostringstream reply;
        //map<string,Classifier*>::iterator it=db.find(object);
        //if (it!=db.end())
        {
            cmdClassifier.addVocab(Vocab::encode("forget"));
            cmdClassifier.addString(object.c_str());
            printf("Sending clearing request: %s\n",cmdClassifier.toString().c_str());
            rpcClassifier.write(cmdClassifier,replyClassifier);
            printf("Received reply: %s\n",replyClassifier.toString().c_str());
        }
    }

    // release resources
    mutexResources.post();
}


/**********************************************************/
void IOL2OPCBridge::doLocalization()
{
    // acquire image for classification/training
    acquireImage(true);
    // grab the blobs
    Bottle blobs=getBlobs();
    // get the scores from the learning machine
    Bottle scores=classify(blobs,true);
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
    drawBlobs(blobs,closestBlob,&scores);
    // draw scores histogram
    drawScoresHistogram(blobs,scores,closestBlob);

    // data for memory update
    mutexResourcesMemory.wait();
    memoryBlobs=blobs;
    memoryScores=scores;
    mutexResourcesMemory.post();
}


/**********************************************************/
void IOL2OPCBridge::updateOPC()
{
    if (opc->isConnected())
    {
        // grab resources
        mutexMemoryUpdate.wait();

        mutexResourcesMemory.wait();
        Bottle blobs=memoryBlobs;
        Bottle scores=memoryScores;
        mutexResourcesMemory.post();

        for (int j=0; j<blobs.size(); j++)
        {
            ostringstream tag;
            tag<<"blob_"<<j;

            // find the blob name (or unknown)
            mutexResources.wait();
            string object=findName(scores,tag.str());
            mutexResources.post();

            if (object!=OBJECT_UNKNOWN)
            {
                CvPoint cog=getBlobCOG(blobs,j);
                if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
                    continue;

                // find 3d position
                Vector x;
                if (get3DPosition(cog,x))
                {
                    Bottle *item=blobs.get(j).asList();
                    if (item==NULL)
                        continue;

                    Object *obj=opc->addObject(object);
                    obj->m_ego_position=x;
                    opc->commit(obj);
                }
            }
        }

        // release resources
        mutexMemoryUpdate.post();
    }
}


/**********************************************************/
bool IOL2OPCBridge::configure(ResourceFinder &rf)
{
    name=rf.check("name",Value("iol2opc")).asString().c_str();
    opc=new OPCClient(name);
    if (opc->connect(rf.check("opcName",Value("OPC")).asString().c_str()))
    {
        yError("OPC doesn't seem to be running!");
        return false;
    }

    imgIn.open(("/"+name+"/img:i").c_str());
    blobExtractor.open(("/"+name+"/blobs:i").c_str());
    imgOut.open(("/"+name+"/img:o").c_str());
    imgRtLocOut.open(("/"+name+"/imgLoc:o").c_str());
    imgClassifier.open(("/"+name+"/imgClassifier:o").c_str());
    imgHistogram.open(("/"+name+"/imgHistogram:o").c_str());
    histObjLocPort.open(("/"+name+"/histObjLocation:i").c_str());
    recogTriggerPort.open(("/"+name+"/recog:o").c_str());

    rpcPort.open(("/"+name+"/rpc").c_str());
    rpcClassifier.open(("/"+name+"/classify:rpc").c_str());
    rpcGet3D.open(("/"+name+"/get3d:rpc").c_str());
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
    opcUpdater.setRate(rf.check("memory_update_period",Value(60)).asInt());
    
    histFilterLength=std::max(1,rf.check("hist_filter_length",Value(10)).asInt());

    img.resize(320,240);
    imgRtLoc.resize(320,240);
    img.zero();
    imgRtLoc.zero();
        
    histColorsCode.push_back(cvScalar( 65, 47,213));
    histColorsCode.push_back(cvScalar(122, 79, 58));
    histColorsCode.push_back(cvScalar(154,208, 72));
    histColorsCode.push_back(cvScalar( 71,196,249));
    histColorsCode.push_back(cvScalar(224,176, 96));
    histColorsCode.push_back(cvScalar( 22,118,238));

    attach(rpcPort);

    rtLocalization.start();
    opcUpdater.start();

    return true;
}


/**********************************************************/
bool IOL2OPCBridge::interruptModule()
{
    imgIn.interrupt();
    imgOut.interrupt();
    imgRtLocOut.interrupt();
    imgClassifier.interrupt();
    imgHistogram.interrupt();
    histObjLocPort.interrupt();
    recogTriggerPort.interrupt();
    rpcPort.interrupt();
    blobExtractor.interrupt();
    rpcClassifier.interrupt();
    getClickPort.interrupt();
    rpcGet3D.interrupt();
    opc->interrupt();

    rtLocalization.stop();
    opcUpdater.stop();

    return true;
}


/**********************************************************/
bool IOL2OPCBridge::close()
{
    imgIn.close();
    imgOut.close();
    imgRtLocOut.close();
    imgClassifier.close();
    imgHistogram.close();
    histObjLocPort.close();
    recogTriggerPort.close();
    rpcPort.close();
    blobExtractor.close();
    rpcClassifier.close();
    getClickPort.close();
    rpcGet3D.close();
    opc->close();

    // dispose filters used for scores histogram
    for (map<string,Filter*>::iterator it=histFiltersPool.begin();
         it!=histFiltersPool.end(); it++)
        delete it->second;

    return true;
}


/**********************************************************/
double IOL2OPCBridge::getPeriod()
{
    return 1.0;
}


/**********************************************************/
bool IOL2OPCBridge::updateModule()
{
    return true;
}


/**********************************************************/
bool IOL2OPCBridge::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}


/**********************************************************/
bool IOL2OPCBridge::add_name(const string &name)
{
    Vector clickLocation;
    clickLocation.resize(2);

    if(getClickPosition(clickLocation)) {
        cout << clickLocation[0] << " " << clickLocation[1] << endl;
    } else {
        return false;
    }

    return false;
}


/**********************************************************/
bool IOL2OPCBridge::remove_name(const string &name)
{
    return false;
}



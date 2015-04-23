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

using namespace yarp::math;


/**********************************************************/
class BusyGate
{
    bool &gate;
    bool  owner;
public:
    /**********************************************************/
    BusyGate(bool &g) : gate(g)
    {
        gate=true;
        owner=true;
    }

    /**********************************************************/
    void release()
    {
        owner=false;
    }

    /**********************************************************/
    ~BusyGate()
    {
        if (owner)
            gate=false; 
    }
};


/**********************************************************/
int IOL2OPCBridge::processHumanCmd(const Bottle &cmd, Bottle &b)
{
    int ret=Vocab::encode(cmd.get(0).asString().c_str());
    b.clear();

    if (cmd.size()>1)
    {
        if (cmd.get(1).isList())
            b=*cmd.get(1).asList();
        else
            b=cmd.tail();
    }

    return ret;
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


/**********************************************************/
bool IOL2OPCBridge::get3DPosition(const CvPoint &point, Vector &x)
{
    if (rpcGet3D.getOutputCount()>0)
    {
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("get"));
        cmd.addVocab(Vocab::encode("s2c"));
        Bottle &options=cmd.addList();
        options.addString(camera.c_str());
        options.addInt(point.x);
        options.addInt(point.y);
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
                string object=db.findName(*scores,tag.str());
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
void IOL2OPCBridge::rotate(cv::Mat &src, const double angle, cv::Mat &dst)
{
    int len=std::max(src.cols,src.rows);
    cv::Point2f pt(len/2.0f,len/2.0f);
    cv::Mat r=cv::getRotationMatrix2D(pt,angle,1.0);
    cv::warpAffine(src,dst,r,cv::Size(len,len));
}


/**********************************************************/
void IOL2OPCBridge::drawScoresHistogram(const Bottle &blobs,
                                  const Bottle &scores, const int i)
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
Bottle IOL2OPCBridge::classify(const Bottle &blobs, const bool rtlocalization)
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
void IOL2OPCBridge::burst(const string &tag)
{
    if (trainBurst && (tag!=""))
    {
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("burst"));
        cmd.addVocab(Vocab::encode(tag.c_str()));

        printf("Sending burst training request: %s\n",cmd.toString().c_str());
        rpcClassifier.write(cmd,reply);
        printf("Received reply: %s\n",reply.toString().c_str());
    }
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

    if (trainOnFlipped && (i>=0))
    {
        ImageOf<PixelBgr> imgFlipped=img;

        if (Bottle *item=blobs.get(i).asList())
        {
            CvPoint tl,br;
            tl.x=(int)item->get(0).asDouble();
            tl.y=(int)item->get(1).asDouble();
            br.x=(int)item->get(2).asDouble();
            br.y=(int)item->get(3).asDouble();

            cvSetImageROI((IplImage*)imgFlipped.getIplImage(),cvRect(tl.x,tl.y,br.x-tl.x,br.y-tl.y));
            cvFlip(imgFlipped.getIplImage(),imgFlipped.getIplImage(),1);
            cvResetImageROI((IplImage*)imgFlipped.getIplImage());

            imgClassifier.write(imgFlipped);

            printf("Sending training request (for flipped image): %s\n",cmd.toString().c_str());
            rpcClassifier.write(cmd,reply);
            printf("Received reply (for flipped image): %s\n",reply.toString().c_str());
        }
    }

    // release resources
    mutexResources.post();
}


/**********************************************************/
void IOL2OPCBridge::improve_train(const string &object, const Bottle &blobs,
                            const int i)
{
    CvPoint ref_cog=getBlobCOG(blobs,i);
    if ((ref_cog.x==RET_INVALID) || (ref_cog.y==RET_INVALID))
        return;

    double t0=Time::now();
    while (Time::now()-t0<improve_train_period)
    {
        // acquire image for training
        acquireImage();

        // grab the blobs
        Bottle blobs=getBlobs();

        // failure handling
        if (blobs.size()==0)
            continue;

        // enforce 2D consistency
        int exploredBlob=-1;
        double curMinDist=10.0;
        double curMinDist2=curMinDist*curMinDist;
        for (int i=0; i<blobs.size(); i++)
        {
            CvPoint cog=getBlobCOG(blobs,i);
            if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
                continue;

            double dx=ref_cog.x-cog.x;
            double dy=ref_cog.y-cog.y;
            double dist2=dx*dx+dy*dy;
            if (dist2<curMinDist2)
            {
                exploredBlob=i;
                curMinDist2=dist2;
            }
        }

        // no candidate found => skip
        if (exploredBlob<0)
            continue;

        // train the classifier
        train(object,blobs,exploredBlob);

        // draw the blobs highlighting the explored one
        drawBlobs(blobs,exploredBlob);
    }
}


/**********************************************************/
bool IOL2OPCBridge::interruptableAction(const string &action,
                                  deque<string> *param,
                                  const string &object,
                                  const Bottle &blobs,
                                  const int iBlob)
{
    // remap "hold" into "take" without final "drop"
    string actionRemapped=action;
    if (actionRemapped=="hold")
        actionRemapped="take";

    Bottle cmdMotor,replyMotor;

    RpcClient *port;
    if (action=="grasp")
    {
        port=&rpcMotorGrasp;

        CvPoint cog=getBlobCOG(blobs,iBlob);
        cmdMotor.addString("grasp");
        Bottle &point=cmdMotor.addList();
        point.addInt(cog.x);
        point.addInt(cog.y);
    }
    else
    {
        port=&rpcMotor;

        cmdMotor.addVocab(Vocab::encode(actionRemapped.c_str()));
        if (action=="drop")
            cmdMotor.addString("over");
        cmdMotor.addString(object.c_str());
        if (action=="drop")
            cmdMotor.addString("gently");

        if (param!=NULL)
            for (size_t i=0; i<param->size(); i++)
                cmdMotor.addString((*param)[i].c_str());
    }

    actionInterrupted=false;
    enableInterrupt=true;   
    port->write(cmdMotor,replyMotor);
    bool ack=(replyMotor.get(0).asVocab()==Vocab::encode("ack"));

    if ((action=="grasp") && !ack)
    {
        string why=replyMotor.get(1).asString().c_str();
        string sentence="Hmmm. The ";
        sentence+=object;
        if (why=="too_far")
            sentence+=" seems too far. Could you push it closer?";
        else
            sentence+=" seems in bad position for me. Could you help moving it a little bit?";

        speaker.speak(sentence);
    }

    // this switch might be turned on asynchronously
    // by a request received on a dedicated port
    if (actionInterrupted)
    {
        reinstateMotor();
        home();
    }
    // drop the object in the hand
    else if (ack && ((action=="take") || (action=="grasp")))
    {
        cmdMotor.clear();
        cmdMotor.addVocab(Vocab::encode("drop"));
        rpcMotor.write(cmdMotor,replyMotor);
    }

    enableInterrupt=false;
    return !actionInterrupted;
}


/**********************************************************/
void IOL2OPCBridge::interruptMotor()
{
    if (enableInterrupt)
    {
        actionInterrupted=true;  // keep this line before the call to write
        enableInterrupt=false;
        Bottle cmdMotorStop,replyMotorStop;
        cmdMotorStop.addVocab(Vocab::encode("interrupt"));
        rpcMotorStop.write(cmdMotorStop,replyMotorStop);

        speaker.speak("Ouch!");
    }
}


/**********************************************************/
void IOL2OPCBridge::point(const string &object)
{
    motorHelper("point",object);
}


/**********************************************************/
void IOL2OPCBridge::look(const string &object)
{
    motorHelper("look",object);
}


/**********************************************************/
void IOL2OPCBridge::point(const Bottle &blobs, const int i)
{
    motorHelper("point",blobs,i);
}


/**********************************************************/
void IOL2OPCBridge::look(const Bottle &blobs, const int i, const Bottle &options)
{
    motorHelper("look",blobs,i,options);
}


/**********************************************************/
int IOL2OPCBridge::recognize(const string &object, Bottle &blobs,
                       Classifier **ppClassifier)
{
    map<string,Classifier*>::iterator it=db.find(object);
    if (it==db.end())
    {
        // if not, create a brand new one
        db[object]=new Classifier(object,classification_threshold);
        it=db.find(object);
        printf("created classifier for %s\n",object.c_str());
    }

    // acquire image for classification/training
    acquireImage();

    // grab the blobs
    blobs=getBlobs();

    // failure handling
    if (blobs.size()==0)
        return RET_INVALID;

    // get the scores from the learning machine
    Bottle scores=classify(blobs);

    // failure handling
    if (scores.size()==1)
    {
        if (scores.get(0).asString()=="failed")
        {
            speaker.speak("Ooops! Sorry, something went wrong in my brain");
            return RET_INVALID;
        }
    }

    // find the best blob
    int recogBlob=db.processScores(it->second,scores);

    // draw the blobs highlighting the recognized one (if any)
    drawBlobs(blobs,recogBlob);

    // prepare output
    if (ppClassifier!=NULL)
        *ppClassifier=it->second;

    return recogBlob;
}


/**********************************************************/
int IOL2OPCBridge::recognize(Bottle &blobs, Bottle &scores, string &object)
{
    object=OBJECT_UNKNOWN;

    // acquire image for classification/training
    acquireImage();

    // grab the blobs
    blobs=getBlobs();

    // failure handling
    if (blobs.size()==0)
        return RET_INVALID;

    // get the scores from the learning machine
    scores=classify(blobs);

    // failure handling
    if (scores.size()==1)
    {
        if (scores.get(0).asString()=="failed")
        {
            speaker.speak("Ooops! Sorry, something went wrong in my brain");
            return RET_INVALID;
        }
    }

    // handle the human-pointed object
    if (whatGood)
    {
        int closestBlob=findClosestBlob(blobs,whatLocation);
        drawBlobs(blobs,closestBlob);
        look(blobs,closestBlob);

        ostringstream tag;
        tag<<"blob_"<<closestBlob;
        object=db.findName(scores,tag.str());
        return closestBlob;
    }
    else
    {
        speaker.speak("Ooops! Sorry, I missed where you pointed at");
        return RET_INVALID;
    }
}


/**********************************************************/
void IOL2OPCBridge::execName(const string &object)
{
    Bottle replyHuman;
    if (!trackStopGood)
    {
        speaker.speak("Ooops! Sorry, I missed where you pointed at");
        replyHuman.addString("nack");
        rpcHuman.reply(replyHuman);
        return;
    }

    map<string,Classifier*>::iterator it=db.find(object);
    if (it==db.end())
    {
        // if not, create a brand new one
        db[object]=new Classifier(object,classification_threshold);
        it=db.find(object);
        printf("created classifier for %s\n",object.c_str());
    }

    // acquire image for training
    acquireImage();

    // grab the blobs
    Bottle blobs=getBlobs();

    // failure handling
    if (blobs.size()==0)
    {
        speaker.speak("Ooops! Sorry, I cannot see any object");
        replyHuman.addString("nack");
        rpcHuman.reply(replyHuman);
        return;
    }

    // run the normal procedure straightaway
    Bottle scores=classify(blobs);

    // failure handling
    if (scores.size()==1)
    {
        if (scores.get(0).asString()=="failed")
        {
            speaker.speak("Ooops! Sorry, something went wrong in my brain");
            replyHuman.addString("nack");
            rpcHuman.reply(replyHuman);
            return;
        }
    }

    db.processScores(it->second,scores);

    // find the closest blob
    int closestBlob=findClosestBlob(blobs,trackStopLocation);

    // draw the blobs highlighting the detected one (if any)
    drawBlobs(blobs,closestBlob);

    // train
    burst("start");
    train(object,blobs,closestBlob);
    improve_train(object,blobs,closestBlob);
    burst("stop");
    triggerRecogInfo(object,blobs,closestBlob,"creation");
    ostringstream reply;
    reply<<"All right! Now I know what a "<<object;
    reply<<" is";
    speaker.speak(reply.str());
    look(blobs,closestBlob);

    replyHuman.addString("ack");
    rpcHuman.reply(replyHuman);
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

        // clear the memory too
        if (rpcMemory.getOutputCount()>0)
        {
            mutexResourcesMemory.wait();
            for (map<string,int>::iterator id=memoryIds.begin(); id!=memoryIds.end(); id++)
            {
                Bottle cmdMemory,replyMemory;
                cmdMemory.addVocab(Vocab::encode("del"));
                Bottle &bid=cmdMemory.addList().addList();
                bid.addString("id");
                bid.addInt(id->second);
                rpcMemory.write(cmdMemory,replyMemory);
            }
            memoryIds.clear();
            mutexResourcesMemory.post();
        }

        db.clear();
        speaker.speak("I have forgotten everything");
        replyHuman.addString("ack");
    }
    else    // forget specific object
    {
        ostringstream reply;
        map<string,Classifier*>::iterator it=db.find(object);
        if (it!=db.end())
        {
            cmdClassifier.addVocab(Vocab::encode("forget"));
            cmdClassifier.addString(object.c_str());
            printf("Sending clearing request: %s\n",cmdClassifier.toString().c_str());
            rpcClassifier.write(cmdClassifier,replyClassifier);
            printf("Received reply: %s\n",replyClassifier.toString().c_str());

            // remove the item from the memory too
            if (rpcMemory.getOutputCount()>0)
            {
                mutexResourcesMemory.wait();
                map<string,int>::iterator id=memoryIds.find(object);
                map<string,int>::iterator memoryIdsEnd=memoryIds.end();
                mutexResourcesMemory.post();

                if (id!=memoryIdsEnd)
                {
                    Bottle cmdMemory,replyMemory;
                    cmdMemory.addVocab(Vocab::encode("del"));
                    Bottle &bid=cmdMemory.addList().addList();
                    bid.addString("id");
                    bid.addInt(id->second);
                    rpcMemory.write(cmdMemory,replyMemory);

                    mutexResourcesMemory.wait();
                    memoryIds.erase(id);
                    mutexResourcesMemory.post();
                }
            }

            db.erase(it);
            reply<<object<<" forgotten";
            speaker.speak(reply.str());
            replyHuman.addString("ack");
        }
        else
        {
            printf("%s object is unknown\n",object.c_str());
            reply<<"I do not know any "<<object;
            speaker.speak(reply.str());
            replyHuman.addString("nack");
        }        
    }

    rpcHuman.reply(replyHuman);

    // release resources
    mutexResources.post();
}


/**********************************************************/
void IOL2OPCBridge::execWhere(const string &object, const Bottle &blobs,
                        const int recogBlob, Classifier *pClassifier,
                        const string &recogType)
{
    Bottle cmdHuman,valHuman,replyHuman;

    // some known object has been recognized
    if (recogBlob>=0)
    {
        ostringstream reply;
        reply<<"I think this is the "<<object;
        speaker.speak(reply.str());
        printf("I think the %s is blob %d\n",object.c_str(),recogBlob);

        // issue a [point] and wait for action completion
        point(object);

        speaker.speak("Am I right?");

        replyHuman.addString("ack");
        replyHuman.addInt(recogBlob);
    }
    // no known object has been recognized in the scene
    else
    {
        ostringstream reply;
        reply<<"I have not found any "<<object;
        reply<<", is it so?";
        speaker.speak(reply.str());
        printf("No object recognized\n");

        replyHuman.addString("nack");
    }

    rpcHuman.reply(replyHuman);

    // enter the human interaction mode to refine the knowledge
    bool ok=false;
    while (!ok)
    {
        replyHuman.clear();
        rpcHuman.read(cmdHuman,true);

        if (isStopping())
            return;

        int type=processHumanCmd(cmdHuman,valHuman);
        // do nothing
        if (type==Vocab::encode("skip"))
        {
            speaker.speak("Skipped");
            replyHuman.addString("ack");
            ok=true;
        }
        // good job is done
        else if (type==Vocab::encode("ack"))
        {
            // reinforce if an object is available
            if ((recogBlob>=0) && (pClassifier!=NULL))
            {
                burst("start");
                train(object,blobs,recogBlob);
                improve_train(object,blobs,recogBlob);
                burst("stop");
                pClassifier->positive();
                triggerRecogInfo(object,blobs,recogBlob,"recognition");
                updateClassifierInMemory(pClassifier);
            }

            speaker.speak("Cool!");
            replyHuman.addString("ack");
            ok=true;
        }
        // misrecognition
        else if (type==Vocab::encode("nack"))
        {
            // update the threshold if an object is available
            if ((recogBlob>=0) && (pClassifier!=NULL))
            {
                pClassifier->negative();
                updateClassifierInMemory(pClassifier);
            }

            // handle the human-pointed object
            CvPoint loc;
            if (pointedLoc.getLoc(loc))
            {
                int closestBlob=findClosestBlob(blobs,loc);
                burst("start");
                train(object,blobs,closestBlob);
                improve_train(object,blobs,closestBlob);
                burst("stop");
                triggerRecogInfo(object,blobs,closestBlob,recogType);
                speaker.speak("Oooh, I see");                
                look(blobs,closestBlob);
            }
            else
                speaker.speak("Ooops! Sorry, I missed where you pointed at");

            replyHuman.addString("ack");
            ok=true;
        }
        else
        {
            speaker.speak("Hmmm hmmm hmmm! Try again");
            replyHuman.addString("nack");
        }

        rpcHuman.reply(replyHuman);
    }
}


/**********************************************************/
void IOL2OPCBridge::execWhat(const Bottle &blobs, const int pointedBlob,
                       const Bottle &scores, const string &object)
{
    Bottle cmdHuman,valHuman,replyHuman;
    Classifier *pClassifier=NULL;

    // some known object has been recognized
    if (object!=OBJECT_UNKNOWN)
    {
        ostringstream reply;
        reply<<"I think it is the "<<object;
        speaker.speak(reply.str());
        speaker.speak("Am I right?");
        printf("I think the blob %d is the %s\n",pointedBlob,object.c_str());

        // retrieve the corresponding classifier
        map<string,Classifier*>::iterator it=db.find(object);
        if (it!=db.end())
            pClassifier=it->second;

        replyHuman.addString("ack");
        replyHuman.addString(object.c_str());
    }
    // no known object has been recognized in the scene
    else
    {
        speaker.speak("I do not know this object");
        speaker.speak("What is it?");
        printf("No object recognized\n");
        replyHuman.addString("nack");
    }

    rpcHuman.reply(replyHuman);

    // enter the human interaction mode to refine the knowledge
    bool ok=false;
    while (!ok)
    {
        replyHuman.clear();
        rpcHuman.read(cmdHuman,true);

        if (isStopping())
            return;

        int type=processHumanCmd(cmdHuman,valHuman);
        // do nothing
        if (type==Vocab::encode("skip"))
        {
            speaker.speak("Skipped");
            replyHuman.addString("ack");
            ok=true;
        }
        // good job is done
        else if ((object!=OBJECT_UNKNOWN) && (type==Vocab::encode("ack")))
        {
            // reinforce if an object is available
            if ((pointedBlob>=0) && (pClassifier!=NULL))
            {
                burst("start");
                train(object,blobs,pointedBlob);
                improve_train(object,blobs,pointedBlob);
                burst("stop");
                db.processScores(pClassifier,scores);
                pClassifier->positive();
                triggerRecogInfo(object,blobs,pointedBlob,"recognition");
                updateClassifierInMemory(pClassifier);
            }

            speaker.speak("Cool!");
            replyHuman.addString("ack");
            ok=true;
        }
        // misrecognition
        else if (type==Vocab::encode("nack"))
        {
            // update the threshold
            if ((pointedBlob>=0) && (pClassifier!=NULL))
            {
                db.processScores(pClassifier,scores);
                pClassifier->negative();
                updateClassifierInMemory(pClassifier);
            }

            speaker.speak("Sorry");
            replyHuman.addString("ack");
            ok=true;
        }
        // handle new/unrecognized/misrecognized object
        else if ((type==Vocab::encode("name")) && (valHuman.size()>0))
        {
            string objectName=valHuman.get(0).asString().c_str();

            // check whether the object is already known
            // and, if not, allocate space for it
            map<string,Classifier*>::iterator it=db.find(objectName);
            if (it==db.end())
            {
                db[objectName]=new Classifier(objectName,classification_threshold);
                it=db.find(objectName);
                speaker.speak("Oooh, I see");
                printf("created classifier for %s\n",objectName.c_str());
            }
            else
            {
                // update the threshold for the case of misrecognition
                if ((pClassifier!=NULL) && (object!=objectName) && (object!=OBJECT_UNKNOWN))
                {
                    db.processScores(pClassifier,scores);
                    pClassifier->negative();
                    updateClassifierInMemory(pClassifier);
                }

                ostringstream reply;
                reply<<"Sorry, I should have recognized the "<<objectName;
                speaker.speak(reply.str());
            }

            // trigger the classifier
            if (pointedBlob>=0)
            {
                burst("start");
                train(objectName,blobs,pointedBlob);
                improve_train(objectName,blobs,pointedBlob);
                burst("stop");
                triggerRecogInfo(objectName,blobs,pointedBlob,
                                 (object==OBJECT_UNKNOWN)?"creation":"recognition");
            }

            db.processScores(it->second,scores);

            replyHuman.addString("ack");
            ok=true;
        }
        else
        {
            speaker.speak("Hmmm hmmm hmmm! Try again");
            replyHuman.addString("nack");
        }

        rpcHuman.reply(replyHuman);
    }
}


/**********************************************************/
void IOL2OPCBridge::execReinforce(const string &object,
                            const Vector &position)
{
    bool ret=false;
    if (db.find(object)!=db.end())
    {
        burst("start");
        ret=doExploration(object,position);
        burst("stop");
    }

    Bottle replyHuman(ret?"ack":"nack");
    rpcHuman.reply(replyHuman);
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
bool IOL2OPCBridge::get3DPositionFromMemory(const string &object,
                                      Vector &position)
{
    bool ret=false;
    if (rpcMemory.getOutputCount()>0)
    {
        // grab resources
        mutexMemoryUpdate.wait();

        mutexResourcesMemory.wait();
        map<string,int>::iterator id=memoryIds.find(object);
        map<string,int>::iterator memoryIdsEnd=memoryIds.end();
        mutexResourcesMemory.post();

        if (id!=memoryIdsEnd)
        {
            // get the relevant properties
            // [get] (("id" <num>) ("propSet" ("position_3d")))
            Bottle cmdMemory,replyMemory;
            cmdMemory.addVocab(Vocab::encode("get"));
            Bottle &content=cmdMemory.addList();
            Bottle &list_bid=content.addList();
            list_bid.addString("id");
            list_bid.addInt(id->second);
            Bottle &list_propSet=content.addList();
            list_propSet.addString("propSet");
            Bottle &list_items=list_propSet.addList();
            list_items.addString("position_3d");
            rpcMemory.write(cmdMemory,replyMemory);

            // retrieve 3D position
            if (replyMemory.get(0).asVocab()==Vocab::encode("ack"))
            {
                if (Bottle *propField=replyMemory.get(1).asList())
                {
                    if (propField->check("position_3d"))
                    {
                        if (Bottle *pPos=propField->find("position_3d").asList())
                        {
                            if (pPos->size()>=3)
                            {
                                position.resize(3);
                                position[0]=pPos->get(0).asDouble();
                                position[1]=pPos->get(1).asDouble();
                                position[2]=pPos->get(2).asDouble();
                                ret=true;
                            }
                        }
                    }
                }
            }
        }

        // release resources
        mutexMemoryUpdate.post();
    }

    return ret;
}


/**********************************************************/
void IOL2OPCBridge::updateMemory()
{
    if (rpcMemory.getOutputCount()>0)
    {
        // grab resources
        mutexMemoryUpdate.wait();

        // load memory on connection event
        if (scheduleLoadMemory)
        {
            loadMemory();
            scheduleLoadMemory=false;
        }

        mutexResourcesMemory.wait();
        Bottle blobs=memoryBlobs;
        Bottle scores=memoryScores;
        mutexResourcesMemory.post();

        set<int> avalObjIds;
        for (int j=0; j<blobs.size(); j++)
        {
            ostringstream tag;
            tag<<"blob_"<<j;

            // find the blob name (or unknown)
            mutexResources.wait();
            string object=db.findName(scores,tag.str());
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

                    // prepare position_2d property
                    Bottle position_2d;
                    Bottle &list_2d=position_2d.addList();
                    list_2d.addString(("position_2d_"+camera).c_str());
                    Bottle &list_2d_c=list_2d.addList();
                    list_2d_c.addDouble(item->get(0).asDouble());
                    list_2d_c.addDouble(item->get(1).asDouble());
                    list_2d_c.addDouble(item->get(2).asDouble());
                    list_2d_c.addDouble(item->get(3).asDouble());

                    // prepare position_3d property
                    Bottle position_3d;
                    Bottle &list_3d=position_3d.addList();
                    list_3d.addString("position_3d");
                    Bottle &list_3d_c=list_3d.addList();
                    list_3d_c.addDouble(x[0]);
                    list_3d_c.addDouble(x[1]);
                    list_3d_c.addDouble(x[2]);

                    mutexResourcesMemory.wait();
                    map<string,int>::iterator id=memoryIds.find(object);
                    map<string,int>::iterator memoryIdsEnd=memoryIds.end();
                    mutexResourcesMemory.post();

                    Bottle cmdMemory,replyMemory;
                    if (id==memoryIdsEnd)      // the object is not available => [add]
                    {
                        cmdMemory.addVocab(Vocab::encode("add"));
                        Bottle &content=cmdMemory.addList();
                        Bottle &list_entity=content.addList();
                        list_entity.addString("entity");
                        list_entity.addString("object");
                        Bottle &list_name=content.addList();
                        list_name.addString("name");
                        list_name.addString(object.c_str());
                        content.append(position_2d);
                        content.append(position_3d);
                        rpcMemory.write(cmdMemory,replyMemory);

                        if (replyMemory.size()>1)
                        {
                            // store the id for later usage
                            if (replyMemory.get(0).asVocab()==Vocab::encode("ack"))
                            {
                                if (Bottle *idField=replyMemory.get(1).asList())
                                {
                                    int id=idField->get(1).asInt();
                                    mutexResourcesMemory.wait();
                                    memoryIds[object]=id;
                                    mutexResourcesMemory.post();

                                    avalObjIds.insert(id);
                                }
                                else
                                    continue;
                            }
                        }
                    }
                    else    // the object is already available => [set]
                    {
                        // prepare id property
                        Bottle bid;
                        Bottle &list_bid=bid.addList();
                        list_bid.addString("id");
                        list_bid.addInt(id->second);

                        cmdMemory.addVocab(Vocab::encode("set"));
                        Bottle &content=cmdMemory.addList();
                        content.append(bid);
                        content.append(position_2d);
                        content.append(position_3d);
                        rpcMemory.write(cmdMemory,replyMemory);

                        avalObjIds.insert(id->second);
                    }
                }
            }
        }

        // remove position properties of objects not in scene
        mutexResourcesMemory.wait();
        for (map<string,int>::iterator it=memoryIds.begin(); it!=memoryIds.end(); it++)
        {
            int id=it->second;
            if (avalObjIds.find(id)==avalObjIds.end())
            {
                Bottle cmdMemory,replyMemory;
                cmdMemory.addVocab(Vocab::encode("del"));
                Bottle &content=cmdMemory.addList();
                Bottle &list_bid=content.addList();
                list_bid.addString("id");
                list_bid.addInt(id);
                Bottle &list_propSet=content.addList();
                list_propSet.addString("propSet");
                Bottle &list_items=list_propSet.addList();
                list_items.addString(("position_2d_"+camera).c_str());
                list_items.addString("position_3d");
                rpcMemory.write(cmdMemory,replyMemory);
            }
        }
        mutexResourcesMemory.post();

        // release resources
        mutexMemoryUpdate.post();
    }
}


/**********************************************************/
void IOL2OPCBridge::updateClassifierInMemory(Classifier *pClassifier)
{
    if ((rpcMemory.getOutputCount()>0) && (pClassifier!=NULL))
    {
        string objectName=pClassifier->getName();

        // prepare classifier_thresholds property
        Bottle classifier_property;
        Bottle &list_classifier=classifier_property.addList();
        list_classifier.addString("classifier_thresholds");
        list_classifier.addList().append(pClassifier->toBottle());

        mutexResourcesMemory.wait();
        map<string,int>::iterator id=memoryIds.find(objectName);
        map<string,int>::iterator memoryIdsEnd=memoryIds.end();
        mutexResourcesMemory.post();

        Bottle cmdMemory,replyMemory;
        if (id==memoryIdsEnd)      // the object is not available => [add]
        {
            cmdMemory.addVocab(Vocab::encode("add"));
            Bottle &content=cmdMemory.addList();
            Bottle &list_entity=content.addList();
            list_entity.addString("entity");
            list_entity.addString("object");
            Bottle &list_name=content.addList();
            list_name.addString("name");
            list_name.addString(objectName.c_str());
            content.append(classifier_property);
            rpcMemory.write(cmdMemory,replyMemory);

            if (replyMemory.size()>1)
            {
                // store the id for later usage
                if (replyMemory.get(0).asVocab()==Vocab::encode("ack"))
                {
                    if (Bottle *idField=replyMemory.get(1).asList())
                    {
                        mutexResourcesMemory.wait();
                        memoryIds[objectName]=idField->get(1).asInt();
                        mutexResourcesMemory.post();
                    }
                }
            }
        }
        else    // the object is already available => [set]
        {
            // prepare id property
            Bottle bid;
            Bottle &list_bid=bid.addList();
            list_bid.addString("id");
            list_bid.addInt(id->second);

            cmdMemory.addVocab(Vocab::encode("set"));
            Bottle &content=cmdMemory.addList();
            content.append(bid);
            content.append(classifier_property);
            rpcMemory.write(cmdMemory,replyMemory);
        }
    }
}


/**********************************************************/
void IOL2OPCBridge::updateObjCartPosInMemory(const string &object,
                                       const Bottle &blobs,
                                       const int i)
{
    if ((rpcMemory.getOutputCount()>0) && (i!=RET_INVALID) && (i<blobs.size()))
    {
        mutexResourcesMemory.wait();
        map<string,int>::iterator id=memoryIds.find(object);
        map<string,int>::iterator memoryIdsEnd=memoryIds.end();
        mutexResourcesMemory.post();

        Bottle *item=blobs.get(i).asList();
        if ((id!=memoryIdsEnd) && (item!=NULL))
        {
            CvPoint cog=getBlobCOG(blobs,i);
            if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
                return;

            Vector x;
            if (get3DPosition(cog,x))
            {
                Bottle cmdMemory,replyMemory;

                // prepare id property
                Bottle bid;
                Bottle &list_bid=bid.addList();
                list_bid.addString("id");
                list_bid.addInt(id->second);

                // prepare position_2d property
                Bottle position_2d;
                Bottle &list_2d=position_2d.addList();
                list_2d.addString(("position_2d_"+camera).c_str());
                Bottle &list_2d_c=list_2d.addList();
                list_2d_c.addDouble(item->get(0).asDouble());
                list_2d_c.addDouble(item->get(1).asDouble());
                list_2d_c.addDouble(item->get(2).asDouble());
                list_2d_c.addDouble(item->get(3).asDouble());

                // prepare position_3d property
                Bottle position_3d;
                Bottle &list_3d=position_3d.addList();
                list_3d.addString("position_3d");
                Bottle &list_3d_c=list_3d.addList();
                list_3d_c.addDouble(x[0]);
                list_3d_c.addDouble(x[1]);
                list_3d_c.addDouble(x[2]);

                cmdMemory.addVocab(Vocab::encode("set"));
                Bottle &content=cmdMemory.addList();
                content.append(bid);
                content.append(position_2d);
                content.append(position_3d);
                rpcMemory.write(cmdMemory,replyMemory);
            }
        }
    }
}


/**********************************************************/
void IOL2OPCBridge::triggerRecogInfo(const string &object, const Bottle &blobs,
                               const int i, const string &recogType)
{
    if ((recogTriggerPort.getOutputCount()>0) && (i!=RET_INVALID) && (i<blobs.size()))
    {
        CvPoint cog=getBlobCOG(blobs,i);
        if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
            return;

        Vector x;
        if (get3DPosition(cog,x))
        {
            Property &msg=recogTriggerPort.prepare();
            msg.clear();

            Bottle pos; pos.addList().read(x);
            msg.put("label",object.c_str());
            msg.put("position_3d",pos.get(0));
            msg.put("type",recogType.c_str());

            recogTriggerPort.write();
        }
    }
}


/**********************************************************/
void IOL2OPCBridge::loadMemory()
{
    printf("Loading memory ...\n");
    // grab resources
    mutexResourcesMemory.wait();

    // purge internal databases
    memoryIds.clear();
    db.clear();

    // ask for all the items stored in memory
    Bottle cmdMemory,replyMemory,replyMemoryProp;
    cmdMemory.addVocab(Vocab::encode("ask"));
    Bottle &content=cmdMemory.addList().addList();
    content.addString("entity");
    content.addString("==");
    content.addString("object");
    rpcMemory.write(cmdMemory,replyMemory);
    
    if (replyMemory.size()>1)
    {
        if (replyMemory.get(0).asVocab()==Vocab::encode("ack"))
        {
            if (Bottle *idField=replyMemory.get(1).asList())
            {
                if (Bottle *idValues=idField->get(1).asList())
                {
                    // cycle over items
                    for (int i=0; i<idValues->size(); i++)
                    {
                        int id=idValues->get(i).asInt();

                        // get the relevant properties
                        // [get] (("id" <num>) ("propSet" ("name" "classifier_thresholds")))
                        cmdMemory.clear();
                        cmdMemory.addVocab(Vocab::encode("get"));
                        Bottle &content=cmdMemory.addList();
                        Bottle &list_bid=content.addList();
                        list_bid.addString("id");
                        list_bid.addInt(id);
                        Bottle &list_propSet=content.addList();
                        list_propSet.addString("propSet");
                        Bottle &list_items=list_propSet.addList();
                        list_items.addString("name");
                        list_items.addString("classifier_thresholds");
                        rpcMemory.write(cmdMemory,replyMemoryProp);

                        // update internal databases
                        if (replyMemoryProp.get(0).asVocab()==Vocab::encode("ack"))
                        {
                            if (Bottle *propField=replyMemoryProp.get(1).asList())
                            {
                                if (propField->check("name"))
                                {
                                    string object=propField->find("name").asString().c_str();
                                    memoryIds[object]=id;

                                    if (propField->check("classifier_thresholds"))
                                        db[object]=new Classifier(*propField->find("classifier_thresholds").asList());
                                    else
                                        db[object]=new Classifier(object,classification_threshold);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    printf("Objects in memory: %d\n",(int)db.size());
    for (map<string,Classifier*>::iterator it=db.begin(); it!=db.end(); it++)
    {
        string object=it->first;
        string properties=it->second->toBottle().toString().c_str();
        printf("classifier for %s: memory_id=%d; properties=%s\n",
               object.c_str(),memoryIds[object],properties.c_str());
    }

    // release resources
    mutexResourcesMemory.post();
    printf("Memory loaded\n");
}


/**********************************************************/
bool IOL2OPCBridge::configure(ResourceFinder &rf)
{
    name=rf.check("name",Value("iol2opc")).asString().c_str();
    camera=rf.check("camera",Value("left")).asString().c_str();
    if ((camera!="left") && (camera!="right"))
        camera="left";

    imgIn.open(("/"+name+"/img:i").c_str());
    blobExtractor.open(("/"+name+"/blobs:i").c_str());
    imgOut.open(("/"+name+"/img:o").c_str());
    imgRtLocOut.open(("/"+name+"/imgLoc:o").c_str());
    imgClassifier.open(("/"+name+"/imgClassifier:o").c_str());
    imgHistogram.open(("/"+name+"/imgHistogram:o").c_str());
    histObjLocPort.open(("/"+name+"/histObjLocation:i").c_str());
    recogTriggerPort.open(("/"+name+"/recog:o").c_str());

    rpcPort.open(("/"+name+"/rpc").c_str());
    rpcHuman.open(("/"+name+"/human:rpc").c_str());
    rpcClassifier.open(("/"+name+"/classify:rpc").c_str());
    rpcMotor.open(("/"+name+"/motor:rpc").c_str());
    rpcMotorGrasp.open(("/"+name+"/motor_grasp:rpc").c_str());
    rpcGet3D.open(("/"+name+"/get3d:rpc").c_str());
    rpcMotorStop.open(("/"+name+"/motor_stop:rpc").c_str());
    rxMotorStop.open(("/"+name+"/motor_stop:i").c_str());
    rxMotorStop.setManager(this);

    memoryReporter.setManager(this);
    rpcMemory.setReporter(memoryReporter);
    rpcMemory.open(("/"+name+"/memory:rpc").c_str());

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

    attention.setManager(this);
    attention.start();

    rtLocalization.setManager(this);
    rtLocalization.setRate(rf.check("rt_localization_period",Value(30)).asInt());
    rtLocalization.start();

    exploration.setRate(rf.check("exploration_period",Value(30)).asInt());
    exploration.setManager(this);

    memoryUpdater.setManager(this);
    memoryUpdater.setRate(rf.check("memory_update_period",Value(60)).asInt());
    memoryUpdater.start();
    
    improve_train_period=rf.check("improve_train_period",Value(0.0)).asDouble();
    trainOnFlipped=rf.check("train_flipped_images",Value("off")).asString()=="on";
    trainBurst=rf.check("train_burst_images",Value("off")).asString()=="on";
    classification_threshold=rf.check("classification_threshold",Value(0.5)).asDouble();
    histFilterLength=std::max(1,rf.check("hist_filter_length",Value(10)).asInt());

    img.resize(320,240);
    imgRtLoc.resize(320,240);
    img.zero();
    imgRtLoc.zero();

    attach(rpcPort);
        
    histColorsCode.push_back(cvScalar( 65, 47,213));
    histColorsCode.push_back(cvScalar(122, 79, 58));
    histColorsCode.push_back(cvScalar(154,208, 72));
    histColorsCode.push_back(cvScalar( 71,196,249));
    histColorsCode.push_back(cvScalar(224,176, 96));
    histColorsCode.push_back(cvScalar( 22,118,238));

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
    rpcHuman.interrupt();
    blobExtractor.interrupt();
    rpcClassifier.interrupt();
    rpcMotor.interrupt();
    rpcMotorGrasp.interrupt();
    rpcGet3D.interrupt();
    rpcMotorStop.interrupt();
    rxMotorStop.interrupt();
    pointedLoc.interrupt();
    speaker.interrupt();
    rpcMemory.interrupt();

    rtLocalization.stop();
    memoryUpdater.stop();
    attention.stop();

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
    rpcHuman.close();
    blobExtractor.close();
    rpcClassifier.close();
    rpcMotor.close();
    rpcMotorGrasp.close();
    rpcGet3D.close();
    rpcMotorStop.close();
    rxMotorStop.close();
    pointedLoc.close();
    speaker.close();
    rpcMemory.close();

    // dispose filters used for scores histogram
    for (map<string,Filter*>::iterator it=histFiltersPool.begin();
         it!=histFiltersPool.end(); it++)
        delete it->second;

    return true;
}


/**********************************************************/
bool IOL2OPCBridge::updateModule()
{
    Bottle cmdHuman,valHuman,replyHuman;
    rpcHuman.read(cmdHuman,true);

    BusyGate busyGate(busy);

    if (isStopping())
        return false;

    attention.suspend();

    int rxCmd=processHumanCmd(cmdHuman,valHuman);
    if ((rxCmd==Vocab::encode("attention")) && (valHuman.size()>0))
        if (valHuman.get(0).asString()=="stop")
            skipGazeHoming=true;

    if (!skipGazeHoming)
    {
        home("gaze");

        // this wait-state gives the memory
        // time to be updated with the 3D
        // location of the objects
        Time::delay(0.1);
    }

    skipGazeHoming=false;

    else if ((rxCmd==Vocab::encode("name")) && (valHuman.size()>0))
    {        
        string activeObject=valHuman.get(0).asString().c_str();
        execName(activeObject);
    }
    else if ((rxCmd==Vocab::encode("forget")) && (valHuman.size()>0))
    {        
        string activeObject=valHuman.get(0).asString().c_str();

        mutexMemoryUpdate.wait();
        execForget(activeObject);
        mutexMemoryUpdate.post();
    }
    else if ((rxCmd==Vocab::encode("where")) && (valHuman.size()>0))
    {        
        Bottle blobs;
        Classifier *pClassifier;
        string activeObject=valHuman.get(0).asString().c_str();

        mutexMemoryUpdate.wait();
        string recogType=(db.find(activeObject)==db.end())?"creation":"recognition";
        int recogBlob=recognize(activeObject,blobs,&pClassifier);
        updateObjCartPosInMemory(activeObject,blobs,recogBlob);
        execWhere(activeObject,blobs,recogBlob,pClassifier,recogType);
        mutexMemoryUpdate.post();
    }
    else if (rxCmd==Vocab::encode("what"))
    {
        // avoid being distracted by the human hand
        // while it is being removed: save the last
        // pointed object
        whatGood=pointedLoc.getLoc(whatLocation);
        Time::delay(1.0);

        Bottle blobs,scores;
        string activeObject;
        int pointedBlob=recognize(blobs,scores,activeObject);
        execWhat(blobs,pointedBlob,scores,activeObject);
    }

    return true;
}


/**********************************************************/
double IOL2OPCBridge::getPeriod()
{
    return 1.0;
}




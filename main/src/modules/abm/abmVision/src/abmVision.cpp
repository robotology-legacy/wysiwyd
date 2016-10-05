/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Maxime Petit
* email:   m.petit@imperial.ac.uk
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

#include <abmVision.h>
#include "wrdac/subsystems/subSystem_ABM.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;
using namespace iCub::iKin;
using namespace yarp::math;


abmVision::~abmVision()
{
}

/*
* Configure method. Receive a previously initialized
* resource finder object. Use it to configure your module.
* If you are migrating from the old Module, this is the
* equivalent of the "open" method.
*/

bool abmVision::configure(yarp::os::ResourceFinder &rf) {

    bool    bEveryThingisGood = true;
    bool    bOptionnalModule = true;
    moduleName = rf.check("name", Value("/abmVision"), "module name (string)").asString();

    /*
    * before continuing, set the module name before getting any other parameters,
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());


    Bottle &camerasGroup=rf.findGroup("cameras");
    Bottle &tweakGroup=rf.findGroup("tweak");

    if (camerasGroup.check("file"))
    {
        rf_cameras.setQuiet();
        camerasGroup.check("context")?
        rf_cameras.setDefaultContext(camerasGroup.find("context").asString().c_str()):
        rf_cameras.setDefaultContext(rf.getContext().c_str());
        rf_cameras.setDefaultConfigFile(camerasGroup.find("file").asString().c_str());
        rf_cameras.configure(0,NULL);
    }


    rf_tweak.setQuiet();
    rf_tweak.setDefaultContext(rf.getContext().c_str());
    tweakFile=tweakGroup.check("file",Value("tweak.ini")).asString().c_str();
    tweakOverwrite=(tweakGroup.check("overwrite",Value("off")).asString()=="off");
    rf_tweak.setDefaultConfigFile(tweakFile.c_str());
    rf_tweak.configure(0,NULL);


    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "abmVision", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    while (!iCub->connect())
    {
        yWarning() << "iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }
    iCub->opc->checkout();

    //configureOPC(rf);

    if (!iCub->getABMClient())
    {
        iCub->say("ABM not connected");
        yWarning() << "ABM NOT CONNECTED";
    }

    // Open handler port
    handlerPortName = "/";
    handlerPortName += getName() + "/rpc";         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {
        yInfo() << getName() << ": Unable to open port " << handlerPortName;
        bEveryThingisGood = false;
    }

    attach(handlerPort);                  // attach to port

    bEveryThingisGood = initializeEye();

    if (!bEveryThingisGood || !bOptionnalModule)
        yInfo() << " Some dependencies are not running (iCubClient, ABM?)";
    else
    {
        yInfo() << " ----------------------------------------------";
        yInfo() << " abmVision ready !";
    }

    return bEveryThingisGood;
}

bool abmVision::interruptModule() {
    return true;
}

bool abmVision::close() {
    handlerPort.close();

    delete iCub;

    return true;
}

bool abmVision::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "help") {
        yInfo() << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString() == "get") {

        string camera = "left";

        //Acquire 3D position of a joint
        Vector v3DPoint(3);
        v3DPoint[0] = -0.33;
        v3DPoint[1] = -0.15;
        v3DPoint[2] = 0.1;
        yDebug() << "v3DPoint done!" ;

        //Acquire heads joints value
        Vector vHead(6);
        vHead[0] = -26.538*M_PI/180;
        vHead[1] = -0.78*M_PI/180;
        vHead[2] = 0.044*M_PI/180;
        vHead[3] = 0.044*M_PI/180;
        vHead[4] = 0.986*M_PI/180;
        vHead[5] = -0.004*M_PI/180;
        yDebug() << "vHead done!" ;

        //Acquire torso joints value
        Vector vTorso(3);
        vTorso[0] = 0.011*M_PI/180;
        vTorso[1] = -0.005*M_PI/180;
        vTorso[2] = 0.044*M_PI/180;
        yDebug() << "vTorso done!" ;

        //vector to receive the 2D projection
        Vector vPixel;

        if(projectPoint(camera, v3DPoint, vHead, vTorso, vPixel)){
            yInfo() << "From 3D to 2D done! vPixel = " << vPixel.toString();

        } else {
            yError() << "[abmVision] Call of projectPoint is wrong!" ;
            reply.addString("ERROR");
            return false;
        }

        reply.addString("ack");
    }


    return true;
}

/* Called periodically every getPeriod() seconds */
bool abmVision::updateModule() {
    return true;
}

double abmVision::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}

bool abmVision::projectPoint(const string &type, const Vector &v3DPoint, const Vector vHead, const Vector vTorso, Vector &vPixels){

    LockGuard lg(mutex);

    if (v3DPoint.length()<3)
    {
        yError("Not enough values given for the point!");
        return false;
    }

    bool isLeft=(type=="left");

    Matrix  *Prj=(isLeft?PrjL:PrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);

    if (Prj!=NULL)
    {
        Vector q(8);
        q[0]=vTorso[0];
        q[1]=vTorso[1];
        q[2]=vTorso[2];
        q[3]=vHead[0];
        q[4]=vHead[1];
        q[5]=vHead[2];
        q[6]=vHead[3];
        q[7]=vHead[4]+vHead[5]/(isLeft?2.0:-2.0);

        yDebug() << "vector q done" ;

        Vector xo=v3DPoint;
        // impose homogeneous coordinates
        if (xo.length()<4)
            xo.push_back(1.0);
        else
        {
            xo=xo.subVector(0,3);
            xo[3]=1.0;
        }

        yDebug() << "Before getH" ;
        yDebug() << "q  = " << q.toString();
        yDebug() << "xo = " << xo.toString();
        yDebug() << "eye->getH(q) = " << eye->getH(q).toString();

        // find position wrt the camera frame
        Vector xe=SE3inv(eye->getH(q))*xo;
        yDebug() << "xe is " << xe.toString();

        yDebug() << "Before vPixel update" ;
        // find the 2D projection
        yDebug() << "*Prj is " << Prj->toString();
        vPixels=*Prj*xe;
        vPixels=vPixels/vPixels[2];
        vPixels.pop_back();
        return true;
    }
    else
    {
        yError("Unspecified projection matrix for %s camera!",type.c_str());
        return false;
    }
}

bool abmVision::initializeEye(){

    iCubHeadCenter eyeC("right_v2");

    eyeL=new iCubEye("left_v2");
    eyeR=new iCubEye("right_v2");


    // remove constraints on the links
    // we use the chains for logging purpose
    eyeL->setAllConstraints(false);
    eyeR->setAllConstraints(false);

    // release links
    eyeL->releaseLink(0); eyeC.releaseLink(0); eyeR->releaseLink(0);
    eyeL->releaseLink(1); eyeC.releaseLink(1); eyeR->releaseLink(1);
    eyeL->releaseLink(2); eyeC.releaseLink(2); eyeR->releaseLink(2);

    // get the absolute reference frame of the head
    //Vector q(eyeC.getDOF(),0.0);
    //eyeCAbsFrame=eyeC.getH(q);
    // ... and its inverse
    //invEyeCAbsFrame=SE3inv(eyeCAbsFrame);

    // get the length of the half of the eyes baseline
    //eyesHalfBaseline=0.5*norm(eyeL->EndEffPose().subVector(0,2)-eyeR->EndEffPose().subVector(0,2));

    bool ret;

    // get camera projection matrix
    ret=getCamPrj(rf_cameras,"CAMERA_CALIBRATION_LEFT",&PrjL,true);
    if (tweakOverwrite)
    {
        Matrix *Prj;
        if (getCamPrj(rf_tweak,"CAMERA_CALIBRATION_LEFT",&Prj,true))
        {
            delete PrjL;
            PrjL=Prj;
        }
    }

    yDebug() << "check if ret is true/false: " << ret;

    if (ret)
    {
        cxl=(*PrjL)(0,2);
        cyl=(*PrjL)(1,2);
        invPrjL=new Matrix(pinv(PrjL->transposed()).transposed());
    }
    else
        PrjL=invPrjL=NULL;

    // get camera projection matrix
    ret=getCamPrj(rf_cameras,"CAMERA_CALIBRATION_RIGHT",&PrjR,true);
    if (tweakOverwrite)
    {
        Matrix *Prj;
        if (getCamPrj(rf_tweak,"CAMERA_CALIBRATION_RIGHT",&Prj,true))
        {
            delete PrjR;
            PrjR=Prj;
        }
    }

    if (ret)
    {
        cxr=(*PrjR)(0,2);
        cyr=(*PrjR)(1,2);
        invPrjR=new Matrix(pinv(PrjR->transposed()).transposed());
    }
    else
        PrjR=invPrjR=NULL;

    /*Vector Kp(1,0.001), Ki(1,0.001), Kd(1,0.0);
    Vector Wp(1,1.0),   Wi(1,1.0),   Wd(1,1.0);
    Vector N(1,10.0),   Tt(1,1.0);
    Matrix satLim(1,2);

    satLim(0,0)=0.05;
    satLim(0,1)=10.0;

    pid=new parallelPID(0.05,Kp,Ki,Kd,Wp,Wi,Wd,N,Tt,satLim);

    Vector z0(1,0.5);
    pid->reset(z0);
    dominantEye="left";*/


    return true;
}

/************************************************************************/
bool abmVision::getCamPrj(const ResourceFinder &rf, const string &type,
               Matrix **Prj, const bool verbose)
{

    yDebug() << "in getCamPrj " ;

    ResourceFinder &_rf=const_cast<ResourceFinder&>(rf);
    *Prj=NULL;

    if (!_rf.isConfigured())
        return false;

    yDebug() << "[getCamPrj] check message " ;

    string message=_rf.findFile("from").c_str();
    if (!message.empty())
    {

        yDebug() << "[getCamPrj] message not empty " ;
        message+=": intrinsic parameters for "+type;
        Bottle &parType=_rf.findGroup(type.c_str());
        if (parType.check("fx") && parType.check("fy") &&
            parType.check("cx") && parType.check("cy"))
        {
            double fx=parType.find("fx").asDouble();
            double fy=parType.find("fy").asDouble();
            double cx=parType.find("cx").asDouble();
            double cy=parType.find("cy").asDouble();

            if (verbose)
            {
                yInfo("%s found:",message.c_str());
                yInfo("fx = %g",fx);
                yInfo("fy = %g",fy);
                yInfo("cx = %g",cx);
                yInfo("cy = %g",cy);
            }

            *Prj=new Matrix(eye(3,4));

            Matrix &K=**Prj;
            K(0,0)=fx; K(1,1)=fy;
            K(0,2)=cx; K(1,2)=cy;

            yDebug() << "[getCamPrj] will return true " ;

            return true;
        }
    }
    else
    {
        message=_rf.find("from").asString().c_str();
        message+=": intrinsic parameters for "+type;
    }

    if (verbose)
        yWarning("%s not found!",message.c_str());

    yDebug() << "[getCamPrj] will return false! " ;
    return false;
}


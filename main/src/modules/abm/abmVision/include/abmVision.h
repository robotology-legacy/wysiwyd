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

#ifndef _ABMVISION_H_
#define _ABMVISION_H_

#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>


class abmVision : public yarp::os::RFModule {
private:
    std::string moduleName;
    std::string sKeyWord;

    std::string handlerPortName;
    yarp::os::Port handlerPort;             // a port to handle messages 

    wysiwyd::wrdac::ICubClient *iCub;

    iCub::iKin::iCubEye *eyeL;
    iCub::iKin::iCubEye *eyeR;

    yarp::sig::Matrix *PrjL, *invPrjL;
    yarp::sig::Matrix *PrjR, *invPrjR;
    double  cxl, cyl;
    double  cxr, cyr;

    yarp::os::Mutex                 mutex;

    yarp::os::ResourceFinder rf_cameras;
    yarp::os::ResourceFinder rf_tweak;
    std::string tweakFile;
    bool tweakOverwrite;

    bool projectPoint(const std::string &type, const yarp::sig::Vector &vTarget, const yarp::sig::Vector vHead, const yarp::sig::Vector vTorso, yarp::sig::Vector &vPixels); //based on iKinGazeCtrl/localizer
    bool initializeEye();

    bool getCamPrj(const yarp::os::ResourceFinder &rf, const std::string &type, yarp::sig::Matrix **Prj, const bool verbose);

public:
    /**
    * document your methods too.
    */
    ~abmVision();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();
};


#endif // __ABMVISION_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------


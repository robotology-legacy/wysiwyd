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


class abmVision : public yarp::os::RFModule {
private:
    std::string moduleName;
    std::string sKeyWord;

    std::string handlerPortName;

    yarp::os::Port handlerPort;             // a port to handle messages 

    wysiwyd::wrdac::ICubClient *iCub;

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


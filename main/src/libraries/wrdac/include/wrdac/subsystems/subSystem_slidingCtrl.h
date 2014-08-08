/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com
 * website: http://efaa.upf.edu/ 
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

#ifndef __EFAA_SUBSYSTEM_SLIDINGCONTROL_H__
#define __EFAA_SUBSYSTEM_SLIDINGCONTROL_H__


#define SUBSYSTEM_SLIDING_CONTROLLER     "slidingController"

#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/knowledge/object.h"
#include "slidingController_IDL.h"

namespace wysiwyd{namespace wrdac{

/**
* \ingroup wrdac_clients
*
* SubSystem for sliding motor control. Actually a simple IDL instantiation.
*/
    class SubSystem_SlidingController : public SubSystem
{
private:
    yarp::os::Port idlPortL;
    yarp::os::Port idlPortR;

protected:
    virtual bool connect() 
    { 
        bool success = true;
        success &= yarp::os::Network::connect(idlPortL.getName(), "/slidingController/left/rpc");    // to be done from outside the code :)
        success &= yarp::os::Network::connect(idlPortR.getName(), "/slidingController/right/rpc");  // to be done from outside the code :)
        success &= clientIDL_slidingController_left->yarp().attachAsClient(idlPortL);
        success &= clientIDL_slidingController_right->yarp().attachAsClient(idlPortR);
        return success;
    }

public:
    slidingController_IDL* clientIDL_slidingController_left;
    slidingController_IDL* clientIDL_slidingController_right;

    SubSystem_SlidingController(const std::string &masterName) :SubSystem(masterName)
    {
        clientIDL_slidingController_left = new slidingController_IDL();
        clientIDL_slidingController_right = new slidingController_IDL();

        idlPortL.open("/" + masterName + "/slidingCtrlIDL/left:rpc");
        idlPortL.open("/" + masterName + "/slidingCtrlIDL/right:rpc");

        m_type = SUBSYSTEM_SLIDING_CONTROLLER;
    }

    virtual void Close() 
    { 
        idlPortL.interrupt(); 
        idlPortL.close(); 
        idlPortR.interrupt(); 
        idlPortR.close(); 
        delete clientIDL_slidingController_left;
        delete clientIDL_slidingController_right;
    };
};

}}//Namespace
#endif



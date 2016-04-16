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

#ifndef __EFAA_SUBSYSTEM_RT_H__
#define __EFAA_SUBSYSTEM_RT_H__

#define SUBSYSTEM_REACTABLE     "reactable"

#include <iostream>
#include <iterator>
#include <algorithm>
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/knowledge/rtObject.h"

namespace wysiwyd{namespace wrdac{

/**
* \ingroup wrdac_clients
*
* SubSystem for dealing with postures using ctpService (see ctpService in icub)
*/
class SubSystem_Reactable: public SubSystem
{
protected:
    virtual bool connect();

public:
    yarp::os::Port portRTrpc;
    yarp::os::BufferedPort<yarp::os::Bottle> portRTin;

    SubSystem_Reactable(const std::string &masterName);

    virtual void Close();;
    
    /**
    * Display a virtual object on the reactable
    * @param o the object to be displayed
    */ 
    void SendOSC(yarp::os::Bottle &oscMsg);

    /**
    * Read from osc forwarding
    */ 
    yarp::os::Bottle* ReadOSC(bool shouldWait);

    /**
    * Display a virtual object on the reactable
    * @param o the object to be displayed
    */ 
    void DisplayPosition(RTObject *o, bool convertFromRobotCoordinates);
        
    /**
    * Ask the module to refresh the calibration matrix from RFH
    * @param o the object to be displayed
    */ 
    void RefreshCalibration();
};

}}//Namespace
#endif



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
    virtual bool connect() 
    { 
        bool success = true;
        success &= yarp::os::Network::connect(portRTrpc.getName(), "/reactable2opc/command:i");
        success &= yarp::os::Network::connect("/reactable2opc/osc:o", portRTin.getName());
        return success;
    }

public:
    yarp::os::Port portRTrpc;
    yarp::os::BufferedPort<yarp::os::Bottle> portRTin;

    SubSystem_Reactable(const std::string &masterName):SubSystem(masterName)
    {
        portRTrpc.open( ("/" + m_masterName + "/reactable:rpc").c_str());
        portRTin.open(("/" + m_masterName + "/reactable/osc:i").c_str());
        m_type = SUBSYSTEM_REACTABLE;
    }

    virtual void Close() 
    {
        portRTrpc.interrupt();portRTrpc.close();
        portRTin.interrupt();portRTin.close();
    };
    
    /**
    * Display a virtual object on the reactable
    * @param o the object to be displayed
    */ 
    void SendOSC(yarp::os::Bottle &oscMsg)
    {
            yarp::os::Bottle cmd; 
            cmd.addString("osc");
            for(int i=0; i<oscMsg.size(); i++)
                cmd.add(oscMsg.get(i));
            std::cout<<"OSC>>"<<cmd.toString().c_str()<<std::endl;
            portRTrpc.write(cmd);
    }

    /**
    * Read from osc forwarding
    */ 
    yarp::os::Bottle* ReadOSC(bool shouldWait)
    {
            yarp::os::Bottle* cmd = portRTin.read(shouldWait);
            return cmd;
    }

    /**
    * Display a virtual object on the reactable
    * @param o the object to be displayed
    */ 
    void DisplayPosition(RTObject *o, bool convertFromRobotCoordinates)
    {
            yarp::os::Bottle cmd; 
            if (convertFromRobotCoordinates)
                cmd.addString("addObject");
            else
                cmd.addString("sendBackObject");
            cmd.addList() = o->asBottle();
            portRTrpc.write(cmd);
    }
        
    /**
    * Ask the module to refresh the calibration matrix from RFH
    * @param o the object to be displayed
    */ 
    void RefreshCalibration()
    {
            yarp::os::Bottle cmd; 
            cmd.addString("recalibrate");
            portRTrpc.write(cmd);
    }
};

}}//Namespace
#endif



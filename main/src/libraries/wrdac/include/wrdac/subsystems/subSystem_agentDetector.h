#ifndef SUBSYSTEM_AGENTDETECTOR_H
#define SUBSYSTEM_AGENTDETECTOR_H

/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
 * website: http://wysiwyd.upf.edu/
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

#define SUBSYSTEM_AGENTDETECTOR           "agentDetector"

#include <iostream>
#include "wrdac/subsystems/subSystem.h"

namespace wysiwyd{
namespace wrdac{

/**
        * \ingroup wrdac_clients
        *
        * SubSystem for agentDetector
        */
class SubSystem_agentDetector : public SubSystem
{
protected:
    virtual bool connect() {
        return yarp::os::Network::connect(portRPC.getName(), "/agentDetector/rpc");
    }

public:
    yarp::os::Port portRPC;
    SubSystem_agentDetector(const std::string &masterName) : SubSystem(masterName) {
        portRPC.open(("/" + m_masterName + "/agentDetector:rpc").c_str());
        m_type = SUBSYSTEM_AGENTDETECTOR;
    }

    virtual void Close() {
        portRPC.interrupt();
        portRPC.close();
    }

    bool changeDefaultName(std::string new_name) { // need to be extended for several agents
        yarp::os::Bottle bReq, bResp;
        bReq.addString("change_partner_name");
        bReq.addString(new_name);
        portRPC.write(bReq, bResp);

        if(bResp.get(0).asString()=="ack")
            return true;
        else
            return false;
    }

    void pause() {
        yarp::os::Bottle bReq, bResp;
        bReq.addString("pause");
        portRPC.write(bReq, bResp);
    }

    void resume() {
        yarp::os::Bottle bReq, bResp;
        bReq.addString("resume");
        portRPC.write(bReq, bResp);
    }
};
}
}//Namespace

#endif // SUBSYSTEM_AGENTDETECTOR_H

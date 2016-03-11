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

#ifndef __EFAA_SUBSYSTEM_ATTENTION_H__
#define __EFAA_SUBSYSTEM_ATTENTION_H__

#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/knowledge/object.h"

#define SUBSYSTEM_ATTENTION     "attention"

namespace wysiwyd {
namespace wrdac {

/**
* \ingroup wrdac_clients
*
* SubSystem for attention and gaze (see attentionSelector module)
*/
class SubSystem_Attention: public SubSystem
{
protected:
    yarp::os::RpcClient attentionSelector;
    virtual bool connect()
    {
        return yarp::os::Network::connect(attentionSelector.getName(),"/attentionSelector/rpc");
    }

public:    
    SubSystem_Attention(const std::string &masterName) : SubSystem(masterName)
    {
        attentionSelector.open(("/"+m_masterName+"/attention:rpc").c_str());
        m_type = SUBSYSTEM_ATTENTION;
    }
    virtual ~SubSystem_Attention() {}

    virtual void Close()
    {
        attentionSelector.interrupt();
        attentionSelector.close();
    }

    /**
    * Track object by name.
    * @param name object name to track.
    * @return true in case of successfull motor command, false
    *         otherwise.
    */
    bool track(const std::string &name)
    {
        yarp::os::Bottle bCmd,bRep;
        bCmd.addString("track");
        bCmd.addString(name.c_str());
        if (attentionSelector.write(bCmd,bRep))
            if (bRep.get(0).asString()=="ack")
                return true;

        return false;
    }

    /**
    * Track object by id.
    * @param id object id to track.
    * @return true in case of successfull motor command, false
    *         otherwise.
    */
    bool track(const int id)
    {
        yarp::os::Bottle bCmd,bRep;
        bCmd.addString("track");
        bCmd.addInt(id);
        if (attentionSelector.write(bCmd,bRep))
            if (bRep.get(0).asString()=="ack")
                return true;

        return false;
    }

    /**
    * Enable auto mode.
    * @return true in case of successfull motor command, false
    *         otherwise.
    */
    bool enableAutoMode()
    {
        yarp::os::Bottle bCmd,bRep;
        bCmd.addString("auto");
        if (attentionSelector.write(bCmd,bRep))
            if (bRep.get(0).asString()=="ack")
                return true;

        return false;
    }

    /**
    * Stop attention.
    * @return true in case of successfull motor command, false
    *         otherwise.
    */
    bool stop()
    {
        yarp::os::Bottle bCmd,bRep;
        bCmd.addString("sleep");
        if (attentionSelector.write(bCmd,bRep))
            if (bRep.get(0).asString()=="ack")
                return true;

        return false;
    }

    /**
    * Retrieve current attention status.
    * @param status a string that can be "auto", "object-name",
    *               "quiet".
    * @return true in case of successfull motor command, false
    *         otherwise.
    */
    bool getStatus(std::string &status)
    {
        yarp::os::Bottle bCmd,bRep;
        bCmd.addString("stat");
        if (attentionSelector.write(bCmd,bRep))
        {
            if (bRep.get(0).asString()=="ack")
            {
                status=bRep.get(1).asString().c_str();
                return true;
            }
        }

        return false;
    }

};

}

}

#endif



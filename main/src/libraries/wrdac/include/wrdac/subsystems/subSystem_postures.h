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

#ifndef __EFAA_SUBSYSTEM_POSTURES_H__
#define __EFAA_SUBSYSTEM_POSTURES_H__

#define SUBSYSTEM_POSTURES      "postures"


#include <iostream>
#include <algorithm>
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/clients/animation.h"

namespace wysiwyd{namespace wrdac{


/**
* \ingroup wrdac_clients
*
* SubSystem for dealing with postures using ctpService (see ctpService in icub)
*/
class SubSystem_Postures: public SubSystem
{
protected:
    virtual bool connect() 
    { 
        bool success = true;
        success &= yarp::os::Network::connect(ctpHead.getName(), "/ctpservice/head/rpc");
        success &= yarp::os::Network::connect(ctpLeftArm.getName(), "/ctpservice/left_arm/rpc");
        success &= yarp::os::Network::connect(ctpRightArm.getName(), "/ctpservice/right_arm/rpc");
        success &= yarp::os::Network::connect(ctpTorso.getName(), "/ctpservice/torso/rpc");
        return success;
    }

public:
    yarp::os::Port ctpHead;
    yarp::os::Port ctpLeftArm;
    yarp::os::Port ctpRightArm;
    yarp::os::Port ctpTorso;

    SubSystem_Postures(const std::string &masterName):SubSystem(masterName)
    {
        ctpHead.open( ("/" + m_masterName + "/ctp/head:rpc").c_str());
        ctpLeftArm.open( ("/" + m_masterName + "/ctp/left_arm:rpc").c_str());
        ctpRightArm.open( ("/" + m_masterName + "/ctp/right_arm:rpc").c_str());
        ctpTorso.open( ("/" + m_masterName + "/ctp/torso:rpc").c_str());
        m_type = SUBSYSTEM_POSTURES;
    }

    virtual void Close() 
    {
        ctpHead.interrupt();ctpHead.close();
        ctpLeftArm.interrupt();ctpLeftArm.close();
        ctpRightArm.interrupt();ctpRightArm.close();
        ctpTorso.interrupt();ctpTorso.close();
    };
    
    /**
    * Make the robot to move to a given posture within a given time for a specific part
    * @param p The body posture to be used
    * @param timing The time to execute the movement (the function is no blocking)
    * @param partUsed The part to be moved (head / left_arm / right_arm / left_hand / right_hand / torso)
    */ 
    void Execute(BodyPosture &p, double timing, const std::string &partUsed)
    {
        if ( partUsed == "head")
        {
            //Head
            yarp::os::Bottle cmdHead; 
            cmdHead.addString("ctpq");
            cmdHead.addString("time");
            cmdHead.addDouble(timing);
            cmdHead.addString("off");
            cmdHead.addInt(0);
            cmdHead.addString("pos");
            yarp::os::Bottle& sub = cmdHead.addList();
            for(size_t i=0; i<p.head.size();i++)
            {
                sub.addDouble(p.head[i]);
            }
            ctpHead.write(cmdHead);
        }

        if ( partUsed == "left_arm")
        {
            yarp::os::Bottle cmdLeftArm; 
            cmdLeftArm.addString("ctpq");
            cmdLeftArm.addString("time");
            cmdLeftArm.addDouble(timing);
            cmdLeftArm.addString("off");
            cmdLeftArm.addInt(0);
            cmdLeftArm.addString("pos");
            yarp::os::Bottle& subLA = cmdLeftArm.addList();
            for(size_t i=0; i<p.left_arm.size();i++)
            {
                subLA.addDouble(p.left_arm[i]);
            }
            ctpLeftArm.write(cmdLeftArm);
        }

        if ( partUsed == "left_hand")
        {
            yarp::os::Bottle cmdLeftArm; 
            cmdLeftArm.addString("ctpq");
            cmdLeftArm.addString("time");
            cmdLeftArm.addDouble(timing);
            cmdLeftArm.addString("off");
            cmdLeftArm.addInt(7);
            cmdLeftArm.addString("pos");
            yarp::os::Bottle& subRA = cmdLeftArm.addList();
            for(size_t i=7; i<p.left_arm.size();i++)
            {
                subRA.addDouble(p.left_arm[i]);
            }
            ctpLeftArm.write(cmdLeftArm);
        }

        if ( partUsed == "right_arm")
        {
            yarp::os::Bottle cmdRightArm; 
            cmdRightArm.addString("ctpq");
            cmdRightArm.addString("time");
            cmdRightArm.addDouble(timing);
            cmdRightArm.addString("off");
            cmdRightArm.addInt(0);
            cmdRightArm.addString("pos");
            yarp::os::Bottle& subRA = cmdRightArm.addList();
            for(size_t i=0; i<p.right_arm.size();i++)
            {
                subRA.addDouble(p.right_arm[i]);
            }
            ctpRightArm.write(cmdRightArm);
        }

        if ( partUsed == "right_hand")
        {
            yarp::os::Bottle cmdRightArm; 
            cmdRightArm.addString("ctpq");
            cmdRightArm.addString("time");
            cmdRightArm.addDouble(timing);
            cmdRightArm.addString("off");
            cmdRightArm.addInt(7);
            cmdRightArm.addString("pos");
            yarp::os::Bottle& subRA = cmdRightArm.addList();
            for(size_t i=7; i<p.right_arm.size();i++)
            {
                subRA.addDouble(p.right_arm[i]);
            }
            ctpRightArm.write(cmdRightArm);
        }

        if ( partUsed == "torso")
        {
            yarp::os::Bottle cmdTorso; 
            cmdTorso.addString("ctpq");
            cmdTorso.addString("time");
            cmdTorso.addDouble(timing);
            cmdTorso.addString("off");
            cmdTorso.addInt(0);
            cmdTorso.addString("pos");
            yarp::os::Bottle& subTors = cmdTorso.addList();
            for(size_t i=0; i<p.torso.size();i++)
            {
                subTors.addDouble(p.torso[i]);
            }
            ctpTorso.write(cmdTorso);
        }
    }

    /**
    * Make the robot to move to a given posture within a given time
    * @param p The body posture to be used
    * @param timing The time to execute the movement (the function is no blocking)
    */ 
    void Execute(BodyPosture &p, double timing)
    {
        Execute(p,timing,"head");
        Execute(p,timing,"left_arm");
        Execute(p,timing,"right_arm");
        Execute(p,timing,"torso");
    }
};

}}//Namespace
#endif



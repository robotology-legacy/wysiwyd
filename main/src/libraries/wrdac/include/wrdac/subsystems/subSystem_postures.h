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
    virtual bool connect();

public:
    yarp::os::Port ctpHead;
    yarp::os::Port ctpLeftArm;
    yarp::os::Port ctpRightArm;
    yarp::os::Port ctpTorso;

    SubSystem_Postures(const std::string &masterName);

    virtual void Close();
    
    /**
    * Make the robot to move to a given posture within a given time for a specific part
    * @param p The body posture to be used
    * @param timing The time to execute the movement (the function is no blocking)
    * @param partUsed The part to be moved (head / left_arm / right_arm / left_hand / right_hand / torso)
    */ 
    void Execute(BodyPosture &p, double timing, const std::string &partUsed);

    /**
    * Make the robot to move to a given posture within a given time
    * @param p The body posture to be used
    * @param timing The time to execute the movement (the function is no blocking)
    */ 
    void Execute(BodyPosture &p, double timing);
};

}}//Namespace
#endif



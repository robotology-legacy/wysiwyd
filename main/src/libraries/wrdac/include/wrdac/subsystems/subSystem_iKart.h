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

#ifndef __EFAA_SUBSYSTEM_IKART_H__
#define __EFAA_SUBSYSTEM_IKART_H__
#define SUBSYSTEM_IKART         "ikart"
#undef min
#undef max

#include <cmath>
#include <algorithm>
#include <iostream>
#include <iterator>

#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include "wrdac/subsystems/subSystem.h"

namespace wysiwyd{namespace wrdac{

/**
* \ingroup wrdac_clients
*
* SubSystem for dealing with iKart (allows to move and retrieve info from odometer and laser)
*/
class SubSystem_iKart: public SubSystem
{
protected:
    virtual bool connect();

public:
    yarp::os::Port portRpc;
    yarp::os::Port portCmd;
    yarp::os::BufferedPort<yarp::os::Bottle> portOdometry;
    yarp::os::BufferedPort<yarp::os::Bottle> portLaser;

    double odometry_x, odometry_y, odometry_orientation;

    SubSystem_iKart(const std::string &masterName);
    virtual ~SubSystem_iKart();

    virtual void Close();

    void resetOdometer();

    void printOdometryValues();

    void refreshOdometryValues();

    yarp::os::Bottle getLaserValues();

    void turn(double degreeOffset, double speedMS = 5.0);

    void goToWayPoint(const yarp::sig::Vector &wp);

    void rawCommand(double linearSpeed, double angularSpeed);
};

}}//Namespace
#endif



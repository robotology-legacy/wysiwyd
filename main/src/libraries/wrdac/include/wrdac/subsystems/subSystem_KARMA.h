/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: NGUYEN Dong Hai Phuong
* email:   phuong.nguyen@iit.it
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

#ifndef SUBSYSTEM_KARMA_H
#define SUBSYSTEM_KARMA_H

#include <string>
#include <algorithm>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include "wrdac/subsystems/subSystem.h"
#include "wrdac/subsystems/subSystem_ABM.h"
#include "wrdac/subsystems/subSystem_attention.h"

#define SUBSYSTEM_KARMA       "KARMA"

namespace wysiwyd {
    namespace wrdac {

        /**
        * \ingroup wrdac_clients
        *
        * SubSystem to deal with the <b>experimental affordance learning</b>
        * module (a.k.a. <b>KARMA</b>) for motor control.
        *
        * For further details, please refer to the KARMA main page:
        * http://robotology.github.io/karma/doxygen/doc/html/group__karmaMotor.html
        */
        class SubSystem_KARMA : public SubSystem
        {
        protected:
            SubSystem_ABM* SubABM;
            bool ABMconnected;

            SubSystem_Attention* SubATT;
            bool ATTconnected;

            std::string robot;

            yarp::os::RpcClient stopPort;
            yarp::os::RpcClient rpcPort;
            yarp::os::RpcClient visionPort;
            yarp::os::RpcClient finderPort;

            //testing Cartesian interface
            yarp::dev::PolyDriver driverL;
            yarp::dev::PolyDriver driverR;
            yarp::dev::PolyDriver driverHL;
            yarp::dev::PolyDriver driverHR;

            yarp::dev::ICartesianControl *iCartCtrlL;
            yarp::dev::ICartesianControl *iCartCtrlR;

            /********************************************************************************/
            void appendTarget(yarp::os::Bottle& b, const yarp::sig::Vector &tCenter);

            void appendDouble(yarp::os::Bottle& b, const double &v);

            /********************************************************************************/
            bool prepare();

            void closeICartClient();



            /********************************************************************************/
            bool sendCmd(yarp::os::Bottle &cmd, const bool disableATT=false);

            /********************************************************************************/
            bool connect();

        public:
            /**
            * Default constructor.
            * @param masterName stem-name used to open up ports.
            */
            SubSystem_KARMA(const std::string &masterName, const std::string &robot);

            /**
            * Clean up resources.
            */
            void Close();

            yarp::sig::Vector applySafetyMargins(const yarp::sig::Vector& in);

            /**
             * @brief push (KARMA): push to certain position, along a direction
             * @param targetCenter: position to push to.
             * @param theta: angle between the y-axis (in robot FoR) and starting position of push action, defines the direction of push action
             * @param radius: radius of the circle with center at @see targetCenter
             * @param options
             * @param sName
             * @return true in case of success release, false otherwise
             */
            bool push(const yarp::sig::Vector &targetCenter, const double theta, const double radius,
                      const yarp::os::Bottle &options = yarp::os::Bottle(),
                      const std::string &sName="target");

            /**
             * @brief draw (KARMA): draw action, along the positive direction of the x-axis (in robot FoR)
             * @param targetCenter: center of a circle
             * @param theta: angle between the y-axis (in robot FoR) and starting position of draw action.
             * @param radius: radius of the circle with center at @see targetCenter
             * @param dist: moving distance of draw action
             * @param options
             * @param sName
             * @return true in case of success release, false otherwise
             */
            bool draw(const yarp::sig::Vector &targetCenter, const double theta,
                      const double radius, const double dist,
                      const yarp::os::Bottle &options = yarp::os::Bottle(),
                      const std::string &sName="target");

            /**
             * @brief vdraw (KARMA): draw action, along the positive direction of the x-axis (in robot FoR)
             * @param targetCenter: center of a circle
             * @param theta: angle between the y-axis (in robot FoR) and starting position of draw action.
             * @param radius: radius of the circle with center at @see targetCenter
             * @param dist: moving distance of draw action
             * @param options
             * @param sName
             * @return true in case of success release, false otherwise
             */
            bool vdraw(const yarp::sig::Vector &targetCenter, const double theta,
                      const double radius, const double dist,
                      const yarp::os::Bottle &options = yarp::os::Bottle(),
                      const std::string &sName="target");

            bool openCartesianClient();

            /**
            * Destructor.
            */
            ~SubSystem_KARMA();
        };
    }
}

#endif // SUBSYSTEM_KARMA_H

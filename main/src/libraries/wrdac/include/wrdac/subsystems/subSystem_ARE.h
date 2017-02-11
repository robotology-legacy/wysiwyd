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

#ifndef __EFAA_SUBSYSTEM_ARE_H__
#define __EFAA_SUBSYSTEM_ARE_H__

#include <string>
#include <algorithm>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "wrdac/subsystems/subSystem.h"
#include "wrdac/subsystems/subSystem_ABM.h"
#include "wrdac/subsystems/subSystem_attention.h"
#include "wrdac/clients/opcClient.h"

#define SUBSYSTEM_ARE       "ARE"

namespace wysiwyd {
    namespace wrdac {

        /**
        * \ingroup wrdac_clients
        *
        * SubSystem to deal with the <b>actionsRenderingEngine</b>
        * module (a.k.a. <b>ARE</b>) for motor control.
        *
        * For further details, please refer to the ARE main page:
        * http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html
        */
        class SubSystem_ARE : public SubSystem
        {
        protected:
            SubSystem_ABM* SubABM;
            bool ABMconnected;

            SubSystem_Attention* SubATT;
            bool ATTconnected;

            OPCClient *opc;

            yarp::os::RpcClient cmdPort;            
            yarp::os::RpcClient rpcPort;
            yarp::os::RpcClient getPort;
            yarp::os::RpcClient calibPort;

            std::string lastlyUsedHand;

            /********************************************************************************/
            void appendCartesianTarget(yarp::os::Bottle& b, const yarp::sig::Vector &t);

            /********************************************************************************/
            void selectHandCorrectTarget(yarp::os::Bottle& options, yarp::sig::Vector& target,
                                         const std::string& objName, const std::string handToUse="");

            /********************************************************************************/
            bool sendCmd(const yarp::os::Bottle &cmd, const bool disableATT=false);

            /********************************************************************************/
            bool sendCmdNoReply(yarp::os::Bottle &cmd);

            /********************************************************************************/
            bool connect();

        public:
            /**
            * Default constructor.
            * @param masterName stem-name used to open up ports.
            */
            SubSystem_ARE(const std::string &masterName);

            /**
            * Clean up resources.
            */
            void Close();

            /********************************************************************************/
            bool getTableHeight(double &height);

            /********************************************************************************/
            yarp::sig::Vector applySafetyMargins(const yarp::sig::Vector& in);

            /**
            * Put the specified part ih home position.
            * @param part the part to be homed ("gaze", "head", "arms",
            *             "fingers", "all"; "all" by default).
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool home(const std::string &part = "all");

            /**
            * Reach the specified [target] and grasp it. Optional parameter
            * "side" or "above" can be supplied to choose the orientation
            * the robot should keep while performing the action (default:
            * "above").
            * @param target Target to grasp in cartesian coordinates
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool take(const std::string &sName, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Reach the specified [target] from one side and then push it
            * laterally. Optional parameter "away" can be supplied in order
            * to have the robot push the object away from its root reference
            * frame.
            * @param target Target to push in cartesian coordinates
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool push(const std::string &sName, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Point at the specified [target] with the index finger.
            * The target can be far away from the iCub, e.g. a body part of the human
            * @param target Target to point to in cartesian coordinates
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool pointfar(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle(),
                       const std::string &sName="target");

            /**
            * Point at the specified [target] with the index finger.
            * @param target Target to point (object name)
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool point(const std::string &sName, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * If an object is held, bring it over the table and drop it on a
            * random position.
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool drop(const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Drop the object on a given target.
            * @param target Target where to drop in cartesian coordinates
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool dropOn(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Bring the hand in the visual field and move it with the
            * purpose of visual exploration.
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool observe(const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Put one hand forward with the palm facing up and wait for an
            * object.
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool expect(const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Put one hand forward with the palm facing up and open the
            * fingers so that the object held in the hand is free to be
            * taken.
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool give(const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Enable/disable arms waving.
            * @param sw enable/disable if true/false.
            * @return true in case of successfull request, false otherwise.
            */
            bool waving(const bool sw);

            /**
            * Look at the specified [target].
            * @param target Target to look at in cartesian coordinates
            * @param options Options of ARE commands ("fixate", 
            *             "wait", (block_eyes ver)).
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool look(const yarp::sig::Vector &target, const yarp::os::Bottle &options = yarp::os::Bottle(),
                      const std::string &sName="target");

            /**
            * Track the specified [target].
            * @param target Target to look at in cartesian coordinates
            * @param options Options of ARE commands ("no_sacc"). 
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool track(const yarp::sig::Vector &target, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Enable/disable impedance control.
            * @param sw enable/disable if true/false.
            * @return true in case of successfull request, false otherwise.
            */
            bool impedance(const bool sw);

            /**
            * Change default arm movement execution time.
            * @param execTime the arm movement execution time given in
            *                 seconds.
            * @return true in case of successfull request, false otherwise.
            */
            bool setExecTime(const double execTime);

            /**
            * Destructor.
            */
            ~SubSystem_ARE();
        };
    }
}

#endif

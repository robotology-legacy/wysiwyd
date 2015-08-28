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
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/knowledge/object.h"

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
            yarp::os::BufferedPort<yarp::os::Bottle> cmdNoWaitPort;
            yarp::os::RpcClient cmdPort;            
            yarp::os::RpcClient rpcPort;
            yarp::os::RpcClient getPort;

            /********************************************************************************/
            void appendCartesianTarget(yarp::os::Bottle& b, const yarp::sig::Vector &t)
            {
                yarp::os::Bottle &sub = b.addList();
                sub.addString("cartesian");
                for (size_t i = 0; i < t.length(); i++)
                    sub.addDouble(t[i]);
            }

            /********************************************************************************/
            bool sendCmd(yarp::os::Bottle &cmd, const bool shouldWait = true)
            {
                if (shouldWait)
                {
                    yarp::os::Bottle bReply;
                    if (cmdPort.write(cmd, bReply))
                        return (bReply.get(0).asVocab() == yarp::os::Vocab::encode("ack"));
                }
                else
                {
                    cmdNoWaitPort.prepare()=cmd;
                    return cmdNoWaitPort.writeStrict();
                }

                return false;
            }

            /********************************************************************************/
            bool connect()
            {
                SubABM = new SubSystem_ABM("from_ARE");
                ABMconnected = (SubABM->Connect());
                std::cout << ((ABMconnected) ? "ARE connected to ABM" : "ARE didn't connect to ABM") << std::endl;

                bool ret = true;
                ret &= yarp::os::Network::connect(cmdPort.getName(),"/actionsRenderingEngine/cmd:io");
                ret &= yarp::os::Network::connect(cmdNoWaitPort.getName(),"/actionsRenderingEngine/cmd:io");
                ret &= yarp::os::Network::connect(rpcPort.getName(),"/actionsRenderingEngine/rpc");
                ret &= yarp::os::Network::connect(getPort.getName(),"/actionsRenderingEngine/get:io");
                return ret;
            }

            bool ABMconnected;
            SubSystem_ABM* SubABM;

        public:
            /**
            * Default constructor.
            * @param masterName stem-name used to open up ports.
            */
            SubSystem_ARE(const std::string &masterName) : SubSystem(masterName)
            {
                cmdPort.open(("/" + masterName + "/" + SUBSYSTEM_ARE + "/cmd:io").c_str());
                cmdNoWaitPort.open(("/" + masterName + "/" + SUBSYSTEM_ARE + "/cmd_nowait:io").c_str());
                rpcPort.open(("/" + masterName + "/" + SUBSYSTEM_ARE + "/rpc").c_str());
                getPort.open(("/" + masterName + "/" + SUBSYSTEM_ARE + "/get:io").c_str());
                m_type = SUBSYSTEM_ARE;
            }

            /**
            * Clean up resources.
            */
            void Close()
            {
                cmdPort.interrupt();
                cmdNoWaitPort.interrupt();
                rpcPort.interrupt();
                getPort.interrupt();
                SubABM->Close();

                cmdPort.close();
                cmdNoWaitPort.close();
                rpcPort.close();
                getPort.close();
            }

            /********************************************************************************/
            bool getTableHeight(double &height)
            {
                yarp::os::Bottle bCmd, bReply;
                bCmd.addVocab(yarp::os::Vocab::encode("get"));
                bCmd.addVocab(yarp::os::Vocab::encode("table"));
                getPort.write(bCmd, bReply);

                yarp::os::Value vHeight = bReply.find("table_height");
                if (vHeight.isNull()) {
                    yError("No table height specified in ARE!");
                    return false;
                }
                else {
                    height = vHeight.asDouble();
                    return true;
                }
            }

            /********************************************************************************/
            yarp::sig::Vector applySafetyMargins(const yarp::sig::Vector& in)
            {
                yarp::sig::Vector out = in;

                double height;
                if (getTableHeight(height)) {
                    if (out[2] < height) {
                        out[2] = height + 0.03; // TODO: Add offset in config file
                    }
                }

                if (out[0] > -0.1) {
                    out[0] = -0.1;
                }

                return out;
            }

            /**
            * Put the specified part ih home position.
            * @param part the part to be homed ("gaze", "head", "arms",
            *             "fingers", "all"; "all" by default).
            * @param shouldWait is the function blocking?
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool home(const std::string &part = "all", const bool shouldWait = true)
            {
                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("home"));
                bCmd.addString(part.c_str());
                // send the result of recognition to the ABM
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(part, "argument"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "home", "action", lArgument, true);
                }
                bool bReturn = sendCmd(bCmd, shouldWait);
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(part, "argument"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "home", "action", lArgument, false);
                }

                return bReturn;
            }

            /**
            * Reach the specified [target] and grasp it. Optional parameter
            * "side" or "above" can be supplied to choose the orientation
            * the robot should keep while performing the action (default:
            * "above").
            * @param target Target to grasp in cartesian coordinates
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @param shouldWait is the function blocking?
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool take(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle(),
                const bool shouldWait = true)
            {
                yarp::sig::Vector target = applySafetyMargins(targetUnsafe);
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "take", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("take"));
                appendCartesianTarget(bCmd, target);
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd, shouldWait);

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "take", "action", lArgument, false);
                }

                return bReturn;
            }

            /**
            * Reach the specified [target] from one side and then push it
            * laterally. Optional parameter "away" can be supplied in order
            * to have the robot push the object away from its root reference
            * frame.
            * @param target Target to grasp in cartesian coordinates
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @param shouldWait is the function blocking?
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool push(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle(),
                const bool shouldWait = true)
            {
                yarp::sig::Vector target = applySafetyMargins(targetUnsafe);
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "push", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("push"));
                appendCartesianTarget(bCmd, target);
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd, shouldWait);

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "push", "action", lArgument, false);
                }
                return bReturn;

            }

            /**
            * Point at the specified [target] with the index finger.
            * @param target Target to grasp in cartesian coordinates
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @param shouldWait is the function blocking?
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool point(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle(),
                const bool shouldWait = true)
            {
                yarp::sig::Vector target = applySafetyMargins(targetUnsafe);
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "point", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("point"));
                appendCartesianTarget(bCmd, target);
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd, shouldWait);

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "point", "action", lArgument, false);
                }
                return bReturn;
            }

            /**
            * If an object is held, bring it over the table and drop it on a
            * random position.
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @param shouldWait is the function blocking?
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool drop(const yarp::os::Bottle &options = yarp::os::Bottle(), const bool shouldWait = true)
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "drop", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("drop"));
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd, shouldWait);

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "drop", "action", lArgument, true);
                }
                return bReturn;

            }

            /**
            * Drop the object on a given target.
            * @param target Target where to drop in cartesian coordinates
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @param shouldWait is the function blocking?
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool dropOn(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle(),
                const bool shouldWait = true)
            {
                yarp::sig::Vector target = applySafetyMargins(targetUnsafe);
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "dropOn", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("drop"));
                bCmd.addString("over");
                appendCartesianTarget(bCmd, target);
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd, shouldWait);
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "dropOn", "action", lArgument, false);
                }
                return bReturn;
            }

            /**
            * Bring the hand in the visual field and move it with the
            * purpose of visual exploration.
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @param shouldWait is the function blocking?
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool observe(const yarp::os::Bottle &options = yarp::os::Bottle(), const bool shouldWait = true)
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "observe", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("observe"));
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd, shouldWait);
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "observe", "action", lArgument, false);
                }

                return bReturn;
            }

            /**
            * Put one hand forward with the palm facing up and wait for an
            * object.
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @param shouldWait is the function blocking?
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool expect(const yarp::os::Bottle &options = yarp::os::Bottle(), const bool shouldWait = true)
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "expect", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("expect"));
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd, shouldWait);
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "expect", "action", lArgument, false);
                }

                return bReturn;
            }

            /**
            * Put one hand forward with the palm facing up and open the
            * fingers so that the object held in the hand is free to be
            * taken.
            * @param options Options of ARE commands ("no_head", "no_gaze",
            *             "no_sacc", "still", "left", "right").
            * @param shouldWait is the function blocking?
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool give(const yarp::os::Bottle &options = yarp::os::Bottle(), const bool shouldWait = true)
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "give", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("give"));
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd, shouldWait);
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "give", "action", lArgument, false);
                }
                return bReturn;
            }

            /**
            * Enable/disable arms waving.
            * @param sw enable/disable if true/false.
            * @return true in case of successfull request, false otherwise.
            */
            bool waving(const bool sw)
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "waving", "action", lArgument, true);
                }
                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("waveing"));
                bCmd.addString(sw ? "on" : "off");
                bool bReturn = rpcPort.asPort().write(bCmd);

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "waving", "action", lArgument, false);
                }
                return bReturn;
            }

            /**
            * Enable/disable impedance control.
            * @param sw enable/disable if true/false.
            * @return true in case of successfull request, false otherwise.
            */
            bool impedance(const bool sw)
            {
                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("impedance"));
                bCmd.addString(sw ? "on" : "off");
                return rpcPort.asPort().write(bCmd);
            }

            /**
            * Change default arm movement execution time.
            * @param execTime the arm movement execution time given in
            *                 seconds.
            * @return true in case of successfull request, false otherwise.
            */
            bool setExecTime(const double execTime)
            {
                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("time"));
                bCmd.addDouble(execTime);
                return rpcPort.asPort().write(bCmd);
            }
        };

    }
}

#endif



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

            yarp::os::RpcClient cmdPort;
            yarp::os::RpcClient rpcPort;
            yarp::os::RpcClient getPort;
            yarp::os::RpcClient d2kPort;

            std::string lastlyUsedHand;

            /********************************************************************************/
            void appendCartesianTarget(yarp::os::Bottle& b, const yarp::sig::Vector &t)
            {
                yarp::os::Bottle &sub=b.addList();
                sub.addString("cartesian");
                for (size_t i=0; i<t.length(); i++)
                    sub.addDouble(t[i]);
            }

            /********************************************************************************/
            void selectHandCorrectTarget(yarp::os::Bottle& options, yarp::sig::Vector& target,
                                         const std::string handToUse="")
            {
                std::string hand="";
                for (int i=0; i<options.size(); i++)
                {
                    yarp::os::Value val=options.get(i);
                    if (val.isString())
                    {
                        std::string item=val.asString();
                        if ((item=="left") || (item=="right"))
                        {
                            hand=item;
                            break;
                        }
                    }
                }

                // always choose hand
                if (hand.empty())
                {
                    if (handToUse.empty())
                    {
                        hand=(target[1]>0.0?"right":"left"); 
                        options.addString(hand.c_str());
                    }
                    else
                        hand=lastlyUsedHand;
                }

                // apply depth2kin correction
                if (d2kPort.getOutputCount()>0)
                {
                    yarp::os::Bottle cmd,reply;
                    cmd.addString("getPoint");
                    cmd.addString(hand.c_str());
                    cmd.addDouble(target[0]);
                    cmd.addDouble(target[1]);
                    cmd.addDouble(target[2]);
                    d2kPort.write(cmd,reply);
                    target[0]=reply.get(1).asDouble();
                    target[1]=reply.get(2).asDouble();
                    target[2]=reply.get(3).asDouble();
                }

                lastlyUsedHand=hand;
            }

            /********************************************************************************/
            bool sendCmd(yarp::os::Bottle &cmd, const bool disableATT=false)
            {
                bool ret=false;

                std::string status;
                if (ATTconnected && disableATT)
                {
                    SubATT->getStatus(status);
                    if (status!="quiet")
                        SubATT->stop(); 
                }

                yarp::os::Bottle bReply;
                if (cmdPort.write(cmd,bReply))
                    ret=(bReply.get(0).asVocab()==yarp::os::Vocab::encode("ack"));

                if (ATTconnected && disableATT)
                {
                    if (status=="auto")
                        SubATT->enableAutoMode();
                    else if (status!="quiet")
                        SubATT->track(status);
                }

                return ret;
            }

            /********************************************************************************/
            bool connect()
            {
                if (ABMconnected=SubABM->Connect())
                    yInfo()<<"ARE connected to ABM";
                else
                    yWarning()<<"ARE didn't connect to ABM";

                if (ATTconnected=SubATT->Connect())
                    yInfo()<<"ARE connected to Attention";
                else
                    yWarning()<<"ARE didn't connect to Attention";

                bool ret=true;
                ret&=yarp::os::Network::connect(cmdPort.getName(),"/actionsRenderingEngine/cmd:io");
                ret&=yarp::os::Network::connect(rpcPort.getName(),"/actionsRenderingEngine/rpc");
                ret&=yarp::os::Network::connect(getPort.getName(),"/actionsRenderingEngine/get:io");

                if (yarp::os::Network::connect(d2kPort.getName(),"/depth2kin/rpc"))
                    yInfo()<<"ARE connected to depth2kin";
                else
                    yWarning()<<"ARE didn't connect to depth2kin";

                return ret;
            }

        public:
            /**
            * Default constructor.
            * @param masterName stem-name used to open up ports.
            */
            SubSystem_ARE(const std::string &masterName) : SubSystem(masterName)
            {
                SubABM = new SubSystem_ABM(m_masterName+"/from_ARE");
                SubATT = new SubSystem_Attention(m_masterName+"/from_ARE");

                cmdPort.open(("/" + masterName + "/" + SUBSYSTEM_ARE + "/cmd:io").c_str());
                rpcPort.open(("/" + masterName + "/" + SUBSYSTEM_ARE + "/rpc").c_str());
                getPort.open(("/" + masterName + "/" + SUBSYSTEM_ARE + "/get:io").c_str());
                d2kPort.open(("/" + masterName + "/" + SUBSYSTEM_ARE + "/d2k:io").c_str());
                m_type = SUBSYSTEM_ARE;
                lastlyUsedHand="";
            }

            /**
            * Clean up resources.
            */
            void Close()
            {
                cmdPort.interrupt();
                rpcPort.interrupt();
                getPort.interrupt();
                d2kPort.interrupt();

                SubABM->Close();
                SubATT->Close();

                cmdPort.close();
                rpcPort.close();
                getPort.close();
                d2kPort.close();
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
                yarp::sig::Vector out=in;
                out[0]=std::min(out[0],-0.1);

                double height;
                if (getTableHeight(height))
                    if (out[2]<height)
                        out[2]=height+0.03; // TODO: Add offset in config file
                
                return out;
            }

            /**
            * Put the specified part ih home position.
            * @param part the part to be homed ("gaze", "head", "arms",
            *             "fingers", "all"; "all" by default).
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool home(const std::string &part = "all")
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
                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(part, "argument"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
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
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool take(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle(), std::string sName = "target")
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(targetUnsafe.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>("take", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "take", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("take"));
                
                yarp::sig::Vector target=targetUnsafe;
                yarp::os::Bottle opt=options;
                selectHandCorrectTarget(opt,target);
                target=applySafetyMargins(target);
                appendCartesianTarget(bCmd,target);
                bCmd.append(opt);

                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(targetUnsafe.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>("take", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
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
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool push(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle(), std::string sName="target")
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(targetUnsafe.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>("push", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "push", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("push"));
                
                yarp::sig::Vector target=targetUnsafe;
                yarp::os::Bottle opt=options;
                selectHandCorrectTarget(opt,target);
                target=applySafetyMargins(target);
                appendCartesianTarget(bCmd,target);
                bCmd.append(opt);

                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(targetUnsafe.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>("push", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
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
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool point(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle(), std::string sName="target")
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(targetUnsafe.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>("point", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "point", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("point"));
                
                yarp::sig::Vector target=targetUnsafe;
                yarp::os::Bottle opt=options;
                selectHandCorrectTarget(opt,target);
                target=applySafetyMargins(target);
                appendCartesianTarget(bCmd,target);
                bCmd.append(opt);

                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(targetUnsafe.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>("point", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
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
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool drop(const yarp::os::Bottle &options = yarp::os::Bottle())
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    lArgument.push_back(std::pair<std::string, std::string>("drop", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    SubABM->sendActivity("action", "drop", "action", lArgument, true);
                }

                // we don't need d2k correction
                // because the drop takes place
                // on a random location
                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("drop"));
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("drop", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
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
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool dropOn(const yarp::sig::Vector &targetUnsafe, const yarp::os::Bottle &options = yarp::os::Bottle())
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(targetUnsafe.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "dropOn", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("drop"));
                bCmd.addString("over");
                
                yarp::sig::Vector target=targetUnsafe;
                yarp::os::Bottle opt=options;
                selectHandCorrectTarget(opt,target,lastlyUsedHand);
                target=applySafetyMargins(target);
                appendCartesianTarget(bCmd,target);
                bCmd.append(opt);

                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(targetUnsafe.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
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
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool observe(const yarp::os::Bottle &options = yarp::os::Bottle())
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
                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
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
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool expect(const yarp::os::Bottle &options = yarp::os::Bottle())
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
                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
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
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool give(const yarp::os::Bottle &options = yarp::os::Bottle())
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
                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
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
                std::string status;
                bReturn ? status = "success" : status = "failed";

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "waving", "action", lArgument, false);
                }
                return bReturn;
            }

            /**
            * Look at the specified [target].
            * @param target Target to look at in cartesian coordinates
            * @param options Options of ARE commands ("fixate", (block_eyes 
            *             ver)).
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool look(const yarp::sig::Vector &target, const yarp::os::Bottle &options = yarp::os::Bottle(), std::string sName="target")
            {
                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>("look", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "look", "action", lArgument, true);
                }

                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("look"));
                appendCartesianTarget(bCmd, target);
                bCmd.append(options);
                bool bReturn = sendCmd(bCmd,true);
                std::string status;
                bReturn ? status = "success" : status = "failed";

                if (ABMconnected)
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(target.toString().c_str(), "vector"));
                    lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
                    lArgument.push_back(std::pair<std::string, std::string>("look", "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
                    lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
                    lArgument.push_back(std::pair<std::string, std::string>("ARE", "subsystem"));
                    SubABM->sendActivity("action", "look", "action", lArgument, false);
                }
                return bReturn;
            }

            /**
            * Track the specified [target].
            * @param target Target to look at in cartesian coordinates
            * @param options Options of ARE commands ("no_sacc"). 
            * @return true in case of successfull motor command, false
            *         otherwise.
            */
            bool track(const yarp::sig::Vector &target, const yarp::os::Bottle &options = yarp::os::Bottle())
            {
                // track() is meant for streaming => no point in gating the activity continuously
                yarp::os::Bottle bCmd;
                bCmd.addVocab(yarp::os::Vocab::encode("track"));
                appendCartesianTarget(bCmd, target);
                bCmd.append(options);
                return sendCmd(bCmd);
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

            /**
            * Destructor.
            */
            ~SubSystem_ARE()
            {
                delete SubABM;
                delete SubATT;
            }
        };
    }
}

#endif



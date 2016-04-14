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

#ifndef __EFAA_SUBSYSTEM_ABM_H__
#define __EFAA_SUBSYSTEM_ABM_H__

#define SUBSYSTEM_ABM           "abm"

#include <iostream>
#include "wrdac/subsystems/subSystem.h"

namespace wysiwyd{
    namespace wrdac{

        /**
        * \ingroup wrdac_clients
        *
        * SubSystem for ABM (see autoBiographicMemory module)
        */
        class SubSystem_ABM : public SubSystem
        {
        protected:
            virtual bool connect() {
                if (yarp::os::Network::isConnected(portRPC.getName(), "/autobiographicalMemory/rpc")){
                    return true;
                }
                else {
                    yWarning(" in SubSytem_ABM: cannot connect, tyring connection.");
                    bool ret = yarp::os::Network::connect(portRPC.getName(), "/autobiographicalMemory/rpc"); 
                    if (!ret) yWarning(" connection to ABM impossible: check that ABM is running");
                    return ret;
                }
            }

        public:
            yarp::os::Port portRPC;
            SubSystem_ABM(const std::string &masterName) :SubSystem(masterName){
                portRPC.open(("/" + m_masterName + "/abm:rpc").c_str());
                m_type = SUBSYSTEM_ABM;
            }
            virtual ~SubSystem_ABM() {}

            virtual void Close() { portRPC.interrupt(); portRPC.close(); }

            void sendActivity(
                const std::string& activityType,
                const std::string& activyInformation,
                const std::string& activityContext,
                const std::list<std::pair<std::string, std::string> >& arguments,
                bool fBegin = true)
            {
                yarp::os::Bottle
                    bMain,          // main information about the activity
                    bArgument,      // Argument of the activity
                    bBegin;         // information about begining or end of the activity

                bMain.addString(activityType.c_str());
                bMain.addString(activyInformation.c_str());
                bMain.addString(activityContext.c_str());

                bArgument.addString("arguments");


                for (std::list<std::pair<std::string, std::string> >::const_iterator itArg = arguments.begin(); itArg != arguments.end(); itArg++)
                {
                    yarp::os::Bottle  bTemp;
                    bTemp.clear();
                    bTemp.addString(itArg->first);
                    bTemp.addString(itArg->second);
                    bArgument.addList() = bTemp;
                }

                bBegin.addString("begin");
                bBegin.addInt((int)fBegin);

                yarp::os::Bottle bSnapshot;
                bSnapshot.clear();
                bSnapshot.addString("snapshot");
                bSnapshot.addList() = bMain;
                bSnapshot.addList() = bArgument;
                bSnapshot.addList() = bBegin;
                if (connect())  portRPC.write(bSnapshot);
            }

            yarp::os::Bottle requestFromString(const std::string &sInput)
            {
                yarp::os::Bottle bReplyRequest;
                //send the SQL query within a bottle to autobiographicalMemory
                yarp::os::Bottle bQuery;
                bQuery.addString("request");
                bQuery.addString(sInput.c_str());
                portRPC.write(bQuery, bReplyRequest);
                return bReplyRequest;
            }

            yarp::os::Bottle resetKnowledge()
            {
                yarp::os::Bottle bReplyRequest;
                //send the SQL query within a bottle to autobiographicalMemory
                yarp::os::Bottle bQuery;
                bQuery.addString("resetKnowledge");
                portRPC.write(bQuery, bReplyRequest);
                return bReplyRequest;
            }

            yarp::os::Bottle triggerStreaming(int iCurrentInstance, bool realtime=false, bool includeAugmented=true, double speedMultiplier=1.0, std::string robot="icubSim")
            {
                yarp::os::Bottle bSend,
                    bReceived;

                bSend.addString("triggerStreaming");
                bSend.addInt(iCurrentInstance);

                yarp::os::Bottle bRealtime;
                bRealtime.addString("realtime");
                bRealtime.addInt((int) realtime);
                bSend.addList() = bRealtime;

                yarp::os::Bottle bAugmented;
                bAugmented.addString("includeAugmented");
                bAugmented.addInt((int) includeAugmented);
                bSend.addList() = bAugmented;

                yarp::os::Bottle bSpeedMultiplier;
                bSpeedMultiplier.addString("speedMultiplier");
                bSpeedMultiplier.addDouble(speedMultiplier);
                bSend.addList() = bSpeedMultiplier;


                yarp::os::Bottle bRobot;
                bRobot.addString("robot");
                bRobot.addString(robot);
                bSend.addList() = bRobot;

                portRPC.write(bSend, bReceived);

                return bReceived;
            }

            yarp::os::Bottle rpcCommand(yarp::os::Bottle &bRequest)
            {
                yarp::os::Bottle bReply;

                yInfo() << " [rpcCommand] bRequest = " << bRequest.toString() ;
                //send the SQL query within a bottle to autobiographicalMemory
                portRPC.write(bRequest, bReply);
                return bReply;
            }

        };
    }
}//Namespace
#endif

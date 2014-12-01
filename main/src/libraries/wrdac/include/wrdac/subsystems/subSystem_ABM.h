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
			virtual bool connect() { return yarp::os::Network::connect(portRPC.getName(), "/autobiographicalMemory/rpc"); }

		public:
			yarp::os::Port portRPC;
			SubSystem_ABM(const std::string &masterName) :SubSystem(masterName){
				portRPC.open(("/" + m_masterName + "/abm:rpc").c_str());
				m_type = SUBSYSTEM_ABM;
			}

			virtual void Close() { portRPC.interrupt(); portRPC.close(); };

			void sendActivity(
				std::string activityType,
				std::string activyInformation,
				std::string activityContext,
				std::list<std::pair<std::string, std::string> > arguments,
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


				for (std::list<std::pair<std::string, std::string> >::iterator itArg = arguments.begin(); itArg != arguments.end(); itArg++)
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
				portRPC.write(bSnapshot);
			}

			yarp::os::Bottle requestFromString(std::string &sInput)
			{
				yarp::os::Bottle bReplyRequest;
				//send the SQL query within a bottle to autobiographicalMemory
				yarp::os::Bottle bQuery;
				bQuery.addString("request");
				bQuery.addString(sInput.c_str());
				portRPC.write(bQuery, bReplyRequest);
				return bReplyRequest;
			}

		};

	}
}//Namespace
#endif



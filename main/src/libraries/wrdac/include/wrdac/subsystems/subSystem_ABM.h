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
#include <list>
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
            virtual bool connect();

        public:
            yarp::os::Port portRPC;
            SubSystem_ABM(const std::string &masterName);
            virtual ~SubSystem_ABM();

            virtual void Close();

            void sendActivity(
                const std::string& activityType,
                const std::string& activyInformation,
                const std::string& activityContext,
                const std::list<std::pair<std::string, std::string> >& arguments,
                bool fBegin = true);

            yarp::os::Bottle requestFromString(const std::string &sInput);

            yarp::os::Bottle resetKnowledge();

            yarp::os::Bottle triggerStreaming(int iCurrentInstance, bool realtime=false, bool includeAugmented=true, double speedMultiplier=1.0, std::string robot="icubSim", bool blocking=false);

            yarp::os::Bottle rpcCommand(yarp::os::Bottle &bRequest);
            yarp::os::Bottle addImgStreamProvider();
            yarp::os::Bottle addDataStreamProvider();
            yarp::os::Bottle removeImgStreamProvider();
            yarp::os::Bottle removeDataStreamProvider();
        };
    }
}//Namespace
#endif

/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau
 * email:   gregoire.pointeau@inserm.fr
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

#include "wrdac/clients/icubClient.h"

// STD
#include <vector>
#include <string>
#include <sstream>


class bodyReservoir : public yarp::os::RFModule {
private:

    wysiwyd::wrdac::ICubClient  *iCub;

    double      period;

    yarp::os::Port  rpcPort;
    yarp::os::Port  portToDumper;

    yarp::os::Bottle        pointObject(std::string sObject);
    yarp::os::Bottle        waveAtAgent(std::string sAgent);

    std::string sAgentName;

public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    //RPC & scenarios
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
};

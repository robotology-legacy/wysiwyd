#ifndef SUBSYSTEM_AGENTDETECTOR_H
#define SUBSYSTEM_AGENTDETECTOR_H

/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
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

#define SUBSYSTEM_AGENTDETECTOR           "agentDetector"

#include <iostream>
#include "wrdac/subsystems/subSystem.h"

namespace wysiwyd{
namespace wrdac{

/**
        * \ingroup wrdac_clients
        *
        * SubSystem for agentDetector
        */
class SubSystem_agentDetector : public SubSystem
{
protected:
    virtual bool connect();

public:
    yarp::os::Port portRPC;
    SubSystem_agentDetector(const std::string &masterName);

    virtual void Close();

    bool changeDefaultName(std::string new_name);

    void pause();

    void resume();
};
}
}//Namespace

#endif // SUBSYSTEM_AGENTDETECTOR_H

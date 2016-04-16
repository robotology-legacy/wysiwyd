#ifndef SUBSYSTEM_BABBLING_H
#define SUBSYSTEM_BABBLING_H

/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Maxime Petit
 * email:   m.petit@imperial.ac.uk
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

#define SUBSYSTEM_BABBLING           "babbling"

#include <iostream>
#include "wrdac/subsystems/subSystem.h"

namespace wysiwyd{
namespace wrdac{

/**
        * \ingroup wrdac_clients
        *
        * SubSystem for babbling
        */
class SubSystem_babbling : public SubSystem
{
protected:
    virtual bool connect();

public:
    yarp::os::Port portRPC;
    SubSystem_babbling(const std::string &masterName);

    virtual void Close();

    //whole arm babbling
    bool babblingArm(std::string babblingLimb, double train_duration = -1.0);

    //single joint babbling
    bool babbling(int jointNumber, std::string babblingLimb, double train_duration = -1.0);

};
}
}//Namespace

#endif // SUBSYSTEM_BABBLING_H

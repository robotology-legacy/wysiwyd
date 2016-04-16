/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
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

#ifndef SUBSYSTEM_IOL2OPC_H
#define SUBSYSTEM_IOL2OPC_H

#define SUBSYSTEM_IOL2OPC           "iol2opc"

#include <iostream>
#include "wrdac/subsystems/subSystem.h"

namespace wysiwyd{
namespace wrdac{

/**
        * \ingroup wrdac_clients
        *
        * SubSystem for iol2opc
        */
class SubSystem_IOL2OPC : public SubSystem
{
protected:
    virtual bool connect();

public:
    yarp::os::Port portRPC;
    SubSystem_IOL2OPC(const std::string &masterName);

    virtual void Close();

    bool changeName(const std::string &old_name, const std::string &new_name);

    void pause();

    void resume();
};
}
}//Namespace

#endif // SUBSYSTEM_IOL2OPC_H

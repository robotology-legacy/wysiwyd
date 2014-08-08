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

#ifndef __EFAA_SUBSYSTEM_ATTENTION_H__
#define __EFAA_SUBSYSTEM_ATTENTION_H__


#define SUBSYSTEM_ATTENTION     "attention"

#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/knowledge/object.h"

namespace wysiwyd{namespace wrdac{

/**
* \ingroup wrdac_clients
*
* SubSystem for attention and gaze (see attentionSelector module)
*/
class SubSystem_Attention: public SubSystem
{
protected:
    virtual bool connect() { return yarp::os::Network::connect(attentionSelector.getName(), "/attentionSelector/rpc"); }

public:
    yarp::os::Port attentionSelector;
    SubSystem_Attention(const std::string &masterName) : SubSystem(masterName)
    {
        attentionSelector.open(("/"+m_masterName+"/attention:rpc").c_str());
        m_type = SUBSYSTEM_ATTENTION;
    }
    virtual void Close() {attentionSelector.interrupt();attentionSelector.close(); }
};


}}//Namespace
#endif



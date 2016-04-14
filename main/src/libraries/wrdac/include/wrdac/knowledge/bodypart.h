/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
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

#ifndef BODYPART_H
#define BODYPART_H

#include <yarp/sig/Vector.h>
#include "object.h"

namespace wysiwyd{
namespace wrdac{

/**
* \ingroup wrdac_representations
*
* Represents a body part of the robot
*/
class Bodypart: public Object
{
    friend class OPCClient;
public:
    Bodypart();
    Bodypart(const Bodypart &b);
    /**
    * Joint number
    */
    int m_joint_number;

    /**
    * Tactile number
    */
    int m_tactile_number;

    /**
    * The part itself, e.g. left_arm, right_arm, ...
    */
    std::string m_part;

    virtual bool isType(std::string _entityType)
    {
        if (_entityType == "bodypart")
            return true;
        else
            return this->Object::isType(_entityType);
    }
    virtual yarp::os::Bottle asBottle();
    virtual bool             fromBottle(const yarp::os::Bottle &b);
    virtual std::string      toString();
};

} //namespace wrdac
} //namespace wysiwyd

#endif // BODYPART_H

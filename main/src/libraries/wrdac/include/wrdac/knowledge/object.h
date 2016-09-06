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

#ifndef __EFAA_OBJECT_H__
#define __EFAA_OBJECT_H__

#include "entity.h"

namespace wysiwyd{
namespace wrdac{

enum class ObjectArea : int {HUMAN = 1, ROBOT = 2, SHARED = 3, NOTREACHABLE = 4};

/**
* \ingroup wrdac_representations
*
* Represent any physical entity (including objects and agents) that can be stored within the OPC.
*/
class Object: public Entity
{
    friend class OPCClient;

public:
    Object();
    Object(const Object &b);

    /**
    * Position of the Object, in the initial ego-centered reference frame of the agent mainting the OPC (initial root of the iCub when navigation is calibrated).
    */
    yarp::sig::Vector m_ego_position;

    /**
    * Orientation of the Object, in the initial ego-centered reference frame of the agent mainting the OPC (initial root of the iCub when navigation is calibrated).
    */
    yarp::sig::Vector m_ego_orientation;

    /**
    * Dimensions of the Object, in meters.
    */
    yarp::sig::Vector m_dimensions;

    /**
    * Mean color of the object (r,g,b) used mainly for debugging/displaying purposes.
    */
    yarp::sig::Vector m_color;

    /**
    * Is the object present in the scene
    */
    double m_present;

    /**
    * A measurement of the object saliency [0,1]
    */
    double m_saliency;

    /**
    * Whether the object is accessible by only the robot, only the human, both or neither agent
    */
    ObjectArea m_objectarea;

    virtual bool    isType(std::string _entityType)
    {
        if (_entityType == EFAA_OPC_ENTITY_OBJECT) {
            return true;
        } else {
            return this->Entity::isType(_entityType);
        }
    }

    virtual yarp::os::Bottle asBottle();
    virtual bool             fromBottle(const yarp::os::Bottle &b);
    virtual std::string      toString();

    /**
    * Express a point given in the initial ego-centered reference frame in respect to the object reference frame.
    *@param vInitialRoot the point to transform
    *@return The transformed vector (x y z)
    */
    yarp::sig::Vector getSelfRelativePosition(const yarp::sig::Vector &vInitialRoot);
};

}} //namespaces

#endif


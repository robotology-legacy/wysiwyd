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

#ifndef __EFAA_RTOBJECT_H__
#define __EFAA_RTOBJECT_H__

#include "object.h"

namespace wysiwyd{namespace wrdac{

/**
* \ingroup wrdac_representations
*
* Represent objects that are perceived by the Reactable.
*/
class RTObject: public Object
{friend class OPCClient;
public:
                       RTObject();   
                       RTObject(const RTObject &b);
    /**
    * Position of the Object, in the Reactable referential.
    */
    yarp::sig::Vector  m_rt_position;

    virtual bool    isType(std::string _entityType)
		{
			if (_entityType == EFAA_OPC_ENTITY_RTOBJECT)
				return true;
			else
				return this->Object::isType(_entityType);
		}   
    virtual yarp::os::Bottle asBottle();
    virtual bool             fromBottle(yarp::os::Bottle b);
    virtual std::string      toString();
};

}} //namespaces

#endif


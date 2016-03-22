/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Lallée Stéphane
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

#ifndef __EFAA_ADJECTIVE_H__
#define __EFAA_ADJECTIVE_H__

#include "entity.h"

namespace wysiwyd{namespace wrdac{
    
    /**
    * \ingroup wrdac_representations
    *
    * Represent a quality (color, size, etc...)
    */
    class Adjective:public Entity
    {friend class OPCClient;
    public:
                        Adjective();
                        Adjective(const Adjective &b);
                std::string  m_quality;
                        
        virtual bool    isType(std::string _entityType)
        {
            if (_entityType == EFAA_OPC_ENTITY_ADJECTIVE)
                return true;
            else
                return this->Entity::isType(_entityType);
        }

        virtual yarp::os::Bottle asBottle();
        virtual bool             fromBottle(const yarp::os::Bottle &b);
        virtual std::string      toString();
    };

}}//Namespaces

#endif


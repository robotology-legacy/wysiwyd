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

#ifndef __EFAA_RELATION_H__
#define __EFAA_RELATION_H__

#include "entity.h"

namespace wysiwyd{namespace wrdac{
    
    /**
    * \ingroup wrdac_representations
    *
    * Represent a relation between two entities.
    */
    class Relation
    {friend class OPCClient;

    private:
       int   m_opcId;

    public:
        std::string  m_verb;
        std::string  m_subject;
        std::string  m_object;
        std::string  m_complement_place;
        std::string  m_complement_time;
        std::string  m_complement_manner;

        Relation(){}
        Relation(yarp::os::Bottle &b);
        Relation(
            std::string subject, 
            std::string verb, 
            std::string object = "none", 
            std::string complement_place = "none",
            std::string complement_time = "none",
            std::string complement_manner = "none");

        Relation(
            Entity* subject, 
            Entity* verb,
            Entity* object = NULL, 
            Entity* complement_place = NULL,
            Entity* complement_time = NULL,
            Entity* complement_manner = NULL
            );

        yarp::os::Bottle    asBottle(bool ignoreID = false);
		/**
		* Return the relation as a bottle without the argument to "none"
		*/
		yarp::os::Bottle	asLightBottle(bool ignoreID = false);
        void                fromBottle(yarp::os::Bottle b);
        virtual std::string toString();

        /**
        * Get the opc ID of the relation.
        */
        int        ID();

        /**
        * Get the name of the relation's subject.
        */
        std::string        subject();
        
        /**
        * Get the name of the relation's object.
        */
        std::string        object();

        /**
        * Get the type of the relation.
        */
        std::string        verb();

        /**
        * Get the complement of place of the relation.
        */
        std::string        complement_place();
        
        /**
        * Get the complement of time of the relation.
        */
        std::string        complement_time();
        
        /**
        * Get the complement of manner of the relation.
        */
        std::string        complement_manner();
        
        bool                operator==(const Relation &b);
        bool                operator<(const Relation &b);
        bool                operator>(const Relation &b);
    };

}}; //namespaces

#endif


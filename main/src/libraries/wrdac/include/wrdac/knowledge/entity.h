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

#ifndef __EFAA_ENTITY_H__
#define __EFAA_ENTITY_H__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <string>
#include <map>
#include <list>
#include <iostream>
#include <sstream>
#include <wrdac/tags.h>

namespace wysiwyd{namespace wrdac{

    //Anticipated declaration of relations (they are used by agents)
    class Relation;

    /**
    * \ingroup wrdac_representations
    *
    * Represent any entity that can be stored within the OPC.
    * All representations derive from entities.
    */
    class Entity
    {friend class OPCClient;
        
    


    //Writing access to name/id is granted only to OPCClient
    private:
        int                 m_opc_id;
        std::string         m_name;
        std::map<std::string, std::string > m_properties;

    //The entity status when the class is updated by the OPCClient
    yarp::os::Bottle        m_original_entity;
    yarp::os::Bottle        asBottleOnlyModifiedProperties();

    protected:
        std::string         m_entity_type;

    public:
                            Entity();
                            Entity(const Entity &b);
                            Entity(yarp::os::Bottle &b);
        


        /**
        * Test if an entity is inheriting a given type
        */ 
        virtual bool    isType(std::string _entityType)
        {
            return false;
        }

        /**
        * Return the entity as a bottle.
        */ 
        virtual yarp::os::Bottle asBottle();

        /**
        * Fill entity fields from a bottle representation
        * @param b a pointer to the bottle containing the entity representation
        */
        virtual bool        fromBottle(yarp::os::Bottle b);

        /**
        * Return a human readable description of the entity
        */
        virtual std::string      toString();


        //Getters
        /**
        * Return the name of an entity (which has to be unique within the OPC)
        */
        std::string             name() const {return m_name.c_str();}

        void                    changeName(std::string sName) { m_name = sName; }
        
        /**
        * Return the id of an entity (which has to be unique within the OPC)
        */
        int                 opc_id() const {return m_opc_id;}

        /**
        * Returns the name of an entity contained in a property, returns the string NULL if property is not set
        */
        std::string         properties(std::string p){ if (m_properties.find(p)!=m_properties.end()) return m_properties[p]; else return "NULL";}

        /**
        * Return the specific type of an entity (object, agent, rt_object, adjective).
        */
        std::string         entity_type(){return m_entity_type;}


        bool                operator==(const Entity &b);
        bool                operator<(const Entity &b);
        bool                operator>(const Entity &b);
    };

}} //namespaces

#endif



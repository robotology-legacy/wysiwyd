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

#ifndef __EFAA_AGENT_H__
#define __EFAA_AGENT_H__

#include "object.h"
#include "relation.h"

namespace wysiwyd{namespace wrdac{
    
/**
* \ingroup wrdac_representations
*
* Represent a drive with a name, an actual value and 2 borns representing the homeostasis range.
*
*/
struct Drive
{
    std::string name;
    double value, homeoStasisMin, homeoStasisMax, decay;

    Drive(std::string d_name, double d_value, double d_homeo_min, double d_homeo_max, double d_decay = 0.05)
    {

        //todo : check the min/max
        name = d_name;
        value = d_value;
        homeoStasisMin = d_homeo_min;
        homeoStasisMax = d_homeo_max;
		decay = d_decay;
    }

    Drive()
    {
        name = "defaultDrive";
        value = 0.5;
        homeoStasisMin = 0.0;
        homeoStasisMax = 1.0;
		decay = 0.05;
    }

    Drive(const Drive &b)
    {
        name = b.name;
        value = b.value;
        homeoStasisMin = b.homeoStasisMin;
        homeoStasisMax = b.homeoStasisMax;
		decay = b.decay;
    }

    yarp::os::Bottle asBottle()
    {
        yarp::os::Bottle b;
        yarp::os::Bottle sub;
        sub.addString("name");
        sub.addString(name.c_str());
        b.addList() = sub;
        sub.clear();
        sub.addString("hStMin");
        sub.addDouble(homeoStasisMin);
        b.addList() = sub;
        sub.clear();
        sub.addString("hStMax");
        sub.addDouble(homeoStasisMax);
        b.addList() = sub;
        sub.clear();
        sub.addString("value");
        sub.addDouble(value);
        b.addList() = sub;
        sub.clear();
        sub.addString("decay");
        sub.addDouble(decay);
        b.addList() = sub;
        return b;
    }

    bool fromBottle(yarp::os::Bottle b)
    {
        name = b.find("name").asString().c_str();
        value = b.find("value").asDouble();
        homeoStasisMin = b.find("hStMin").asDouble();
        homeoStasisMax = b.find("hStMax").asDouble();
        decay = b.find("decay").asDouble();
          return true;
    }
};

/**
* \ingroup wrdac_representations
*
* Represent a the body of an agent. Joints are stored as a dictionnary of string, position
*
*/
struct Body
{
    std::map<std::string, yarp::sig::Vector> m_parts;

    Body()
    {
            m_parts[EFAA_OPC_BODY_PART_TYPE_HEAD].resize(3,0.0);
            m_parts[EFAA_OPC_BODY_PART_TYPE_SHOULDER_C].resize(3,0.0);
            m_parts[EFAA_OPC_BODY_PART_TYPE_SHOULDER_L].resize(3,0.0);
            m_parts[EFAA_OPC_BODY_PART_TYPE_SHOULDER_R].resize(3,0.0);
            m_parts[EFAA_OPC_BODY_PART_TYPE_SPINE].resize(3,0.0);
            m_parts[EFAA_OPC_BODY_PART_TYPE_HAND_R].resize(3,0.0);
            m_parts[EFAA_OPC_BODY_PART_TYPE_HAND_L].resize(3,0.0);
            m_parts[EFAA_OPC_BODY_PART_TYPE_ELBOW_R].resize(3,0.0);
            m_parts[EFAA_OPC_BODY_PART_TYPE_ELBOW_L].resize(3,0.0);
			//m_parts[EFAA_OPC_BODY_PART_TYPE_LEFT_HIP].resize(3,0.0);
			//m_parts[EFAA_OPC_BODY_PART_TYPE_LEFT_KNEE].resize(3,0.0);
			//m_parts[EFAA_OPC_BODY_PART_TYPE_LEFT_FOOT].resize(3,0.0);
			//m_parts[EFAA_OPC_BODY_PART_TYPE_RIGHT_HIP].resize(3,0.0);
			//m_parts[EFAA_OPC_BODY_PART_TYPE_RIGHT_KNEE].resize(3,0.0);
			//m_parts[EFAA_OPC_BODY_PART_TYPE_RIGHT_FOOT].resize(3,0.0);
    }

    yarp::os::Bottle asBottle()
        {
            yarp::os::Bottle b;
            for(std::map<std::string,yarp::sig::Vector>::iterator part = m_parts.begin(); part != m_parts.end(); part++)
            {
                yarp::os::Bottle sub;
                sub.addString(part->first.c_str());
                yarp::os::Bottle position;
                position.addDouble(part->second[0]);
                position.addDouble(part->second[1]);
                position.addDouble(part->second[2]);
                sub.addList() = position;
                b.addList()= sub;
            }
            return b;
        }

        bool fromBottle(yarp::os::Bottle b)
        {
            for(std::map<std::string,yarp::sig::Vector>::iterator part = m_parts.begin(); part != m_parts.end(); part++)
            {
                yarp::os::Bottle* position = b.find(part->first.c_str()).asList();
                part->second[0] = position->get(0).asDouble();
                part->second[1] = position->get(1).asDouble();
                part->second[2] = position->get(2).asDouble();
            }
            return true;
        }
};

/**
* \ingroup wrdac_representations
*
* Represent an agent.
* An agent is a Object which possesses emotions and beliefs.
* Beliefs are encoded as a list of relations.
*/
class Agent: public Object
{friend class OPCClient;
private:
    std::list<Relation>      m_belief;
public:
                        Agent();
                        Agent(const Agent &b);

    std::map<std::string, double> m_emotions_intrinsic;
    std::map<std::string, Drive>  m_drives;
    Body                m_body;
            
    virtual bool    isType(std::string _entityType)
        {
			if (_entityType == EFAA_OPC_ENTITY_AGENT)
				return true;
			else
				return this->Object::isType(_entityType);
		}

    virtual yarp::os::Bottle asBottle();
    virtual bool             fromBottle(yarp::os::Bottle b);
    virtual std::string      toString();

    /**
    * Add the belief of a relation to the agent.
    * @param r a relation that the agent will believe to be true
    */
    bool                addBelief(Relation r);

    /**
    * Remove the belief of a relation from the agent.
    * @param r a relation that the agent will believe not to be true
    */
    bool                removeBelief(Relation r);
    
    /**
    * Check if some relation is believed by an agent.
    * @param r a relation to check
    */
    bool                checkBelief(Relation r);
        
    /**
    * Get a read-only copy of the agent believes.
    */
    const std::list<Relation> &beliefs();

    bool                operator==(const Agent &b);
};

}} //namespaces

#endif



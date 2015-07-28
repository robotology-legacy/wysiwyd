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

#include <wrdac/knowledge/entity.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;


Entity::Entity(Bottle &b)
{
    this->fromBottle(b);
}

Entity::Entity()
{
    this->m_opc_id = -1;
    this->m_entity_type = "unknown";
    this->m_name = "";
}

Entity::Entity(const Entity &b)
{
	this->m_name = b.m_name;
	this->m_entity_type = b.m_entity_type;
	this->m_opc_id = b.m_opc_id;
}

Bottle Entity::asBottle()
{
    Bottle b,bId;
 /*   bId.addString("id");
    bId.addInt(this->m_opc_id);
	b.addList() = bId;*/

    Bottle bName;
    bName.addString("name");
    bName.addString(m_name.c_str());
    b.addList() = bName;

    Bottle bEntity;
    bEntity.addString("entity");
    bEntity.addString(m_entity_type.c_str());
    b.addList() = bEntity;
		
	Bottle bProperties;
	bProperties.addString("intrinsic_properties");
	Bottle& bPropertiesGlob = bProperties.addList();
	for(map<string, string>::iterator prop = m_properties.begin(); prop != m_properties.end(); prop++)
	{
		Bottle bPropertiesValues;
		bPropertiesValues.addString(prop->first.c_str());
		bPropertiesValues.addString(prop->second.c_str());
		bPropertiesGlob.addList() = bPropertiesValues;
	}
	b.addList() = bProperties;
    return b;
}

Bottle Entity::asBottleOnlyModifiedProperties()
{
    Bottle current_entity = this->asBottle();

    Bottle new_entity;
    for(int p=0; p<current_entity.size();p++)
    {
        string currentTag = current_entity.get(p).asList()->get(0).asString().c_str();
        Value currentValue = current_entity.find(currentTag.c_str());
        Value originalValue = m_original_entity.find(currentTag.c_str()); 
     
	//std::cout<<"Debug: tag:"<<currentTag<<std::endl;
	//std::cout<<"Debug: cValue:"<<currentValue.toString()<<std::endl;
	//std::cout<<"Debug: oValue:"<<originalValue.toString()<<std::endl;
        if (currentValue.toString() != originalValue.toString())
        {
            Bottle bSub;
            bSub.addString(currentTag.c_str());
            bSub.add(currentValue);

            new_entity.addList()=bSub;
        }
    }
    return new_entity;
}

bool Entity::fromBottle(Bottle b)
{
    if (!b.check("name") || !b.check("entity"))
        return false;
	std::string test = b.toString().c_str();

//	m_opc_id = b->find("id").asInt();
    m_name = b.find("name").asString().c_str();
    m_entity_type = b.find("entity").asString().c_str();
		
	Bottle* bProperties = b.find("intrinsic_properties").asList();
	
	for(int i=0; i< bProperties->size() ; i++)
	{
		std::string pTag = bProperties->get(i).asList()->get(0).asString().c_str();
		std::string pVal = bProperties->get(i).asList()->get(1).asString().c_str();
		m_properties[pTag] = pVal;
	}

    return true;
}

string Entity::toString()
{
    std::ostringstream oss;
    oss << "**************"<<endl;
    oss <<"Name : " <<    m_name<<endl;
    oss <<"ID : "    <<    m_opc_id<<endl;
    oss <<"Type : "    <<    m_entity_type<<endl;
    oss <<"Intrinsic properties: "<<endl;
	for(map<string, string>::iterator prop = m_properties.begin(); prop != m_properties.end(); prop++)
	{
		oss<<'\t'<<prop->first.c_str()<<" : "<<prop->second.c_str()<<endl;

	}
    return oss.str();
}


bool Entity::operator==(const Entity &b)
{
	return (this->m_opc_id == b.m_opc_id);
}


bool Entity::operator<(const Entity &b)
{
	return (this->m_opc_id < b.m_opc_id );
}


bool Entity::operator>(const Entity &b)
{
	return (this->m_opc_id > b.m_opc_id );
}


void Entity::changeName(std::string sName)
{
    m_name = sName;
}

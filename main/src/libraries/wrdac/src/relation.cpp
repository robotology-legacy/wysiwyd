/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com * A copy of the license can be found at
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


#include <wrdac/knowledge/relation.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

Relation::Relation(
            string subject, 
            string verb, 
            string object, 
            string complement_place,
            string complement_time,
            string complement_manner)
{   
    m_opcId = -1;
     m_subject = subject;
     m_verb = verb;
     m_object = object;
    m_complement_place = complement_place;
    m_complement_time = complement_time;
    m_complement_manner = complement_manner;
}

Relation::Relation(
            Entity* subject, 
            Entity* verb,
            Entity* object, 
            Entity* complement_place,
            Entity* complement_time,
            Entity* complement_manner
            )
{
    m_opcId = -1;
	if (subject)
		m_subject = subject->name();
	else
		m_subject = "none";

	if (verb)
		m_verb = verb->name();
	else
		m_verb = "none";

    if (object)
        m_object = object->name();
    else
        m_object = "none";

    if (complement_place)
        m_complement_place = complement_place->name();
    else
        m_complement_place = "none";

    if (complement_time)
        m_complement_time = complement_time->name();
    else
        m_complement_time = "none";

    if (complement_manner)
        m_complement_manner = complement_manner->name();
    else
        m_complement_manner = "none";
}

Relation::Relation(Bottle &b)
{
    this->fromBottle(b);
}

Bottle Relation::asBottle(bool ignoreID)
{
    Bottle b;
    Bottle bSub;
    bSub.addString("entity");
    bSub.addString(EFAA_OPC_ENTITY_RELATION);
    b.addList()=bSub;

	if (!ignoreID)
	{
		bSub.clear();
		bSub.addString("id");
		bSub.addInt(m_opcId);
		b.addList()=bSub;
	}
    bSub.clear();
    bSub.addString("rSubject");
    bSub.addString(m_subject.c_str());
    b.addList()=bSub;
    bSub.clear();
    bSub.addString("rVerb");
    bSub.addString(m_verb.c_str());
    b.addList()=bSub;
    bSub.clear();
    bSub.addString("rObject");
    bSub.addString(m_object.c_str());
    b.addList()=bSub;
    bSub.clear();
    bSub.addString("rCompPlace");
    bSub.addString(m_complement_place.c_str());
    b.addList()=bSub;
    bSub.clear();
    bSub.addString("rCompTime");
    bSub.addString(m_complement_time.c_str());
    b.addList()=bSub;
    bSub.clear();
    bSub.addString("rCompManner");
    bSub.addString(m_complement_manner.c_str());
    b.addList()=bSub;
    return b;
}

Bottle Relation::asLightBottle(bool ignoreID)
{
    Bottle b;
    Bottle bSub;
    bSub.addString("entity");
    bSub.addString(EFAA_OPC_ENTITY_RELATION);
    b.addList()=bSub;

    bSub.clear();
    bSub.addString("rSubject");
    bSub.addString(m_subject.c_str());
    b.addList()=bSub;
    bSub.clear();
    bSub.addString("rVerb");
    bSub.addString(m_verb.c_str());
    b.addList()=bSub;
    bSub.clear();
    bSub.addString("rObject");
	bSub.addString(m_object.c_str());
	b.addList()=bSub;
	if (m_complement_place != "none")
	{
		bSub.clear();
		bSub.addString("rCompPlace");
		bSub.addString(m_complement_place.c_str());
		b.addList()=bSub;
	}
	if (m_complement_time != "none")
	{
		bSub.clear();
		bSub.addString("rCompTime");
		bSub.addString(m_complement_time.c_str());
		b.addList()=bSub;
	}
	if (m_complement_manner != "none")
	{
		bSub.clear();
		bSub.addString("rCompManner");
		bSub.addString(m_complement_manner.c_str());
		b.addList()=bSub;
	}
    return b;
}




void Relation::fromBottle(Bottle b)
{        
    m_opcId = b.find("id").asInt();
    m_subject = b.find("rSubject").toString().c_str();
    m_verb = b.find("rVerb").asString().c_str();
    m_object = b.find("rObject").asString().c_str();
    m_complement_place = b.find("rCompPlace").asString().c_str();
    m_complement_time = b.find("rCompTime").asString().c_str();
    m_complement_manner = b.find("rCompManner").asString().c_str();
}

string Relation::toString()
{    
    std::ostringstream oss;
    oss<< subject() << " " <<verb()<<" ";
    if (object() != "none")
        oss<<object()<<" ";
    if (complement_place() != "none")
        oss<<complement_place()<<" ";
    if (complement_time() != "none")
        oss<<complement_time()<<" ";
    if (complement_manner() != "none")
        oss<<complement_manner()<<" ";
    //oss<<" (ID="<<m_opcId<<")"<<endl;
    return oss.str();
}

int Relation::ID()
{    
    return m_opcId;
}

string Relation::subject() const
{    
    return m_subject;
}

string Relation::object() const
{    
    return m_object;
}

string Relation::verb() const
{    
    return m_verb;
}

string Relation::complement_place() const
{    
    return m_complement_place;
}

string Relation::complement_time() const
{    
    return m_complement_time;
}

string Relation::complement_manner() const
{    
    return m_complement_manner;
}

bool Relation::operator>(const Relation &b)
{
    if (this->m_subject != b.m_subject)
    {
        return (this->m_subject >  b.m_subject);
    }   
    else if (this->m_opcId != b.m_opcId)
    {
        return (this->m_opcId > b.m_opcId);
    }
    else if (this->m_verb != b.m_verb)
    {
        return (this->m_verb > b.m_verb);
    }
    else if (this->m_object != b.m_object)
    {
        return (this->m_object > b.m_object);
    }
    else if (this->m_complement_place != b.m_complement_place)
    {
        return (this->m_complement_place > b.m_complement_place);
    }
    else if (this->m_complement_time != b.m_complement_time)
    {
        return (this->m_complement_time > b.m_complement_time);
    }
    else if (this->m_complement_manner != b.m_complement_manner)
    {
        return (this->m_complement_manner > b.m_complement_manner);
    }
    return false;

}

bool Relation::operator<(const Relation &b)
{
    return (!(this->operator>(b))&&!(this->operator==(b)));
}

bool Relation::operator==(const Relation &b)
{
    return ( 
        this->m_opcId == b.m_opcId &&
        this->m_subject == b.m_subject &&
        this->m_verb == b.m_verb &&
        this->m_object == b.m_object &&
        this->m_complement_place == b.m_complement_place &&
        this->m_complement_time == b.m_complement_time &&
        this->m_complement_manner == b.m_complement_manner
        );
}




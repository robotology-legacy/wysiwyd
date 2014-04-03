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


#include <knowledge/agent.h>
#include <algorithm>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

Agent::Agent():Object()
{    
    m_entity_type = EFAA_OPC_ENTITY_AGENT;
}

Agent::Agent(const Agent &b):Object(b)
{
	this->m_belief = b.m_belief;
	this->m_emotions_intrinsic = b.m_emotions_intrinsic;
	this->m_drives= b.m_drives;
	this->m_body = b.m_body;
}

Bottle Agent::asBottle()
{
    //Get the Object bottle
    Bottle b = this->Object::asBottle();
    Bottle bSub;
    bSub.addString("belief");
    Bottle& bSubIds = bSub.addList();
    for(list<Relation>::iterator it = m_belief.begin(); it != m_belief.end(); it++)
    {
        Bottle& bSub2 = bSubIds.addList();
        bSub2 = it->asBottle();
    }
    b.addList() = bSub;

	Bottle bSubEmotions;
	bSubEmotions.addString("emotions");
	Bottle& bSubEmoList = bSubEmotions.addList();
    for(map<string, double>::iterator it = m_emotions_intrinsic.begin(); it != m_emotions_intrinsic.end(); it++)
    {
		Bottle& bSubEmo = bSubEmoList.addList();
		bSubEmo.addString(it->first.c_str());
		bSubEmo.addDouble(it->second);
    }
	b.addList() = bSubEmotions;

	Bottle bSubDrives,bSubSub;
	bSubDrives.addString("drives");
	for(map<string, Drive>::iterator it = m_drives.begin(); it != m_drives.end(); it++)
	{
		bSubSub.addList() = it->second.asBottle();
	}
	bSubDrives.addList() = bSubSub;
	b.addList() = bSubDrives;
	          
	Bottle bSubBody;
    bSubBody.addString("body");
    bSubBody.addList().copy(m_body.asBottle());
    b.addList() = bSubBody;  

    return b;
}

bool Agent::fromBottle(Bottle b)
{
    if (!this->Object::fromBottle(b))
        return false;

    if (!b.check("belief")||!b.check("emotions"))
        return false;

    m_belief.clear();
    Bottle* beliefs = b.find("belief").asList();
    for(int i=0; i<beliefs->size() ; i++)
    {
        Bottle* bRelation = beliefs->get(i).asList();
        Relation r(*bRelation);
        m_belief.push_back(r);
    }

	m_emotions_intrinsic.clear();
    Bottle* emotions = b.find("emotions").asList();
    for(int i=0; i<emotions->size() ; i++)
    {
        Bottle* bEmo = emotions->get(i).asList();
		string emotionName = bEmo->get(0).asString().c_str();
		double emotionValue = bEmo->get(1).asDouble();
		m_emotions_intrinsic[emotionName.c_str()] = emotionValue;
    }

	m_drives.clear();
    Bottle* drivesProperty = b.find("drives").asList();
	string drivesDebug = drivesProperty->toString().c_str();
    for(int i=0; i<drivesProperty->size() ; i++)
    {
			Bottle* bD = drivesProperty->get(i).asList();
			string drivesDebug1 = bD->toString().c_str();
			Drive currentDrive;
			currentDrive.fromBottle(*bD);
			m_drives[currentDrive.name] = currentDrive;

    }
	           
	Bottle* bodyProperty = b.find("body").asList();
    m_body.fromBottle(*bodyProperty);

    return true;
}

string Agent::toString()
{    
    std::ostringstream oss;
    oss<< this->Object::toString();

    oss<<"Believes that : \n";
    for(list<Relation>::iterator it = m_belief.begin(); it != m_belief.end(); it++)
    {
        oss<< it->toString() <<endl;
    }

	oss<<"Emotions:"<<endl;
	   for(map<string,double>::iterator it = m_emotions_intrinsic.begin(); it != m_emotions_intrinsic.end(); it++)
    {
        oss<< '\t'<<it->first <<" : "<<it->second<<endl;
    }

	oss<<"Drives:"<<endl;
	for(map<string,Drive>::iterator it = m_drives.begin(); it != m_drives.end(); it++)
    {
        oss<< '\t'<<it->first <<" : "<<it->second.name<<"("<<it->second.value<<")"<<endl;
    }
    return oss.str();
}

bool Agent::addBelief(Relation r)
{
	//Check if this relation is already present
	list<Relation>::iterator it = find(m_belief.begin(),m_belief.end(),r);
	if (it != m_belief.end())
	{
		//cout<<"Agent "+name()+" already believes that " + r.toString();
		return false;
	}

	m_belief.push_back(r);
	//cout<<"Agent "+name()+" now believes that " + r.toString();
	return true;
}

bool Agent::removeBelief(Relation r)
{
	//Check if this relation is already present
	list<Relation>::iterator it = find(m_belief.begin(),m_belief.end(),r);
	if (it != m_belief.end())
	{
		m_belief.remove(r);
		//cout<<"Agent "+name()+" do not believes anymore that " + r.toString();
		return true;
	}
	//cout<<"Agent "+name()+" didn not believe that " + r.toString();
	return false;
}

bool Agent::checkBelief(Relation r)
{
	return (find(m_belief.begin(),m_belief.end(),r) != m_belief.end());
}

list<Relation> Agent::beliefs()
{
	return m_belief;
}




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


#include <wrdac/knowledge/adjective.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

Adjective::Adjective():Entity()
{
    m_entity_type = EFAA_OPC_ENTITY_ADJECTIVE;
}


Adjective::Adjective(const Adjective &b):Entity(b)
{
	this->m_entity_type = EFAA_OPC_ENTITY_ADJECTIVE;
	this->m_quality = b.m_quality;
}
        
Bottle Adjective::asBottle()
{
    //Get the Object bottle
    Bottle b = this->Entity::asBottle();
    Bottle bSub;
    bSub.addString("qualityType");
    bSub.addString(m_quality.c_str());
    b.addList() = bSub;
    return b;
}

bool Adjective::fromBottle(Bottle b)
{
    if (!this->Entity::fromBottle(b))
        return false;

    if (!b.check("qualityType"))
        return false;

    m_quality = b.find("qualityType").asString().c_str();
    return true;
}

string Adjective::toString()
{    
    std::ostringstream oss;
    oss<< this->Entity::toString();
    oss <<"Qualifs a " << m_quality<<endl;
    return oss.str();
}



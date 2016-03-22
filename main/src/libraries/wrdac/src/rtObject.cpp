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


#include <wrdac/knowledge/rtObject.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

RTObject::RTObject():Object()
{    
    m_entity_type = EFAA_OPC_ENTITY_RTOBJECT;
    m_rt_position.resize(3,0.0);
    m_present = 1.0;
}


RTObject::RTObject(const RTObject &b):Object(b)
{
    this->m_entity_type = b.m_entity_type;
    this->m_rt_position = b.m_rt_position;
    this->m_present = 1.0;
}

Bottle RTObject::asBottle()
{
    //Get the Object bottle
    Bottle b = this->Object::asBottle();
    
    Bottle bSub;
    bSub.addString("rt_position_x");
    bSub.addDouble(m_rt_position[0]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString("rt_position_y");
    bSub.addDouble(m_rt_position[1]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString("rt_position_z");
    bSub.addDouble(m_rt_position[2]);
    b.addList() = bSub;
    return b;
}

string RTObject::toString()
{    
    std::ostringstream oss;
    oss<< this->Object::toString();
    oss<<"rt position : \t\t";
    oss<< m_rt_position.toString(3,3)<<endl;
    return oss.str();
}

bool RTObject::fromBottle(const Bottle &b)
{
    if (!this->Object::fromBottle(b))
        return false;

    if (!b.check("rt_position_x") || 
        !b.check("rt_position_y") ||
        !b.check("rt_position_z"))
    {
        return false;
    }

    m_rt_position[0] = b.find("rt_position_x").asDouble();
    m_rt_position[1] = b.find("rt_position_y").asDouble();
    m_rt_position[2] = b.find("rt_position_z").asDouble();
    return true;
}



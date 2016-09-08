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


#include <wrdac/knowledge/object.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

Object::Object():Entity()
{
    m_entity_type = EFAA_OPC_ENTITY_OBJECT;
    m_ego_position.resize(3,0.0);
    m_ego_orientation.resize(3,0.0);
    m_dimensions.resize(3,0.1);
    m_color.resize(3,255.0);
    m_saliency = 0.0;
    m_value = 0.0;
    m_present = 0.0;
    m_objectarea = ObjectArea::NOTREACHABLE;

    // by default, place object 25cm in front of the robot
    // to avoid collisions when pointing to an object
    m_ego_position[0] = -0.25;
}

Object::Object(const Object &b):Entity(b)
{
    this->m_entity_type = b.m_entity_type;
    this->m_color = b.m_color;
    this->m_dimensions = b.m_dimensions;
    this->m_ego_orientation = b.m_ego_orientation;
    this->m_ego_position = b.m_ego_position;
    this->m_present = b.m_present;
    this->m_saliency = b.m_saliency;
    this->m_objectarea = b.m_objectarea;
    this->m_value = b.m_value;
}

Bottle Object::asBottle()
{
    //Get the entity bottle
    Bottle b = this->Entity::asBottle();

    Bottle bSub;
    //Add Object specific properties
    //Position
    bSub.addString(EFAA_OPC_OBJECT_ROBOTPOSX_TAG);
    bSub.addDouble(m_ego_position[0]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_ROBOTPOSY_TAG);
    bSub.addDouble(m_ego_position[1]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_ROBOTPOSZ_TAG);
    bSub.addDouble(m_ego_position[2]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_ROBOTPOS_TAG);
    bSub.addList().read(m_ego_position);
    b.addList() = bSub;
    bSub.clear();

    //Orientation
    bSub.addString(EFAA_OPC_OBJECT_ROBOTORX_TAG);
    bSub.addDouble(m_ego_orientation[0]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_ROBOTORY_TAG);
    bSub.addDouble(m_ego_orientation[1]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_ROBOTORZ_TAG);
    bSub.addDouble(m_ego_orientation[2]);
    b.addList() = bSub;
    bSub.clear();

    //Dimension
    bSub.addString(EFAA_OPC_OBJECT_RTDIMX_TAG);
    bSub.addDouble(m_dimensions[0]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_RTDIMY_TAG);
    bSub.addDouble(m_dimensions[1]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_RTDIMZ_TAG);
    bSub.addDouble(m_dimensions[2]);
    b.addList() = bSub;
    bSub.clear();

    //Color
    bSub.addString(EFAA_OPC_OBJECT_GUI_COLOR_R);
    bSub.addDouble(m_color[0]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_GUI_COLOR_G);
    bSub.addDouble(m_color[1]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_GUI_COLOR_B);
    bSub.addDouble(m_color[2]);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_SALIENCY);
    bSub.addDouble(m_saliency);
    b.addList() = bSub;
    bSub.clear();
    bSub.addString(EFAA_OPC_OBJECT_VALUE);
    bSub.addDouble(m_value);
    b.addList() = bSub;
    bSub.clear();

    //Present
    bSub.addString(EFAA_OPC_OBJECT_PRESENT_TAG);
    bSub.addDouble(m_present);
    b.addList() = bSub;
    bSub.clear();

    //ObjectArea
    bSub.addString("object_area");
    bSub.addInt(static_cast<int>(m_objectarea));
    b.addList() = bSub;
    bSub.clear();

    return b;
}

bool Object::fromBottle(const Bottle &b)
{
    if (!this->Entity::fromBottle(b))
        return false;

    if (!b.check(EFAA_OPC_OBJECT_ROBOTPOSX_TAG) ||
        !b.check(EFAA_OPC_OBJECT_ROBOTPOSY_TAG) ||
        !b.check(EFAA_OPC_OBJECT_ROBOTPOSZ_TAG) ||
        !b.check(EFAA_OPC_OBJECT_RTDIMX_TAG) ||
        !b.check(EFAA_OPC_OBJECT_RTDIMY_TAG) ||
        !b.check(EFAA_OPC_OBJECT_RTDIMZ_TAG) ||
        !b.check(EFAA_OPC_OBJECT_ROBOTORX_TAG) ||
        !b.check(EFAA_OPC_OBJECT_ROBOTORY_TAG) ||
        !b.check(EFAA_OPC_OBJECT_ROBOTORZ_TAG) ||
        !b.check(EFAA_OPC_OBJECT_GUI_COLOR_R) ||
        !b.check(EFAA_OPC_OBJECT_GUI_COLOR_G) ||
        !b.check(EFAA_OPC_OBJECT_GUI_COLOR_B) ||
        !b.check(EFAA_OPC_OBJECT_PRESENT_TAG) ||
        !b.check("object_area")               ||
        !b.check(EFAA_OPC_OBJECT_VALUE))
    {
        return false;
    }

    m_ego_position[0] = b.find(EFAA_OPC_OBJECT_ROBOTPOSX_TAG).asDouble();
    m_ego_position[1] = b.find(EFAA_OPC_OBJECT_ROBOTPOSY_TAG).asDouble();
    m_ego_position[2] = b.find(EFAA_OPC_OBJECT_ROBOTPOSZ_TAG).asDouble();
    m_dimensions[0] = b.find(EFAA_OPC_OBJECT_RTDIMX_TAG).asDouble();
    m_dimensions[1] = b.find(EFAA_OPC_OBJECT_RTDIMY_TAG).asDouble();
    m_dimensions[2] = b.find(EFAA_OPC_OBJECT_RTDIMZ_TAG).asDouble();
    m_ego_orientation[0] = b.find(EFAA_OPC_OBJECT_ROBOTORX_TAG).asDouble();
    m_ego_orientation[1] = b.find(EFAA_OPC_OBJECT_ROBOTORY_TAG).asDouble();
    m_ego_orientation[2] = b.find(EFAA_OPC_OBJECT_ROBOTORZ_TAG).asDouble();
    m_color[0] = b.find(EFAA_OPC_OBJECT_GUI_COLOR_R).asDouble();
    m_color[1] = b.find(EFAA_OPC_OBJECT_GUI_COLOR_G).asDouble();
    m_color[2] = b.find(EFAA_OPC_OBJECT_GUI_COLOR_B).asDouble();
    m_saliency = b.find(EFAA_OPC_OBJECT_SALIENCY).asDouble();
    m_value = b.find(EFAA_OPC_OBJECT_VALUE).asDouble();
    m_present = b.find(EFAA_OPC_OBJECT_PRESENT_TAG).asDouble();
    m_objectarea = static_cast<ObjectArea>(b.find("object_area").asInt());

    return true;
}

string Object::toString()
{
    std::ostringstream oss;
    oss<< this->Entity::toString();
    oss<<"self xyz : \t";
    oss<< m_ego_position.toString(3,3)<<endl;
    oss<<"self rpy: \t";
    oss<< m_ego_orientation.toString(3,3)<<endl;
    oss<<"size : \t \t";
    oss<< m_dimensions.toString(3,3)<<endl;
    oss<<"color : \t";
    oss<< m_color.toString(3,3)<<endl;
    oss<<"saliency : \t";
    oss<< m_saliency<<endl;
    oss<<"value : \t";
    oss<< m_value<<endl;
    oss<<"present : \t";
    oss<< m_present<<endl;
    oss<<"object area : \t";
    oss<< static_cast<int>(m_objectarea)<<endl;
    return oss.str();
}

Vector Object::getSelfRelativePosition(const Vector &vInitialRoot)
{
    Vector targetAbsolute(4,1.0);
    targetAbsolute.setSubvector(0,vInitialRoot);
    Vector targetRelative(4,1.0);

    //Translation
    targetAbsolute[0] = targetAbsolute[0] - this->m_ego_position[0];
    targetAbsolute[1] = targetAbsolute[1] - this->m_ego_position[1];

    //Rotation
    double theta = this->m_ego_orientation[2] * M_PI/180.0;
    targetRelative[0] = targetAbsolute[0] * cos(theta) + targetAbsolute[1]*sin(theta);
    targetRelative[1] = -targetAbsolute[0]*sin(theta) + targetAbsolute[1]*cos(theta);
    targetRelative[2] = targetAbsolute[2];
    return targetRelative;
}

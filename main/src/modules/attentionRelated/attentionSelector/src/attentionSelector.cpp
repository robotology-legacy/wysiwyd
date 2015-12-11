// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Stéphane Lallée, moved from EFAA by Maxime Petit
* email:   stephane.lallee@gmail.com
* website: http://wysiwyd.upf.edu/
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* $WYSIWYD_ROOT/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include "attentionSelector.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

/************************************************************************/
bool attentionSelectorModule::configure(yarp::os::ResourceFinder &rf) {

    moduleName = rf.check("name",
        Value("attentionSelector"),
        "module name (string)").asString();

    setName(moduleName.c_str());

    opcName = rf.check("opcName",
        Value("OPC"),
        "Opc name (string)").asString();

    trackSwitchingPeriod = rf.check("trackSwitchingPeriod",
        Value(1.0)).asDouble();


    opc = new OPCClient(moduleName.c_str());
    if (!opc->connect(opcName)) {
        yError() << getName() << ": Unable to connect to OPC";
        delete opc;
        return false;
    }

    icub_client = new ICubClient(getName(), "icubClient", "client_ARE.ini");
    if (!icub_client->connectSubSystems()) {
        yError() << getName() << ": Unable to connect to ARE";
        delete opc;
        delete icub_client;
        return false;
    }
    are = icub_client->getARE();

    string handlerPortName = "/";
    handlerPortName += getName() + "/rpc";
    handlerPort.open(handlerPortName.c_str());
    attach(handlerPort);

    icub = NULL;
    aState = s_waiting;
    trackedObject = "none";
    x_coord = 0.0;
    y_coord = 0.0;
    z_coord = 0.0;
    trackedCoordinates = false;

    autoSwitch = true;

    if (rf.check("noAutoSwitch"))
    {
        yInfo() << " autoSwitch set to off";
        autoSwitch = false;
    }

    return true;
}

/************************************************************************/
bool attentionSelectorModule::interruptModule() {
    opc->interrupt();
    handlerPort.interrupt();
    return true;
}

/************************************************************************/
bool attentionSelectorModule::close() {
    handlerPort.close();
    icub_client->close();
    opc->close();
    delete icub_client;
    delete opc;
    return true;
}

/************************************************************************/
bool attentionSelectorModule::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "track <string name> : track the object with the given opc name \n" +
        "track <int id> : track the object with the given opc id \n" +
        "track <double x> <double y> <double z> : track with the object coordinates \n" +
        "auto : switch attention between present objects \n" +
        "sleep : pauses the head control until next command \n" +
        "stat : returns \"auto\",\"quiet\",\"<object_name_to_track>\" \n" +
        "help \n" +
        "quit \n";

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "help") {
        yInfo() << helpMessage;
        reply.addString("ack");
        return true;
    }
    else if (command.get(0).asString() == "track") {
        autoSwitch = false;
        if (command.get(1).isInt()) {
            trackedObject = dynamic_cast<Object*>(opc->getEntity(command.get(1).asInt()))->name();
            trackedCoordinates = false;
        }
        else if (command.get(1).isString()) {
            trackedObject = dynamic_cast<Object*>(opc->getEntity(command.get(1).asString().c_str()))->name();
            trackedCoordinates = false;
        }
        else {
            trackedCoordinates = true;

            x_coord = command.get(1).asDouble();
            y_coord = command.get(2).asDouble();
            z_coord = command.get(3).asDouble();
        }
        aState = s_tracking;
        reply.addString("ack");
        return true;
    }
    else if (command.get(0).asString() == "auto") {
        autoSwitch = true;
        reply.addString("ack");
        return true;
    }
    else if (command.get(0).asString() == "sleep") {
        autoSwitch = false;
        trackedObject = "none";
        reply.addString("ack");
        return true;
    }
    else if (command.get(0).asString() == "look") {
        autoSwitch = false;
        trackedObject = "none";
        trackedCoordinates = false;

        Vector xyz(3);
        xyz[0] = command.get(1).asDouble();
        xyz[1] = command.get(2).asDouble();
        xyz[2] = command.get(3).asDouble();

        are->look(xyz);
        reply.addString("ack");
        return true;
    }
    else if (command.get(0).asString() == "stat") {
        reply.addString("ack");
        if (autoSwitch)
            reply.addString("auto");
        else if (trackedObject!="none")
            reply.addString(trackedObject.c_str());
        else
            reply.addString("quiet");
        return true;
    }

    reply.addString("nack");
    return true;
}


/***************************************************************************/
bool attentionSelectorModule::updateModule() {

    if (!opc->isConnected())
        if (!opc->connect("OPC"))
            return true;

    opc->checkout();
    list<Entity*> entities = opc->EntitiesCache();
    presentObjects.clear();
    for (auto& entity : entities)
    {
        if (entity->isType(EFAA_OPC_ENTITY_ACTION)) {
            yDebug() << "Ignoring relation...";
        }
        else
        {
            if (entity->name() == "icub") {
                icub = dynamic_cast<Agent*>(entity);
            } else {
                ////!!! ONLY RT_OBJECT and AGENTS ARE TRACKED !!!
                //if ( ( (*it)->isType(EFAA_OPC_ENTITY_RTOBJECT) || (*it)->isType(EFAA_OPC_ENTITY_AGENT) ) && (dynamic_cast<Object*>(*it))->m_present )
                //{
                //    //yDebug() << "push back " << (*it)->name() << *it;
                //    presentObjects.push_back(dynamic_cast<Object*>(*it));
                //}
                //yDebug() << "object name is : " << dynamic_cast<Object*>(*it)->name() << " with type " << dynamic_cast<Object*>(*it)->entity_type() ;
                //yDebug() << "Check " << (*it)->isType(EFAA_OPC_ENTITY_OBJECT) ;
                //yDebug() << "Position = {" <<  (dynamic_cast<Object*>(*it))->m_ego_position[0] << ", " << (dynamic_cast<Object*>(*it))->m_ego_position[1] << ", " << (dynamic_cast<Object*>(*it))->m_ego_position[2] << "}" ;
                // EVERY OBJECT CAN BE TRACKED, INCLUDE ABSENT OBJECT ONL IF SALIENCY IS NOT NUL AND OBJECT NOT IN 0 0 0
                if ((entity->isType(EFAA_OPC_ENTITY_OBJECT) || entity->isType(EFAA_OPC_ENTITY_RTOBJECT) || entity->isType(EFAA_OPC_ENTITY_AGENT)))
                {
                    if (!( dynamic_cast<Object*>(entity)->m_ego_position[0] == 0.0 && dynamic_cast<Object*>(entity)->m_ego_position[1] == 0.0 && dynamic_cast<Object*>(entity)->m_ego_position[2] == 0.0))
                    {
                        if ((dynamic_cast<Object*>(entity))->m_present)
                        {
                            //yDebug() << "push back (present) " << (*it)->name() ;
                            presentObjects.push_back(dynamic_cast<Object*>(entity)->name());
                        }
                        else if ((dynamic_cast<Object*>(entity))->m_saliency > 0.0)
                        {
                            //yDebug() << "push back (saliency) " << (*it)->name() ;
                            presentObjects.push_back(dynamic_cast<Object*>(entity)->name());
                        }
                    }
                }


            }
        }
    }

    if (icub == NULL)
        icub = opc->addOrRetrieveEntity<Agent>("icub");

    if (presentObjects.size() <= 0)
    {
        yWarning() << "Unable to get any lookable entity from OPC";
    }
    else if (autoSwitch)
    {
        exploring();
    }

    if (trackedCoordinates)
    {
        yInfo() << "Tracking coordinates: " << x_coord << " " << y_coord << " " << z_coord << ".";
        Vector newTarget(3); newTarget[0] = x_coord; newTarget[1] = y_coord; newTarget[2] = z_coord;
        if (isFixationPointSafe(newTarget))
            are->track(newTarget);
    }
    else if (trackedObject != "none")
    {
        yInfo() << "Tracking locked on object " << trackedObject << ".";
        Object* oTracked = dynamic_cast<Object*>(opc->getEntity(trackedObject));
        Vector newTarget = oTracked->m_ego_position;
        if (isFixationPointSafe(newTarget))
            are->track(newTarget);
    }
    return true;
}


/************************************************************************/
double attentionSelectorModule::getPeriod() {
    return 0.01;
}

bool attentionSelectorModule::isFixationPointSafe(const Vector &fp)
{
    if (fp[0] < -0.015)
        return true;
    else
        return false;
}

/************************************************************************/
void attentionSelectorModule::exploring() {

    double maxSalience = 0;
    string nameTrackedObject = "none";
    if (presentObjects.size() == 0)
    {
        aState = s_exploring; return;
    }

    for (auto& presentObject : presentObjects)
    {
        Object* o = dynamic_cast<Object*>(opc->getEntity(presentObject));
        if (o && maxSalience < o->m_saliency)
        {
            maxSalience = o->m_saliency;
            nameTrackedObject = o->name();
        }
    }

    if (nameTrackedObject != "none")
    {
        yInfo() << "Most salient object is: " << nameTrackedObject << " with saliency=" << maxSalience;
        trackedObject = nameTrackedObject;
    }
    else
    {
        if (Time::now() > timeLastSwitch + trackSwitchingPeriod)
        {
            int rndID = rand() % presentObjects.size();
            trackedObject = presentObjects[rndID];
            yInfo() << "Now track object " << rndID << trackedObject;
            timeLastSwitch = Time::now();
        }
    }
}


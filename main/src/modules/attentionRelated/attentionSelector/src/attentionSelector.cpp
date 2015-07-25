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

#include "iCub/attentionSelector.h"


using namespace wysiwyd::wrdac;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/************************************************************************/
bool attentionSelectorModule::configure(yarp::os::ResourceFinder &rf) {    

    moduleName            = rf.check("name", 
        Value("attentionSelector"), 
        "module name (string)").asString();

    setName(moduleName.c_str());

    opcName             = rf.check("opcName", 
        Value("OPC"), 
        "Opc name (string)").asString();

    trackSwitchingPeriod = rf.check("trackSwitchingPeriod", 
        Value(1.0)).asDouble();


    opc = new OPCClient(moduleName.c_str());
    opc->connect(opcName);
    icub = NULL;

    handlerPortName = "/";
    handlerPortName +=  getName() + "/rpc";

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    gazePortName = "/";
    gazePortName += getName() + "/gaze";
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local", gazePortName.c_str());

    igaze=NULL;
    if (clientGazeCtrl.open(option)) {
        clientGazeCtrl.view(igaze);
    }
    else
    {
        cout<<"Invalid gaze polydriver"<<endl;
        return false;
    }
    igaze->storeContext(&store_context_id);

    double neckTrajTime = rf.check("neckTrajTime", 
        Value(0.75)).asDouble();
    igaze->setNeckTrajTime(neckTrajTime);

    double bindNeckPitchMin = rf.check("neckPitchBindMin", Value(-25.0)).asDouble();
    double bindNeckPitchMax = rf.check("neckPitchBindMax", Value(25.0)).asDouble();
    igaze->bindNeckPitch(bindNeckPitchMin, bindNeckPitchMax);

    attach(handlerPort);                  // attach to port

    aState = s_waiting;
    trackedObject = NULL;
    x_coord = 0.0;
    y_coord = 0.0;
    z_coord = 0.0;
    trackedCoordinates = false ;
    autoSwitch = true;

    return true ;
}

/************************************************************************/
bool attentionSelectorModule::interruptModule() {
    opc->interrupt();
    handlerPort.interrupt();
    return true;
}

/************************************************************************/
bool attentionSelectorModule::close() {
    opc->close();
    handlerPort.close();

    igaze->restoreContext(store_context_id);
    if (clientGazeCtrl.isValid())
        clientGazeCtrl.close();

    return true;
}

/************************************************************************/
bool attentionSelectorModule::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) + 
        " commands are: \n" +  
        "track <string name> : track the object with the given opc name \n" +
        "track <int id> : track the object with the given opc id \n" + 
        "track <double x> <double y> <double z> : track with the object coordinates\n" +
        "auto : switch attention between present objects \n" + 
        "sleep : pauses the head control until next command" +
        "help \n" + 
        "quit \n" ;

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="track") {
        autoSwitch = false;
        if (command.get(1).isInt()) {
            trackedObject = dynamic_cast<Object*>(opc->getEntity(command.get(1).asInt()));
            trackedCoordinates = false ;
        }
        else if (command.get(1).isString()) {
            trackedObject = dynamic_cast<Object*>(opc->getEntity(command.get(1).asString().c_str()));
            trackedCoordinates = false ;
        }
        else {
            trackedCoordinates = true ;

            x_coord = command.get(1).asDouble() ;
            y_coord = command.get(2).asDouble() ;
            z_coord = command.get(3).asDouble() ;

            cout << "after coordinates" << endl ;
        }
        aState = s_tracking; 
        reply.addString("ack");
    }
    else if (command.get(0).asString()=="auto") {
        autoSwitch = true;
        reply.addString("ack");
    }   
    else if (command.get(0).asString()=="sleep") {
        autoSwitch = false;
        trackedObject = NULL;
        reply.addString("ack");
    }   
    else if (command.get(0).asString()=="look") {
        autoSwitch = false;
        trackedObject = NULL;
        trackedCoordinates = false ;

        Vector xyz(3);
        xyz[0] = command.get(1).asDouble() ;
        xyz[1] = command.get(2).asDouble() ;
        xyz[2] = command.get(3).asDouble() ;

        igaze->lookAtFixationPoint(xyz);
        reply.addString("ack");
    }   
    else if (command.get(0).asString()=="waitMotionDone") {
        igaze->waitMotionDone();
        reply.addString("ack");
    }   
    else if (command.get(0).asString()=="getFixationPoint") {
        Vector v;
        igaze->getFixationPoint(v);
        reply.addString("ack");
        reply.addDouble(v[0]);
        reply.addDouble(v[1]);
        reply.addDouble(v[2]);
    }   
    else if (command.get(0).asString()=="getHeadPose") {
        Vector x,o;
        igaze->getHeadPose(x,o);
        reply.addString("ack");
        reply.addDouble(x[0]);
        reply.addDouble(x[1]);
        reply.addDouble(x[2]);
        reply.addDouble(o[0]);
        reply.addDouble(o[1]);
        reply.addDouble(o[2]);
        reply.addDouble(o[3]);
    }
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
    for(list<Entity*>::iterator it=entities.begin(); it !=entities.end(); it++)
    {
        if ((*it)->name() == "icub")
            icub = dynamic_cast<Agent*>(*it);
        else
        {
            //!!! ONLY RT_OBJECT and AGENTS ARE TRACKED !!!
            if ( ( (*it)->isType(EFAA_OPC_ENTITY_RTOBJECT) || (*it)->isType(EFAA_OPC_ENTITY_AGENT) ) && (dynamic_cast<Object*>(*it))->m_present )
            {
                presentObjects.push_back(dynamic_cast<Object*>(*it));
            }
        }
    }

    if (icub == NULL)
        icub = opc->addOrRetrieveEntity<Agent>("icub");

    if (presentObjects.size() <= 0)
    {
        cout<<"Unable to get any lookable entity from OPC"<<endl;
    }
    else if (autoSwitch)
    {
        exploring();
    }

    if(trackedCoordinates)
    {
        cout<<"Tracking tracking coordinates: "<<x_coord<<" "<<y_coord<<" "<<z_coord<<"."<<endl;
        Vector newTarget(3); newTarget[0]=x_coord;newTarget[1]=y_coord;newTarget[2]=z_coord;
        if (isFixationPointSafe(newTarget))
            igaze->lookAtFixationPoint(newTarget);
    }
    else if (trackedObject != NULL)
    {
        cout<<"Tracking locked on object "<<trackedObject->name()<<"."<<endl;
        Vector newTarget = icub->getSelfRelativePosition(trackedObject->m_ego_position);
        if (isFixationPointSafe(newTarget))
            igaze->lookAtFixationPoint(newTarget);
    }
    return true;
}


/************************************************************************/
double attentionSelectorModule::getPeriod() {   
    return 0.01;
}

bool attentionSelectorModule::isFixationPointSafe(Vector fp)
{
    if (fp[0] < -0.015 )
        return true;
    else
        return false;
}

/************************************************************************/
void attentionSelectorModule::exploring() {

    double maxSalience = 0;
    string nameTrackedObject = "none";
    if (presentObjects.size()==0)
    {aState = s_exploring; return;}

    Object* mostSalient = *presentObjects.begin();
    for(vector<Object*>::iterator it = presentObjects.begin(); it!= presentObjects.end(); it++)
    {
        if (maxSalience < (*it)->m_saliency )
        {
            maxSalience = (*it)->m_saliency;
            nameTrackedObject = (*it)->name();
            mostSalient = *it;   
        }
        //cout<<(*it)->name()<<"\´s saliency is " <<(*it)->m_saliency<<endl;
    }

    if (nameTrackedObject != "none")
    {
        cout<<"Most salient is : "<<mostSalient->name()<<" with saliency="<<mostSalient->m_saliency<<endl;
        trackedObject = mostSalient;
    }
    else
    {
        if(Time::now()>timeLastSwitch + trackSwitchingPeriod)
        {
            int rndID = rand()%presentObjects.size();
            trackedObject = presentObjects[rndID];
            timeLastSwitch = Time::now();
        }
    }
}



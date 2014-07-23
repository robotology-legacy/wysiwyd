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
    useSaliency = rf.check("useSaliency");

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
        trackingCounter = 0;
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
           trackedObject = (Object*)opc->getEntity(command.get(1).asInt());
           trackedCoordinates = false ;
       }
       else if (command.get(1).isString()) {
           trackedObject = (Object*)opc->getEntity(command.get(1).asString().c_str());
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
			icub = (Agent*)(*it);
		else
		{
			//!!! ONLY RT_OBJECT and AGENTS ARE TRACKED !!!
			if ( ( (*it)->isType(EFAA_OPC_ENTITY_OBJECT) || (*it)->isType(EFAA_OPC_ENTITY_AGENT) ) && ((Object*)(*it))->m_present )
			{
				presentObjects.push_back((Object*)(*it));
			}
		}
	}
    if (useSaliency)
        updateSaliency();

	if (icub == NULL)
		icub = opc->addAgent("icub");

    if (presentObjects.size() <= 0)
    {
        cout<<"Unable to get any lookable entity from OPC"<<endl;
        //trackedObject = NULL;
        //aState = s_waiting;
    }

    switch (aState)
    {
        case s_waiting:
        {
            cout<<".";
            aState = s_exploring;
        }
        break;

        case s_exploring:
        {
            if (autoSwitch)
            {
                cout<<"Exploration... Trying to find a nice object"<<endl;
                exploring();
            }
        }
        break;

        case s_tracking:
        {
            //cout<<"Tracking..."<<endl;
            tracking();
            trackingCounter++;
        }
        break;
    }

    if (trackingCounter * getPeriod() > trackSwitchingPeriod )
    {
        if (autoSwitch)
        {
            cout<<"Bored with current object. Trying to find a new one."<<endl;
            trackingCounter = 0;
            aState = s_waiting;
        }
        else
        {
            if(trackedCoordinates)
                cout<<"Tracking tracking coordinates: "<<x_coord<<" "<<y_coord<<" "<<z_coord<<"."<<endl;
            else if (trackedObject != NULL)
                cout<<"Tracking locked on object "<<trackedObject->name()<<"."<<endl;
            else
                cerr<<"Object is null and should not be "<<endl;

        }
    }

    return true;
}

/************************************************************************/
void attentionSelectorModule::updateSaliency() {   
    cout<<"Updating Saliency"<<endl;
    //Update the object model
    for(vector< Object* >::iterator it=presentObjects.begin(); it!=presentObjects.end();it++)
    {    bool appeared,disappeared;
        if (presentObjectsLastStep.find((*it)->name()) != presentObjectsLastStep.end())
        {
            Vector lastPos = presentObjectsLastStep[ (*it)->name() ].o.m_ego_position;
            Vector currentPos = (*it)->m_ego_position;

            //Get instantaneous speed of the object
            double speed = sqrt(pow(lastPos[0] - currentPos[0],2.0) + pow(lastPos[1] - currentPos[1],2.0) + pow(lastPos[2] - currentPos[2],2.0)) / getPeriod(); 
            double acceleration = speed - presentObjectsLastStep[ (*it)->name() ].speed;
            presentObjectsLastStep[ (*it)->name() ].speed = speed;
            presentObjectsLastStep[ (*it)->name() ].acceleration = acceleration;

            appeared = (!presentObjectsLastStep[ (*it)->name() ].o.m_present) && (*it)->m_present;
            disappeared = presentObjectsLastStep[ (*it)->name() ].o.m_present && !(*it)->m_present;
        }
        else
        {
            presentObjectsLastStep[ (*it)->name() ].speed = 0.0;
            presentObjectsLastStep[ (*it)->name() ].acceleration = 0.0;
            appeared = true;
            disappeared = false;
        }

        if (appeared)
            (*it)->m_saliency = 1.0;
        
        (*it)->m_saliency += presentObjectsLastStep[ (*it)->name() ].acceleration * ACCELERATION_COEFFICIENT;


        if (this->trackedObject == (*it))
            (*it)->m_saliency -= FOCUS_LEAKING;
        //else
        //    (*it)->m_saliency += FOCUS_LEAKING / 2.0;

        (*it)->m_saliency = max(0.0, min((*it)->m_saliency , 1.0));

        presentObjectsLastStep[ (*it)->name() ].o.fromBottle( (*it)->asBottle() );
        //cout<<(*it)->name()<<"\´s saliency is " <<(*it)->m_saliency<<endl;
    }

    //if (this->trackedObject != NULL && this->trackedObject->m_saliency < 0.3)
    //{
    //    aState = s_exploring;
    //}

    //Commit the saliency changes
    //opc->commit();
}

/************************************************************************/
double attentionSelectorModule::getPeriod() {   
    return 0.05;
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

    if (presentObjects.size()==0)
        {aState = s_exploring; return;}

    if (useSaliency)
    {
        Object* mostSalient = *presentObjects.begin();
        for(vector<Object*>::iterator it = presentObjects.begin(); it!= presentObjects.end(); it++)
        {
            if (mostSalient->m_saliency <= (*it)->m_saliency )
            {
                mostSalient = (*it);
            }
            //cout<<(*it)->name()<<"\´s saliency is " <<(*it)->m_saliency<<endl;
        }

        trackedObject = mostSalient;
        cout<<"Choosed to track : "<<trackedObject->name()<<endl;
        aState = s_tracking; 
    }
    else
    {
        //cout<<"Inside exploring";
        //cout<<"size is"<<presentObjects.size()<<endl;
        if (presentObjects.size()==0)
            {aState = s_exploring; return;}
            
        bool fSuitable = false;
        int count = 0;
        while (!fSuitable && count< 10)
        {   
            int rndID = rand()%presentObjects.size();
            //cout<<"Size is "<<presentObjects.size()<<" chosen is "<<rndID<<endl;
            Vector positionRelativeToRobot = icub->getSelfRelativePosition(presentObjects[rndID]->m_ego_position);
            fSuitable = isFixationPointSafe(positionRelativeToRobot);
            trackedObject = presentObjects[rndID];
            count++;
        }
        if (count==10)
            aState = s_exploring;
        else
        {
           cout<<"Choosed to track : "<<trackedObject->name()<<endl;
           aState = s_tracking; 
        }
    }
}

/************************************************************************/
void attentionSelectorModule::tracking() {

    if (trackedObject == NULL && !trackedCoordinates)
    {
        aState = s_waiting;
        return;
    }
    else if (trackedCoordinates == true) {

        //cout << "in trackedCoordinates == true" << endl ;

        Vector newTarget ;
        newTarget.push_back(x_coord) ;
        newTarget.push_back(y_coord) ;
        newTarget.push_back(z_coord) ;

        //cout << "target : (" << newTarget[0] << ", " << newTarget[1] << ", " << newTarget[2] << ")" << endl ; 

		if (isFixationPointSafe(newTarget))
			igaze->lookAtFixationPoint(newTarget);

    }
    else
    {
        //Check if the tracked ID is still in the OPC
        bool found = false;
        
        //cout << "in track by name or id" << endl ;

		for(vector<Object*>::iterator it=presentObjects.begin();it!=presentObjects.end();it++)
        {
            //cout<<"Debug : "<<(*it)->name()<<" -- ";
			if (trackedObject != NULL && trackedObject->name() == (*it)->name())
			{
				found = true;
				break;
			}
        }
        if (!found)
        {
            cout<<"Object not found..."<<endl;
            return;
        }

        Vector newTarget = icub->getSelfRelativePosition(trackedObject->m_ego_position);
        //cout << "target : (" << newTarget[0] << ", " << newTarget[1] << ", " << newTarget[2] << ")" << endl ; 
		if (isFixationPointSafe(newTarget))
			igaze->lookAtFixationPoint(newTarget);
    }
 
}   



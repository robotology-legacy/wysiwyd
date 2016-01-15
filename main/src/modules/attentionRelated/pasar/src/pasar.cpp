// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project IST-270490
* Authors: Stéphane Lallée, Grégoire Pointeau
* email:   stephane.lallee@gmail.com, greg.pointeau@gmail.com
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

#include "pasar.h"
#include "wrdac/subsystems/subSystem_ABM.h"


using namespace wysiwyd::wrdac;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/************************************************************************/
bool PasarModule::configure(yarp::os::ResourceFinder &rf) {
    std::string opcName;
    std::string gazePortName;
    std::string handlerPortName;
    std::string saliencyPortName;

    string moduleName = rf.check("name", Value("pasar")).asString().c_str();
    setName(moduleName.c_str());


    //    moduleName = rf.check("name",
    //        Value("pasar")).asString();
    //    setName(moduleName.c_str());

    //Parameters
    pTopDownAppearanceBurst = rf.check("parameterTopDownAppearanceBurst",
        Value(3.)).asDouble();
    pTopDownDisappearanceBurst = rf.check("parameterTopDownDisappearanceBurst",
        Value(2.)).asDouble();
    pTopDownAccelerationCoef = rf.check("parameterTopDownAccelerationCoef",
        Value(0.1)).asDouble();
    //pLeakyIntegrationA        =  rf.check("parameterLeakyIntegrationA", 
    //    Value(0.9)).asDouble(); 
    pTopDownInhibitionReturn = rf.check("parameterInhibitionReturn",
        Value(0.05)).asDouble();
    pExponentialDecrease = rf.check("ExponentialDecrease",
        Value(0.9)).asDouble();
    pTopDownWaving = rf.check("pTopDownWaving",
        Value(0.2)).asDouble();
    dBurstOfPointing = rf.check("pBurstOfPointing",
        Value(0.2)).asDouble();
    dthresholdAppear = rf.check("thresholdAppear", Value(1.)).asDouble();
    dthresholdDisappear = rf.check("thresholdDisappear", Value(2.)).asDouble();

    //check for decrease
    if (pExponentialDecrease >= 1 || pExponentialDecrease <= 0.0)   pExponentialDecrease = 0.95;

    presentRightHand.first = false;
    presentRightHand.second = false;
    presentLeftHand.first = false;
    presentLeftHand.first = false;

    rightHandt1 = Vector(3, 0.0);
    rightHandt2 = Vector(3, 0.0);
    leftHandt1 = Vector(3, 0.0);
    leftHandt2 = Vector(3, 0.0);

    thresholdMovementAccel = rf.check("thresholdMovementAccel",
        Value(0.02)).asDouble();
    thresholdWaving = rf.check("thresholdWaving",
        Value(0.02)).asDouble();
    thresholdSaliency = rf.check("thresholdSaliency",
        Value(0.005)).asDouble();

    isControllingMotors = rf.check("motorControl",
        Value(0)).asInt() == 1;
    //Ports

    opcName = rf.check("opc", Value("OPC")).asString().c_str();
    opc = new OPCClient(moduleName);
    while (!opc->connect(opcName))
    {
        cout << "Waiting connection to OPC..." << endl;
        Time::delay(1.0);
    }

    opc->checkout();


    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "pasar", "pasar.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    if (!iCub->connect())
    {
        yInfo() << "iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    abm = true;
    if (!iCub->getABMClient())
    {
        abm = false;
        yWarning() << " WARNING ABM NOT CONNECTED, MODULE CANNOT START";
    }


    if (!Network::connect("/agentDetector/skeleton:o", ("/" + moduleName + "/skeleton:i").c_str()))
    {
        isSkeletonIn = false;
    }
    else
    {
        yInfo() << " is connected to skeleton";
        isSkeletonIn = true;
    }

    isPointing = rf.find("isPointing").asInt() == 1;
    isWaving = rf.find("isWaving").asInt() == 1;

    yInfo() << " pointing: " << isPointing;
    yInfo() << " waving: " << isWaving;

    if (!handlerPort.open(("/" + moduleName + "/rpc").c_str())) {
        cout << getName() << ": Unable to open port rpc" << endl;
        return false;
    }

    attach(handlerPort);                  // attach to port
    trackedObject = "";
    presentObjectsLastStep.clear();
    initTime = yarp::os::Time::now();
    initializeMapTiming();

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";


    return true;
}

/************************************************************************/
bool PasarModule::interruptModule() {
    opc->interrupt();
    handlerPort.interrupt();
    return true;
}

/************************************************************************/
bool PasarModule::close() {
    opc->close();
    handlerPort.interrupt();
    handlerPort.close();
    return true;
}

/************************************************************************/
bool PasarModule::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "track <string name> : track the object with the given opc name \n" +
        "track <int id> : track the object with the given opc id \n" +
        "track <double x> <double y> <double z> : track with the object coordinates\n" +
        "auto : switch attention between present objects \n" +
        "sleep : pauses the head control until next command\n" +
        "help \n" +
        "pointing on/off:  launch or stop pointing\n" +
        "waving on/off:  launch or stop waving\n" +
        "quit \n";

    reply.clear();
    reply.addString("ack");

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "help") {
        reply.addString(helpMessage.c_str());
    }
    else if (command.get(0).asString() == "pointing") {
        if (command.size() != 2)
        {
            reply.addString("error in PASAR: Bottle 'pointing' misses information (on/off)");
        }
        else
        {
            if (command.get(1).asString() == "off")
            {
                isPointing = false;
                yInfo() << " stop pointing";
                reply.addString("stop pointing");
            }
            else if (command.get(1).asString() == "on")
            {
                isPointing = true;
                yInfo() << " start pointing";
                reply.addString("start pointing");
            }
        }
    }
    else if (command.get(0).asString() == "waving") {
        if (command.size() != 2)
        {
            reply.addString("error in PASAR: Botte 'waving' misses information (on/off)");
        }
        else
        {
            if (command.get(1).asString() == "off")
            {
                isWaving = false;
                yInfo() << " stop waving";
                reply.addString("stop waving");
            }
            else if (command.get(1).asString() == "on")
            {
                isWaving = true;
                yInfo() << " start waving";
                reply.addString("start waving");
            }
        }
    }
    else if (command.get(0).asString() == "set") {
        reply.addString("set");
        /* if (command.get(1).asString()=="leak")
        {
        reply.addString("leak");
        this->pLeakyIntegrationA = command.get(2).asDouble();
        }*/
        //if
        //{
        reply.addString("ir");
        this->pTopDownInhibitionReturn = command.get(2).asDouble();
        //  }
    }
    return true;
}

/***************************************************************************/
bool PasarModule::updateModule()
{
    opc->checkout();
    entities = opc->EntitiesCache();

    presentObjects.clear();
    presentLastSpeed = presentCurrentSpeed;
    presentCurrentSpeed.clear();


    double now = Time::now() - initTime;


    for (auto &entity : entities)
    {
        if (entity->name() != "icub")
        {
            //!!! ONLY OBJECTS, RT_OBJECT and AGENTS ARE TRACKED !!!

            if (entity->isType(EFAA_OPC_ENTITY_RTOBJECT) || entity->isType(EFAA_OPC_ENTITY_AGENT) || entity->isType(EFAA_OPC_ENTITY_OBJECT))
            {
                if (entity->isType(EFAA_OPC_ENTITY_RTOBJECT))
                {
                    RTObject * rto = dynamic_cast<RTObject*>(entity);
                    presentObjects[entity->opc_id()].o.fromBottle(rto->asBottle());
                    presentObjects[entity->opc_id()].o.m_saliency = rto->m_saliency;

                }

                if (entity->isType(EFAA_OPC_ENTITY_OBJECT))
                {
                    Object * ob = dynamic_cast<Object*>(entity);
                    presentObjects[entity->opc_id()].o.fromBottle(ob->asBottle());
                    presentObjects[entity->opc_id()].o.m_saliency = ob->m_saliency;

                }

                if (entity->isType(EFAA_OPC_ENTITY_AGENT))
                {
                    Agent *ag = dynamic_cast<Agent*>(entity);
                    presentObjects[entity->opc_id()].o.fromBottle(ag->asBottle());
                    presentObjects[entity->opc_id()].o.m_saliency = ag->m_saliency;

                }
            }
        }
    }
    //if (presentObjects[ (*it)->name() ].o.m_saliency > 0)
    //    cout<<" salience : " << (*it)->name() << " " << presentObjects[ (*it)->name() ].o.m_saliency << endl;

    if (presentObjectsLastStep.size() > 0)
    {
        //Compute top down saliency (concept based)
        saliencyTopDown();
        if (isPointing) saliencyPointing();
        if (isWaving)   saliencyWaving();

        //Leaky integrate
        saliencyLeakyIntegration();

        //Get the most salient object and track it
        auto mostSalientObject = presentObjects.begin();
        for (auto it = presentObjects.begin(); it != presentObjects.end(); it++)
        {
            //  cout<<"Saliency ("<<it->second.o.name()<<") = "<<it->second.o.m_saliency<<endl;
            if (it->second.o.m_saliency > mostSalientObject->second.o.m_saliency)
                mostSalientObject = it;
        }

        trackedObject = mostSalientObject->second.o.name();

        if (presentObjects[mostSalientObject->first].o.m_saliency > 0.0)
        {
            cout << "Tracking : " << trackedObject << " Salience : " << presentObjects[mostSalientObject->first].o.m_saliency << endl;
        }


        //Update the OPC values
        for (list<Entity*>::iterator it = entities.begin(); it != entities.end(); it++)
        {
            if (presentObjects.find((*it)->opc_id()) != presentObjects.end())
            {
                (dynamic_cast<Object*>(*it))->m_saliency = presentObjects[(*it)->opc_id()].o.m_saliency;
            }
        }
        opc->commit();
    }
    presentObjectsLastStep = presentObjects;

    return true;
}



/*
*   Update the salience according to the informations contained in the OPC (acceleration, appareance, disappareance)
*
*/
/************************************************************************/
void PasarModule::saliencyTopDown() {
    //cout<<"TopDown Saliency"<<endl;
    double now = yarp::os::Time::now() - initTime;
    //Add up the top down saliency
    for (auto &it : presentObjects)
    {
        if (presentObjectsLastStep.find(it.first) != presentObjectsLastStep.end() && it.second.o.name() != "cursor_0" && it.second.o.name() != "icub")
        {
            //Objects appears/disappears
            bool appeared, disappeared;


            //            yInfo() << "\t\t entity: " << it.second.o.name() << " last seen: " << it.second.lastTimeSeen << " now: " << now << "  diff is: " << now - it.second.lastTimeSeen;


            // If the object is absent:
            // if the lastTimeSeen was more than a threshold, the object dissapeared
            disappeared = (now - it.second.lastTimeSeen > dthresholdDisappear) && (!it.second.o.m_present) && it.second.present;

            // If the object is present:
            // if the lastTimeSeen was more than a threshold, the object appeared
            appeared = (now - it.second.lastTimeSeen > dthresholdAppear) && (it.second.o.m_present) && !it.second.present;

            if (it.second.o.m_present) it.second.lastTimeSeen = now;



            //instantaneous speed/acceleration
            Vector lastPos = presentObjectsLastStep[it.first].o.m_ego_position;
            Vector currentPos = it.second.o.m_ego_position;
            presentCurrentSpeed[it.first] = pair<double, double>(lastPos[0] - currentPos[0], lastPos[1] - currentPos[1]);

            double acceleration = sqrt(pow(presentCurrentSpeed[it.first].first - presentLastSpeed[it.first].first, 2.) + pow(presentCurrentSpeed[it.first].second - presentLastSpeed[it.first].second, 2.));

            presentObjects[it.first].acceleration = acceleration;

            //Use the world model (CONCEPTS <=> RELATIONS) to modulate the saliency
            if (appeared)
            {
                it.second.o.m_saliency += pTopDownAppearanceBurst;
                it.second.present = true;
                yInfo() << "\t\t APPEARANCE OF: " << it.second.o.name();

                if (iCub->getABMClient()->Connect())
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(it.second.o.name(), "appear"));
                    iCub->getABMClient()->sendActivity("event",
                        "appearance",
                        "pasar",
                        lArgument,
                        true);
                }
            }

            if (disappeared)
            {
                it.second.o.m_saliency += pTopDownDisappearanceBurst;
                it.second.present = false;
                yInfo() << "\t\t DISAPPEARANCE OF: " << it.second.o.name();
                if (iCub->getABMClient()->Connect())
                {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>(it.second.o.name(), "disappear"));
                    iCub->getABMClient()->sendActivity("event",
                        "disappearance",
                        "pasar",
                        lArgument,
                        true);
                }
            }

            if (acceleration > thresholdMovementAccel)
            {
                it.second.o.m_saliency += pTopDownAccelerationCoef;
                yInfo() << " moving object:" << it.second.o.name() << " salience : " << it.second.o.m_saliency << " acceleration: " << acceleration;
            }
        }
    }
}

/************************************************************************/
void PasarModule::saliencyNormalize() {
    //cout<<"Normalizing Saliency"<<endl;

    //Get the max
    map< int, ObjectModel >::iterator mostSalientObject = presentObjects.begin();

    for (map< int, ObjectModel >::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
    {
        if (it->second.o.m_saliency > mostSalientObject->second.o.m_saliency)
            mostSalientObject = it;
    }

    //Normalize
    double maxS = mostSalientObject->second.o.m_saliency;
    if (maxS == 0) maxS = 1.;
    for (map< int, ObjectModel >::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
    {
        it->second.o.m_saliency /= maxS;
    }
}

/*
*   Decrease of the salience through time.
*   Exponential facotr pExponentialDecrease < 1
*/
/************************************************************************/
void PasarModule::saliencyLeakyIntegration() {
    //cout<<"Leaky integration"<<endl;
    //cout<<"Membrane Activity : "<<endl;
    for (auto &it : presentObjects)
    {
        if (it.second.o.name() != "")
        {
            it.second.o.m_saliency *= pExponentialDecrease;
        }
        if (it.second.o.m_saliency < thresholdSaliency)
            it.second.o.m_saliency = 0.0;
    }
}

/************************************************************************/
double PasarModule::getPeriod() {
    return 0.1;
}

bool PasarModule::isFixationPointSafe(Vector fp)
{
    if (fp[0] < -0.015)
        return true;
    else
        return false;
}



/*
* increase the salience of the closest object from the right hand
*
*/
void PasarModule::saliencyPointing()
{

    bool wasPresent = false;
    Agent *ag;
    // founding the agent:
    Vector vec;
    for (auto &it : presentObjects){
        if (it.second.o.entity_type() == EFAA_OPC_ENTITY_AGENT
            && it.second.present
            && (it.second.o.name() != "iCub")){
            ag = dynamic_cast<Agent*>(&it.second.o);
            Vector vec = ag->m_body.m_parts["handRight"];
            wasPresent = true;
        }
    }

    if (!wasPresent){
        yInfo() << " in PASAR:saliencyPointing no human agent present";
        return;
    }

    //    yInfo() << " righthand is:" << vec.toString();

    double x = vec[0];
    double y = vec[1];
    double z = vec[2];

    double closest = 10e5;
    string objectPointed = "none";

    for (auto &it : presentObjects)
    {
        if (it.second.o.name() != ag->name())
        {
            double distance;

            //!!! ONLY RT_OBJECT and AGENTS ARE TRACKED !!!

            distance = sqrt(
                (x - it.second.o.m_ego_position[0])*(x - it.second.o.m_ego_position[0]) +
                (y - it.second.o.m_ego_position[1])*(y - it.second.o.m_ego_position[1]) +
                (z - it.second.o.m_ego_position[2])*(z - it.second.o.m_ego_position[2])
                );
            //                yInfo() << " distance from " << (*it)->name() << " is " << distance;
            if (distance < closest)
            {
                closest = distance;
                objectPointed = it.second.o.name();
            }
        }
    }



    if (objectPointed != "none")
    {
        yInfo() << " pointed object is: \t" << objectPointed;
        for (auto &it : presentObjects)
        {
            if (it.second.o.name() == objectPointed)
            {
                it.second.o.m_saliency += dBurstOfPointing;
            }
        }
    }
    else
    {
        yInfo() << " pasar: no object pointed";
    }
}


/*
*  Increase the saliency of the agent waving
*
*/
void PasarModule::saliencyWaving()
{

    bool wasPresent = false;
    Agent *ag;
    // founding the agent:
    Vector vec;
    for (auto &it : presentObjects){
        if (it.second.o.entity_type() == EFAA_OPC_ENTITY_AGENT
            && it.second.present
            && (it.second.o.name() != "iCub")){
            ag = dynamic_cast<Agent*>(&it.second.o);
            Vector vec = ag->m_body.m_parts["handRight"];
            wasPresent = true;
        }
    }

    if (!wasPresent){
        yInfo() << " in PASAR:saliencyWaving no human agent present";
        return;
    }


    if (!ag || !ag->m_present)
    {
        presentRightHand.first = presentRightHand.second;
        presentLeftHand.first = presentLeftHand.second;

        presentRightHand.second = false;
        presentLeftHand.second = false;

        return;
    }


    Vector vecRight = ag->m_body.m_parts["handRight"];
    Vector vecLeft = ag->m_body.m_parts["handLeft"];

    Vector speedRightt1(3);
    Vector speedRightt2(3);

    Vector speedLeftt1(3);
    Vector speedLeftt2(3);

    Vector accelRight(3);
    Vector accelLeft(3);


    speedRightt2[0] = (rightHandt1[0] - rightHandt2[0]);
    speedRightt2[1] = (rightHandt1[1] - rightHandt2[1]);
    speedRightt2[2] = (rightHandt1[2] - rightHandt2[2]);

    speedRightt1[0] = (vecRight[0] - rightHandt1[0]);
    speedRightt1[1] = (vecRight[1] - rightHandt1[1]);
    speedRightt1[2] = (vecRight[2] - rightHandt1[2]);

    speedLeftt1[0] = (vecLeft[0] - leftHandt1[0]);
    speedLeftt1[1] = (vecLeft[1] - leftHandt1[1]);
    speedLeftt1[2] = (vecLeft[2] - leftHandt1[2]);

    speedLeftt2[0] = (leftHandt1[0] - leftHandt2[0]);
    speedLeftt2[1] = (leftHandt1[1] - leftHandt2[1]);
    speedLeftt2[2] = (leftHandt1[2] - leftHandt2[2]);

    accelRight[0] = (speedRightt1[0] - speedRightt2[0]);
    accelRight[1] = (speedRightt1[1] - speedRightt2[1]);
    accelRight[2] = (speedRightt1[2] - speedRightt2[2]);

    accelLeft[0] = (speedLeftt1[0] - speedLeftt2[0]);
    accelLeft[1] = (speedLeftt1[1] - speedLeftt2[1]);
    accelLeft[2] = (speedLeftt1[2] - speedLeftt2[2]);

    // get the norm of the accel vector
    double dAccelLeft = sqrt(accelLeft[0] * accelLeft[0] + accelLeft[1] * accelLeft[1] + accelLeft[2] * accelLeft[2]);
    double dAccelRight = sqrt(accelRight[0] * accelRight[0] + accelRight[1] * accelRight[1] + accelRight[2] * accelRight[2]);


    yInfo() << " right hand waving: " << dAccelRight << "\t left hand waving: " << dAccelLeft;

    // if the acceleration is made on 3 consecutive frames
    if (presentRightHand.first && presentRightHand.second)
    {
        if (dAccelRight > thresholdWaving)
        {
            ag->m_saliency += pTopDownWaving;
            yInfo() << "\t\t\t\t\tagent is waving right hand";
        }
        rightHandt2 = rightHandt1;
        rightHandt1 = vecRight;
    }

    if (presentLeftHand.first && presentLeftHand.second)
    {
        if (dAccelLeft > thresholdWaving)
        {
            ag->m_saliency += pTopDownWaving;
            yInfo() << "\t\tagent is waving left hand";
        }
        leftHandt2 = leftHandt1;
        leftHandt1 = vecLeft;
    }
    opc->commit();

    presentRightHand.first = presentRightHand.second;
    presentLeftHand.first = presentLeftHand.second;

    presentRightHand.second = true;
    presentLeftHand.second = true;
}



void PasarModule::initializeMapTiming()
{
    opc->checkout();
    entities = opc->EntitiesCache();
    double now = yarp::os::Time::now() - initTime;
    presentObjects.clear();

    for (auto &entity : entities){

        if (entity->name() != "icub")
        {
            //!!! ONLY OBJECTS, RT_OBJECT and AGENTS ARE TRACKED !!!

            if (entity->isType(EFAA_OPC_ENTITY_RTOBJECT) || entity->isType(EFAA_OPC_ENTITY_AGENT) || entity->isType(EFAA_OPC_ENTITY_OBJECT))
            {
                Object * ob = dynamic_cast<Object*>(entity);
                presentObjects[entity->opc_id()].o = *ob;
                presentObjects[entity->opc_id()].lastTimeSeen = ob->m_present ? now : now - 10;
                presentObjects[entity->opc_id()].present = ob->m_present;
            }
        }
    }
}
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

#include "iCub/pasar.h"


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

    moduleName = rf.check("name",
        Value("pasar"),
        "module name (string)").asString();
    setName(moduleName.c_str());

    //Parameters
    pTopDownAppearanceBurst = rf.check("parameterTopDownAppearanceBurst",
        Value(0.5)).asDouble();
    pTopDownDisappearanceBurst = rf.check("parameterTopDownDisappearanceBurst",
        Value(0.5)).asDouble();
    pTopDownAccelerationCoef = rf.check("parameterTopDownAccelerationCoef",
        Value(0.1)).asDouble();
    //pLeakyIntegrationA		=  rf.check("parameterLeakyIntegrationA", 
    //    Value(0.9)).asDouble(); 
    pTopDownInhibitionReturn = rf.check("parameterInhibitionReturn",
        Value(0.05)).asDouble();
    pExponentialDecrease = rf.check("ExponentialDecrease",
        Value(0.9)).asDouble();

    //check for decrease
    if (pExponentialDecrease >= 1)   pExponentialDecrease = 0.95;

    thresholdMovementAccel = rf.check("thresholdMovementAccel",
        Value(0.0)).asDouble();
    thresholdSaliency = rf.check("thresholdSaliency",
        Value(0.005)).asDouble();

    isControllingMotors = rf.check("motorControl",
        Value(0)).asInt() == 1;
    //Ports
    opcName = rf.check("opcName",
        Value("OPC"),
        "Opc name (string)").asString();
    opc = new OPCClient(moduleName.c_str());
    opc->connect(opcName);
    if (!opc->isConnected())
        if (!opc->connect("OPC"))
            return false;
    opc->checkout();

    icub = opc->addOrRetrieveAgent("icub");

    saliencyPortName = "/";
    saliencyPortName += getName() + "/saliency:i";
    if (!saliencyInput.open(saliencyPortName.c_str())) {
        cout << getName() << ": Unable to open port " << saliencyPortName << endl;
        return false;
    }

    saliencyPortName = "/";
    saliencyPortName += getName() + "/saliency:o";
    if (!saliencyOutput.open(saliencyPortName.c_str())) {
        cout << getName() << ": Unable to open port " << saliencyPortName << endl;
        return false;
    }

    handlerPortName = "/";
    handlerPortName += getName() + "/rpc";
    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    gazePortName = "/";
    gazePortName += getName() + "/gaze";
    Property option;
    option.put("device", "gazecontrollerclient");
    option.put("remote", "/iKinGazeCtrl");
    option.put("local", gazePortName.c_str());

    if (isControllingMotors)
    {
        igaze = NULL;
        if (clientGazeCtrl.open(option)) {
            clientGazeCtrl.view(igaze);
        }
        else
        {
            cout << "Invalid gaze polydriver" << endl;
            return false;
        }
        igaze->storeContext(&store_context_id);

        double neckTrajTime = rf.check("neckTrajTime", Value(0.75)).asDouble();
        igaze->setNeckTrajTime(neckTrajTime);
    }

    attach(handlerPort);                  // attach to port
    trackedObject = "";
    presentObjectsLastStep.clear();
    return true;
}

/************************************************************************/
bool PasarModule::interruptModule() {
    opc->interrupt();
    handlerPort.interrupt();
    saliencyInput.interrupt();
    saliencyOutput.interrupt();
    return true;
}

/************************************************************************/
bool PasarModule::close() {
    opc->close();
    handlerPort.close();
    saliencyInput.close();
    saliencyOutput.close();

    igaze->restoreContext(store_context_id);
    if (clientGazeCtrl.isValid())
        clientGazeCtrl.close();

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
        "sleep : pauses the head control until next command" +
        "help \n" +
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

    opc->update();
    list<Entity*> entities = opc->EntitiesCache();
    presentObjects.clear();
    presentLastSpeed = presentCurrentSpeed;
    presentCurrentSpeed.clear();
    for (list<Entity*>::iterator it = entities.begin(); it != entities.end(); it++)
    {
        if ((*it)->name() != "icub")
        {
            //!!! ONLY RT_OBJECT and AGENTS ARE TRACKED !!!
            if (((*it)->isType(EFAA_OPC_ENTITY_OBJECT)))
            {

                if ((*it)->isType(EFAA_OPC_ENTITY_RTOBJECT))
                {
                    RTObject * rto = dynamic_cast<RTObject*>(*it);
                    presentObjects[(*it)->name()].o.fromBottle(rto->asBottle());
                    presentObjects[(*it)->name()].o.m_saliency = rto->m_saliency;
                    presentObjects[(*it)->name()].speed = 0.0;
                    presentObjects[(*it)->name()].acceleration = 0.0;
                    presentObjects[(*it)->name()].restingSteps = 0;

                }

                if ((*it)->isType(EFAA_OPC_ENTITY_AGENT))
                {
                    Agent *ag = dynamic_cast<Agent*>(*it);
                    presentObjects[(*it)->name()].o.fromBottle(ag->asBottle());
                    presentObjects[(*it)->name()].o.m_saliency = ag->m_saliency;
                    presentObjects[(*it)->name()].speed = 0.0;
                    presentObjects[(*it)->name()].acceleration = 0.0;
                    presentObjects[(*it)->name()].restingSteps = 0;

                }/*
                 presentObjects[ (*it)->name() ].o.fromBottle( (*it)->asBottle() );
                 presentObjects[ (*it)->name() ].o.m_saliency = 0.0;
                 presentObjects[ (*it)->name() ].speed = 0.0;
                 presentObjects[ (*it)->name() ].acceleration = 0.0;
                 presentObjects[ (*it)->name() ].restingSteps = 0;*/
            }
        }
        //if (presentObjects[ (*it)->name() ].o.m_saliency > 0)
        //    cout<<" salience : " << (*it)->name() << " " << presentObjects[ (*it)->name() ].o.m_saliency << endl;

    }
    if (presentObjectsLastStep.size() > 0)
    {
        //Retrieve the bottom-up saliency (vision based) and attribute it to objects
        //saliencyBottomUp();

        //Compute top down saliency (concept based)
        saliencyTopDown();

        //Normalize
        //saliencyNormalize();

        //Inhinbition of return
        //        if(trackedObject!= "")
        //            presentObjects[trackedObject].o.m_saliency = max(0.0, presentObjects[trackedObject].o.m_saliency - pTopDownInhibitionReturn);


        //Leaky integrate
        saliencyLeakyIntegration();

        //Get the most salient object and track it
        map< string, ObjectModel >::iterator mostSalientObject = presentObjects.begin();
        for (map< string, ObjectModel >::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
        {
            //  cout<<"Saliency ("<<it->second.o.name()<<") = "<<it->second.o.m_saliency<<endl;
            if (it->second.o.m_saliency > mostSalientObject->second.o.m_saliency)
                mostSalientObject = it;
        }

        trackedObject = mostSalientObject->first;

        if (presentObjects[trackedObject].o.m_saliency > 0.0)
        {
            cout << "Tracking : " << trackedObject << " Salience : " << presentObjects[trackedObject].o.m_saliency << endl;
        }


        if (isControllingMotors && isFixationPointSafe(presentObjects[trackedObject].o.m_ego_position))
            igaze->lookAtFixationPoint(presentObjects[trackedObject].o.m_ego_position);

        //Prepare the output img
        ImageOf<PixelRgb> &img = saliencyOutput.prepare();
        img.copy(imageOut);
        saliencyOutput.write();

        //Update the OPC values
        for (list<Entity*>::iterator it = entities.begin(); it != entities.end(); it++)
        {
            if (presentObjects.find((*it)->name()) != presentObjects.end())
            {
                ((Object*)(*it))->m_saliency = presentObjects[(*it)->name()].o.m_saliency;
            }
        }
        opc->commit();
    }
    presentObjectsLastStep = presentObjects;
    return true;
}


/*
*   Update the salience according to visual clues
*   needs
*
*/
void PasarModule::saliencyBottomUp()
{
    ImageOf<PixelMono> * currentSaliency = saliencyInput.read(false);
    if (currentSaliency)
    {
        imageOut.copy(*currentSaliency);

        //Define ROI for every visible object
        for (map<string, ObjectModel>::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
        {
            if (it->second.o.m_present)
            {
                Vector o = it->second.o.m_ego_position;
                Vector d = it->second.o.m_dimensions;

                //Get the 3d position of every corner of the bounding box
                vector<Vector> p3dCorners;
                p3dCorners.resize(8);
                p3dCorners[0] = o;
                p3dCorners[0][0] += d[0] / 2.0;
                p3dCorners[0][1] += d[1] / 2.0;
                //p3dCorners[0][2] += d[2]/2.0;
                p3dCorners[0][2] += d[2];

                p3dCorners[1] = o;
                p3dCorners[1][0] += d[0] / 2.0;
                p3dCorners[1][1] += d[1] / 2.0;
                //p3dCorners[1][2] -= d[2]/2.0;
                p3dCorners[1][2] -= 0.0;

                p3dCorners[2] = o;
                p3dCorners[2][0] += d[0] / 2.0;
                p3dCorners[2][1] -= d[1] / 2.0;
                //p3dCorners[2][2] += d[2]/2.0;
                p3dCorners[2][2] += d[2];

                p3dCorners[3] = o;
                p3dCorners[3][0] += d[0] / 2.0;
                p3dCorners[3][1] -= d[1] / 2.0;
                //p3dCorners[3][2] -= d[2]/2.0;
                p3dCorners[3][2] -= 0.0;

                p3dCorners[4] = o;
                p3dCorners[4][0] -= d[0] / 2.0;
                p3dCorners[4][1] += d[1] / 2.0;
                //p3dCorners[4][2] += d[2]/2.0;
                p3dCorners[4][2] += d[2];

                p3dCorners[5] = o;
                p3dCorners[5][0] -= d[0] / 2.0;
                p3dCorners[5][1] += d[1] / 2.0;
                //p3dCorners[5][2] -= d[2]/2.0;
                p3dCorners[5][2] -= 0.0;

                p3dCorners[6] = o;
                p3dCorners[6][0] -= d[0] / 2.0;
                p3dCorners[6][1] -= d[1] / 2.0;
                //p3dCorners[6][2] += d[2]/2.0;
                p3dCorners[6][2] += d[2];

                p3dCorners[7] = o;
                p3dCorners[7][0] -= d[0] / 2.0;
                p3dCorners[7][1] -= d[1] / 2.0;
                //p3dCorners[7][2] -= d[2]/2.0;
                p3dCorners[7][2] -= 0.0;

                //Find the 2D bounding box
                vector<Vector> p2dCorners(8);
                Vector pxTL(2);
                pxTL = 999;
                Vector pxBR(2);
                pxBR = 0;
                for (int corner = 0; corner < 8; corner++)
                {
                    Vector px(2);
                    igaze->get2DPixel(0, p3dCorners[corner], px);
                    p2dCorners[corner].resize(2);
                    p2dCorners[corner] = px;

                    if (px[0]<pxTL[0])
                        pxTL[0] = px[0];
                    if (px[0]>pxBR[0])
                        pxBR[0] = px[0];
                    if (px[1]<pxTL[1])
                        pxTL[1] = px[1];
                    if (px[1]>pxBR[1])
                        pxBR[1] = px[1];
                }

                //Clamp
                pxTL[0] = max(0.0, pxTL[0]);
                pxTL[0] = min((double)currentSaliency->width(), pxTL[0]);
                pxTL[1] = max(0.0, pxTL[1]);
                pxTL[1] = min((double)currentSaliency->height(), pxTL[1]);
                pxBR[0] = max(0.0, pxBR[0]);
                pxBR[0] = min((double)currentSaliency->width(), pxBR[0]);
                pxBR[1] = max(0.0, pxBR[1]);
                pxBR[1] = min((double)currentSaliency->height(), pxBR[1]);

                //Get the average saliency of this ROI
                float avg = 0.0;
                int count = 0;
                for (int x = (int)pxTL[0]; x < (int)pxBR[0]; x++)
                {
                    for (int y = (int)pxTL[1]; y < (int)pxBR[1]; y++)
                    {
                        avg += currentSaliency->pixel(x, y) / 255.0;
                        count++;
                    }
                }

                //Draw stuff
                PixelRgb color;
                color.r = (unsigned char)it->second.o.m_color[0];
                color.g = (unsigned char)it->second.o.m_color[1];
                color.b = (unsigned char)it->second.o.m_color[2];
                if (pxTL[0] != 0 && pxTL[1] != 0 && pxBR[0] != 0 && pxBR[1] != 0)
                {
                    //Draw cube
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[1][0], (int)p2dCorners[1][1], (int)p2dCorners[5][0], (int)p2dCorners[5][1]);
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[5][0], (int)p2dCorners[5][1], (int)p2dCorners[7][0], (int)p2dCorners[7][1]);
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[7][0], (int)p2dCorners[7][1], (int)p2dCorners[3][0], (int)p2dCorners[3][1]);
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[3][0], (int)p2dCorners[3][1], (int)p2dCorners[1][0], (int)p2dCorners[1][1]);

                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[0][0], (int)p2dCorners[0][1], (int)p2dCorners[4][0], (int)p2dCorners[4][1]);
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[4][0], (int)p2dCorners[4][1], (int)p2dCorners[6][0], (int)p2dCorners[6][1]);
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[6][0], (int)p2dCorners[6][1], (int)p2dCorners[2][0], (int)p2dCorners[2][1]);
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[2][0], (int)p2dCorners[2][1], (int)p2dCorners[0][0], (int)p2dCorners[0][1]);

                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[0][0], (int)p2dCorners[0][1], (int)p2dCorners[1][0], (int)p2dCorners[1][1]);
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[5][0], (int)p2dCorners[5][1], (int)p2dCorners[4][0], (int)p2dCorners[4][1]);
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[7][0], (int)p2dCorners[7][1], (int)p2dCorners[6][0], (int)p2dCorners[6][1]);
                    yarp::sig::draw::addSegment(imageOut, color, (int)p2dCorners[3][0], (int)p2dCorners[3][1], (int)p2dCorners[2][0], (int)p2dCorners[2][1]);
                }
                if (count > 0)
                    it->second.o.m_saliency = avg / (double)count;
                else
                    it->second.o.m_saliency = 0.1;

            }
        }
        //saliencyNormalize();
    }
}


/*
*   Update the salience according to the informations contained in the OPC (acceleration, appareance, disappareance)
*
*/
/************************************************************************/
void PasarModule::saliencyTopDown() {
    //cout<<"TopDown Saliency"<<endl;

    //Add up the top down saliency
    for (map<string, ObjectModel >::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
    {
        if (presentObjectsLastStep.find(it->first) != presentObjectsLastStep.end() && it->first != "cursor_0" && it->first != "icub")
        {
            //Objects appears/disappears
            bool appeared, disappeared;
            appeared = it->second.o.m_present && !presentObjectsLastStep[it->first].o.m_present;
            disappeared = !it->second.o.m_present && presentObjectsLastStep[it->first].o.m_present;

            //instantaneous speed/acceleration
            Vector lastPos = presentObjectsLastStep[it->first].o.m_ego_position;
            Vector currentPos = it->second.o.m_ego_position;
            presentCurrentSpeed[it->first] = pair<double, double>(lastPos[0] - currentPos[0], lastPos[1] - currentPos[1]);

            double acceleration = sqrt(pow(presentCurrentSpeed[it->first].first - presentLastSpeed[it->first].first, 2.) + pow(presentCurrentSpeed[it->first].second - presentLastSpeed[it->first].second, 2.));

            presentObjects[it->first].acceleration = acceleration;

            //Use the world model (CONCEPTS <=> RELATIONS) to modulate the saliency
            if (appeared)
            {
                //cout<<it->second.o.name()<<" Appearance burst"<<endl;
                it->second.o.m_saliency += pTopDownAppearanceBurst;
            }

            if (disappeared)
            {
                //cout<<it->second.o.name()<<" Disappearance burst"<<endl;
                it->second.o.m_saliency += pTopDownDisappearanceBurst;
            }


            if (acceleration > thresholdMovementAccel)
            {
                it->second.o.m_saliency += pTopDownAccelerationCoef;
                //cout << "ca bouge !!! " << it->second.o.name() << " salience : " << acceleration << endl;
            }
        }
    }
}

/************************************************************************/
void PasarModule::saliencyNormalize() {
    //cout<<"Normalizing Saliency"<<endl;

    //Get the max
    map< string, ObjectModel >::iterator mostSalientObject = presentObjects.begin();

    for (map< string, ObjectModel >::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
    {
        if (it->second.o.m_saliency > mostSalientObject->second.o.m_saliency)
            mostSalientObject = it;
    }

    //Normalize
    double maxS = mostSalientObject->second.o.m_saliency;
    if (maxS == 0) maxS = 1.;
    for (map< string, ObjectModel >::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
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
    for (map< string, ObjectModel >::iterator it = presentObjects.begin(); it != presentObjects.end(); it++)
    {
        if (it->first != "")
        {
            it->second.o.m_saliency *= pExponentialDecrease;
        }
        if (it->second.o.m_saliency < thresholdSaliency)
            it->second.o.m_saliency = 0.0;
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




#include "rt2objCol.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wysiwyd::wrdac;
using namespace std;


Reactable2OPC::Reactable2OPC()
{
    H2ICUB = eye(4);
    H2RT = eye(4);
    isCalibrated = false;
    portCalibration.open("/reactable2opc/calibration:rpc");

    w = new OPCClient("reactable2opc");
    w->isVerbose = false;

    while ( !w->connect("OPC") || !Network::connect("/reactable2opc/calibration:rpc","/referenceFrameHandler/rpc"))
    {
        cout<<"Waiting for connection to OPC or rfh..."<<endl;
        Time::delay(1.0);
    }
    //

    //Load the id/name mapping from a config file
    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("reactable2opc");
    rf.setDefaultConfigFile("mapping.ini");
    rf.configure(0,NULL);
    bool invertYaxis = rf.check("invertYaxis",Value(0)).asInt() == 1;
    bool invertXaxis = rf.check("invertXaxis",Value(0)).asInt() == 1;
    if (invertYaxis)
    {
        cout<<"Using inverted Y axis"<<endl;
        YaxisFactor = -1;
    }
    else
        YaxisFactor = 1;

    if (invertXaxis)
    {
        cout<<"Using inverted X axis"<<endl;
        XaxisFactor = -1;
    }
    else
        XaxisFactor = 1;

    loadObjectsDatabase(rf);

    //Open the OSC socket for reception
    OSC_INPUT_TABLE_PORT = rf.check("OSC_TABLE_INPUT_PORT", Value(3333)).asInt();
    int oscReceiverPort = rf.check("OSC_INPUT_PORT",Value(3334)).asInt();
    oscThreadReceiver = new OscThread(w, oscReceiverPort);
    oscThreadReceiver->start();

    //Open the OSC socket for writing
    int oscEmitterPort = rf.check("OSC_OUTPUT_PORT",Value(3335)).asInt();
    string oscEmitterEndPoint = rf.check("OSC_OUTPUT_ADRESS",Value("10.0.0.106")).asString().c_str();
    oscEmitter = new UdpTransmitSocket(IpEndpointName( oscEmitterEndPoint.c_str(), oscEmitterPort ));
    yarp2osc = new DataPort(oscEmitter);
    yarp2osc->useCallback();
    yarp2osc->open("/reactable2opc/command:i");
    cout<<"Targeting OSC to" <<oscEmitterEndPoint<<":"<<oscEmitterPort<< endl;
    cout<<"OSC & TUIO listening..." << endl;

    checkCalibration();
}

Reactable2OPC::~Reactable2OPC()
{
    oscThreadReceiver->forceBreak();
    oscThreadReceiver->stop();

    w->interrupt();
    w->close();

    portCalibration.interrupt();
    portCalibration.close();

    yarp2osc->interrupt();
    yarp2osc->close();
}

void Reactable2OPC::loadObjectsDatabase(ResourceFinder& rf)
{
    int objectsCount = rf.find("objectsCount").asInt();
    for(int i=0;i<objectsCount;i++)
    {
        ostringstream oGrpName; oGrpName<<"object_"<<i;
        Bottle cObject = rf.findGroup(oGrpName.str().c_str());
        string oName = cObject.find("name").asString().c_str();
        int id = cObject.find("id").asInt();
        cout<<"Mapping ("<<id<<" , "<<oName<<")"<<endl;
        idMap[id] = oName;

        //Store the offset of the marker locally
        Bottle* bOffset = cObject.find("marker-offset").asList();
        idOffsets[id].resize(3);
        idOffsets[id][0] = bOffset->get(0).asDouble();
        idOffsets[id][1] = bOffset->get(1).asDouble();
        idOffsets[id][2] = bOffset->get(2).asDouble();

        //Push the object to OPC
        RTObject* o = w->addOrRetrieveEntity<RTObject>(oName);
        Bottle* bDim = cObject.find("dimensions").asList();
        o->m_dimensions[0] = bDim->get(0).asDouble();
        o->m_dimensions[1] = bDim->get(1).asDouble();
        o->m_dimensions[2] = bDim->get(2).asDouble();

        Bottle* bColor = cObject.find("color").asList();
        o->m_color[0] = bColor->get(0).asDouble();
        o->m_color[1] = bColor->get(1).asDouble();
        o->m_color[2] = bColor->get(2).asDouble();

        o->m_present = 0.0;
        w->commit(o);
        //cout<<o->toString()<<endl;
    }
}

bool Reactable2OPC::checkCalibration(bool forceRefresh)
{
    if (isCalibrated && !forceRefresh)
        return true;

    if (portCalibration.getOutputCount()>0)
    {
        Bottle bCmd;
        bCmd.clear();
        bCmd.addString("mat");
        bCmd.addString("reactable_both");
        bCmd.addString("icub");

        Bottle reply;
        reply.clear();
        portCalibration.write(bCmd, reply);
        if (reply.get(0).asString() == "nack")
        {
            cout<<"Transformation matrix not retrieved"<<endl;
            return false;
        }
        else
        {
            Bottle* bMat = reply.get(1).asList();
            H2ICUB.resize(4,4);
            oscThreadReceiver->H2ICUB.resize(4,4);
            for(int i=0; i<4; i++)
            {
                for(int j=0; j<4; j++)
                {
                    H2ICUB(i,j)=bMat->get(4*i+j).asDouble();
                    oscThreadReceiver->H2ICUB(i,j) = H2ICUB(i,j);
                }
            }
            cout<<"Transformation matrix retrieved"<<endl<<H2ICUB.toString(3,3).c_str()<<endl;
            oscThreadReceiver->XaxisFactor = this->XaxisFactor;
            oscThreadReceiver->YaxisFactor = this->YaxisFactor;
            isCalibrated = true;
            oscThreadReceiver->isCalibrated = true;
        }

        bCmd.clear();
        bCmd.addString("mat");
        bCmd.addString("icub");
        bCmd.addString("reactable_both");

        reply.clear();
        portCalibration.write(bCmd, reply);
        if (reply.get(0).asString() == "nack")
        {
            cout<<"Inverse Transformation matrix not retrieved"<<endl;
            return false;
        }
        else
        {
            Bottle* bMat = reply.get(1).asList();
            H2RT.resize(4,4);
            for(int i=0; i<4; i++)
            {
                for(int j=0; j<4; j++)
                {
                    H2RT(i,j)=bMat->get(4*i+j).asDouble();
                }
            }
            cout<<"Inverse Transformation matrix retrieved"<<endl<<H2RT.toString(3,3).c_str()<<endl;
            isCalibrated = true;
        }
    }
    return false;
}

void Reactable2OPC::addTuioObject(TuioObject *tobj) {

    std::cout << "add obj " << tobj->getSymbolID() << " (" << tobj->getSessionID() << ") "<< tobj->getX() << " " << tobj->getY() << " " << tobj->getAngle() << std::endl;

    if (idMap.find(tobj->getSymbolID()) == idMap.end() )
    {
        cout<<"Unknown object with ID "<<tobj->getSymbolID()<<" added"<<endl;
        return;
    }

    if (!w->isConnected())
    {
        cout<<"Not connected to OPC"<<endl;
        return;
    }

    RTObject* o = w->addOrRetrieveEntity<RTObject>(idMap[tobj->getSymbolID()]);
    //w->update(o);

    Vector rtPosition(4);
    rtPosition(0) = XaxisFactor * tobj->getX();
    rtPosition(1) = YaxisFactor * tobj->getY();
    rtPosition(2) = 0;
    rtPosition(3) = 1;

    o->m_rt_position[0] = rtPosition[0];
    o->m_rt_position[1] = rtPosition[1];
    o->m_rt_position[2] = rtPosition[2];
    cout<<o->toString()<<endl;
    if ( checkCalibration() )
    {
        Vector icubPos(4);
        icubPos = H2ICUB*rtPosition;
        o->m_ego_position[0] = icubPos[0] + idOffsets[tobj->getSymbolID()][0];
        o->m_ego_position[1] = icubPos[1] + idOffsets[tobj->getSymbolID()][1];
        o->m_ego_position[2] = icubPos[2] + idOffsets[tobj->getSymbolID()][2];
    }

    o->m_present = 1.0;
    o->m_ego_orientation[0] = 0.0;
    o->m_ego_orientation[1] = 0.0;
    o->m_ego_orientation[2] = tobj->getAngle();
    w->isVerbose = true;
    w->commit(o);
    w->isVerbose = false;
    //std::cout <<o->toString() << std::endl;
}

void Reactable2OPC::updateTuioObject(TuioObject *tobj) {
    addTuioObject(tobj);
}

void Reactable2OPC::removeTuioObject(TuioObject *tobj) {
    std::cout << "del obj " << tobj->getSymbolID() << " (" << tobj->getSessionID() << ")" << std::endl;
    if (idMap.find(tobj->getSymbolID()) == idMap.end() )
    {
        cout<<"Unknown object with ID "<<tobj->getSymbolID()<<" removed"<<endl;
        return;
    }

    if (!w->isConnected())
    {
        cout<<"Not connected to OPC"<<endl;
        return;
    }

    string objectName = idMap[tobj->getSymbolID()];
    RTObject *o = dynamic_cast<RTObject*>(w->getEntity(objectName));
    if(o) {
        o->m_present = 0.0;
        w->commit(o);
    }
}

void Reactable2OPC::addTuioCursor(TuioCursor *tcur) {
    std::cout << "add cur " << tcur->getCursorID() << " (" <<  tcur->getSessionID() << ") " << tcur->getX() << " " << tcur->getY() << std::endl;

    if (!w->isConnected())
    {
        cout<<"Not connected to OPC"<<endl;
        return;
    }
    ostringstream curName;
    curName<<"cursor_"<<tcur->getCursorID();
    RTObject* o = w->addOrRetrieveEntity<RTObject>(curName.str().c_str());
    //w->update(o);

    Vector rtPosition(4);
    rtPosition(0) = XaxisFactor * tcur->getX();
    rtPosition(1) = YaxisFactor * tcur->getY();
    rtPosition(2) = 0;
    rtPosition(3) = 1;

    o->m_rt_position[0] = rtPosition[0];
    o->m_rt_position[1] = rtPosition[1];
    o->m_rt_position[2] = rtPosition[2];

    if ( checkCalibration() )
    {
        Vector icubPos(4);
        icubPos = H2ICUB*rtPosition;
        o->m_ego_position[0] = icubPos[0];
        o->m_ego_position[1] = icubPos[1];
        o->m_ego_position[2] = icubPos[2];
    }

    o->m_present = 1.0;
    o->m_ego_orientation[0] = 0.0;
    o->m_ego_orientation[1] = 0.0;
    o->m_ego_orientation[2] = 0.0;

    o->m_dimensions[0] = 0.015;
    o->m_dimensions[1] = 0.015;
    o->m_dimensions[2] = 0.001;

    o->m_color[0] = 255;
    o->m_color[1] = 100;
    o->m_color[2] = 100;

    w->commit(o);
}

void Reactable2OPC::updateTuioCursor(TuioCursor *tcur) {
    std::cout << "set cur " << tcur->getCursorID() << " (" <<  tcur->getSessionID() << ") " << tcur->getX() << " " << tcur->getY()
              << " " << tcur->getMotionSpeed() << " " << tcur->getMotionAccel() << " " << std::endl;
    addTuioCursor(tcur);
}

void Reactable2OPC::removeTuioCursor(TuioCursor *tcur) {
    std::cout << "rem cur " << tcur->getCursorID() << " (" <<  tcur->getSessionID() << ") "<< std::endl;

    ostringstream curName;
    curName<<"cursor_"<<tcur->getCursorID();
    RTObject *o = dynamic_cast<RTObject*>(w->getEntity(curName.str()));
    if(o) {
        o->m_present = 0.0;
        w->commit(o);
    }
}

void  Reactable2OPC::refresh(TuioTime frameTime) {
    if (yarp2osc->shouldRecalibrate)
    {
        yarp2osc->shouldRecalibrate = false;
        checkCalibration(true);
    }
}

//UGLY - If anyone passes tehre and has time to loose you can move it to main.cpp
int main(int argc, char* argv[])
{
    //if (argc >= 2 && strcmp(argv[1], "-h") == 0)
    //{
    //    std::cout << "usage: Reactable2OPC [port]\n";
    //    return 0;
    //}

    //int port = 3333;
    //if (argc >= 2) port = atoi(argv[1]);

    Network yarp;
    if (!yarp.checkNetwork())
        return false;

    Reactable2OPC *dump = new Reactable2OPC();
    TuioClient client(dump->OSC_INPUT_TABLE_PORT);
    client.addTuioListener(dump);
    client.connect(true);
    delete dump;

    return 0;
}

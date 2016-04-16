#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_reactable.h"

bool wysiwyd::wrdac::SubSystem_Reactable::connect()
{
    bool success = true;
    success &= yarp::os::Network::connect(portRTrpc.getName(), "/reactable2opc/command:i");
    success &= yarp::os::Network::connect("/reactable2opc/osc:o", portRTin.getName());
    return success;
}

wysiwyd::wrdac::SubSystem_Reactable::SubSystem_Reactable(const std::string &masterName):SubSystem(masterName)
{
    portRTrpc.open( ("/" + m_masterName + "/reactable:rpc").c_str());
    portRTin.open(("/" + m_masterName + "/reactable/osc:i").c_str());
    m_type = SUBSYSTEM_REACTABLE;
}

void wysiwyd::wrdac::SubSystem_Reactable::Close()
{
    portRTrpc.interrupt();portRTrpc.close();
    portRTin.interrupt();portRTin.close();
}

void wysiwyd::wrdac::SubSystem_Reactable::SendOSC(yarp::os::Bottle &oscMsg)
{
    yarp::os::Bottle cmd;
    cmd.addString("osc");
    for(int i=0; i<oscMsg.size(); i++)
        cmd.add(oscMsg.get(i));
    std::cout<<"OSC>>"<<cmd.toString().c_str()<<std::endl;
    portRTrpc.write(cmd);
}

yarp::os::Bottle *wysiwyd::wrdac::SubSystem_Reactable::ReadOSC(bool shouldWait)
{
    yarp::os::Bottle* cmd = portRTin.read(shouldWait);
    return cmd;
}

void wysiwyd::wrdac::SubSystem_Reactable::DisplayPosition(wysiwyd::wrdac::RTObject *o, bool convertFromRobotCoordinates)
{
    yarp::os::Bottle cmd;
    if (convertFromRobotCoordinates)
        cmd.addString("addObject");
    else
        cmd.addString("sendBackObject");
    cmd.addList() = o->asBottle();
    portRTrpc.write(cmd);
}

void wysiwyd::wrdac::SubSystem_Reactable::RefreshCalibration()
{
    yarp::os::Bottle cmd;
    cmd.addString("recalibrate");
    portRTrpc.write(cmd);
}

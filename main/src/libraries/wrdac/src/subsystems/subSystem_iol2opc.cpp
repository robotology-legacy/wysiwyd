#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_iol2opc.h"

bool wysiwyd::wrdac::SubSystem_IOL2OPC::connect() {
    if(yarp::os::Network::isConnected(portRPC.getName(), "/iol2opc/rpc")) {
        return true;
    } else {
        return yarp::os::Network::connect(portRPC.getName(), "/iol2opc/rpc");
    }
}

wysiwyd::wrdac::SubSystem_IOL2OPC::SubSystem_IOL2OPC(const std::string &masterName) : SubSystem(masterName) {
    portRPC.open(("/" + m_masterName + "/iol2opc:rpc").c_str());
    m_type = SUBSYSTEM_IOL2OPC;
}

void wysiwyd::wrdac::SubSystem_IOL2OPC::Close() {
    portRPC.interrupt();
    portRPC.close();
}

bool wysiwyd::wrdac::SubSystem_IOL2OPC::changeName(const std::string &old_name, const std::string &new_name) {
    yarp::os::Bottle bReq, bResp;
    bReq.addString("change_name");
    bReq.addString(old_name);
    bReq.addString(new_name);
    portRPC.write(bReq, bResp);

    return bResp.get(0).asBool();
}

void wysiwyd::wrdac::SubSystem_IOL2OPC::pause() {
    yarp::os::Bottle bReq, bResp;
    bReq.addString("pause");
    portRPC.write(bReq, bResp);
}

void wysiwyd::wrdac::SubSystem_IOL2OPC::resume() {
    yarp::os::Bottle bReq, bResp;
    bReq.addString("resume");
    portRPC.write(bReq, bResp);
}

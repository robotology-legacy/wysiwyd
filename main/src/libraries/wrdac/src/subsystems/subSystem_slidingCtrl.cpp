#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_slidingCtrl.h"

bool wysiwyd::wrdac::SubSystem_SlidingController::connect()
{
    bool success = true;
    if(!yarp::os::Network::isConnected(idlPortL.getName(), "/slidingController/left/rpc")) {
        success &= yarp::os::Network::connect(idlPortL.getName(), "/slidingController/left/rpc");    // to be done from outside the code :)
    }
    if(!yarp::os::Network::isConnected(idlPortR.getName(), "/slidingController/right/rpc")) {
        success &= yarp::os::Network::connect(idlPortR.getName(), "/slidingController/right/rpc");  // to be done from outside the code :)
    }
    success &= clientIDL_slidingController_left->yarp().attachAsClient(idlPortL);
    success &= clientIDL_slidingController_right->yarp().attachAsClient(idlPortR);
    return success;
}

wysiwyd::wrdac::SubSystem_SlidingController::SubSystem_SlidingController(const std::string &masterName) :SubSystem(masterName)
{
    clientIDL_slidingController_left = new slidingController_IDL();
    clientIDL_slidingController_right = new slidingController_IDL();

    idlPortL.open("/" + masterName + "/slidingCtrlIDL/left:rpc");
    idlPortR.open("/" + masterName + "/slidingCtrlIDL/right:rpc");

    m_type = SUBSYSTEM_SLIDING_CONTROLLER;
}

void wysiwyd::wrdac::SubSystem_SlidingController::Close()
{
    idlPortL.interrupt();
    idlPortL.close();
    idlPortR.interrupt();
    idlPortR.close();
    delete clientIDL_slidingController_left;
    delete clientIDL_slidingController_right;
}

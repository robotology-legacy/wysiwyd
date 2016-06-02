#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_ABM.h"

bool wysiwyd::wrdac::SubSystem_ABM::connect() {
    if (yarp::os::Network::isConnected(portRPC.getName(), "/autobiographicalMemory/rpc")){
        return true;
    }
    else {
        yDebug("SubSystem_ABM: not connected, try to connect");
        bool ret = yarp::os::Network::connect(portRPC.getName(), "/autobiographicalMemory/rpc");
        if (!ret) yWarning(" connection to ABM impossible: check that ABM is running");
        return ret;
    }
}


wysiwyd::wrdac::SubSystem_ABM::SubSystem_ABM(const std::string &masterName) :SubSystem(masterName){
    portRPC.open(("/" + m_masterName + "/abm:rpc").c_str());
    m_type = SUBSYSTEM_ABM;
}


wysiwyd::wrdac::SubSystem_ABM::~SubSystem_ABM() {}


void wysiwyd::wrdac::SubSystem_ABM::Close() { portRPC.interrupt(); portRPC.close(); }


void wysiwyd::wrdac::SubSystem_ABM::sendActivity(const std::string &activityType, const std::string &activyInformation, const std::string &activityContext, const std::list<std::pair<std::string, std::string> > &arguments, bool fBegin)
{
    yarp::os::Bottle
            bMain,          // main information about the activity
            bArgument,      // Argument of the activity
            bBegin;         // information about begining or end of the activity

    bMain.addString(activityType.c_str());
    bMain.addString(activyInformation.c_str());
    bMain.addString(activityContext.c_str());

    bArgument.addString("arguments");


    for (std::list<std::pair<std::string, std::string> >::const_iterator itArg = arguments.begin(); itArg != arguments.end(); itArg++)
    {
        yarp::os::Bottle  bTemp;
        bTemp.clear();
        bTemp.addString(itArg->first);
        bTemp.addString(itArg->second);
        bArgument.addList() = bTemp;
    }

    bBegin.addString("begin");
    bBegin.addInt((int)fBegin);

    yarp::os::Bottle bSnapshot;
    bSnapshot.clear();
    bSnapshot.addString("snapshot");
    bSnapshot.addList() = bMain;
    bSnapshot.addList() = bArgument;
    bSnapshot.addList() = bBegin;
    if (connect())  portRPC.write(bSnapshot);
}


yarp::os::Bottle wysiwyd::wrdac::SubSystem_ABM::requestFromString(const std::string &sInput)
{
    yarp::os::Bottle bReplyRequest;
    //send the SQL query within a bottle to autobiographicalMemory
    yarp::os::Bottle bQuery;
    bQuery.addString("request");
    bQuery.addString(sInput.c_str());
    portRPC.write(bQuery, bReplyRequest);
    return bReplyRequest;
}


yarp::os::Bottle wysiwyd::wrdac::SubSystem_ABM::resetKnowledge()
{
    yarp::os::Bottle bReplyRequest;
    //send the SQL query within a bottle to autobiographicalMemory
    yarp::os::Bottle bQuery;
    bQuery.addString("resetKnowledge");
    portRPC.write(bQuery, bReplyRequest);
    return bReplyRequest;
}

yarp::os::Bottle wysiwyd::wrdac::SubSystem_ABM::addImgStreamProvider()
{
    yarp::os::Bottle bRequest, bReply;
    bRequest.addString("addImgStreamProvider");
    portRPC.write(bRequest, bReply);
    return bReply;
}

yarp::os::Bottle wysiwyd::wrdac::SubSystem_ABM::removeImgStreamProvider()
{
    yarp::os::Bottle bRequest, bReply;
    bRequest.addString("removeImgStreamProvider");
    portRPC.write(bRequest, bReply);
    return bReply;
}

yarp::os::Bottle wysiwyd::wrdac::SubSystem_ABM::addDataStreamProvider()
{
    yarp::os::Bottle bRequest, bReply;
    bRequest.addString("addDataStreamProvider");
    portRPC.write(bRequest, bReply);
    return bReply;
}

yarp::os::Bottle wysiwyd::wrdac::SubSystem_ABM::removeDataStreamProvider()
{
    yarp::os::Bottle bRequest, bReply;
    bRequest.addString("removeDataStreamProvider");
    portRPC.write(bRequest, bReply);
    return bReply;
}

yarp::os::Bottle wysiwyd::wrdac::SubSystem_ABM::triggerStreaming(int iCurrentInstance, bool realtime, bool includeAugmented, double speedMultiplier, std::string robot, bool blocking)
{
    yarp::os::Bottle bSend,
            bReceived;

    bSend.addString("triggerStreaming");
    bSend.addInt(iCurrentInstance);

    yarp::os::Bottle bRealtime;
    bRealtime.addString("realtime");
    bRealtime.addInt((int) realtime);
    bSend.addList() = bRealtime;

    yarp::os::Bottle bBlocking;
    bBlocking.addString("blocking");
    bBlocking.addInt((int) blocking);
    bSend.addList() = bBlocking;

    yarp::os::Bottle bAugmented;
    bAugmented.addString("includeAugmented");
    bAugmented.addInt((int) includeAugmented);
    bSend.addList() = bAugmented;

    yarp::os::Bottle bSpeedMultiplier;
    bSpeedMultiplier.addString("speedMultiplier");
    bSpeedMultiplier.addDouble(speedMultiplier);
    bSend.addList() = bSpeedMultiplier;


    yarp::os::Bottle bRobot;
    bRobot.addString("robot");
    bRobot.addString(robot);
    bSend.addList() = bRobot;

    portRPC.write(bSend, bReceived);

    return bReceived;
}


yarp::os::Bottle wysiwyd::wrdac::SubSystem_ABM::rpcCommand(yarp::os::Bottle &bRequest)
{
    yarp::os::Bottle bReply;

    yInfo() << " [rpcCommand] bRequest = " << bRequest.toString() ;
    //send the SQL query within a bottle to autobiographicalMemory
    portRPC.write(bRequest, bReply);
    return bReply;
}

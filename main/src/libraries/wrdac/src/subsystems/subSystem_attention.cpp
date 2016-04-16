#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_attention.h"

wysiwyd::wrdac::SubSystem_Attention::SubSystem_Attention(const std::string &masterName) : SubSystem(masterName)
{
    attentionSelector.open(("/"+m_masterName+"/attention:rpc").c_str());
    m_type = SUBSYSTEM_ATTENTION;
}

wysiwyd::wrdac::SubSystem_Attention::~SubSystem_Attention() {}


bool wysiwyd::wrdac::SubSystem_Attention::connect()
{
    return yarp::os::Network::connect(attentionSelector.getName(),"/attentionSelector/rpc");
}

void wysiwyd::wrdac::SubSystem_Attention::Close()
{
    attentionSelector.interrupt();
    attentionSelector.close();
}

bool wysiwyd::wrdac::SubSystem_Attention::track(const std::string &name)
{
    yarp::os::Bottle bCmd,bRep;
    bCmd.addString("track");
    bCmd.addString(name.c_str());
    if (attentionSelector.write(bCmd,bRep))
        if (bRep.get(0).asString()=="ack")
            return true;

    return false;
}

bool wysiwyd::wrdac::SubSystem_Attention::track(const int id)
{
    yarp::os::Bottle bCmd,bRep;
    bCmd.addString("track");
    bCmd.addInt(id);
    if (attentionSelector.write(bCmd,bRep))
        if (bRep.get(0).asString()=="ack")
            return true;

    return false;
}

bool wysiwyd::wrdac::SubSystem_Attention::enableAutoMode()
{
    yarp::os::Bottle bCmd,bRep;
    bCmd.addString("auto");
    if (attentionSelector.write(bCmd,bRep))
        if (bRep.get(0).asString()=="ack")
            return true;

    return false;
}

bool wysiwyd::wrdac::SubSystem_Attention::stop()
{
    yarp::os::Bottle bCmd,bRep;
    bCmd.addString("sleep");
    if (attentionSelector.write(bCmd,bRep))
        if (bRep.get(0).asString()=="ack")
            return true;

    return false;
}

bool wysiwyd::wrdac::SubSystem_Attention::getStatus(std::string &status)
{
    yarp::os::Bottle bCmd,bRep;
    bCmd.addString("stat");
    if (attentionSelector.write(bCmd,bRep))
    {
        if (bRep.get(0).asString()=="ack")
        {
            status=bRep.get(1).asString().c_str();
            return true;
        }
    }

    return false;
}

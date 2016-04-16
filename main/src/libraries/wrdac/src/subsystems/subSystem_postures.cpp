#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_postures.h"

using namespace wysiwyd::wrdac;

bool wysiwyd::wrdac::SubSystem_Postures::connect()
{
    bool success = true;
    success &= yarp::os::Network::connect(ctpHead.getName(), "/ctpservice/head/rpc");
    success &= yarp::os::Network::connect(ctpLeftArm.getName(), "/ctpservice/left_arm/rpc");
    success &= yarp::os::Network::connect(ctpRightArm.getName(), "/ctpservice/right_arm/rpc");
    success &= yarp::os::Network::connect(ctpTorso.getName(), "/ctpservice/torso/rpc");
    return success;
}

wysiwyd::wrdac::SubSystem_Postures::SubSystem_Postures(const std::string &masterName):SubSystem(masterName)
{
    ctpHead.open( ("/" + m_masterName + "/ctp/head:rpc").c_str());
    ctpLeftArm.open( ("/" + m_masterName + "/ctp/left_arm:rpc").c_str());
    ctpRightArm.open( ("/" + m_masterName + "/ctp/right_arm:rpc").c_str());
    ctpTorso.open( ("/" + m_masterName + "/ctp/torso:rpc").c_str());
    m_type = SUBSYSTEM_POSTURES;
}

void wysiwyd::wrdac::SubSystem_Postures::Close()
{
    ctpHead.interrupt();ctpHead.close();
    ctpLeftArm.interrupt();ctpLeftArm.close();
    ctpRightArm.interrupt();ctpRightArm.close();
    ctpTorso.interrupt();ctpTorso.close();
}

void wysiwyd::wrdac::SubSystem_Postures::Execute(wysiwyd::wrdac::BodyPosture &p, double timing, const std::string &partUsed)
{
    if ( partUsed == "head")
    {
        //Head
        yarp::os::Bottle cmdHead;
        cmdHead.addString("ctpq");
        cmdHead.addString("time");
        cmdHead.addDouble(timing);
        cmdHead.addString("off");
        cmdHead.addInt(0);
        cmdHead.addString("pos");
        yarp::os::Bottle& sub = cmdHead.addList();
        for(size_t i=0; i<p.head.size();i++)
        {
            sub.addDouble(p.head[i]);
        }
        ctpHead.write(cmdHead);
    }

    if ( partUsed == "left_arm")
    {
        yarp::os::Bottle cmdLeftArm;
        cmdLeftArm.addString("ctpq");
        cmdLeftArm.addString("time");
        cmdLeftArm.addDouble(timing);
        cmdLeftArm.addString("off");
        cmdLeftArm.addInt(0);
        cmdLeftArm.addString("pos");
        yarp::os::Bottle& subLA = cmdLeftArm.addList();
        for(size_t i=0; i<p.left_arm.size();i++)
        {
            subLA.addDouble(p.left_arm[i]);
        }
        ctpLeftArm.write(cmdLeftArm);
    }

    if ( partUsed == "left_hand")
    {
        yarp::os::Bottle cmdLeftArm;
        cmdLeftArm.addString("ctpq");
        cmdLeftArm.addString("time");
        cmdLeftArm.addDouble(timing);
        cmdLeftArm.addString("off");
        cmdLeftArm.addInt(7);
        cmdLeftArm.addString("pos");
        yarp::os::Bottle& subRA = cmdLeftArm.addList();
        for(size_t i=7; i<p.left_arm.size();i++)
        {
            subRA.addDouble(p.left_arm[i]);
        }
        ctpLeftArm.write(cmdLeftArm);
    }

    if ( partUsed == "right_arm")
    {
        yarp::os::Bottle cmdRightArm;
        cmdRightArm.addString("ctpq");
        cmdRightArm.addString("time");
        cmdRightArm.addDouble(timing);
        cmdRightArm.addString("off");
        cmdRightArm.addInt(0);
        cmdRightArm.addString("pos");
        yarp::os::Bottle& subRA = cmdRightArm.addList();
        for(size_t i=0; i<p.right_arm.size();i++)
        {
            subRA.addDouble(p.right_arm[i]);
        }
        ctpRightArm.write(cmdRightArm);
    }

    if ( partUsed == "right_hand")
    {
        yarp::os::Bottle cmdRightArm;
        cmdRightArm.addString("ctpq");
        cmdRightArm.addString("time");
        cmdRightArm.addDouble(timing);
        cmdRightArm.addString("off");
        cmdRightArm.addInt(7);
        cmdRightArm.addString("pos");
        yarp::os::Bottle& subRA = cmdRightArm.addList();
        for(size_t i=7; i<p.right_arm.size();i++)
        {
            subRA.addDouble(p.right_arm[i]);
        }
        ctpRightArm.write(cmdRightArm);
    }

    if ( partUsed == "torso")
    {
        yarp::os::Bottle cmdTorso;
        cmdTorso.addString("ctpq");
        cmdTorso.addString("time");
        cmdTorso.addDouble(timing);
        cmdTorso.addString("off");
        cmdTorso.addInt(0);
        cmdTorso.addString("pos");
        yarp::os::Bottle& subTors = cmdTorso.addList();
        for(size_t i=0; i<p.torso.size();i++)
        {
            subTors.addDouble(p.torso[i]);
        }
        ctpTorso.write(cmdTorso);
    }
}

void wysiwyd::wrdac::SubSystem_Postures::Execute(wysiwyd::wrdac::BodyPosture &p, double timing)
{
    Execute(p,timing,"head");
    Execute(p,timing,"left_arm");
    Execute(p,timing,"right_arm");
    Execute(p,timing,"torso");
}

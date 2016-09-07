#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_KARMA.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

void wysiwyd::wrdac::SubSystem_KARMA::appendTarget(yarp::os::Bottle &b, const yarp::sig::Vector &t)
{
    yarp::os::Bottle &sub=b;
    for (size_t i=0; i<t.length(); i++)
        sub.addDouble(t[i]);
}

void wysiwyd::wrdac::SubSystem_KARMA::appendDouble(yarp::os::Bottle &b, const double &v)
{
    yarp::os::Bottle &sub=b;
    sub.addDouble(v);
}


bool wysiwyd::wrdac::SubSystem_KARMA::sendCmd(yarp::os::Bottle &cmd, const bool disableATT)
{
    bool ret=false;

    std::string status;
    if (ATTconnected && disableATT)
    {
        SubATT->getStatus(status);
        if (status!="quiet")
            SubATT->stop();
    }

    yarp::os::Bottle bReply;
    if (rpcPort.write(cmd,bReply))
        ret=(bReply.get(0).asVocab()==yarp::os::Vocab::encode("ack"));

    if (ATTconnected && disableATT)
    {
        if (status=="auto")
            SubATT->enableAutoMode();
        else if (status!="quiet")
            SubATT->track(status);
    }

    return ret;
}

bool wysiwyd::wrdac::SubSystem_KARMA::connect()
{
    ABMconnected=SubABM->Connect();
    if (ABMconnected)
        yInfo()<<"KARMA connected to ABM";
    else
        yWarning()<<"KARMA didn't connect to ABM";

    ATTconnected=SubATT->Connect();
    if (ATTconnected)
        yInfo()<<"KARMA connected to Attention";
    else
        yWarning()<<"KARMA didn't connect to Attention";

    bool ret=true;
    ret&=yarp::os::Network::connect(stopPort.getName(),"/karmaMotor/stop:i");
    ret&=yarp::os::Network::connect(rpcPort.getName(),"/karmaMotor/rpc");

    if (yarp::os::Network::connect(visionPort.getName(),"/karmaMotor/vision:i"))
        yInfo()<<"KARMA connected to tool tip vision";
    else
        yWarning()<<"KARMA didn't connect to tool tip vision";

    if (yarp::os::Network::connect(finderPort.getName(),"/karmaMotor/finder:rpc"))
        yInfo()<<"KARMA connected to tool dimensions solver";
    else
        yWarning()<<"KARMA didn't connect to tool dimensions solver";

    openCartesianClient();

    return ret;
}

wysiwyd::wrdac::SubSystem_KARMA::SubSystem_KARMA(const std::string &masterName, const std::string &robot) : SubSystem(masterName)
{
    SubABM = new SubSystem_ABM(m_masterName+"/from_KARMA");
    SubATT = new SubSystem_Attention(m_masterName+"/from_KARMA");

    this->robot = robot;

    stopPort.open(("/" + masterName + "/" + SUBSYSTEM_KARMA + "/stop:i").c_str());
    rpcPort.open(("/" + masterName + "/" + SUBSYSTEM_KARMA + "/rpc").c_str());
    visionPort.open(("/" + masterName + "/" + SUBSYSTEM_KARMA + "/vision:i").c_str());
    finderPort.open(("/" + masterName + "/" + SUBSYSTEM_KARMA + "/finder:rpc").c_str());
    m_type = SUBSYSTEM_KARMA;
}

void wysiwyd::wrdac::SubSystem_KARMA::Close()
{
    stopPort.interrupt();
    rpcPort.interrupt();
    visionPort.interrupt();
    finderPort.interrupt();

    SubABM->Close();
    SubATT->Close();

    driverL.close();
    driverR.close();
    driverHL.close();
    driverHR.close();

    stopPort.close();
    rpcPort.close();
    visionPort.close();
    finderPort.close();
}

yarp::sig::Vector wysiwyd::wrdac::SubSystem_KARMA::applySafetyMargins(const yarp::sig::Vector &in)
{
    yarp::sig::Vector out=in;
    out[0]=std::min(out[0],-0.1);

    return out;
}

bool wysiwyd::wrdac::SubSystem_KARMA::prepare()
{
    Vector xd(3,0.0), xdL(3,0.0), xdR(3,0.0), odL(3,0.0), odR(3,0.0);
    double travelTime = 0.5;

    int contextL, contextR;
    iCartCtrlL->storeContext(&contextL);
    iCartCtrlR->storeContext(&contextR);

    iCartCtrlL->setTrajTime(travelTime);
    iCartCtrlR->setTrajTime(travelTime);

    Bottle options;
    Bottle &straightOpt=options.addList();
    straightOpt.addString("straightness");
    straightOpt.addDouble(10.0);
    iCartCtrlL->tweakSet(options);
    iCartCtrlR->tweakSet(options);

    iCartCtrlL->getPose(xd,odL);
    iCartCtrlR->getPose(xd,odR);
    xdL[0] = 0.0;
    xdL[1] = -0.3;
    xdL[2] = 0.5;

    xdR[0] = 0.0;
    xdR[1] = 0.3;
    xdR[2] = 0.5;

    iCartCtrlR->goToPose(xdR,odR,1.0);
    iCartCtrlL->goToPose(xdL,odL,1.0);

    iCartCtrlL->waitMotionDone(0.1,4.0);
    iCartCtrlR->waitMotionDone(0.1,4.0);

    iCartCtrlL->stopControl();
    iCartCtrlR->stopControl();

    iCartCtrlL->restoreContext(contextL);
    iCartCtrlL->deleteContext(contextL);

    iCartCtrlR->restoreContext(contextR);
    iCartCtrlR->deleteContext(contextR);

    return true;
}

bool wysiwyd::wrdac::SubSystem_KARMA::push(const yarp::sig::Vector &targetCenter, const double theta, const double radius, const yarp::os::Bottle &options, const std::string &sName)
{
    prepare();

    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "vector"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("push", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "push", "action", lArgument, true);
    }

    yarp::os::Bottle bCmd;
    bCmd.addVocab(yarp::os::Vocab::encode("push"));

    yarp::sig::Vector target=targetCenter;
    yarp::os::Bottle opt=options;

    target=applySafetyMargins(target);
    appendTarget(bCmd,target);
    appendDouble(bCmd,theta);
    appendDouble(bCmd,radius);
    bCmd.append(opt);

    bool bReturn = sendCmd(bCmd,true);
    std::string status;
    bReturn ? status = "success" : status = "failed";

    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "vector"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("push", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "push", "action", lArgument, false);
    }

    return bReturn;

}

bool wysiwyd::wrdac::SubSystem_KARMA::draw(const yarp::sig::Vector &targetCenter, const double theta, const double radius, const double dist, const yarp::os::Bottle &options, const std::string &sName)
{
    prepare();

    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "vector"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(dist).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("draw", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "draw", "action", lArgument, true);
    }

    yarp::os::Bottle bCmd;
    bCmd.addVocab(yarp::os::Vocab::encode("draw"));

    yarp::sig::Vector target=targetCenter;
    yarp::os::Bottle opt=options;

    target=applySafetyMargins(target);
    appendTarget(bCmd,target);
    appendDouble(bCmd,theta);
    appendDouble(bCmd,radius);
    appendDouble(bCmd,dist);
    bCmd.append(opt);

    bool bReturn = sendCmd(bCmd,true);
    std::string status;
    bReturn ? status = "success" : status = "failed";

    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "vector"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(dist).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("draw", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "draw", "action", lArgument, false);
    }

    return bReturn;
}

bool wysiwyd::wrdac::SubSystem_KARMA::vdraw(const yarp::sig::Vector &targetCenter, const double theta, const double radius, const double dist, const yarp::os::Bottle &options, const std::string &sName)
{
    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "vector"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(dist).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("virtual-draw", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "virtual-draw", "action", lArgument, true);
    }

    yarp::os::Bottle bCmd;
    bCmd.addVocab(yarp::os::Vocab::encode("vdraw"));

    yarp::sig::Vector target=targetCenter;
    yarp::os::Bottle opt=options;

    target=applySafetyMargins(target);
    appendTarget(bCmd,target);
    appendDouble(bCmd,theta);
    appendDouble(bCmd,radius);
    appendDouble(bCmd,dist);
    bCmd.append(opt);

    bool bReturn = sendCmd(bCmd,true);
    std::string status;
    bReturn ? status = "success" : status = "failed";

    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "vector"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(dist).c_str(), "double"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("virtual-draw", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "virtual-draw", "action", lArgument, false);
    }
    return bReturn;
}

bool wysiwyd::wrdac::SubSystem_KARMA::openCartesianClient()
{
    std::string name = "KARMA";

    Property optionL("(device cartesiancontrollerclient)");
    optionL.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
    optionL.put("local",("/"+name+"/cart_ctrl/left_arm").c_str());

    Property optionR("(device cartesiancontrollerclient)");
    optionR.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
    optionR.put("local",("/"+name+"/cart_ctrl/right_arm").c_str());

    Property optionHL("(device remote_controlboard)");
    optionHL.put("remote",("/"+robot+"/left_arm").c_str());
    optionHL.put("local",("/"+name+"/hand_ctrl/left_arm").c_str());

    Property optionHR("(device remote_controlboard)");
    optionHR.put("remote",("/"+robot+"/right_arm").c_str());
    optionHR.put("local",("/"+name+"/hand_ctrl/right_arm").c_str());


    if (!driverL.open(optionL))
    {
        return false;
    }
    if (!driverR.open(optionR))
    {
        driverL.close();
        return false;
    }

    if (!driverHL.open(optionHL))
    {
        driverL.close();
        driverR.close();
        return false;
    }

    if (!driverHR.open(optionHR))
    {
        driverL.close();
        driverR.close();
        driverHL.close();
        return false;
    }

    driverL.view(iCartCtrlL);
    driverR.view(iCartCtrlR);

    return true;

}

void wysiwyd::wrdac::SubSystem_KARMA::closeICartClient()
{
    driverL.close();
    driverR.close();
    driverHL.close();
    driverHR.close();
}

wysiwyd::wrdac::SubSystem_KARMA::~SubSystem_KARMA()
{
    delete SubABM;
    delete SubATT;
    iCartCtrlL->stopControl();
    iCartCtrlR->stopControl();
    delete iCartCtrlL;
    delete iCartCtrlR;
}


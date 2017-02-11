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

void wysiwyd::wrdac::SubSystem_KARMA::selectHandCorrectTarget(yarp::os::Bottle &options, const std::string &targetName,
                                                              yarp::sig::Vector &target, const std::string handToUse)
{
    std::string hand="";
    for (int i=0; i<options.size(); i++)
    {
        yarp::os::Value val=options.get(i);
        if (val.isString())
        {
            std::string item=val.asString();
            if ((item=="left") || (item=="right"))
            {
                hand=item;
                break;
            }
        }
    }

    // always choose hand
    if (hand.empty())
    {
        if (handToUse.empty())
        {
            hand=(target[1]>0.0?"right":"left");
//            options.addString(hand.c_str());
        }
        else
            hand=handToUse;
    }

    // apply 3D correction
    if (calibPort.getOutputCount()>0)
    {
        yarp::os::Bottle cmd,reply;
        cmd.addString("get_location");
        cmd.addString(hand);
        cmd.addString(targetName);
        cmd.addString("iol-"+hand);
        calibPort.write(cmd,reply);
        target[0]=reply.get(1).asDouble();
        target[1]=reply.get(2).asDouble();
        target[2]=reply.get(3).asDouble();

        Bottle opt;
        opt.addString("fixate");
        SubARE->look(target,opt,targetName);
    }

//    lastlyUsedHand=hand;
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
        yDebug()<<"KARMA didn't connect to Attention";

    AREconnected=SubARE->Connect();
    if (AREconnected)
        yInfo()<<"KARMA connected to ARE";
    else
        yWarning()<<"KARMA didn't connect to ARE";

    bool ret=true;
    if(!yarp::os::Network::isConnected(stopPort.getName(),"/karmaMotor/stop:i")) {
        ret&=yarp::os::Network::connect(stopPort.getName(),"/karmaMotor/stop:i");
    }
    if(!yarp::os::Network::isConnected(rpcPort.getName(),"/karmaMotor/rpc")) {
        ret&=yarp::os::Network::connect(rpcPort.getName(),"/karmaMotor/rpc");
    }

    if(!yarp::os::Network::isConnected(visionPort.getName(),"/karmaMotor/vision:i")) {
        if (yarp::os::Network::connect(visionPort.getName(),"/karmaMotor/vision:i"))
            yInfo()<<"KARMA connected to tool tip vision";
        else
            yWarning()<<"KARMA didn't connect to tool tip vision";
    }

    if(!yarp::os::Network::isConnected(finderPort.getName(),"/karmaMotor/finder:rpc")) {
        if (yarp::os::Network::connect(finderPort.getName(),"/karmaMotor/finder:rpc"))
            yInfo()<<"KARMA connected to tool dimensions solver";
        else
            yWarning()<<"KARMA didn't connect to tool dimensions solver";
    }

    if(!yarp::os::Network::isConnected(calibPort.getName(),"/iolReachingCalibration/rpc")) {
        if (yarp::os::Network::connect(calibPort.getName(),"/iolReachingCalibration/rpc"))
            yInfo()<<"KARMA connected to calibrator";
        else
            yWarning()<<"KARMA didn't connect to calibrator";
    }

    openCartesianClient();

    if (AREconnected)
    {
        hasTable = SubARE->getTableHeight(tableHeight);
        if (hasTable)
            yInfo("[SubSystem_KARMA] table height: %f",tableHeight);
        else
            yWarning("[SubSystem_KARMA] no table object");
    }
    else
    {
        tableHeight = 0.0;
        yWarning("[SubSystem_KARMA] not connected to ARE");
    }

    return ret;
}

wysiwyd::wrdac::SubSystem_KARMA::SubSystem_KARMA(const std::string &masterName, const std::string &robot) : SubSystem(masterName)
{
    SubABM = new SubSystem_ABM(m_masterName+"/from_KARMA");
    SubATT = new SubSystem_Attention(m_masterName+"/from_KARMA");
    SubARE = new SubSystem_ARE(m_masterName+"/from_KARMA");

    this->robot = robot;

    stopPort.open(("/" + masterName + "/" + SUBSYSTEM_KARMA + "/stop:i").c_str());
    rpcPort.open(("/" + masterName + "/" + SUBSYSTEM_KARMA + "/rpc").c_str());
    visionPort.open(("/" + masterName + "/" + SUBSYSTEM_KARMA + "/vision:i").c_str());
    finderPort.open(("/" + masterName + "/" + SUBSYSTEM_KARMA + "/finder:rpc").c_str());
    calibPort.open(("/" + masterName + "/" + SUBSYSTEM_KARMA + "/calib:io").c_str());
    m_type = SUBSYSTEM_KARMA;
}

void wysiwyd::wrdac::SubSystem_KARMA::Close()
{
    stopPort.interrupt();
    rpcPort.interrupt();
    visionPort.interrupt();
    finderPort.interrupt();
    calibPort.interrupt();

    SubABM->Close();
    SubATT->Close();
    SubARE->Close();

    driverL.close();
    driverR.close();
    driverHL.close();
    driverHR.close();

    stopPort.close();
    rpcPort.close();
    visionPort.close();
    finderPort.close();
    calibPort.close();
}

yarp::sig::Vector wysiwyd::wrdac::SubSystem_KARMA::applySafetyMargins(const yarp::sig::Vector &in)
{
    yarp::sig::Vector out=in;
    out[0]=std::min(out[0],-0.1);

    return out;
}

bool wysiwyd::wrdac::SubSystem_KARMA::returnArmSafely(std::string armType)
{
    Vector xL(3,0.0), xR(3,0.0), xdL(3,0.0), xdR(3,0.0), odL(3,0.0), odR(3,0.0);
    double travelTime = 2.0;

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

    iCartCtrlL->getPose(xL,odL);
    iCartCtrlR->getPose(xR,odR);

    if (armType == "selectable")
    {
        if (xL[2]<xR[2])
            armType = "left";
        else
            armType = "right";
    }
    yInfo("[SubSystem_KARMA] return %s arm before home",armType.c_str());
    if (armType == "left")
    {
        xdL = xL;
        xdL[2] = std::min(0.05,xL[2] + 0.1);
        yInfo("[SubSystem_KARMA] xdL = %s",xdL.toString().c_str());
        iCartCtrlL->goToPose(xdL,odL,1.0);
        iCartCtrlL->waitMotionDone(0.1,4.0);
        iCartCtrlL->stopControl();
        iCartCtrlL->restoreContext(contextL);
        iCartCtrlL->deleteContext(contextL);
    }
    else if (armType == "right")
    {
        xdR = xR;
        xdR[2] = std::min(0.05,xR[2] + 0.1);
        yInfo("[SubSystem_KARMA] xdR = %s",xdR.toString().c_str());
        iCartCtrlR->goToPose(xdR,odR,1.0);
        iCartCtrlR->waitMotionDone(0.1,4.0);
        iCartCtrlR->stopControl();
        iCartCtrlR->restoreContext(contextR);
        iCartCtrlR->deleteContext(contextR);
    }

    return true;
}

bool wysiwyd::wrdac::SubSystem_KARMA::chooseArm(const std::string &armType)
{
    Vector dimTool(3,0.0);
    yarp::os::Bottle bCmd;
    bCmd.addVocab(yarp::os::Vocab::encode("tool"));
    bCmd.addVocab(yarp::os::Vocab::encode("attach"));
    bCmd.addString(armType);
    appendTarget(bCmd,dimTool);

    bool bReturn = sendCmd(bCmd,true);
    std::string status;
    bReturn ? status = "success" : status = "failed";

    return bReturn;
}

void wysiwyd::wrdac::SubSystem_KARMA::chooseArmAuto()
{
    yarp::os::Bottle bCmd;
    bCmd.addVocab(yarp::os::Vocab::encode("tool"));
    bCmd.addVocab(yarp::os::Vocab::encode("remove"));


    sendCmd(bCmd,true);
}

bool wysiwyd::wrdac::SubSystem_KARMA::pushAside(const std::string &objName,
                                                const yarp::sig::Vector &objCenter, const double &targetPosY,
                                                const double &theta,
                                                const std::string &armType,
                                                const yarp::os::Bottle &options, const std::string &sName)
{
    // Calculate the pushing distance (radius) for push with Karma
    Vector object = objCenter;
    Bottle opt = options;
    double zOffset = 0.05;
    double actionOffset = 0.1;              // add 10cm offset to deal with object dimension
    yInfo ("[subSystem_KARMA] object center OPC  : %s",object.toString().c_str());
    selectHandCorrectTarget(opt,objName,object);    // target is calibrated by this method
    yInfo ("[subSystem_KARMA] object center calib: %s",object.toString().c_str());
    double radius = fabs(object[1] - targetPosY);
    yInfo ("objectY = %f",object[1]);
    yInfo ("targetPosYRight = %f",targetPosY);
    yInfo ("radius = %f",radius);
    Vector targetCenter = object;
    targetCenter[1] = targetPosY;

    if (hasTable)
        targetCenter[2] = tableHeight+zOffset;

    yInfo ("object height = %f",targetCenter[2]);

    // Choose arm
    bool armChoose = false;
    if (armType =="right" || armType == "left")
        armChoose = chooseArm(armType);

    // Call push (no calibration)
    bool pushSucceed = push(targetCenter,theta,radius + actionOffset,options,sName);

    //if (pushSucceed)
    //    returnArmSafely(armType);

    if (armChoose)
        chooseArmAuto();

    return pushSucceed;
}

bool wysiwyd::wrdac::SubSystem_KARMA::pushFront(const std::string &objName,
                                                const yarp::sig::Vector &objCenter, const double &targetPosXFront,
                                                const std::string &armType,
                                                const yarp::os::Bottle &options, const std::string &sName)
{
    // Calculate the pushing distance (radius) for push with Karma
    Vector object = objCenter;
    Bottle opt = options;
    double zOffset = 0.1;
    double actionOffset = 0.1;              // add 10cm offset to deal with object dimension
    yInfo ("[subSystem_KARMA] object center OPC  : %s",object.toString().c_str());
    selectHandCorrectTarget(opt,objName,object);    // target is calibrated by this method
    yInfo ("[subSystem_KARMA] object center calib: %s",object.toString().c_str());
    double radius = fabs(object[0] - targetPosXFront);
    yInfo ("objectX = %f",object[0]);
    yInfo ("targetPosXFront = %f",targetPosXFront);
    yInfo ("radius = %f",radius);
    Vector targetCenter = object;
    targetCenter[0] = targetPosXFront;

    if (hasTable)
        targetCenter[2] = tableHeight+zOffset;

    yInfo ("object height = %f",targetCenter[2]);

    // Choose arm
    bool armChoose = false;
    if (armType =="right" || armType == "left")
        armChoose = chooseArm(armType);

    // Call push (no calibration)
    bool pushSucceed = push(targetCenter,-90,radius + actionOffset,options,sName);

    //if (pushSucceed)
    //    returnArmSafely(armType);

    if (armChoose)
        chooseArmAuto();

    return pushSucceed;
}

bool wysiwyd::wrdac::SubSystem_KARMA::push(const yarp::sig::Vector &targetCenter,
                                           const double theta, const double radius,
                                           const yarp::os::Bottle &options, const std::string &sName)
{
    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "targetCenter"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "theta"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "radius"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("push", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "karmapush", "action", lArgument, true);
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
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "targetCenter"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "theta"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "radius"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("push", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "karmapush", "action", lArgument, false);
    }

    return bReturn;

}

bool wysiwyd::wrdac::SubSystem_KARMA::pullBack(const std::string &objName,
                                               const yarp::sig::Vector &objCenter, const double &targetPosXBack,
                                               const std::string &armType,
                                               const yarp::os::Bottle &options, const std::string &sName)
{
    // Calculate the pulling distance (dist) for pull with Karma
    Vector object = objCenter;
    Bottle opt = options;
    double zOffset = -0.04;
    double actionOffset = 0.05;                     // add 5cm offset to deal with object dimension
    yInfo ("[subSystem_KARMA] object center OPC  : %s",object.toString().c_str());
    selectHandCorrectTarget(opt,objName,object);            // target is calibrated by this method
    yInfo ("[subSystem_KARMA] object center calib: %s",object.toString().c_str());
    double dist = fabs(object[0] - targetPosXBack); // dist in pulling ~ radius in pushing; radius in pulling ~ radius in pushing
    yInfo ("objectX = %f",object[0]);
    yInfo ("targetPosXBack = %f",targetPosXBack);
    yInfo ("dist = %f",dist);
    Vector targetCenter = object;

    if (hasTable)
        targetCenter[2] = std::max(tableHeight,targetCenter[2]);
    targetCenter[2] += zOffset;

    yInfo ("object height = %f",targetCenter[2]);

    // Choose arm
    bool armChoose = false;
    if (armType =="right" || armType == "left")
        armChoose = chooseArm(armType);

    // Call draw (no calibration)
    bool drawSucceed = draw(targetCenter,90,actionOffset,dist + actionOffset,options,sName);

    //if (drawSucceed)
    //    returnArmSafely(armType);

    if (armChoose)
        chooseArmAuto();

    return drawSucceed;
}

bool wysiwyd::wrdac::SubSystem_KARMA::draw(const yarp::sig::Vector &targetCenter,
                                           const double theta, const double radius,
                                           const double dist,
                                           const yarp::os::Bottle &options, const std::string &sName)
{
    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "targetCenter"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "theta"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "radius"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(dist).c_str(), "dist"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("pull", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "karmapull", "action", lArgument, true);
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
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "targetCenter"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "theta"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "radius"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(dist).c_str(), "dist"));
        lArgument.push_back(std::pair<std::string, std::string>(options.toString().c_str(), "options"));
        lArgument.push_back(std::pair<std::string, std::string>("pull", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "object"));
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
        lArgument.push_back(std::pair<std::string, std::string>(status, "status"));
        lArgument.push_back(std::pair<std::string, std::string>("KARMA", "subsystem"));
        SubABM->sendActivity("action", "karmapull", "action", lArgument, false);
    }

    return bReturn;
}

bool wysiwyd::wrdac::SubSystem_KARMA::vdraw(const std::string &targetName,
                                            const yarp::sig::Vector &targetCenter,
                                            const double theta, const double radius,
                                            const double dist,
                                            const yarp::os::Bottle &options, const std::string &sName)
{
    if (ABMconnected)
    {
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "targetCenter"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "theta"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "radius"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(dist).c_str(), "dist"));
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
    selectHandCorrectTarget(opt,targetName,target);
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
        lArgument.push_back(std::pair<std::string, std::string>(targetCenter.toString().c_str(), "targetCenter"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(theta).c_str(), "theta"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(radius).c_str(), "radius"));
        lArgument.push_back(std::pair<std::string, std::string>(std::to_string(dist).c_str(), "dist"));
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

wysiwyd::wrdac::SubSystem_KARMA::~SubSystem_KARMA()
{
    delete SubABM;
    delete SubATT;
    delete SubARE;
    driverL.close();
    driverR.close();
    driverHL.close();
    driverHR.close();
}

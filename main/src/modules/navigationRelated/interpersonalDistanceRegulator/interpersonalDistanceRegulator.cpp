#include "interpersonalDistanceRegulator.h"

bool InterpersonalDistanceRegulator::configure(yarp::os::ResourceFinder &rf)
{
    isPaused = false;
    isQuitting = false;

    string moduleName = rf.check("name",Value("interpersonalDistanceRegulator")).asString().c_str();
    setName(moduleName.c_str());

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();
    preferedDistanceToPeople = rf.check("preferedDistanceToPeople",Value(0.9)).asDouble();
    fwdSpeed = rf.check("forwardSpeed", Value(0.075)).asDouble();
    backwdSpeed = rf.check("backwardSpeed", Value(0.075)).asDouble();
    turningSpeed = rf.check("angularSpeed", Value(0.075)).asDouble();

    ikart = new SubSystem_iKart(moduleName);
    while (!ikart->Connect())
    {
        cout << "Connecting IDR to iKart..." << endl;
        Time::delay(1.0);
    }

    opc = new OPCClient(moduleName);
    while (!opc->connect("OPC"))
    {
        cout << "Connecting InterpersonalThread to OPC..." << endl;
        Time::delay(1.0);
    }
    cout<<"Configuration done."<<endl;

    rpc.open ( ("/"+moduleName+"/rpc").c_str());
    attach(rpc);
    return true;
}

bool InterpersonalDistanceRegulator::updateModule()
{
    if (isQuitting)
        return false;

    if (!isPaused)
    {
        partner = opc->addOrRetrieveAgent("partner");

        if (partner->m_present)
        {
            yarp::sig::Vector p = partner->m_ego_position;
            double linearSpeed = 0.0;
            double angularSpeed = 0.0;

            double angularRange = 0.2;
            if (p[1]<-angularRange / 2.0)
                angularSpeed = -turningSpeed;
            else if (p[1]>angularRange / 2.0)
                angularSpeed = turningSpeed;

            cout << "[interpersonalThread] Position of partner is : " << p.toString(3, 3).c_str() << endl;
            double range = 0.1;
            if (-p[0]<preferedDistanceToPeople - range / 2.0)
                linearSpeed = -backwdSpeed;
            else if (-p[0]>preferedDistanceToPeople + range / 2.0)
                linearSpeed = fwdSpeed;

            ikart->rawCommand(linearSpeed, angularSpeed);
        }
    }
    return true;
}

bool InterpersonalDistanceRegulator::respond(const Bottle& cmd, Bottle& reply)
{
    reply.addString("NACK");
    return true;
}

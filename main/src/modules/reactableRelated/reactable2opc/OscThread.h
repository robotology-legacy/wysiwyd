#ifndef INCLUDED_OSCTHREAD_H
#define INCLUDED_OSCTHREAD_H

#include <osc/OscReceivedElements.h>
#include <osc/OscPacketListener.h>

#include <ip/UdpSocket.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include <yarp/os/Thread.h>
#include <yarp/math/Math.h>
#include <wrdac/helpers.h>
#include <iostream>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace yarp::math;

class OscThread: public yarp::os::Thread, public osc::OscPacketListener
{
    int port;
    UdpListeningReceiveSocket *s;
    OPCClient *opc;
    Port oscFwding;

public:
    Matrix          H2ICUB;
    bool            isCalibrated;
    int         YaxisFactor;
    int         XaxisFactor;

    OscThread(OPCClient * _opc, int _port = 7000);
    void forceBreak();

protected:
    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    virtual void ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint);

};

#endif

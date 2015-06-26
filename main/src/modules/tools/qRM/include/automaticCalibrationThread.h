#include "wrdac/clients/icubClient.h"
#include <time.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;


class AutomaticCalibrationThread: public RateThread
{
    OPCClient* opc; 
    RTObject* cursor;
    BufferedPort<Bottle> cartesianPort;
    yarp::os::Port rfhPort;
    vector< Vector > iCubPos;
    vector< Vector > rtPos;
    string scalingMode;

public:

    AutomaticCalibrationThread(int period, string _scalingMode = "ical"):RateThread(period)
    {
        scalingMode = _scalingMode;
        cartesianPort.open("/automaticCalibrationThread/cartesian:i");
        rfhPort.open("/automaticCalibrationThread/rfh:rpc");

        opc = new OPCClient("automaticCalibrationThread");
        while(!opc->connect("OPC"))
        {
            cout<<"Connecting CalibrationThread to OPC..."<<endl;
            Time::delay(1.0);
        }     

        int attemptCount = 0;
        while(attemptCount<5&&!Network::connect("/icub/cartesianController/right_arm/state:o",cartesianPort.getName()))
        {
            cout<<"Connecting CalibrationThread to cartesian status..."<<endl;
            Time::delay(1.0);
            attemptCount++;
        }    
        while(!Network::connect(rfhPort.getName(), "/referenceFrameHandler/rpc"))
        {
            cout<<"Connecting InterpersonalThread to RFH..."<<endl;
            Time::delay(1.0);
        }
        cursor = opc->addRTObject("cursor_0");
    }

    void threadRelease()
    {
        opc->interrupt();
        opc->close();
    }

    virtual void run()
    {
        opc->update(cursor);

        if (cursor->m_present)
        {
            cout<<"Trying to acquire a calibration point..."<<endl;
            Bottle* bPos = cartesianPort.read(false);
            if(bPos)
            {
                Vector tmp(3,0.0);
                tmp[0]= bPos->get(0).asDouble();
                tmp[1]= bPos->get(1).asDouble();
                tmp[3]= bPos->get(2).asDouble();
                rtPos.push_back(cursor->m_rt_position);
                iCubPos.push_back(tmp);
                cout<<"Got "<<iCubPos.size()<<" pairs !"<<endl;
            }
        }
    }
    void clear()
    {
        rtPos.clear();
        iCubPos.clear();
    }

    void updateCalibration()
    {
        Bottle clrBottle;
        clrBottle.addString("clear");
        rfhPort.write(clrBottle);

        double t0 = Time::now();
        for(unsigned int i=0; i<rtPos.size(); i+=3)
        {
            Bottle botRPH, botRPHRep;
            botRPH.addString("add");
            botRPH.addString("reactable_both");
            Bottle &cooTable = botRPH.addList();
            cooTable.addDouble(rtPos[i][0]);
            cooTable.addDouble(rtPos[i][1]);
            cooTable.addDouble(rtPos[i][2]);
            Bottle &cooiCub = botRPH.addList();
            cooiCub.addDouble(iCubPos[i][0]);
            cooiCub.addDouble(iCubPos[i][1]);
            cooiCub.addDouble(iCubPos[i][2]);
            rfhPort.write(botRPH,botRPHRep);
            cout<<"Sent to RFH: "<<botRPH.toString().c_str()<<endl;
            cout<<"Got from RFH: "<<botRPHRep.toString().c_str()<<endl;
        }

        //Get the calibration matrix
        Bottle calibBottle, calibReply;
        calibBottle.addString(scalingMode);
        calibBottle.addString("reactable_both");
        rfhPort.write(calibBottle,calibReply);
        cout<<"Calibrated ! "<<calibReply.toString().c_str()<<endl;

        calibBottle.clear();
        calibBottle.addString("save");
        rfhPort.write(calibBottle,calibReply);
        cout<<"Saved to file ! "<<calibReply.toString().c_str()<<endl;

        double t1 = Time::now();
        cout<<"Calibration time : "<<t1-t0<<endl;

        clrBottle.clear();
        clrBottle.addString("clear");
        rfhPort.write(clrBottle);
    }
};

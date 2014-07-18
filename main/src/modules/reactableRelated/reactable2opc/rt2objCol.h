#ifndef INCLUDED_Reactable2OPC_H
#define INCLUDED_Reactable2OPC_H

#include "TuioListener.h"
#include "TuioClient.h"

#include <math.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <map>
#include <ostream>
#include <wrdac/helpers.h>

//OSC Layer
#include "OscThread.h"
#include "osc/OscOutboundPacketStream.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wysiwyd::wrdac;
using namespace std;

using namespace TUIO;

Matrix	    H2ICUB;
Matrix		H2RT;

class DataPort : public BufferedPort<Bottle> {
UdpTransmitSocket* osc;

public:
bool shouldRecalibrate;

	DataPort(UdpTransmitSocket* _osc)
{
	osc = _osc;
	shouldRecalibrate = false;
}
     virtual void onRead(Bottle& cmdBot) {
	
	cout<<"Catched command."<<cmdBot.toString().c_str()<<endl;
	string cmdWord = cmdBot.get(0).asString().c_str();
		if (cmdWord == "recalibrate")
		{
			shouldRecalibrate = true;
		}

		if ( cmdWord == "osc")
		{
			cout<<"Catched a raw osc command. Forwarding to OSC: "<<cmdBot.toString().c_str()<<endl;
			char buffer[255];
			osc::OutboundPacketStream p( buffer, 255 );
			p << osc::BeginBundleImmediate;
			p << osc::BeginMessage( cmdBot.get(1).asString().c_str() ) ;
			for(int i=2; i<cmdBot.size();i++)
			{
				if (cmdBot.get(i).isString())
					p << cmdBot.get(i).asString().c_str() ;
				else if (cmdBot.get(i).isInt())
					p << cmdBot.get(i).asInt() ;
				else if (cmdBot.get(i).isDouble())
					p << (float)cmdBot.get(i).asDouble() ;

			}
			p << osc::EndMessage;
			p << osc::EndBundle;
			osc->Send( p.Data(), p.Size() );
		}

		if (cmdWord == "addObject")
		{
			//if (H2RT != NULL)
			{
				RTObject o;
				o.fromBottle(*cmdBot.get(1).asList());

				cout<<"Get virtual object in iCub coordinates. Forwarding to OSC: "<<o.toString().c_str()<<endl;
                Vector rtPosTmp(4);
                Vector icubPosTmp(4);	
    			icubPosTmp(0) = o.m_ego_position(0);
    			icubPosTmp(1) = o.m_ego_position(1);
    			icubPosTmp(2) = o.m_ego_position(2);
    			icubPosTmp(3) = 1.0;
                rtPosTmp = H2RT*icubPosTmp;

                o.m_rt_position(0) = rtPosTmp(0);
                o.m_rt_position(1) = rtPosTmp(1);
                o.m_rt_position(2) = rtPosTmp(2);

				char buffer[255];
				osc::OutboundPacketStream p( buffer, 255 );
				p << osc::BeginBundleImmediate;
				p << osc::BeginMessage( "/object" );
				p << "add";
				p << o.name().c_str();
				p << (float)o.m_rt_position[0];
				p << (float)o.m_rt_position[1];
				p << (float)o.m_dimensions[0];
				p << (float)o.m_dimensions[1];
				p << (int)o.m_color[0];
				p << (int)o.m_color[1];
				p << (int)o.m_color[2];
				p << osc::EndMessage;
				p << osc::EndBundle;
				osc->Send( p.Data(), p.Size() );
			}
		}  

		if (cmdWord == "sendBackObject")
		{
			//if (H2RT != NULL)
			{
				RTObject o;
				o.fromBottle(*cmdBot.get(1).asList());

				cout<<"Get virtual object in RT coordinates. Forwarding to OSC: "<<o.toString().c_str()<<endl;

				char buffer[255];
				osc::OutboundPacketStream p( buffer, 255 );
				p << osc::BeginBundleImmediate;
				p << osc::BeginMessage( "/object" );
				p << "add";
				p << o.name().c_str();
				p << (float)o.m_rt_position[0];
				p << (float)o.m_rt_position[1];
				p << (float)o.m_dimensions[0];
				p << (float)o.m_dimensions[1];
				p << (int)o.m_color[0];
				p << (int)o.m_color[1];
				p << (int)o.m_color[2];
				p << osc::EndMessage;
				p << osc::EndBundle;
				osc->Send( p.Data(), p.Size() );
			}
		} 
     }
};

class Reactable2OPC : public TuioListener {

	OscThread	*oscThreadReceiver;
	UdpTransmitSocket *oscEmitter;
	DataPort*	  yarp2osc;

	OPCClient			*w;
	Port	  			portCalibration;
	map<int, string> 	idMap;
	map<int, string> 	idMapCur;
	map<int, Vector> 	idOffsets;

	bool		isCalibrated;
	int			YaxisFactor;
	int			XaxisFactor;


public:
		int	OSC_INPUT_TABLE_PORT;

		Reactable2OPC();
		~Reactable2OPC();

		//TODO: Load objects names / dimensions / association with fiducial ID from a conf file
		void loadObjectsDatabase(ResourceFinder &rf);

		bool checkCalibration(bool forceRefresh = false);

		void addTuioObject(TuioObject *tobj);
		void updateTuioObject(TuioObject *tobj);
		void removeTuioObject(TuioObject *tobj);

		void addTuioCursor(TuioCursor *tcur);
		void updateTuioCursor(TuioCursor *tcur);
		void removeTuioCursor(TuioCursor *tcur);

		void refresh(TuioTime frameTime);
};

#endif /* INCLUDED_Reactable2OPC_H */

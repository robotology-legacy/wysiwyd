#include "OscThread.h"

OscThread::OscThread(OPCClient * _opc, int _port)
{
	opc = _opc;
	port = _port;
}

bool OscThread::threadInit()
{
	cout<<"Opening OSC listener on port " << port <<endl;
	s = new UdpListeningReceiveSocket(
		IpEndpointName( IpEndpointName::ANY_ADDRESS, port ),
        this );
	isCalibrated	 = false;
    oscFwding.open("/reactable2opc/osc:o");
	return true;
}

void OscThread::run()
{
	//while(!this->isStopping())
	//{
		s->RunUntilSigInt();
	//}
}

void OscThread::forceBreak()
{
	s->AsynchronousBreak();
}

void OscThread::threadRelease()
{
	s->AsynchronousBreak();
	delete s;
}

void OscThread::ProcessMessage( const osc::ReceivedMessage& m, 
			const IpEndpointName& remoteEndpoint )
{
    (void) remoteEndpoint; // suppress unused parameter warning

    try{
            string keyword = m.AddressPattern();
        if( keyword == "/object")
		{
		cout<<"Received virtual object from osc"<<endl;
			//if (H2ICUB != NULL)

				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				const char *type = (arg++)->AsString();
				const char *name = (arg++)->AsString();
            			string typeword = type;
				if (typeword == "add")
				{
					float x = (arg++)->AsFloat();
					float y = (arg++)->AsFloat();
					float dimx = (arg++)->AsFloat();
					float dimy = (arg++)->AsFloat();
					int r = (arg++)->AsInt32();
					int g = (arg++)->AsInt32();
					int b = (arg++)->AsInt32();
					cout<<"Adding object "<<name<<endl;

					if (!opc->isConnected())	
					{
						cout<<"Not connected to OPC"<<endl;
						return;
					}
					if (!isCalibrated)
					{
						cout<<"Not calibrated"<<endl;
						return;
					}

                    RTObject* o = opc->addOrRetrieveEntity<RTObject>(name);
					opc->update(o);

					Vector rtPosition(4);
					rtPosition(0) = XaxisFactor * x;
					rtPosition(1) = YaxisFactor * y;
					rtPosition(2) = 0;
					rtPosition(3) = 1;

					o->m_rt_position[0] = rtPosition[0];
					o->m_rt_position[1] = rtPosition[1];
					o->m_rt_position[2] = rtPosition[2];
					cout<<o->toString()<<endl;

					Vector icubPos(4);
					icubPos = H2ICUB * rtPosition;
					o->m_ego_position[0] = icubPos[0];//+ idOffsets[tobj->getSymbolID()][0];
					o->m_ego_position[1] = icubPos[1];//+ idOffsets[tobj->getSymbolID()][1];
					o->m_ego_position[2] = icubPos[2];//+ idOffsets[tobj->getSymbolID()][2];

                    o->m_dimensions[0] = dimx;
                    o->m_dimensions[1] = dimy;
                    o->m_dimensions[2] = 0.01;

					o->m_present = true;
					o->m_ego_orientation[0] = 0.0;
					o->m_ego_orientation[1] = 0.0;
					o->m_ego_orientation[2] = 0.0;//tobj->getAngle();

					o->m_color[0] = r;
					o->m_color[1] = g;
					o->m_color[2] = b;//tobj->getAngle();

					//opc->isVerbose = true;
					opc->commit(o);
					//opc->isVerbose = false;
			cout<<"Added"<<endl;
				}
				if (typeword == "remove")
				{
					if (!opc->isConnected())
					{
						cout<<"Not connected to OPC"<<endl;
						return;
					}
			cout<<"Removing object"<<endl;
            RTObject *o = dynamic_cast<RTObject*>(opc->getEntity(name));
                    if(o) {
                        o->m_present = false;
                        //opc->isVerbose = true;
                        opc->commit(o);
                        //opc->isVerbose = false;
			cout<<"Removed"<<endl;
                    }
				}

        	}
		else if( keyword == "/event")
		{  
              osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
                const char *subject = (arg++)->AsString();
                const char *verb = (arg++)->AsString();
                const char *cobj = "none";
                const char *cplace = "none";
                const char *ctime = "none";
                const char *cmanner = "none";
		double timeLife = 5.0;
                if (arg != m.ArgumentsEnd())
                    cobj = (arg++)->AsString();
                if (arg != m.ArgumentsEnd())
                    cplace = (arg++)->AsString();
                if (arg != m.ArgumentsEnd())
                    ctime = (arg++)->AsString();
                if (arg != m.ArgumentsEnd())
		    cmanner = (arg++)->AsString();
		if (arg != m.ArgumentsEnd())
		    timeLife = (arg++)->AsFloat();

                Relation r(subject,verb,cobj,cplace,ctime,cmanner);
		cout<<"Received relation (/Event) from OSC "<<r.toString()<< "with a lifetime of "<<timeLife<< "Trying to add : ";
		opc->addRelation(r, timeLife);
		cout<<endl;
          }
		else if( keyword == "/revent")
		{
              osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
                const char *subject = (arg++)->AsString();
                const char *verb = (arg++)->AsString();
                const char *cobj = "none";
                const char *cplace = "none";
                const char *ctime = "none";
                const char *cmanner = "none";
		double timeLife = 5.0;
                if (arg != m.ArgumentsEnd())
                    cobj = (arg++)->AsString();
                if (arg != m.ArgumentsEnd())
                    cplace = (arg++)->AsString();
                if (arg != m.ArgumentsEnd())
                    ctime = (arg++)->AsString();
                if (arg != m.ArgumentsEnd())
		    cmanner = (arg++)->AsString();
		if (arg != m.ArgumentsEnd())
		    timeLife = (arg++)->AsFloat();

                Relation r(subject,verb,cobj,cplace,ctime,cmanner);
		cout<<"Received relation remove request (/revent) from OSC "<<r.toString()<< "Trying to remove : ";
		opc->removeRelation(r);
		cout<<endl;
          } 
          else if ( keyword == "/bottle")
          {
            Bottle bFwd;
			osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
			const char *fwdMsg = (arg++)->AsString();
            cout<<"OSC input (bottle forwarding) : "<<fwdMsg<<endl;
			bFwd.addString(fwdMsg);
			oscFwding.write(bFwd);
          }
		  else
          {
            cout<<"OSC input : Unknown format"<<endl;
          }
    }catch( osc::Exception& e ){

        std::cout << "error while parsing message: "
            << m.AddressPattern() << ": " << e.what() << "\n";
    }
}

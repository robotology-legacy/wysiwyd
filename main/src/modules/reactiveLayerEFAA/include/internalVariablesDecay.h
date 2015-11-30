#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <wrdac/helpers.h>

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

class InternalVariablesDecay: public RateThread
{
    OPCClient *opc;
    list<Entity*> presentAgents;
    double lastUpdateTimer;
    double emotionalDecay;

    void retrievePresentAgents()
    {
        Bottle condition;

        Bottle isAgent;
        isAgent.addString(EFAA_OPC_ENTITY_TAG);
        isAgent.addString("==");
        isAgent.addString(EFAA_OPC_ENTITY_AGENT);

        Bottle isPresent;
        isPresent.addString(EFAA_OPC_OBJECT_PRESENT_TAG);
        isPresent.addString("==");
        isPresent.addInt(1);

        condition.addList() = isAgent;
        condition.addString("&&");
        condition.addList() = isPresent;        

        string debug = condition.toString().c_str();
        presentAgents = opc->Entities(condition);
    }

public:


    InternalVariablesDecay(int period, double _emotionalDecay = 0.1):RateThread(period)
    {
        opc = new OPCClient("InternalVariablesDecay");
        while(!opc->connect("OPC"))
        {
            cout<<"Connecting InternalVariablesDecay to OPC..."<<endl;
            Time::delay(1.0);
        }
        retrievePresentAgents();
        emotionalDecay = _emotionalDecay;
        lastUpdateTimer = Time::now();
    }

    virtual void run()
    {
        retrievePresentAgents();

        double deltaTime = Time::now() - lastUpdateTimer;
        lastUpdateTimer = Time::now();

        for(list<Entity*>::iterator aIt = presentAgents.begin(); aIt != presentAgents.end() ; aIt++)
        {
            Agent* currentAgent = dynamic_cast<Agent*>(*aIt);
            opc->update(currentAgent);

            //Decay drives
            for(map<string,Drive>::iterator d=currentAgent->m_drives.begin(); d != currentAgent->m_drives.end(); d++)
            {
                d->second.value -= d->second.decay * deltaTime;
                d->second.value = max(0.0, min(1.0, d->second.value));
            }

            //Decay Emotions
            for(map<string,double>::iterator d=currentAgent->m_emotions_intrinsic.begin(); d != currentAgent->m_emotions_intrinsic.end(); d++)
            {
                d->second -= emotionalDecay * deltaTime;
                d->second = max(0.0, min(1.0, d->second));
            }

            /*			cout<<"EMOTIONS : "<<endl;
            for(map<string,double>::iterator d=currentAgent->m_emotions_intrinsic.begin(); d != currentAgent->m_emotions_intrinsic.end(); d++)
            {
            cout<<'\t'<<d->first<<'\t'<<d->second<<endl;
            }*/		
        }
        opc->commit();
    }

    void threadRelease()
    {
        opc->interrupt();
        opc->close();
    }
};

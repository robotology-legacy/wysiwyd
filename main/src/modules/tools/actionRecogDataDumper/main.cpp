/*
* Copyright(C) 2016 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
* Authors: Ugo Pattacini
* email : ugo.pattacini@iit.it
* Permission is granted to copy, distribute, and / or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* wysiwyd / license / gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU General
* Public License for more details
*/

#include <string>
#include <list>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <wrdac/clients/icubClient.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;


/*******************************************************/
class Recorder : public RFModule
{
    Stamp dumpStamp;
    BufferedPort<Bottle> dumpPort;
    RpcServer rpcPort;

    ICubClient icubclient;
    Mutex mutex;

    string agentName;
    string actionTag;
    string objectTag;
    double period;
    int gate;

public:
    /*******************************************************/
    Recorder() : icubclient("actionRecogDataDumper") { }

    /*******************************************************/
    bool configure(ResourceFinder &rf)
    {
        agentName=rf.check("agent-name",Value("partner")).asString();
        period=rf.check("period",Value(0.05)).asDouble();

        if (!icubclient.connect())
        {
            yError() << " iCubClient : Some dependencies are not running (OPC?)...";
            return false;
        }

        dumpPort.open("/actionRecogDataDumper/data/dump:o");
        rpcPort.open("/actionRecogDataDumper/rpc");
        attach(rpcPort);

        actionTag="none";
        objectTag="none";
        gate=0;

        return true;
    }

    /*******************************************************/
    double getPeriod()
    {
        return period;
    }

    /*******************************************************/
    bool updateModule()
    {
        LockGuard lg(mutex);

        double dumpTime=Time::now();
        Bottle &bDump=dumpPort.prepare();
        bDump.clear();

        bDump.addString(actionTag);
        bDump.addString(objectTag);

        icubclient.opc->checkout();

        // agent body + position
        agentName = icubclient.getPartnerName();
        if (Entity *e=icubclient.opc->getEntity(agentName))
        {
            if (Agent *agent=dynamic_cast<Agent*>(e))
            {
                bDump.addList()=agent->m_body.asBottle();
                Bottle &bAgent=bDump.addList();
                bAgent.addString(agent->name());
                bAgent.addDouble(agent->m_ego_position[0]);
                bAgent.addDouble(agent->m_ego_position[1]);
                bAgent.addDouble(agent->m_ego_position[2]);
                bAgent.addInt(agent->m_present==1.0?1:0);
            }
        }

        // objects position
        list<Entity*> lEntity=icubclient.opc->EntitiesCache();
        for (list<Entity*>::iterator itEnt=lEntity.begin(); itEnt!=lEntity.end(); itEnt++)
        {
            string entityName=(*itEnt)->name();
            string entityType=(*itEnt)->entity_type();
            if (entityType==EFAA_OPC_ENTITY_OBJECT)
            {
                if (Object *object=dynamic_cast<Object*>(*itEnt))
                {
                    Bottle &bObject=bDump.addList();
                    bObject.addString(object->name());
                    bObject.addDouble(object->m_ego_position[0]);
                    bObject.addDouble(object->m_ego_position[1]);
                    bObject.addDouble(object->m_ego_position[2]);
                    bObject.addInt(object->m_present==1.0?1:0);
                }
            }
        }

        bDump.addInt(gate);

        dumpStamp.update(dumpTime);
        dumpPort.setEnvelope(dumpStamp);
        dumpPort.writeStrict();

        yInfo()<<bDump.toString();

        return true;
    }

    /*******************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        LockGuard lg(mutex);
        int cmd=command.get(0).asVocab();
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (cmd==Vocab::encode("start"))
        {
            if (command.size()>=3)
            {
                actionTag=command.get(1).asString();
                objectTag=command.get(2).asString();
                gate=1;
                if (command.size()>=4)
                {
                    int g=command.get(3).asInt();
                    if (g>0)
                        gate=g;
                }
                reply.addVocab(ack);
            }
            else
                reply.addVocab(nack);
            return true;
        }
        else if (cmd==Vocab::encode("stop"))
        {
            actionTag="none";
            objectTag="none";
            gate=0;
            reply.addVocab(ack);
            return true;
        }
        else if (cmd==Vocab::encode("get"))
        {
            reply.addVocab(ack);
            reply.addString(actionTag);
            reply.addString(objectTag);
            reply.addInt(gate);
            return true;
        }
        else
            return RFModule::respond(command,reply);
    }

    /*******************************************************/
    bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();
        dumpPort.interrupt();
        dumpPort.close();
        icubclient.opc->interrupt();
        icubclient.opc->close();
        icubclient.close();
        return true;
    }

    bool interruptModule()
    {
        rpcPort.interrupt();
        dumpPort.interrupt();
        icubclient.opc->interrupt();
        return true;
    }
};


/*******************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP network seems unavailable!";
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Recorder recorder;
    return recorder.runModule(rf);
}



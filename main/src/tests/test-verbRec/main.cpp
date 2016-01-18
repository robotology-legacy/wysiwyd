/*
* Copyright(C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
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
#include <wrdac/clients/icubClient.h>

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;


/*******************************************************/
class Recorder : public RFModule
{
    BufferedPort<Bottle> dumpPort;
    RpcClient verbPort;
    RpcServer rpcPort;
    OPCClient opc;
    Mutex mutex;

    string agentName;
    string actionTag;
    string objectTag;
    double period;
    int gate;

public:
    /*******************************************************/
    Recorder() : opc("test_verbRec/opc") { }

    /*******************************************************/
    bool configure(ResourceFinder &rf)
    {
        agentName=rf.check("agent-name",Value("partner")).asString();
        actionTag=rf.check("action-tag",Value("none")).asString();
        objectTag=rf.check("object-tag",Value("none")).asString();
        period=rf.check("period",Value(0.1)).asDouble();

        if (!opc.connect("OPC"))
        {
            yError()<<"OPC seems unavailabe!";
            return false;
        }

        dumpPort.open("/test_verbRec/dump:o");
        verbPort.open("/test_verbRec/verb:rpc");
        rpcPort.open("/test_verbRec/rpc");
        attach(rpcPort);

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

        Bottle &bDump=dumpPort.prepare();
        bDump.clear();

        bDump.addString(actionTag);
        bDump.addString(objectTag);

        opc.checkout();        

        // agent body + position
        if (Entity *e=opc.getEntity(agentName))
        {
            if (Agent *agent=dynamic_cast<Agent*>(e))
            {
                bDump.addList()=agent->m_body.asBottle();
                Bottle &bAgent=bDump.addList();
                bAgent.addString(agent->name());
                bAgent.addDouble(agent->m_ego_position[0]);
                bAgent.addDouble(agent->m_ego_position[1]);
                bAgent.addDouble(agent->m_ego_position[2]);
                bAgent.addInt(agent->m_present?1:0);
            }
        }

        // objects position
        list<Entity*> lEntity=opc.EntitiesCache();
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
                    bObject.addInt(object->m_present?1:0);
                }
            }
        }

        bDump.addInt(gate);

        // query verbRec
        if (verbPort.getOutputCount()>0)
        {
            Bottle cmd=bDump;
            Bottle reply;
            verbPort.write(cmd,reply);
            bDump.append(reply);
        }

        yInfo()<<bDump.toString();
        dumpPort.writeStrict();

        return true;
    }

    /*******************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        LockGuard lg(mutex);
        int cmd=command.get(0).asVocab();
        int ack=Vocab::encode("ack");

        if (cmd==Vocab::encode("start"))
        {
            gate=1;
            reply.addVocab(ack);
            return true;
        }
        else if (cmd==Vocab::encode("stop"))
        {
            gate=0;
            reply.addVocab(ack);
            return true;
        }
        else if (cmd==Vocab::encode("get"))
        {
            reply.addVocab(ack);
            reply.addInt(gate);
            return true;
        }
        else
            return RFModule::respond(command,reply);
    }

    /*******************************************************/
    bool close()
    {
        rpcPort.close();
        verbPort.close();
        dumpPort.close();
        opc.close();
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



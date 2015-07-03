#include "opcPopulater.h"



bool opcPopulater::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("opcPopulater")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();
    
    double seed = (double)time(NULL);
    int count = 0;
    while (seed == time(NULL)){ ++count; }
    seed = (int)seed % 100;
    srand((int)seed);

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "opcPopulater", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= false;
    if (!iCub->connect())
    {
        yInfo() << " iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }
    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    return true;
}


bool opcPopulater::close() {
    iCub->close();
    delete iCub;

    return true;
}


bool opcPopulater::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "help") {
        yInfo() << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString() == "populateSpecific") {
        yInfo() << " populateSpecific";
        (populateSpecific(command)) ? reply.addString("populateSpecific done !") : reply.addString("populateSpecific failed !");
    }


    return true;
}

/* Called periodically every getPeriod() seconds */
bool opcPopulater::updateModule() {
    return true;
}


bool opcPopulater::populateSpecific(Bottle bInput){

    if (bInput.size() != 3)
    {
        yWarning() << " in qRM::populateSpecific | wrong number of input";
        return false;
    }


    if (bInput.get(1).toString() == "agent")
    {
        string sName = bInput.get(2).toString();
        Agent* agent = iCub->opc->addAgent(sName);
        agent->m_ego_position[0] = (-1.5) * ((double)rand() / (double)RAND_MAX) - 0.5;
        agent->m_ego_position[1] = (2) * ((double)rand() / (double)RAND_MAX) - 1;
        agent->m_ego_position[2] = 0.60;
        agent->m_present = 1;
        agent->m_color[0] = rand() % 80;
        agent->m_color[1] = rand() % 70 + 180;
        agent->m_color[2] = rand() % 100 + 80;
        iCub->opc->commit(agent);

        agent = NULL;
    }

    if (bInput.get(1).toString() == "object")
    {
        string sName = bInput.get(2).toString();
        Object* obj = iCub->opc->addObject(sName);
        obj->m_ego_position[0] = (-1.5) * ((double)rand() / (double)RAND_MAX) - 0.2;
        obj->m_ego_position[1] = (2) * ((double)rand() / (double)RAND_MAX) - 1;
        obj->m_ego_position[2] = 0.20;
        obj->m_present = 1;
        obj->m_color[0] = rand() % 100 + 80;
        obj->m_color[1] = rand() % 80;
        obj->m_color[2] = rand() % 70 + 180;
        iCub->opc->commit(obj);
        
        iCub->opc->update();

        delete obj;
    }

    if (bInput.get(1).toString() == "rtobject")
    {
        string sName = bInput.get(2).toString();
        RTObject* obj = iCub->opc->addRTObject(sName);
        obj->m_ego_position[0] = (-1.5) * ((double)rand() / (double)RAND_MAX) - 0.2;
        obj->m_ego_position[1] = (2) * ((double)rand() / (double)RAND_MAX) - 1;
        obj->m_present = 1;
        obj->m_color[0] = rand() % 70 + 180;
        obj->m_color[1] = rand() % 100 + 80;
        obj->m_color[2] = rand() % 80;
        iCub->opc->commit(obj);
        delete obj;
    }

    return true;
}

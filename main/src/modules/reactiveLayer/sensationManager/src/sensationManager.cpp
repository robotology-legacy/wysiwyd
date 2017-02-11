#include "sensationManager.h"


bool SensationManager::close() {
    for(auto& sens : sensations) {
        sens->close_ports();
        delete sens;
    }
    sensations.clear();

    rpc_in_port.interrupt();
    rpc_in_port.close();

    return true;
}


bool SensationManager::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name",Value("SensationManager")).asString();
    setName(moduleName.c_str());
    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();

    Bottle grp = rf.findGroup("SENSATIONS");
    if (!grp.isNull()){
        sensationList = *grp.find("sensations").asList();  
        for (int i = 0; i < sensationList.size(); i++)
        {
            string sensation_name = sensationList.get(i).asString();
            // behavior_names.push_back(behavior_name);
            if (sensation_name == "opcSensation") {
                sensations.push_back(new OpcSensation());
            } else if (sensation_name == "test") {
                sensations.push_back(new Test());
            } else{
                yDebug() << "Sensation " + sensation_name + " not implemented";
                return false;
            }
            sensations.back()->configure();
        }
    }else{
        yError()<<"Didn't find any sensation. Please revise your configuration files...";
        return 0;
    }

    // for(std::vector<Behavior*>::iterator it = behaviors.begin(); it != behaviors.end(); ++it) {
    // }
    rpc_in_port.open("/" + moduleName + "/rpc");
    attach(rpc_in_port);
    yInfo("Init done");

    return true;
}


bool SensationManager::updateModule()
{
    for(std::vector<Sensation*>::size_type i = 0; i != sensations.size(); i++) {
        sensations[i]->publish();
    }
    return true;


}

bool SensationManager::respond(const Bottle& cmd, Bottle& reply)
{
    yInfo() << "RPC received in sensationsManager";
    yDebug() << cmd.toString();
    
    reply.clear();
    bool rpl;
    if (cmd.get(0).asString() == "help" )
    {   string help = "\n";
        help += " ['is' + entity_name + entity_tag]  : Turns on/off manual mode (for manual control of drives) \n";
        reply.addString(help);
    }
    else if (cmd.get(0).asString() == "is") {
        for (int i = 0; i < sensationList.size(); i++)
        {
            string sensation_name = sensationList.get(i).asString();
            // behavior_names.push_back(behavior_name);
            if (sensation_name == "opcSensation") {
                rpl = dynamic_cast<OpcSensation*>(sensations[i])->get_property(cmd.get(1).asString(),cmd.get(2).asString());
                reply.addString("ack");
                reply.addInt(rpl);
                return true;
            }
        }
        if (reply.size()==0)
        {    
            reply.addString("nack");
        }
    } else {
        reply.addString("nack");
        reply.addString("Unknown rpc command");
    }
    return true;
}


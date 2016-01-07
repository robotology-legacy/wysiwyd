#include "behaviorManager.h"


bool BehaviorManager::close()
{
    rpc_in_port.interrupt();
    rpc_in_port.close();
    iCub->close();

    for(auto& beh : behaviors) {
        beh->close_ports();
    }

    delete iCub;

    return true;
}

bool BehaviorManager::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name",Value("BehaviorManager")).asString();
    setName(moduleName.c_str());
    yInfo()<<moduleName<<": finding configuration files...";//<<endl;
    period = rf.check("period",Value(1.0)).asDouble();

    Bottle grp = rf.findGroup("BEHAVIORS");
    Bottle behaviorList = *grp.find("behaviors").asList();  

    rpc_in_port.open("/" + moduleName + "/trigger:i");
    yInfo() << "RPC_IN : " << rpc_in_port.getName();
    attach(rpc_in_port);
    for (int i = 0; i<behaviorList.size(); i++)
    {
        string behavior_name = behaviorList.get(i).asString();
        if (behavior_name == "tagging") {
            behaviors.push_back(new Tagging(&mut));
        } else if (behavior_name == "pointing") {
            behaviors.push_back(new Pointing(&mut));
        } else if (behavior_name == "dummy") {
            behaviors.push_back(new Dummy(&mut));
        } else if (behavior_name == "dummy2") {
            behaviors.push_back(new Dummy(&mut));
        } else if (behavior_name == "pointingOrder") {
            behaviors.push_back(new PointingOrder(&mut));
        }  else if (behavior_name == "touchingOrder") {
            behaviors.push_back(new TouchingOrder(&mut));
        }
            // other behaviors here
        else {
            yDebug() << "Behavior " + behavior_name + " not implemented";
            return false;
        }
    }

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "behaviorManager","client.ini",isRFVerbose);
    // char rep = 'n';
    if (!iCub->connect())
    {
        yInfo()<<"iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    // id = 0;
    for(auto& beh : behaviors) {
        beh->configure();
        beh->openPorts(moduleName);
        beh->iCub = iCub;

        if (beh->from_sensation_port_name != "None") {
            while (!Network::connect(beh->from_sensation_port_name, beh->sensation_port_in.getName())) {
                yInfo()<<"Connecting "<< beh->from_sensation_port_name << " to " <<  beh->sensation_port_in.getName();// <<endl;
                yarp::os::Time::delay(0.5);
            }
        }
        if (beh->external_port_name != "None") {
            while (!Network::connect(beh->rpc_out_port.getName(), beh->external_port_name)) {
                yInfo()<<"Connecting "<< beh->rpc_out_port.getName() << " to " <<  beh->external_port_name;// <<endl;
                yarp::os::Time::delay(0.5);
            }   
        }

        // id++;
    }

    // behavior_to_trigger = -1;  // means no behavior to trigger

    yInfo("Init done");

    return true;
}


bool BehaviorManager::updateModule()
{
    return true;
}


bool BehaviorManager::respond(const Bottle& cmd, Bottle& reply)
{
    yDebug() << "RPC received in BM";
    yDebug() << cmd.toString();
    if (cmd.get(0).asString() == "help" )
    {   string help = "\n";
        help += " ['behavior_name']  : Triggers corresponding behavior \n";
        reply.addString(help);
    }
    else
    {
        for(auto& beh : behaviors) {
            if (cmd.get(0).asString() == beh->name) {
        //         Bottle args;
        //         args.clear();
        //         for (int a = 1; a < cmd.size(); a++)
        //         {
        //             args.add(&cmd.get(a));
        //         }
                beh->trigger(/*args*/);
            }
        }
    }
    reply.clear();
    reply.addString("ack");
    yDebug() << "End of BehaviorManager::respond";
    return true;
}

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
    behaviorList = *grp.find("behaviors").asList();  

    rpc_in_port.open("/" + moduleName + "/trigger:i");
    yInfo() << "RPC_IN : " << rpc_in_port.getName();
    attach(rpc_in_port);
    for (int i = 0; i<behaviorList.size(); i++)
    {
        behavior_name = behaviorList.get(i).asString();
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
        }  else if (behavior_name == "reactions") {
            behaviors.push_back(new Reactions(&mut));
        }  else if (behavior_name == "followingOrder") {
            behaviors.push_back(new FollowingOrder(&mut));
        }  else if (behavior_name == "narrate") {
            behaviors.push_back(new Narrate(&mut));
            // other behaviors here
        }  else {
            yDebug() << "Behavior " + behavior_name + " not implemented";
            return false;
        }
    }

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "behaviorManager","client.ini",isRFVerbose);

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
    }
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
    
    reply.clear();

    if (cmd.get(0).asString() == "help" )
    {   string help = "\n";
        help += " ['behavior_name']  : Triggers corresponding behavior \n";
        reply.addString(help);
    }
    else if (cmd.get(0).asString() == "names" ) {
        Bottle names;
        names.clear();
        for(auto& beh : behaviors) {
            names.addString(beh->name);
        }
        reply.addList() = names;
    }
    else
    {
        for(auto& beh : behaviors) {
            yDebug()<<cmd.get(0).asString() <<"    "<< beh->name;
            if (cmd.get(0).asString() == beh->name) {
        //         Bottle args;
        //         args.clear();
        //         for (int a = 1; a < cmd.size(); a++)
        //         {
        //             args.add(&cmd.get(a));
        //         }
                bool aux;
                if (beh->from_sensation_port_name != "None") {
                    if (!Network::isConnected(beh->from_sensation_port_name, beh->sensation_port_in.getName())) {
                        aux =Network::connect(beh->from_sensation_port_name, beh->sensation_port_in.getName());
                        yInfo()<< "The sensations port for "<< beh->name <<" was not connected. \nReconnection status: " << aux;
                    }
                }
                if (beh->external_port_name != "None") {
                    if (!Network::isConnected(beh->rpc_out_port.getName(), beh->external_port_name)) {
                        aux = Network::connect(beh->rpc_out_port.getName(), beh->external_port_name);
                        yInfo()<< "The external port for "<< beh->name <<" was not connected. \nReconnection status: " << aux;
                    }   
                }

                beh->trigger(/*args*/);

                // Add event into ABM
                if (iCub->getABMClient()->Connect()) {
                    std::list<std::pair<std::string, std::string> > lArgument;
                    lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
                    lArgument.push_back(std::pair<std::string, std::string>(cmd.get(0).asString(), "predicate"));
                    lArgument.push_back(std::pair<std::string, std::string>("unknown_object", "object"));
                    lArgument.push_back(std::pair<std::string, std::string>("partner", "recipient"));
                    iCub->getABMClient()->sendActivity("action",
                        cmd.get(0).asString(),
                        "behavior",  // expl: "pasar", "drives"...
                        lArgument,
                        true);
                    yInfo() << cmd.get(0).asString() + " behavior has been recorded in the ABM";
                }
                else{
                    yDebug() << "ABM not connected; no recording of action.";
                }
            }
        }
        reply.addString("ack");
    }
    yDebug() << "End of BehaviorManager::respond";
    return true;
}

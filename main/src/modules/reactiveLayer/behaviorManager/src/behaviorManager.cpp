#include "behaviorManager.h"

bool BehaviorManager::interruptModule()
{
    rpc_in_port.interrupt();

    for(auto& beh : behaviors) {
        beh->interrupt_ports();
    }
    return true;
}

bool BehaviorManager::close()
{
    rpc_in_port.interrupt();
    rpc_in_port.close();
    iCub->close();

    for(auto& beh : behaviors) {
        beh->close_ports();
    }
    behaviors.clear();

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

    for (int i = 0; i<behaviorList.size(); i++)
    {
        behavior_name = behaviorList.get(i).asString();
        if (behavior_name == "tagging") {
            behaviors.push_back(new Tagging(&mut, rf, "tagging"));
        } else if (behavior_name == "pointing") {
            behaviors.push_back(new Pointing(&mut, rf, "pointing"));
        } else if (behavior_name == "dummy") {
            behaviors.push_back(new Dummy(&mut, rf, "dummy"));
        } else if (behavior_name == "dummy2") {
            behaviors.push_back(new Dummy(&mut, rf, "dummy2"));
        }  else if (behavior_name == "reactions") {
            behaviors.push_back(new Reactions(&mut, rf, "reactions"));
        }  else if (behavior_name == "followingOrder") {
            behaviors.push_back(new FollowingOrder(&mut, rf, "followingOrder"));
        }  else if (behavior_name == "narrate") {
            behaviors.push_back(new Narrate(&mut, rf, "narrate"));
        }  else if (behavior_name == "recognitionOrder") {
            behaviors.push_back(new recognitionOrder(&mut, rf, "recognitionOrder"));
        }  else if (behavior_name == "greeting") {
            behaviors.push_back(new recognitionOrder(&mut, rf, "greeting"));
        }  else if (behavior_name == "speech") {
            behaviors.push_back(new Speech(&mut, rf, "speech"));            
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
        yInfo() << "iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }
    if (rf.check("use_ears",Value("false")).asBool())
    {
        yDebug()<<"using ears";
        while (!Network::connect("/ears/behavior:o", rpc_in_port.getName())) {
            yWarning() << "Ears is not reachable";
            yarp::os::Time::delay(0.5);
        }
    }else{
        yDebug()<<"not using ears";
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

    attach(rpc_in_port);
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
    else if (cmd.get(0).asString() == "manual") {
        for(auto& beh : behaviors) {
            if (beh->behaviorName == "followingOrder") {
                if (cmd.get(1).asString() == "on") {
                    yInfo() << "followingOrder behavior manual mode on";
                    dynamic_cast<FollowingOrder *>(beh)->manual = true;
                    reply.addString("ack");
                } else if (cmd.get(1).asString() == "off") {
                    yInfo() << "followingOrder behavior manual mode off";
                    dynamic_cast<FollowingOrder *>(beh)->manual = false;
                    reply.addString("ack");
                }
            }
            else if (beh->behaviorName == "recognitionOrder") {
                if (cmd.get(1).asString() == "on") {
                    yInfo() << "recognitionOrder behavior manual mode on";
                    dynamic_cast<recognitionOrder *>(beh)->manual = true;
                    reply.addString("ack");
                } else if (cmd.get(1).asString() == "off") {
                    yInfo() << "recognitionOrder behavior manual mode off";
                    dynamic_cast<recognitionOrder *>(beh)->manual = false;
                    reply.addString("ack");
                }
            }
        }
    } else if (cmd.get(0).asString() == "names" ) {
        Bottle names;
        names.clear();
        for(auto& beh : behaviors) {
            names.addString(beh->behaviorName);
        }
        reply.addList() = names;
    }
    else
    {
        bool behavior_triggered = false;
        for(auto& beh : behaviors) {
            if (cmd.get(0).asString() == beh->behaviorName) {
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
                        yInfo()<< "The sensations port for "<< beh->behaviorName <<" was not connected. \nReconnection status: " << aux;
                    }
                }
                if (beh->external_port_name != "None") {
                    if (!Network::isConnected(beh->rpc_out_port.getName(), beh->external_port_name)) {
                        aux = Network::connect(beh->rpc_out_port.getName(), beh->external_port_name);
                        yInfo()<< "The external port for "<< beh->behaviorName <<" was not connected. \nReconnection status: " << aux;
                    }   
                }

                Bottle args;
                if (cmd.size()>1){
                    args = cmd.tail();
                }
                yDebug() << "arguments are " << args.toString().c_str();
                // beh->trigger(args);
                behavior_triggered = beh->trigger(args);

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
        if (behavior_triggered)
            reply.addString("ack");
        else{
            reply.addString("nack");
            yDebug()<< "Behavior ' " << cmd.get(0).asString() << " ' not found. \nSend 'names' to see a list of available behaviors. ";
        }
    }
    yDebug() << "End of BehaviorManager::respond";
    return true;
}

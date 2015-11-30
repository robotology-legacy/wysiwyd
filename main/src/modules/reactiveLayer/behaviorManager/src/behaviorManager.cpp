#include "behaviorManager.h"


bool BehaviorManager::close()
{
    iCub->close();
    delete iCub;

    return true;
}


bool BehaviorManager::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name",Value("BehaviorManager")).asString();
    setName(moduleName.c_str());
    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();

    Bottle grp = rf.findGroup("BEHAVIORS");
    Bottle behaviorList = *grp.find("behaviors").asList();  

    rpc_in_port.open("/" + moduleName + "/trigger:i");
    cout << "RPC_IN : " << rpc_in_port.getName();
    attach(rpc_in_port);
    for (int i = 0; i<behaviorList.size(); i++)
    {
        string behavior_name = behaviorList.get(i).asString();
        // behavior_names.push_back(behavior_name);
        if (behavior_name == "tagging") {
            behaviors.push_back(new Tagging());
        } else if (behavior_name == "pointing") {
            behaviors.push_back(new Pointing());
        } else if (behavior_name == "dummy") {
            behaviors.push_back(new Dummy());

        } else if (behavior_name == "pointingOrder") {
            behaviors.push_back(new PointingOrder());

        }
            // other behaviors here
        else{
            yDebug() << "Behavior " + behavior_name + " not implemented";
            return false;
        }
    }

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "behaviorManager","client.ini",isRFVerbose);
    iCub->opc->isVerbose = false;
    char rep = 'n';
    yDebug() << "1";
    while (rep!='y'&&!iCub->connect())
    {
        cout<<"iCubClient : Some dependencies are not running..."<<endl;
        Time::delay(1.0);
    }
    yDebug() << "2";

    //Set the voice
    cout << "3" << endl;
    cout.flush();
    //if (iCub->getSpeechClient())
    SubSystem_Speech* sss = iCub->getSpeechClient(); 
    if (sss) {
        string ttsOptions = rf.check("ttsOptions", yarp::os::Value("iCub")).asString();
        sss->SetOptions(ttsOptions);    
    } else {
        yInfo() << "SPEECH not available.";
    }
    yDebug() << "4";


    // id = 0;
    for(std::vector<Behavior*>::iterator it = behaviors.begin(); it != behaviors.end(); ++it) {
        Behavior* beh = *it;
        beh->configure();
        beh->openPorts(moduleName);
        beh->iCub = iCub;
        // attach(beh->rpc_in_port);
        // trigger_port.open("/" + moduleName + "/trigger" + ":i");

        // trigger_under_ports.push_back(new TriggerCallback(&behavior_to_trigger, id));
        // trigger_under_ports.useCallback();
        // trigger_under_ports.back()->open("/" + moduleName +"/triggers_under/" + beh->name +":i"<);
        
        // trigger_over_ports.push_back(new TriggerCallback(&behavior_to_trigger, id));
        // trigger_over_ports.useCallback();
        // trigger_over_ports.back()->open("/" + moduleName +"/triggers_over/" + beh->name +":i");

        // sensation_input_ports.push_back(new BufferedPort<Bottle>());
        // sensation_input_ports.back()->open("/" + moduleName +"/sensations/" + beh->name +":i");

        // rpc_out_ports.push_back(new Port());
        // rpc_out_ports.back()->open("/" + moduleName +"/rpc_out/" + beh->name +":i");

        if (beh->from_sensation_port_name != "None") {
            while (!Network::connect(beh->from_sensation_port_name, beh->sensation_port_in.getName())) {
                cout<<"Connecting "<< beh->from_sensation_port_name << " to " <<  beh->sensation_port_in.getName() <<endl;
                yarp::os::Time::delay(0.5);
            }
        }
        if (beh->external_port_name != "None") {
            while (!Network::connect(beh->rpc_out_port.getName(), beh->external_port_name)) {
                cout<<"Connecting "<< beh->rpc_out_port.getName() << " to " <<  beh->external_port_name <<endl;
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
    // for(i=0; i < behaviors.size(); i++) {
    //     Bottle *run =trigger_under_ports[i]->read(false):
    //     if (run != NULL) {
    //         Bottle *sensation = sensation_input_ports[behavior_to_trigger]->read();
    //         Bottle cmd = behaviors[behavior_to_trigger]->run(*sensation);
    //         Bottle rply;
    //         rpc_out_ports[behavior_to_trigger]->write(cmd, rply);

    //     }
    // }

    // if (behavior_to_trigger != -1) {

    // }
    return true;


}


bool BehaviorManager::respond(const Bottle& cmd, Bottle& reply)
{
    yDebug() << "RPC received  in BM";
    yDebug() << cmd.toString();
    if (cmd.get(0).asString() == "help" )
    {   string help = "\n";
        help += " ['behavior_name']  : Triggers corresponding behavior \n";
        reply.addString(help);
    }else
    {

        for(std::vector<Behavior*>::size_type i = 0; i != behaviors.size(); i++) {
            if (cmd.get(0).asString() == behaviors[i]->name) {
                behaviors[i]->run();
            }
        }

        reply.addString("ack");
    }
    // strange:
    reply.addString("nack");

    return true;
}

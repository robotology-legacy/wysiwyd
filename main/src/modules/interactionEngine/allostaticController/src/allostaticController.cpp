#include <algorithm>    // std::random_shuffle
#include "allostaticController.h"

bool AllostaticController::close()
{
    return true;
}

int AllostaticController::openPorts(string driveName)
{

    outputm_ports.push_back(new BufferedPort<Bottle>);
    outputM_ports.push_back(new BufferedPort<Bottle>);




    string portName = "/" + moduleName + "/" + driveName;
    string pn = portName + "/min:i";

    if (!outputm_ports.back()->open(pn))
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }
    string targetPortName = "/" + homeo_name + "/" + driveName + "/min:o";
    yarp::os::Time::delay(0.1);
    while(!Network::connect(targetPortName,pn)){
        cout<<"Setting up homeostatic connections... "<< targetPortName << " " << pn <<endl;
        yarp::os::Time::delay(0.5);
    }
    pn = portName + "/max:i";
    cout << "Configuring port " << pn << " ..." << endl;
    yarp::os::Time::delay(0.1);
    if (!outputM_ports.back()->open(pn))
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }
    yarp::os::Time::delay(0.1);
    targetPortName = "/" + homeo_name + "/" + driveName + "/max:o";
    while(!Network::connect(targetPortName, pn))
    {cout<<"Setting up homeostatic connections... "<< targetPortName << " " << pn <<endl;yarp::os::Time::delay(0.5);}


    return 42;
}

bool AllostaticController::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name",Value("AllostaticController")).asString();
    setName(moduleName.c_str());

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    // bool isRFVerbose = false;
    // CHECK if needed
    finding = false;
    pointing = false;



    configureAllostatic(rf);

    cout<<"Configuration done."<<endl;

    // rpc.open ( ("/"+moduleName+"/rpc"));
    // attach(rpc);
    ears_port.open("/" + moduleName + "/ears:o");
    while (!Network::connect(ears_port.getName(), "/ears/rpc")) {
        cout<<"Setting up ears connection from "<< ears_port.getName() <<" to /ears/rpc" <<endl;
        yarp::os::Time::delay(0.5);
    }

    physicalInteraction = false;
    someonePresent = false;

    //iCub->getReactableClient()->SendOSC(yarp::os::Bottle("/event reactable pong start"));

    last_time = yarp::os::Time::now();

    yInfo("Init done");


    return true;
}

void AllostaticController::configureAllostatic(yarp::os::ResourceFinder &rf)
{
    //The homeostatic module should be running in parallel, independent from this, so the objective of
    //this config would be to have a proper list  and connect to each port

    homeo_name = "homeostasis";
    string homeo_rpc_name = "/" + homeo_name + "/rpc";
    string to_h_rpc_name="/"+moduleName+"/toHomeoRPC:o";
    to_homeo_rpc.open(to_h_rpc_name);

    while(!Network::connect(to_h_rpc_name,homeo_rpc_name))
    {
        cout<<"Trying to connect to homeostasis..."<<endl;
        cout << "from " << to_h_rpc_name << " to " << homeo_rpc_name << endl;
        yarp::os::Time::delay(0.2);
    }


    //Initialise the iCub allostatic model. Drives for interaction engine will be read from IE default.ini file
    cout << "Initializing drives..."<<endl;
    Bottle grpAllostatic = rf.findGroup("ALLOSTATIC");
    drivesList = *grpAllostatic.find("drives").asList();
    //iCub->icubAgent->m_drives.clear();
    Bottle cmd;

    priority_sum = 0.;
    double priority;
    for (int d = 0; d<drivesList.size(); d++)
    {
        cmd.clear();
        string driveName = drivesList.get(d).asString();

        cmd.addString("add");
        cmd.addString("conf");
        Bottle drv;
        drv.clear();
        Bottle aux;
        aux.clear();
        aux.addString("name");
        aux.addString(driveName);
        drv.addList()=aux;
        aux.clear();
        drv.addList()=grpAllostatic;
        cmd.addList()=drv;
        Bottle rply;
        rply.clear();
        rply.get(0).asString();
        // cout << cmd.toString() << endl;
        to_homeo_rpc.write(cmd,rply);

        AllostaticDrive alloDrive;
        alloDrive.name = driveName;

        Value cmds = grpAllostatic.check((driveName + "-sensation-on"), Value("None"));
        alloDrive.sensationOnCmd = *cmds.asList();
        cmds = grpAllostatic.check((driveName + "-sensation-off"), Value("None"));
        alloDrive.sensationOffCmd = *cmds.asList();

        cmds = grpAllostatic.check((driveName + "-trigger"), Value("None"));
        if (!(cmds.isString() && cmds.asString() == "None")) {
            alloDrive.triggerCmd = *cmds.asList();
        }

        alloDrive.homeoPort = &to_homeo_rpc;

        alloDrive.inputSensationPort = new BufferedPort<Bottle>;
        string portName = "/" + moduleName + "/" + driveName + "/sensation:i";
        alloDrive.inputSensationPort->open(portName);

        openPorts(driveName);

        string sensationPort = grpAllostatic.check((driveName + "-sensation-port"), Value("None")).asString();
        string pn = "/" + moduleName + "/" + driveName + "/sensation:i";
        while(!Network::connect(sensationPort, pn)) {
            cout<<"Connecting " << sensationPort << " to " << pn << endl;
            yarp::os::Time::delay(0.5);
        }


        // set drive priorities. Default to 1.
        priority = grpAllostatic.check((driveName + "-priority"), Value(1.)).asDouble();
        priority_sum += priority;
        drivePriorities.push_back(priority);

        //Under effects
        // StimulusEmotionalResponse responseUnder;
        // removed: stimuls-response sentences
        string under_port_name = grpAllostatic.check((driveName + "-under-behavior-port"), Value("None")).asString();
        string under_cmd_name = grpAllostatic.check((driveName + "-under-behavior"), Value("None")).asString();

        bool active = false;
        
        if (under_port_name != "None" && under_cmd_name != "None")
        {
            active = true;
            string out_port_name = "/" + moduleName + "/" + driveName + "/under_action:o";
            alloDrive.behaviorUnderPort = new Port();
            alloDrive.behaviorUnderPort->open(out_port_name);
            cout << "trying to connect to " << under_port_name << endl;
            while(!Network::connect(out_port_name,under_port_name))
            {
                cout << ".";
                yarp::os::Time::delay(0.5);
            }
            //responseUnder.rpc_command.clear();
            alloDrive.behaviorUnderCmd = Bottle(under_cmd_name);//.addString(under_cmd_name);
            //cout << endl << endl << responseUnder.rpc_command.toString() << endl << endl;
        }else{
            yInfo() << "No port name";            
        }

        // homeostaticUnderEffects[driveName] = responseUnder;

        //Over effects

        string over_port_name = grpAllostatic.check((driveName + "-under-behavior-port"), Value("None")).asString();
        string over_cmd_name = grpAllostatic.check((driveName + "-over-behavior"), Value("None")).asString();
        if (over_port_name != "None" && over_cmd_name != "None")
        {
            active=true;
            string out_port_name = "/" + moduleName + "/" + driveName + "/over_action:o";
            alloDrive.behaviorOverPort = new Port();
            alloDrive.behaviorOverPort->open(out_port_name);
            cout << "trying to connect to " << over_port_name << endl;
            while(!Network::connect(out_port_name,over_port_name))
            {
                cout << ".";
                yarp::os::Time::delay(0.5);
            }
            //responseOver.rpc_command.clear();
            alloDrive.behaviorOverCmd = Bottle(over_cmd_name);//.addString(over_cmd_name);
        }else{
            yInfo() << "No port name";
        }

        alloDrive.active = active;
        allostaticDrives[driveName] = alloDrive;
    }

    // Normalize drive priorities
    if ( ! Normalize(drivePriorities))
        cout << "Error: Drive priorities sum up to 0." << endl;

    // CHECK if needed
    cout << "done." << endl;
}

bool AllostaticController::createTemporalDrive(string name, double prior)
{
    //Add new drive with default configuration
    cout<<"!!!adding drive"<<endl;
    Bottle cmd,rpl;
    cmd.clear();
    cmd.addString("add");
    cmd.addString("new");
    cmd.addString(name);
    to_homeo_rpc.write(cmd);
    yarp::os::Time::delay(2.0);
    drivesList.addString(name);
    cout << "!!!making drive boolean"<<endl;
    //Remove decay: it will always be either needed or not
    cmd.clear();
    cmd.addString("par");
    cmd.addString(name);
    cmd.addString("dec");
    cmd.addDouble(0.0);
    cout << cmd.toString()<<endl;
    to_homeo_rpc.write(cmd);
    cout << "!!!sending drive out of the CZ"<<endl;
    yarp::os::Time::delay(2.0);
    
    //Set drive out of the boundary
    cmd.clear();
    cmd.addString("par");
    cmd.addString(name);
    cmd.addString("val");
    cmd.addDouble(0.01);
    to_homeo_rpc.write(cmd);
    yarp::os::Time::delay(2.0);

    //Change priorities
    cout << "!!!changing priorities"<<endl;
    double priority = priority_sum*prior;
    priority_sum += priority;
    drivePriorities.push_back(priority);
    Normalize(drivePriorities);

    //temporal drives are only on this vector
    temporalDrivesList.push_back(name);


    //Is everything alright¿?
    return true;

}

bool AllostaticController::Normalize(vector<double>& vec) {
    double sum = 0.;
    for (unsigned int i = 0; i<vec.size(); i++)
        sum += vec[i];
    if (sum == 0.)
        return false;
    for (unsigned int i = 0; i<vec.size(); i++)
        vec[i] /= sum;
    return true;
}

bool AllostaticController::updateModule()
{
    for(std::map<string, AllostaticDrive>::iterator it=allostaticDrives.begin(); it!=allostaticDrives.end(); ++it) {
        // cout << it->second.inputSensationPort->read()->get(0).asInt() << endl;
        if (bool(it->second.inputSensationPort->read()->get(0).asInt())) {
            yDebug() << "Sensation ON";
            it->second.update(SENSATION_ON);
        } else {
            it->second.update(SENSATION_OFF);
        }
    }

    updateAllostatic();

    return true;

    //CMF: here we rather want to read data from the Sensation Module and to transmit it to the related drives.
    // confusion = handleTagging();
    // cout << "confusion handled "<< endl;
    // cout << confusion << endl;

    // if (searchList.size()>0){
    //     cout << "there are elements to search!!!"<<endl;
    //     finding = handleSearch();
    // }
        
    // if (pointList.size()>0){
    //     if (finding)
    //         // Be carfull: both handlePoint (point in response of a human order) and handlePointing (point what you know)
    //         pointing = handlePoint();
    //     cout << "there are elements to point!!!"<<endl;
    // }
    // //learning = handlePointing();
    updateAllostatic();
    //updateEmotions();


    return true;
}

// Return the index of a drive to solve according to priorities and homeostatis levels
// Return -1 if no drive to be solved
DriveOutCZ AllostaticController::chooseDrive() {
    // to be redone

    DriveOutCZ result;
    bool inCZ;
    int numOutCz = 0;
    double random, cum;
    vector<double> outOfCzPriorities(drivePriorities);
    vector<string> names;
    vector<double> min_diff; 
    vector<double> max_diff; 

    for ( int i =0;i<drivesList.size();i++) {
        names.push_back(drivesList.get(i).asString());
        min_diff.push_back(outputm_ports[i]->read()->get(0).asDouble());
        max_diff.push_back(outputM_ports[i]->read()->get(0).asDouble());
        inCZ = min_diff.back() <= 0 && max_diff.back() <= 0;
        if (inCZ) {
            outOfCzPriorities[i] = 0.1;
        }
        else {
            numOutCz ++;
        }
        // cout << "Drive " << i << ", " << drivesList.get(i).asString() << ". Priority: " << outOfCzPriorities[i] << "." << endl;
    }
    if (! numOutCz) {
        result.name = "None";
        return result;
    }
    if ( ! Normalize(outOfCzPriorities)) {
        result.name = "None";
        return result;
    }
    random = Rand::scalar();
    cum = outOfCzPriorities[0];
    int idx = 0;
    while (cum < random) {
        cum += outOfCzPriorities[idx + 1];
        idx++;
    }
    result.name = names[idx];
    if (min_diff[idx] > 0)
        result.level = UNDER;
    if (max_diff[idx] > 0)
        result.level = OVER;
    return result;
}


bool AllostaticController::updateAllostatic()
{


    DriveOutCZ activeDrive = chooseDrive();
    

    // CMF: Commands to ears should rather be in proactivetagging
    if (activeDrive.name == "None") {
        cout << "No drive out of CZ." << endl;
        if ((yarp::os::Time::now()-last_time)>2.0)
                {
                last_time = yarp::os::Time::now();
                Bottle cmd;
                cmd.clear();
                cmd.addString("listen");
                cmd.addString("on");
                ears_port.write(cmd);
            }
        return true;
    }
    else
    {
        cout << "Drive" << activeDrive.name << " out of CZ. Active: " << allostaticDrives[activeDrive.name].active << endl;
        //i = activeDrive.idx;
        
                Bottle cmd;
                cmd.clear();
                cmd.addString("listen");
                cmd.addString("off");
                ears_port.write(cmd);
    }
    bool temporal = false;
    for (unsigned int j = 0;j<temporalDrivesList.size();j++)
    {
        if (activeDrive.name==temporalDrivesList[j])
            {
                temporal = true;
            }
    }

    if (allostaticDrives[activeDrive.name].active) {
        // yInfo() << " [updateAllostatic] Drive " << activeDrive.name << " chosen.";
        allostaticDrives[activeDrive.name].triggerBehavior(activeDrive.level);
    } else {
        yInfo() << "Drive " + activeDrive.name + " is not active";
    }

//     if (activeDrive.level == UNDER)
//     {
        
        
//         // iCub->say(homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].getRandomSentence());
//         Bottle cmd;
//         cmd.clear();
//         cmd.addString("par");
//         cmd.addString("tagging");
//         cmd.addString("val");
//         cmd.addDouble(0.5);
//         //rpc_ports[activeDrive.idx]->write(cmd);
//         to_homeo_rpc.write(cmd);

//         cmd.clear();
//         cmd.addString("par");
//         cmd.addString("tagging");
//         cmd.addString("dec");
//         cmd.addDouble(0.0);
//         //rpc_ports[activeDrive.idx]->write(cmd);
//         to_homeo_rpc.write(cmd);

//         if (homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].active)
//         {
//             yInfo() << " [updateAllostatic] Command will be sent";
//             //yInfo() <<homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].active << homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].rpc_command.toString();
//             Bottle rply;
//             rply.clear();
//             //homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].output_port->write(homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].rpc_command,rply);
//             homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].send();
//             cout << homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].rpc_command << endl;
//             yarp::os::Time::delay(0.1);
//             yInfo() << "[updateAllostatic] reply from homeostatis : " << rply.toString();

//             //clear as soon as sent
//             //homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].rpc_command.clear();
//             yInfo() << "check rpc command is empty : " << homeostaticUnderEffects[drivesList.get(activeDrive.idx).asString()].rpc_command;
//         }
        

//         //d->second.value += (d->second.homeoStasisMax - d->second.homeoStasisMin) / 3.0;
//     }

//     if (activeDrive.level == OVER)
//     {
//         cout << "Drive " << activeDrive.idx << " chosen. Over level." << endl;
// //        iCub->look("partner");
//         // iCub->say(homeostaticOverEffects[drivesList.get(activeDrive.idx).asString()].getRandomSentence());
//         Bottle cmd;
//         cmd.clear();
//         cmd.addString("par");
//         cmd.addString(drivesList.get(activeDrive.idx).asString());
//         cmd.addString("val");
//         cmd.addDouble(0.5);

//         to_homeo_rpc.write(cmd);
//         //d->second.value -= (d->second.homeoStasisMax - d->second.homeoStasisMin) / 3.0;;
//     }

    return true;
}

bool AllostaticController::respond(const Bottle& cmd, Bottle& reply)
{
    if (cmd.get(0).asString() == "help" )
    {   string help = "\n";
        help += " ['need'] ['macro'] [name] [arguments]   : Assigns a value to a specific parameter \n";
        reply.addString(help);
        /*cout << " [par] [drive] [val/min/max/dec] [value]   : Assigns a value to a specific parameter \n"<<
                " [delta] [drive] [val/min/max/dec] [value] : Adds a value to a specific parameter  \n"<<
                " [add] [conf] [drive Bottle]               : Adds a drive to the manager as a drive directly read from conf-file  \n"<<
                " [add] [botl] [drive Bottle]                : Adds a drive to the manager as a Bottle of values of shape \n"<<
                "                                           : (string name, double value, double homeo_min, double homeo_max, double decay = 0.05, bool gradient = true) \n"<<endl;
    */
    }else if (cmd.get(0).asString() == "need")
    {
        if (cmd.get(1).asString() == "macro") //macro is for subgoals or general actions
        {
            if (cmd.get(2).asString() == "search")
            {
                cout << yarp::os::Time::now()<<endl;
                cout<< "haha! time to work!"<<endl;
                string o_name = cmd.get(3).asString();
                //double p = pow(1/cmd.get(4).asDouble() ,5 ) * 100;
                string d_name = "search";
                d_name += "_";
                d_name += o_name;
                //createTemporalDrive(d_name, p);
                cout << "searchList size:"<<endl;

                searchList.push_back(o_name);
                cout << searchList.size() << endl;
                cout << searchList[0]<<endl;
                reply.addString("ack");

            }
        }else if (cmd.get(1).asString() == "primitive")
        {
            if (cmd.get(2).asString() == "point")
            {
                cout << yarp::os::Time::now()<<endl;
                string o_name = cmd.get(3).asString();
                double p = pow(1/cmd.get(4).asDouble() ,5 ) * 100;
                string d_name = "point";
                d_name += "_";
                d_name += o_name;
                //createTemporalDrive(d_name, p);
                //searchList.push_back(o_name);
                //createTemporalDrive(o_name, p);
                pointList.push_back(o_name);
                reply.addString("ack");

            }
        }
    }
    reply.addString("nack");

    return true;
}

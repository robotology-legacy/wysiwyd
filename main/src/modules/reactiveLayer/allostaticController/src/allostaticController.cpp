#include "allostaticController.h"

bool AllostaticController::close()
{
    yDebug() << "Closing port to homeo rpc";
    to_homeo_rpc.interrupt();
    to_homeo_rpc.close();
    for (unsigned int i = 0; i < outputm_ports.size(); i++)
    {
        // yDebug() << "Closing port " + itoa(i) + " to homeo min/max";
        outputm_ports[i]->interrupt();        
        outputm_ports[i]->close();
        outputM_ports[i]->interrupt();        
        outputM_ports[i]->close();
    }

    yDebug() << "Closing AllostaticDrive ports";
    for(std::map<string, AllostaticDrive>::iterator it=allostaticDrives.begin(); it!=allostaticDrives.end(); ++it) {
        it->second.close_ports();
    }

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
        yDebug() << getName() << ": Unable to open port " << pn;// << endl;
    }
    string targetPortName = "/" + homeo_name + "/" + driveName + "/min:o";
    yarp::os::Time::delay(0.1);
    while(!Network::connect(targetPortName,pn)){
        yInfo() <<"Setting up homeostatic connections... "<< targetPortName << " " << pn ;//<<endl;
        yarp::os::Time::delay(0.5);
    }
    pn = portName + "/max:i";
    yInfo() << "Configuring port " << pn << " ...";// << endl;
    yarp::os::Time::delay(0.1);
    if (!outputM_ports.back()->open(pn))
    {
        yDebug() << getName() << ": Unable to open port " << pn ;//<< endl;
    }
    yarp::os::Time::delay(0.1);
    targetPortName = "/" + homeo_name + "/" + driveName + "/max:o";
    while(!Network::connect(targetPortName, pn))
    {yDebug()<<"Setting up homeostatic connections... "<< targetPortName << " " << pn;//endl;
    yarp::os::Time::delay(0.5);}


    return 42;
}

bool AllostaticController::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name",Value("AllostaticController")).asString();
    setName(moduleName.c_str());

    yDebug()<<moduleName<<": finding configuration files...";//<<endl;
    period = rf.check("period",Value(0.5)).asDouble();

    configureAllostatic(rf);


    last_time = yarp::os::Time::now();

    yInfo()<<"Configuration done.";


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
        yDebug()<<"Trying to connect to homeostasis...";//<<endl;
        yDebug() << "from " << to_h_rpc_name << " to " << homeo_rpc_name;// << endl;
        yarp::os::Time::delay(0.2);
    }

    yInfo() << "Initializing drives...";//<<endl;
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
        yInfo() << ("Initializing drive " + driveName);

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
            yDebug()<<"Connecting " << sensationPort << " to " << pn;// << endl;
            yarp::os::Time::delay(0.5);
        }


        // set drive priorities. Default to 1.
        priority = grpAllostatic.check((driveName + "-priority"), Value(1.)).asDouble();
        priority_sum += priority;
        drivePriorities.push_back(priority);

        //Under effects
        string under_port_name = grpAllostatic.check((driveName + "-under-behavior-port"), Value("None")).asString();
        string under_cmd_name = grpAllostatic.check((driveName + "-under-behavior"), Value("None")).asString();

        bool active = false;
        
        if (under_port_name != "None" && under_cmd_name != "None")
        {
            active = true;
            string out_port_name = "/" + moduleName + "/" + driveName + "/under_action:o";
            alloDrive.behaviorUnderPort = new Port();
            alloDrive.behaviorUnderPort->open(out_port_name);
            yDebug() << "trying to connect to " << under_port_name;// << endl;
            while(!Network::connect(out_port_name,under_port_name))
            {
                
                yarp::os::Time::delay(0.5);
            }
            alloDrive.behaviorUnderCmd = Bottle(under_cmd_name);//.addString(under_cmd_name);
        }else{
            yInfo() << "No port name";            
        }

        //Over effects
        string over_port_name = grpAllostatic.check((driveName + "-under-behavior-port"), Value("None")).asString();
        string over_cmd_name = grpAllostatic.check((driveName + "-over-behavior"), Value("None")).asString();
        if (over_port_name != "None" && over_cmd_name != "None")
        {
            active=true;
            string out_port_name = "/" + moduleName + "/" + driveName + "/over_action:o";
            alloDrive.behaviorOverPort = new Port();
            alloDrive.behaviorOverPort->open(out_port_name);
            yDebug() << "trying to connect to " << over_port_name ;//<< endl;
            while(!Network::connect(out_port_name,over_port_name))
            {
                
                yarp::os::Time::delay(0.5);
            }
            alloDrive.behaviorOverCmd = Bottle(over_cmd_name);//.addString(over_cmd_name);
        }else{
            yInfo() << "No port name";
        }




        alloDrive.active = active;
        allostaticDrives[driveName] = alloDrive;
    }

    if ( ! Normalize(drivePriorities))
        yDebug() << "Error: Drive priorities sum up to 0.";// << endl;

    yInfo() << "done.";// << endl;
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
        // yDebug() << it->second.inputSensationPort->read()->get(0).asInt();// << endl;
        if (bool(it->second.inputSensationPort->read()->get(0).asInt())) {
            yDebug() << "Sensation ON";
            it->second.update(SENSATION_ON);
        } else {
            yDebug() << "Sensation OFF";
            it->second.update(SENSATION_OFF);
        }
    }

    updateAllostatic();

    return true;
}

// Return a DriveOutCZ structure indicating the name and level (UNDER or OVER) of the drive to by humour according to priorities and homeostatis levels
// name is "None" if no drive to be solved
DriveOutCZ AllostaticController::chooseDrive() {

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
            outOfCzPriorities[i] = 0.;
        }
        else {
            numOutCz ++;
        }
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

    yInfo() << "Drive " + activeDrive.name + " chosen";

    if (activeDrive.name == "None") {
        yInfo() << "No drive out of CZ." ;
        return true;
    }
    else
    {
        yInfo() << "Drive " + activeDrive.name + " out of CZ." ;
    }

    if (allostaticDrives[activeDrive.name].active) {
        yInfo() << "Trigerring " + activeDrive.name;
        allostaticDrives[activeDrive.name].triggerBehavior(activeDrive.level);
    } else {
        yInfo() << "Drive " + activeDrive.name + " is not active";
    }

    return true;
}


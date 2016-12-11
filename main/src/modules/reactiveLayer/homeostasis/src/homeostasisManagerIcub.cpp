#include "homeostasisManagerIcub.h"
#include <cmath>

bool HomeostaticModule::addNewDrive(string driveName, yarp::os::Bottle& grpHomeostatic)
{    
    yDebug() << "Add drive " + driveName;
    Drive* drv = new Drive(driveName, period);

    drv->setHomeostasisMin(grpHomeostatic.check((driveName + "-homeostasisMin"), Value(drv->homeostasisMin)).asDouble());
    drv->setHomeostasisMax(grpHomeostatic.check((driveName + "-homeostasisMax"), Value(drv->homeostasisMax)).asDouble());
    //cout << "H1 " << grpHomeostatic.check((driveName + "-decay"), Value(drv->decay)).asDouble() ; 
    drv->setDecay(grpHomeostatic.check((driveName + "-decay"), Value(drv->decay)).asDouble());
    drv->setValue((drv->homeostasisMax + drv->homeostasisMin) / 2.);
    drv->setGradient(grpHomeostatic.check((driveName + "-gradient"), Value(drv->gradient)).asInt());
    //cout << "h2 " << drv->gradient ;
    //cout << drv->name << " " << drv->homeostasisMin << " " << drv->homeostasisMax << " " << drv->decay << " " <<drv->gradient ;
    //cout << d ;
    //cout << grpHomeostatic.toString();
    manager->addDrive(drv);
    //cout << "h2 " << manager->drives[0]->gradient ;
   
    openPorts(driveName);

    return true;
}
bool HomeostaticModule::addNewDrive(string driveName)
{

    cout << "adding a default drive..." ;
    Drive* drv = new Drive(driveName, period);


    drv->setHomeostasisMin(drv->homeostasisMin);
    drv->setHomeostasisMax(drv->homeostasisMax);
    drv->setDecay(drv->decay);
    drv->setValue((drv->homeostasisMax + drv->homeostasisMin) / 2.);
    drv->setGradient(false);
    
    manager->addDrive(drv);
    
    yDebug() << "default drive added. Opening ports...";//;
    openPorts(driveName);
    yInfo() << "new drive created successfully!";//;

    return true;
}

int HomeostaticModule::openPorts(string driveName)
{
    //Create Ports
    string portName = "/" + moduleName + "/" + driveName;
    
    string pn = portName + ":i";
    input_ports.push_back(new BufferedPort<Bottle>());
    outputm_ports.push_back(new BufferedPort<Bottle>());
    outputM_ports.push_back(new BufferedPort<Bottle>());

    yInfo() << "Configuring port " << " : " << pn << " ..." ;
    if (!input_ports.back()->open(pn)) 
    {
        yInfo() << getName() << ": Unable to open port " << pn ;
    }
    
    pn = portName + "/min:o";
    yInfo() << "Configuring port " << " : "<< pn << " ..." ;
    if (!outputm_ports.back()->open(pn)) 
    {
        yInfo() << getName() << ": Unable to open port " << pn ;
    }

    pn = portName + "/max:o";
    yInfo() << "Configuring port " << " : "<< pn << " ..." ;
    if (!outputM_ports.back()->open(pn)) 
    {
        yInfo() << getName() << ": Unable to open port " << pn ;
    }

    return 42;
}
/*bool HomeostaticModule::addNewDrive(string driveName, yarp::os::Value grpHomeostatic,int d)
{
    Drive drv = Drive();
    drv.setName(driveName);
    drv.setHomeostasisMin(grpHomeostatic.check((driveName + "-homeostasisMin").c_str(), Value(0.25)).asDouble());
    drv.setHomeostasisMax(grpHomeostatic.check((driveName + "-homeostasisMax").c_str(), Value(0.75)).asDouble());
    drv.setDecay(grpHomeostatic.check((driveName + "-decay").c_str(), Value(0.05)).asDouble());
    drv.setValue((drv.homeostasisMax + drv.homeostasisMin) / 2.0);
    drv.setGradient(grpHomeostatic.check((driveName + "-gradient").c_str()));
    manager->addDrive(&drv,d);
    //Create Ports
    string portName = "/" + moduleName + "/" + driveName;
    if (!input_ports[d].open((portName+":i").c_str())) 
    {
        cout << getName() << ": Unable to open port " << portName << ":i" ;
    }
    if (!output_ports[d].open((portName+":o").c_str())) 
    {
        cout << getName() << ": Unable to open port " << portName << ":o" ;
    }
    return true;
}*/

bool HomeostaticModule::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name",Value("homeostasis")).asString().c_str();
    setName(moduleName.c_str());

    yInfo()<<moduleName<<": finding configuration files...";
    period = rf.check("period",Value(0.1)).asDouble();

    manager = new HomeostasisManager();

    yInfo() << "Initializing drives";
    Bottle grpHomeostatic = rf.findGroup("HOMEOSTATIC");
    Bottle *drivesList = grpHomeostatic.find("drives").asList();
    if (drivesList)
    {
        yInfo() << "Configuration: Found " << drivesList->size() << " drives. " ;
        for (int d = 0; d<drivesList->size(); d++)
        {
            cout << d ;
            //Read Drive Configuration
            string driveName = drivesList->get(d).asString().c_str();
            addNewDrive(driveName, grpHomeostatic);
            
        }
    }
    stress_k = rf.check("stress-k",Value("15.")).asDouble();
    stress_th = rf.check("stress-th",Value("0.55")).asDouble();
    //stress=0;

    stressPort.open("/"+moduleName+"/stress:o");

    yInfo() << "Opening RPC...";
     
    rpc.open ( ("/"+moduleName+"/rpc").c_str());
    attach(rpc);

    yInfo()<<"Configuration done.";
    return true;
}

bool HomeostaticModule::removeDrive(int d)
{
    //Remove drive
    manager->removeDrive(d);
    //close ports
    input_ports[d]->close();
    outputm_ports[d]->close();
    outputM_ports[d]->close();
    delete input_ports[d];
    delete outputm_ports[d];
    delete outputM_ports[d];
    //Remove ports from lists
    input_ports.erase(input_ports.begin()+d);
    outputm_ports.erase(outputm_ports.begin()+d);
    outputM_ports.erase(outputM_ports.begin()+d);
    return true;
}


bool HomeostaticModule::respond(const Bottle& cmd, Bottle& reply)
{
    bool all_drives = false;
    if (cmd.get(0).asString() == "help" )
    {   string help = "\n";
        help += " ['par'] ['drive'] ['val'/'min'/'max'/'dec'] [value]   : Assigns a value to a specific parameter \n";
        help += " ['delta'] ['drive'] ['val'/'min'/'max'/'dec'] [value] : Adds a value to a specific parameter  \n";
        help += " ['add'] ['conf'] [drive Bottle]                       : Adds a drive to the manager as a drive directly read from conf-file  \n";
        help += " ['add'] ['botl'] [drive Bottle]                       : Adds a drive to the manager as a Bottle of values of shape \n";
        help += " ['add'] ['new'] [drive name]                          : Adds a default drive to the manager \n";
        help += " ['rm'] [drive name]                                   : removes a drive from the manager \n";
        help += " ['sleep'] [drive name] [time]                         : prevent drive update for a certain time (in seconds) \n";
        help += " ['sleep'] ['all'] [time]                              : prevent all drive updates for a certain time (in seconds) \n";
        help += " ['names']                                             : returns an ordered list of the drives in the manager \n";
        help += "                                                       : (string name, double value, double homeo_min, double homeo_max, double decay = 0.05, bool gradient = true) \n";
        reply.addString(help);
        /*cout << " [par] [drive] [val/min/max/dec] [value]   : Assigns a value to a specific parameter \n"<<
                " [delta] [drive] [val/min/max/dec] [value] : Adds a value to a specific parameter  \n"<<
                " [add] [conf] [drive Bottle]               : Adds a drive to the manager as a drive directly read from conf-file  \n"<<
                " [add] [botl] [drive Bottle]                : Adds a drive to the manager as a Bottle of values of shape \n"<<
                "                                           : (string name, double value, double homeo_min, double homeo_max, double decay = 0.05, bool gradient = true) \n";
    */
    }    
    else if (cmd.get(0).asString() == "par" )
    {
        
        if (cmd.get(1).asString() == "all") {
            all_drives = true;
        }
        for (unsigned int d = 0; d<manager->drives.size();d++)
        {
            if (all_drives || cmd.get(1).asString() == manager->drives[d]->name)
            {
                if (cmd.get(2).asString()=="val")
                {
                    manager->drives[d]->setValue(cmd.get(3).asDouble());
                }
                else if (cmd.get(2).asString()=="min")
                {
                    manager->drives[d]->setHomeostasisMin(cmd.get(3).asDouble());
                }
                else if (cmd.get(2).asString()=="max")
                {
                    manager->drives[d]->setHomeostasisMax(cmd.get(3).asDouble());
                }
                else if (cmd.get(2).asString()=="dec")
                {
                    manager->drives[d]->setDecay(cmd.get(3).asDouble());
                }
                else if (cmd.get(2).asString()=="reset")
                {
                    manager->drives[d]->reset();
                }
                else
                {
                    reply.addString("Format is: \n - ['par'] [drive_name] [val/min/max/dec/reset] [value]");
                    
                }
                reply.addString("ack");
                yInfo()<<"Received a order";
            }

        }
        reply.addString("nack: wrong drive name");
    }
    else if (cmd.get(0).asString() == "delta" )
    {
        for (unsigned int d = 0; d<manager->drives.size();d++)
        {
            if (cmd.get(1).asString() == manager->drives[d]->name)
            {
                if (cmd.get(2).asString()=="val")
                {
                    manager->drives[d]->deltaValue(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="min")
                {
                    manager->drives[d]->deltaHomeostasisMin(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="max")
                {
                    manager->drives[d]->deltaHomeostasisMax(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="dec")
                {
                    manager->drives[d]->deltaDecay(cmd.get(3).asDouble());
                }
                else
                {
                    reply.addString("nack");
                    cout << "Format is: \n - ['par'] [drive_name] [val/min/max/dec] [value]";
                }
                reply.addString("ack");
                yInfo()<<"Received a order";
            }
        }
    }
    else if (cmd.get(0).asString()=="add")
    {
        if (cmd.get(1).asString()=="conf")
        {
            Bottle *ga = cmd.get(2).asList();
           // cout <<"HOMEOSTATIC"<< ga->toString() ;
            Bottle grpAllostatic = ga->findGroup("ALLOSTATIC");
            //cout <<"HOMEOSTATIC"<< grpAllostatic.toString() ;
            //cout << "Bottle allo check: " << grpAllostatic.find("test-decay").asDouble() ; 

            addNewDrive(cmd.get(2).check("name",yarp::os::Value("")).asString(), grpAllostatic);
            reply.addString("add drive from config bottle: ack");
        }
        else if (cmd.get(1).asString()=="botl")
        {
            Drive d = bDrive(cmd.get(2).asList());
            manager->addDrive(&d);
            openPorts(d.name);
            reply.addString("add drive from bottle: ack");
        }
        else if (cmd.get(1).asString()=="new")
        {
            string d_name = cmd.get(2).asString();
            yInfo() << "adding new drive... " ;
            bool b = addNewDrive(d_name);
            if (b)
                reply.addString("add new drive: ack");
            else
                reply.addString("ack. drive not created");
        }
        else
        {
            reply.addString("nack");
        }

    }
    else if (cmd.get(0).asString()=="rm")
    {
        for (unsigned int d = 0; d<manager->drives.size();d++)
        {
            if (cmd.get(1).asString() == manager->drives[d]->name)
            {
                bool b = removeDrive(d);
                if (b)
                    reply.addString("ack: Successfully removed");
                else
                    reply.addString("ack: Could not remove the drive");

            }
        }
        reply.addString("nack");
    }
    else if (cmd.get(0).asString()=="freeze" || cmd.get(0).asString()=="unfreeze") {
        if (cmd.get(1).asString() == "all") {
            all_drives = true;
        }
        for (unsigned int d = 0; d<manager->drives.size();d++)
        {
            if (all_drives || cmd.get(1).asString() == manager->drives[d]->name)
            {
                if (cmd.get(0).asString()=="freeze") {
                    manager->drives[d]->freeze(); 
                }
                else {
                    manager->drives[d]->unfreeze();
                }
            }
        }
        reply.addString("ack");

    }
    else if (cmd.get(0).asString()=="sleep")
    {
        if (cmd.get(1).asString() != "all")
        {
            for (unsigned int d = 0; d<manager->drives.size();d++)
            {
                if (cmd.get(1).asString() == manager->drives[d]->name)
                {
                    double t = cmd.get(2).asDouble();
                    manager->sleep(d, t);
                    reply.addString("ack: Drive sleep");
                }
            }
        }
        else
        {
            for (unsigned int d = 0; d<manager->drives.size();d++)
            {
                double t = cmd.get(2).asDouble();
                manager->sleep(d, t);
                std::stringstream ss;
                ss << "ack: "<< manager->drives[d]->name << " sleep for " << t << "s";
                reply.addString(ss.str());
            }
        }
        reply.addString("nack");
    }    
    else if (cmd.get(0).asString()=="names")
    {
        Bottle nms;
        nms.clear();
        for (unsigned int i=0;i<manager->drives.size();i++)
        {
            nms.addString(manager->drives[i]->name);
        }
        reply.addList()=nms;
    }
    else if (cmd.get(0).asString() == "ask")
    {
        for (auto drive: manager->drives)
        {
            if (cmd.get(1).asString() == drive->name)
            {
                if (cmd.get(2).asString() == "min") {
                    reply.addDouble(drive->homeostasisMin);
                } else if (cmd.get(2).asString() == "max") {
                    reply.addDouble(drive->homeostasisMax);
                } else {
                    reply.addString("nack");
                }
            }
        }
        if (reply.size() == 0) {
            reply.addString("nack");
        }
    }
    else if (cmd.get(0).asString() == "force")
    {
        for (unsigned int d = 0; d<manager->drives.size();d++)
        {
            manager->drives[d]->freeze();
            manager->drives[d]->reset();
            if (cmd.get(1).asString() == manager->drives[d]->name)
            {
                if (cmd.get(2).asString()=="bottom")
                    manager->drives[d]->setValue(0.);
                else if (cmd.get(2).asString()=="top")
                    manager->drives[d]->setValue(1.);
                else
                {
                    reply.addString("Format is: \n - ['force'] [drive_name] [top/bottom]");
                }
                reply.addString("ack");
                yInfo()<<"Received a order";
            }
            else
            {
                reply.addString("Format is: \n - ['force'] [drive_name] [top/bottom]");
            }

        }
    }  
    else
    {
        reply.addString("nack");
    }
    return true;
}

bool HomeostaticModule::updateModule()
{
    double stress = 0;
    for(unsigned int d = 0; d<manager->drives.size();d++)
    {
        yInfo() << "Going by drive #"<<d << " with name "<< manager->drives[d]->name ;
        
        //yarp::os::Bottle* inp = input_ports[d]->read();
        yarp::os::Bottle* inp;
        inp = input_ports[d]->read(false);
        //cout << inp ;
        
        if(manager->drives[d]->gradient == true)
        {
            if (inp)
            {
                //cout<< "Input: "<<inp->get(0).asString();
                if (manager->drives[d]->name == "avoidance")
                    {
                        processAvoidance(d,inp);
                    }
                else
                {
                    manager->drives[d]->setValue(inp->get(0).asDouble());
                }
            }else{
                //manager->drives[d]->setValue(inp->get(0).asDouble());
            }
        }
        manager->drives[d]->update();
        yarp::os::Bottle &out1 = outputm_ports[d]->prepare();// = output_ports[d]->prepare();
        out1.clear();
        yarp::os::Bottle &out2 = outputM_ports[d]->prepare();
        out2.clear();
        if (manager->drives[d]->name == "avoidance")
        {
            out1.addDouble(-manager->drives[d]->getValue()+manager->drives[d]->homeostasisMin);
            outputm_ports[d]->write();

            double aux = +manager->drives[d]->getValue()-manager->drives[d]->homeostasisMax;
            if (aux<0)
            {
                aux=0;
            }else{
                aux=1;
                //aux = 0-aux;
            }
            out2.addDouble(aux);
            outputM_ports[d]->write();
        }else{
            
            out1.addDouble(-manager->drives[d]->getValue()+manager->drives[d]->homeostasisMin);
            outputm_ports[d]->write();
            
            yDebug() <<"Drive value: " << manager->drives[d]->value;
            yDebug() <<"Drive decay: " << manager->drives[d]->decay;
            yDebug() <<"Drive homeostasisMin: " << manager->drives[d]->homeostasisMin;
            yDebug() <<"Drive homeostasisMax: " << manager->drives[d]->homeostasisMax;
            yDebug() <<"Drive gradient: " << manager->drives[d]->gradient;
            yDebug() <<"Drive period: " << manager->drives[d]->period;
            out2.addDouble(+manager->drives[d]->getValue()-manager->drives[d]->homeostasisMax);
            yDebug()<<out1.get(0).asDouble();
            outputM_ports[d]->write();
        }
        stress += pow(out2.get(0).asDouble(),3) + pow(out1.get(0).asDouble(),3);
        
    }
    Bottle& output=stressPort.prepare();
    output.clear();

    stress = 1./(1.+exp(-stress_k*(-stress+stress_th)));
   
    output.addDouble(stress);
    stressPort.write();

    return true;
}

//Hardcoded function. This must go outside!!!
bool HomeostaticModule::processAvoidance(int d, Bottle* avoidanceBottle)
{
    // double intensity=0;
    // for (int i =0;i<avoidanceBottle->size();i++)
    // {
    //     Bottle* aux = avoidanceBottle->get(i).asList();
    //     if (aux->size() < 8)
    //     {
    //         intensity +=0.5;
    //     }
    //     else
    //     {
    //         intensity += aux->get(7).asDouble();
    //     }
    // }
    // manager->drives[d]->setValue(intensity);
    return true;

}

bool HomeostaticModule::close()
{
    for (unsigned int d=0; d<manager->drives.size(); d++)
    {
        input_ports[d]->interrupt();
        input_ports[d]->close();
        delete input_ports[d];

        outputM_ports[d]->interrupt();
        outputM_ports[d]->close();
        delete outputM_ports[d];

        outputm_ports[d]->interrupt();
        outputm_ports[d]->close();
        delete outputm_ports[d];

        delete manager->drives[d];
    }
    stressPort.interrupt();
    stressPort.close();

    input_ports.clear();
    outputM_ports.clear();
    outputm_ports.clear();

    rpc.interrupt();
    rpc.close();

    delete manager;

    return true;
}

#include "homeostasisManagerIcub.h"
#include <cmath>




bool homeostaticModule::addNewDrive(string driveName, yarp::os::Bottle& grpHomeostatic)
{

    
    Drive* drv = new Drive(driveName);


    drv->setHomeostasisMin(grpHomeostatic.check((driveName + "-homeostasisMin"), Value(drv->homeostasisMin)).asDouble());
    drv->setHomeostasisMax(grpHomeostatic.check((driveName + "-homeostasisMax"), Value(drv->homeostasisMax)).asDouble());
    cout << "H1 " << grpHomeostatic.check((driveName + "-decay"), Value(drv->decay)).asDouble() <<endl; 
    drv->setDecay(grpHomeostatic.check((driveName + "-decay"), Value(drv->decay)).asDouble());
    drv->setValue((drv->homeostasisMax + drv->homeostasisMin) / 2.);
    drv->setGradient(grpHomeostatic.check((driveName + "-gradient"), Value(drv->gradient)).asBool());
    cout << "h2 " << drv->decay << endl;
    //cout << drv->name << " " << drv->homeostasisMin << " " << drv->homeostasisMax << " " << drv->decay << " " <<drv->gradient << endl;
    //cout << d << endl;
    //cout << grpHomeostatic.toString()<<endl;
    manager.addDrive(drv);
     cout << "h2 " << manager.drives[0]->decay << endl;
   
    openPorts(driveName);

    return true;
}

int homeostaticModule::openPorts(string driveName)
{
    //Create Ports
    string portName = "/" + moduleName + "/" + driveName;
    
    string pn = portName + ":i";
    input_ports.push_back(new BufferedPort<Bottle>());
    outputm_ports.push_back(new BufferedPort<Bottle>());
    outputM_ports.push_back(new BufferedPort<Bottle>());

    cout << "Configuring port " << " : " << pn << " ..." << endl;
    if (!input_ports.back()->open(pn)) 
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }
    
    pn = portName + "/min:o";
    cout << "Configuring port " << " : "<< pn << " ..." << endl;
    if (!outputm_ports.back()->open(pn)) 
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }
    

    pn = portName + "/max:o";
    cout << "Configuring port " << " : "<< pn << " ..." << endl;
    if (!outputM_ports.back()->open(pn)) 
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }

    return 42;
}
/*bool homeostaticModule::addNewDrive(string driveName, yarp::os::Value grpHomeostatic,int d)
{
    Drive drv = Drive();
    drv.setName(driveName);
    drv.setHomeostasisMin(grpHomeostatic.check((driveName + "-homeostasisMin").c_str(), Value(0.25)).asDouble());
    drv.setHomeostasisMax(grpHomeostatic.check((driveName + "-homeostasisMax").c_str(), Value(0.75)).asDouble());
    drv.setDecay(grpHomeostatic.check((driveName + "-decay").c_str(), Value(0.05)).asDouble());
    drv.setValue((drv.homeostasisMax + drv.homeostasisMin) / 2.0);
    drv.setGradient(grpHomeostatic.check((driveName + "-gradient").c_str()));
    manager.addDrive(&drv,d);
    //Create Ports
    string portName = "/" + moduleName + "/" + driveName;
    if (!input_ports[d].open((portName+":i").c_str())) 
    {
        cout << getName() << ": Unable to open port " << portName << ":i" << endl;
    }
    if (!output_ports[d].open((portName+":o").c_str())) 
    {
        cout << getName() << ": Unable to open port " << portName << ":o" << endl;
    }
    return true;
}*/

bool homeostaticModule::configure(yarp::os::ResourceFinder &rf)
{
    manager = homeostasisManager();
    
    moduleName = rf.check("name",Value("homeostasis")).asString().c_str();
    setName(moduleName.c_str());

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();

    cout << "Initializing drives";
    Bottle grpHomeostatic = rf.findGroup("HOMEOSTATIC");
    Bottle *drivesList = grpHomeostatic.find("drives").asList();
    cout << "Initializing Drives... " << endl;
    if (drivesList)
    {
        cout << "Configuration: Found " << drivesList->size() << " drives. " << endl;
        for (int d = 0; d<drivesList->size(); d++)
        {
            cout << d << endl;
            //Read Drive Configuration
            string driveName = drivesList->get(d).asString().c_str();
            addNewDrive(driveName, grpHomeostatic);
            
        }
    }
    cout << "Opening RPC..."<< endl;
     
    rpc.open ( ("/"+moduleName+"/rpc").c_str());
    attach(rpc);

	
	cout<<"Configuration done."<<endl;
    return true;
}



bool homeostaticModule::respond(const Bottle& cmd, Bottle& reply)
{
	if (cmd.get(0).asString() == "help" )
    {   string help = "\n";
        help += " [par] [drive] [val/min/max/dec] [value]   : Assigns a value to a specific parameter \n";
        help += " [delta] [drive] [val/min/max/dec] [value] : Adds a value to a specific parameter  \n";
        help += " [add] [conf] [drive Bottle]               : Adds a drive to the manager as a drive directly read from conf-file  \n";
        help += " [add] [botl] [drive Bottle]                : Adds a drive to the manager as a Bottle of values of shape \n";
        help += "                                           : (string name, double value, double homeo_min, double homeo_max, double decay = 0.05, bool gradient = true) \n";
        reply.addString(help);
        /*cout << " [par] [drive] [val/min/max/dec] [value]   : Assigns a value to a specific parameter \n"<<
                " [delta] [drive] [val/min/max/dec] [value] : Adds a value to a specific parameter  \n"<<
                " [add] [conf] [drive Bottle]               : Adds a drive to the manager as a drive directly read from conf-file  \n"<<
                " [add] [botl] [drive Bottle]                : Adds a drive to the manager as a Bottle of values of shape \n"<<
                "                                           : (string name, double value, double homeo_min, double homeo_max, double decay = 0.05, bool gradient = true) \n"<<endl;
    */
    }    
    else if (cmd.get(0).asString() == "par" )
    {
        for (unsigned int d = 0; d<manager.drives.size();d++)
        {
            if (cmd.get(1).asString() == manager.drives[d]->name)
            {
                if (cmd.get(2).asString()=="val")
                {
                    manager.drives[d]->setValue(cmd.get(3).asDouble());
                }
                else if (cmd.get(2).asString()=="min")
                {
                    manager.drives[d]->setHomeostasisMin(cmd.get(3).asDouble());
                }
                else if (cmd.get(2).asString()=="max")
                {
                    manager.drives[d]->setHomeostasisMax(cmd.get(3).asDouble());
                }
                else if (cmd.get(2).asString()=="dec")
                {
                    manager.drives[d]->setDecay(cmd.get(3).asDouble());
                }
                else
                {
                    reply.addString("Format is: \n - ['par'] [drive_name] [val/min/max/dec] [value]");
                    
                }
                reply.addString("ack");
                cout<<"Received a order"<<endl;
            }

        }
        reply.addString("nack: wrong drive name");
    }
    else if (cmd.get(0).asString() == "delta" )
    {
        for (unsigned int d = 0; d<manager.drives.size();d++)
        {
            if (cmd.get(0).asString() == manager.drives[d]->name)
            {
                if (cmd.get(2).asString()=="val")
                {
                    manager.drives[d]->deltaValue(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="min")
                {
                    manager.drives[d]->deltaHomeostasisMin(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="max")
                {
                    manager.drives[d]->deltaHomeostasisMax(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="dec")
                {
                    manager.drives[d]->deltaDecay(cmd.get(3).asDouble());
                }
                else
                {
                    reply.addString("nack");
                    cout << "Format is: \n - ['par'] [drive_name] [val/min/max/dec] [value]"<<endl;
                }
                reply.addString("ack");
                cout<<"Received a order"<<endl;
            }
        }
    }
    else if (cmd.get(0).asString()=="add")
    {
        if (cmd.get(1).asString()=="conf")
        {
            Bottle *ga = cmd.get(2).asList();
           // cout <<"HOMEOSTATIC"<< ga->toString() << endl;
            Bottle grpAllostatic = ga->findGroup("ALLOSTATIC");
           cout <<"HOMEOSTATIC"<< grpAllostatic.toString() << endl;
            cout << "Bottle allo check: " << grpAllostatic.find("test-decay").asDouble() << endl; 

            addNewDrive(cmd.get(2).check("name",yarp::os::Value("")).asString(), grpAllostatic);
            reply.addString("add drive from config bottle: ack");
        }
        else if (cmd.get(1).asString()=="botl")
        {
            Drive d = bDrive(cmd.get(2).asList());
            manager.addDrive(&d);
            openPorts(d.name);
            reply.addString("add drive from bottle: ack");
        }
        else
        {
            reply.addString("nack");
        }

    }
    else if (cmd.get(0).asString()=="names")
    {
        Bottle nms;
        nms.clear();
        for (unsigned int i=0;i<manager.drives.size();i++)
        {
            nms.addString(manager.drives[i]->name);
        }
        reply.addList()=nms;
    }
    else
    {
        reply.addString("nack");
    }
    return true;
}

bool homeostaticModule::updateModule()
{
    for(unsigned int d = 0; d<manager.drives.size();d++)
    {
        cout << "Going by drive #"<<d << " with name "<< manager.drives[d]->name <<endl;
        
        //yarp::os::Bottle* inp = input_ports[d]->read();
        yarp::os::Bottle* inp;
        inp = input_ports[d]->read(false);
        //cout << inp << endl;
        
        if(manager.drives[d]->gradient == true)
        {
            if (inp)
            {
                cout<< "Input: "<<inp->get(0).asString()<<endl;
                if (manager.drives[d]->name == "avoidance")
                    {
                        processAvoidance(d,inp);
                    }
                else
                {
                    manager.drives[d]->setValue(inp->get(0).asDouble());
                }
            }else{
                //manager.drives[d]->setValue(inp->get(0).asDouble());
            }
        }
        manager.drives[d]->update();

        if (manager.drives[d]->name == "avoidance")
        {
            yarp::os::Bottle &out1 = outputm_ports[d]->prepare();// = output_ports[d]->prepare();
            out1.clear();
            out1.addDouble(-manager.drives[d]->getValue()+manager.drives[d]->homeostasisMin);
            outputm_ports[d]->write();
            yarp::os::Bottle &out2 = outputM_ports[d]->prepare();
            out2.clear();
            double aux = +manager.drives[d]->getValue()-manager.drives[d]->homeostasisMax;
            if (aux<0)
            {
                aux=0;
            }else{
                aux=1;
                //aux = 0-aux;
            }
            cout<<manager.drives[d]->getValue()<<endl;
            cout << manager.drives[d]->homeostasisMax<<endl;
            out2.addDouble(aux);
            cout<<out2.get(0).asDouble()<<endl;
            outputM_ports[d]->write();
        }else{
            yarp::os::Bottle &out1 = outputm_ports[d]->prepare();// = output_ports[d]->prepare();
            out1.clear();
            out1.addDouble(-manager.drives[d]->getValue()+manager.drives[d]->homeostasisMin);
            outputm_ports[d]->write();
            yarp::os::Bottle &out2 = outputM_ports[d]->prepare();
            out2.clear();
            cout <<"Drive value: " << manager.drives[d]->value<<endl;
            cout <<"Drive decay: " << manager.drives[d]->decay<<endl;
            cout <<"Drive homeostasisMin: " << manager.drives[0]->homeostasisMin<<endl;
            cout <<"Drive homeostasisMax: " << manager.drives[0]->homeostasisMax<<endl;
            out2.addDouble(+manager.drives[d]->getValue()-manager.drives[d]->homeostasisMax);
            cout<<out1.get(0).asDouble()<<endl;
            outputM_ports[d]->write();
        }
        
        
    }

    return true;
}

//Hardcoded function. This must go outside!!!
bool homeostaticModule::processAvoidance(int d, Bottle* avoidanceBottle)
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
    // manager.drives[d]->setValue(intensity);
    return true;

}

bool homeostaticModule::close()
{
    for (unsigned int d=0;d<manager.drives.size();d++)
    {
        input_ports[d]->interrupt();
        input_ports[d]->close();

        outputM_ports[d]->interrupt();
        outputM_ports[d]->close();

        outputm_ports[d]->interrupt();
        outputm_ports[d]->close();
    }
    rpc.interrupt();
    rpc.close();

    return true;
}

#include "homeostasisManagerIcub.h"
#include <cmath>




bool homeostaticModule::addNewDrive(string driveName, yarp::os::Bottle* grpHomeostatic,int d)
{

    //int asd;
    Drive* drv = new Drive();
    drv->setName(driveName);    

    drv->setHomeostasisMin(grpHomeostatic->check((driveName + "-homeostasisMin").c_str(), Value(0.25)).asDouble());
    drv->setHomeostasisMax(grpHomeostatic->check((driveName + "-homeostasisMax").c_str(), Value(0.75)).asDouble());
    drv->setDecay(grpHomeostatic->check((driveName + "-decay").c_str(), Value(0.05)).asDouble());
    drv->setValue((drv->homeostasisMax + drv->homeostasisMin) / 2.0);
    drv->setGradient(grpHomeostatic->check((driveName + "-gradient").c_str()));
    //cout << drv.name << " " << drv.homeostasisMin << " " << drv.homeostasisMax << " " << drv.decay << " " <<drv.gradient << endl;

    manager.addDrive(drv,d);
    
    openPorts(driveName,d);

    return true;
}

int homeostaticModule::openPorts(string driveName,int d)
{
    input_ports.resize(input_ports.size()+1);
    outputM_ports.resize(outputM_ports.size()+1);
    outputm_ports.resize(outputm_ports.size()+1);

    //Create Ports
    string portName = "/" + moduleName + "/" + driveName;
    
    string pn = portName + ":i";
    input_ports[d] = new BufferedPort<Bottle>;
    outputm_ports[d] = new BufferedPort<Bottle>;
    outputM_ports[d] = new BufferedPort<Bottle>;
    cout << "Configuring port " <<d<< " : " << pn << " ..." << endl;
    if (!input_ports[d]->open((pn).c_str())) 
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }

    pn = portName + "/min:o";
    cout << "Configuring port " <<d<< " : "<< pn << " ..." << endl;
    if (!outputm_ports[d]->open((pn).c_str())) 
    {
        cout << getName() << ": Unable to open port " << pn << endl;
    }

    pn = portName + "/max:o";
    cout << "Configuring port " <<d<< " : "<< pn << " ..." << endl;
    if (!outputM_ports[d]->open((pn).c_str())) 
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
    manager = homeostasisManager(0);
    
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
            addNewDrive(driveName, &grpHomeostatic,d);
            cout<<manager.drives[0]->homeostasisMax<<endl;
            
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
        for (unsigned int d = 0; d<manager.n_drives;d++)
        {
            if (cmd.get(0).asString() == manager.drive_names[d]->c_str())
            {
                if (cmd.get(2).asString()=="val")
                {
                    manager.drives[d]->setValue(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="min")
                {
                    manager.drives[d]->setHomeostasisMin(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="max")
                {
                    manager.drives[d]->setHomeostasisMax(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="dec")
                {
                    manager.drives[d]->setDecay(cmd.get(3).asDouble());
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
    else if (cmd.get(0).asString() == "delta" )
    {
        for (unsigned int d = 0; d<manager.n_drives;d++)
        {
            if (cmd.get(0).asString() == manager.drive_names[d]->c_str())
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
            addNewDrive(cmd.get(2).check("name",yarp::os::Value("")).asString(), 
                cmd.get(2).asList(), 
                manager.n_drives+1);
            reply.addString("add drive from config bottle: ack");
        }
        else if (cmd.get(1).asString()=="botl")
        {
            Drive d = bDrive(cmd.get(2).asList());
            manager.addDrive(&d,manager.n_drives+1);
            openPorts(d.name,manager.n_drives);
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
        for (unsigned int i=0;i<manager.n_drives;i++)
        {
            nms.addString(*manager.drive_names[i]);
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
    cout << manager.n_drives << endl;
    for(unsigned int d = 0; d<manager.n_drives;d++)
    {
        cout << manager.drives[d]->homeostasisMax<<endl;
        cout << "Going by drive #"<<d<<endl;
        
        //yarp::os::Bottle* inp = input_ports[d]->read();
        yarp::os::Bottle* inp;
        inp = input_ports[d]->read(false);
        //cout << inp << endl;
        
        if(manager.drives[d]->gradient == true)
        {
            cout << "gradient = True" << endl;
            if (inp)
            {
                cout<< "Input: "<<inp->get(0).asString()<<endl;
                if (*(manager.drive_names[d])=="avoidance")
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

        if (*(manager.drive_names[d])=="avoidance")
        {
            yarp::os::Bottle &out1 = outputm_ports[d]->prepare();// = output_ports[d].prepare();
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
            yarp::os::Bottle &out1 = outputm_ports[d]->prepare();// = output_ports[d].prepare();
            out1.clear();
            out1.addDouble(-manager.drives[d]->getValue()+manager.drives[d]->homeostasisMin);
            outputm_ports[d]->write();
            yarp::os::Bottle &out2 = outputM_ports[d]->prepare();
            out2.clear();
            cout<<manager.drives[d]->getValue()<<endl;
            cout << manager.drives[d]->homeostasisMax<<endl;
            out2.addDouble(+manager.drives[d]->getValue()-manager.drives[d]->homeostasisMax);
            cout<<out2.get(0).asDouble()<<endl;
            outputM_ports[d]->write();
        }
        
        
    }

    return true;
}

//Hardcoded function. This must go outside!!!
bool homeostaticModule::processAvoidance(int d, Bottle* avoidanceBottle)
{
    double intensity=0;
    for (int i =0;i<avoidanceBottle->size();i++)
    {
        Bottle* aux = avoidanceBottle->get(i).asList();
        if (aux->size() < 8)
        {
            intensity +=0.5;
        }
        else
        {
            intensity += aux->get(7).asDouble();
        }
    }
    manager.drives[d]->setValue(intensity);
    return true;

}

bool homeostaticModule::close()
{
    for (unsigned int d=0;d<manager.n_drives;d++)
    {
        input_ports[d]->close();
        outputM_ports[d]->close();
        outputm_ports[d]->close();
    }
    rpc.close();
    return true;
}
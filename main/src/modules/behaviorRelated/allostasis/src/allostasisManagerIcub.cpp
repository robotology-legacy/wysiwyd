#include "allostasisManagerIcub.h"
#include <cmath>



bool allostaticModule::configure(yarp::os::ResourceFinder &rf)
{
    
    moduleName = rf.check("name",Value("alloManager")).asString().c_str();
    setName(moduleName.c_str());

    cout<<moduleName<<": finding configuration files..."<<endl;
    period = rf.check("period",Value(0.1)).asDouble();

    cout << "Initializing drives";
    Bottle grpallostatic = rf.findGroup("ALLOSTATIC");
    Bottle *drivesList = grpallostatic.find("drives").asList();
    cout << "Initializing Drives... " << endl;
    if (drivesList)
    {
        cout << "Configuration: Found " << drivesList->size() << " drives. " << endl;
        for (int d = 0; d<drivesList->size(); d++)
        {
            cout << d << endl;
            //Read Drive Configuration
            string driveName = drivesList->get(d).asString().c_str();
            addNewDrive(driveName, &grpallostatic,d);
            cout<<manager.drives[0]->allostasisMax<<endl;
            
        }
    }
    cout << "Opening RPC..."<< endl;
     
    rpc.open ( ("/"+moduleName+"/rpc").c_str());
    attach(rpc);

	
	cout<<"Configuration done."<<endl;
    return true;
}



bool allostaticModule::respond(const Bottle& cmd, Bottle& reply)
{
	if (cmd.get(0).asString() == "help" )
    {   string help = "\n";
        help += " [par] [drive] [val/min/max/dec] [value]   : Assigns a value to a specific parameter \n";
        help += " [delta] [drive] [val/min/max/dec] [value] : Adds a value to a specific parameter  \n";
        help += " [add] [conf] [drive Bottle]               : Adds a drive to the manager as a drive directly read from conf-file  \n";
        help += " [add] [botl] [drive Bottle]                : Adds a drive to the manager as a Bottle of values of shape \n";
        help += "                                           : (string name, double value, double allo_min, double allo_max, double decay = 0.05, bool gradient = true) \n";
        reply.addString(help);
        /*cout << " [par] [drive] [val/min/max/dec] [value]   : Assigns a value to a specific parameter \n"<<
                " [delta] [drive] [val/min/max/dec] [value] : Adds a value to a specific parameter  \n"<<
                " [add] [conf] [drive Bottle]               : Adds a drive to the manager as a drive directly read from conf-file  \n"<<
                " [add] [botl] [drive Bottle]                : Adds a drive to the manager as a Bottle of values of shape \n"<<
                "                                           : (string name, double value, double allo_min, double allo_max, double decay = 0.05, bool gradient = true) \n"<<endl;
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
                    manager.drives[d]->setallostasisMin(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="max")
                {
                    manager.drives[d]->setallostasisMax(cmd.get(3).asDouble());
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
                    manager.drives[d]->deltaallostasisMin(cmd.get(3).asDouble());
                }
                else if (cmd.get(2)=="max")
                {
                    manager.drives[d]->deltaallostasisMax(cmd.get(3).asDouble());
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
    else
    {
        reply.addString("nack");
    }
    return true;
}

bool allostaticModule::updateModule()
{
    cout << manager.n_drives << endl;
    for(unsigned int d = 0; d<manager.n_drives;d++)
    {
        cout << manager.drives[d]->allostasisMax<<endl;
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
                manager.drives[d]->setValue(inp->get(0).asDouble());
            }else{
                //manager.drives[d]->setValue(inp->get(0).asDouble());
            }
        }
        manager.drives[d]->update();
        yarp::os::Bottle &out1 = outputm_ports[d]->prepare();// = output_ports[d].prepare();
        out1.clear();
        out1.addDouble(-manager.drives[d]->getValue()+manager.drives[d]->allostasisMin);
        outputm_ports[d]->write();
        yarp::os::Bottle &out2 = outputM_ports[d]->prepare();
        out2.clear();
        cout<<manager.drives[d]->getValue()<<endl;
        cout << manager.drives[d]->allostasisMax<<endl;
        out2.addDouble(+manager.drives[d]->getValue()-manager.drives[d]->allostasisMax);
        cout<<out2.get(0).asDouble()<<endl;
        outputM_ports[d]->write();
        
    }

    return true;
}

bool allostaticModule::close()
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
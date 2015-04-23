

//   Include declaration file
#include "ppsSim.h"
//   Include math library

#include <cstdlib>
#include <vector>



bool ppsSim::configure(yarp::os::ResourceFinder &rf)
{

    elapsedCycles = 0;
    listSize[0]=0;
    listSize[1]=0;
    listSize[2]=0;
    string moduleName = rf.check("name", Value("ppsSim")).asString().c_str();
    //setName(moduleName.c_str());

    string moduleOutput = rf.check("output", Value("/pos:o")).asString().c_str();
    string moduleOutputTarget = rf.check("output", Value("/posT:o")).asString().c_str();
    string moduleInput = "/pos:i";

    bool    bEveryThingisGood = true;

    string port2icubsim = "/" + moduleName + "sim:o";
    if (!portSim.open(port2icubsim.c_str())) {
        cout << getName() << ": Unable to open port " << port2icubsim << endl;
        cout << "The microphone might be turned on" << endl;
        bEveryThingisGood &= false;
    }    

    // create an output port
    std::string port2output = "/" + moduleName + moduleOutput;

    if (!portOutput.open(port2output.c_str())) {
        cout << getName() << ": Unable to open port " << port2output << endl;
        cout << "The microphone might be turned on" << endl;
        bEveryThingisGood &= false;
    }
    std::string port2outputT = "/" + moduleName + moduleOutputTarget;

    if (!portOutputTarget.open(port2outputT.c_str())) {
        cout << getName() << ": Unable to open port " << port2outputT << endl;
        cout << "The microphone might be turned on" << endl;
        bEveryThingisGood &= false;
    }

    std::string port2input = "/" + moduleName + moduleInput;

    if (!portInput.open(port2input.c_str())) {
        cout << getName() << ": Unable to open port " << port2input << endl;
        cout << "The microphone might be turned on" << endl;
        bEveryThingisGood &= false;
    }
    std::string port2world = "/icubSim/world";
    while (!Network::connect(port2icubsim, port2world.c_str()))
    {
        std::cout << "Trying to get input from microphone..." << std::endl;
        yarp::os::Time::delay(1.0);
    }

    cout << moduleName << ": finding configuration files..." << endl;

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    spamTable();
    
    yarp::os::Bottle pos1;

    pos1.clear();
    pos1.addDouble(0.1);
    pos1.addDouble(0.35);
    pos1.addDouble(0.55);
    ppsSim::createObject(pos1, true);

    pos1.clear();
    pos1.addDouble(0);
    pos1.addDouble(0.55);
    pos1.addDouble(0.35);
    ppsSim::createObject(pos1);

    cout<<"Configuration done."<<endl;

    return true;
}

void ppsSim::spamTable()
{
    yarp::os::Bottle size;
    size.clear();
    size.addDouble(0.45);
    size.addDouble(0.45);
    size.addDouble(0.45);

    yarp::os::Bottle pos;
    pos.clear();
    pos.addDouble(0);
    pos.addDouble(0.25);
    pos.addDouble(0.45);

    yarp::os::Bottle colour;
    colour.clear();
    colour.addDouble(0);
    colour.addDouble(0);
    colour.addDouble(0);
    std::string b = "box";

    ppsSim::createObject(b,size, pos, colour,true);
    std::cout<< "Table Spammed" << std::endl;
}

void ppsSim::createObject(std::string ob, yarp::os::Bottle size, yarp::os::Bottle pos, yarp::os::Bottle colour, bool target)
{
    yarp::os::Bottle pair;
    pair.clear();

    cmd.clear();
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString(ob.c_str());
    
    pair.addString(ob);
    
    if(ob=="box"){    
        cmd.addDouble(size.get(0).asDouble());
        cmd.addDouble(size.get(1).asDouble());
        cmd.addDouble(size.get(2).asDouble());
        listSize[0] +=1;
        pair.addInt(listSize[0]);
    }else if(ob == "sph"){
        cmd.addDouble(size.get(0).asDouble());
        listSize[1] +=1;
        pair.addInt(listSize[1]);
    }else if (ob == "cyl"){
        cmd.addDouble(size.get(0).asDouble());
        listSize[2] +=1;
        pair.addInt(listSize[2]);
    }else{
        cout<<"nothing created!!!!"<<endl;
    }

    cmd.addDouble(pos.get(0).asDouble());
    cmd.addDouble(pos.get(1).asDouble());
    cmd.addDouble(pos.get(2).asDouble());
    
    cmd.addDouble(colour.get(0).asDouble());
    cmd.addDouble(colour.get(1).asDouble());
    cmd.addDouble(colour.get(2).asDouble());
    portSim.write(cmd);
    //increase ob list and add ob to the list
    if (target==true){
        targets.append(pair);
    }else{
        objects.append(pair);
    }
}

void ppsSim::createObject(yarp::os::Bottle pos, bool target)
{
    yarp::os::Bottle s;
    yarp::os::Bottle c;
    s.clear();
    c.clear();
    s.addDouble(0.1);
    s.addDouble(0.1);
    s.addDouble(0.1);
    c.addDouble(255);
    c.addDouble(0);
    if(target==true)
        c.addDouble(255);
    else
        c.addDouble(0);

    ppsSim::createObject("box",s,pos,c);
    
}
void ppsSim::getCoordinates(std::string object, int id, bool target)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("get");
    cmd.addString(object);
    cmd.addInt(id);
    yarp::os::Bottle pos;
    pos.clear();
    portSim.write(cmd,pos);

    if (target == false){
        ppsSim::positions.append(pos);
    }else{
        ppsSim::tpositions.append(pos);
    }
}

bool ppsSim::close() {
    portOutput.close();
    portInput.close();
    rpc.close();
    return true;
}


/* Respond function */
bool ppsSim::respond(const yarp::os::Bottle& bCommand, yarp::os::Bottle& bReply)
{
/*
    std::string helpMessage = std::string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "F + (int)Frequency\n" +
        "RF \n" +
        "NI + (int)Noise intensity\n" +
        "WI + (int)Wave Intensity\n" +
        "quit \n";

    bReply.clear();
    std::string keyWord = bCommand.get(0).asString().c_str();

    if (keyWord == "quit") {
        bReply.addString("quitting");
        return false;
    }
    else if (keyWord == "help") {
        cout << helpMessage <<endl;
        bReply.addString("ok");
    }
    else if (keyWord == "F") {
        if (bCommand.size() == 2)
        {
            if (bCommand.get(1).isInt())
            {
                setNewFrequency(bCommand.get(1).asInt());
                bReply.addInt(f);
            }
        }
    }
    else if (keyWord == "RF") {    
                bReply.addInt(newRandomFrequency());
    }
    else if(keyWord == "WI"){
        if (bCommand.size() == 2)
        {
            if (bCommand.get(1).isInt())
            {
                WI = bCommand.get(1).asInt();
                bReply.addInt(f);
            }
        }
    }
    else if(keyWord == "NI"){
        if (bCommand.size() == 2)
        {
            if (bCommand.get(1).isInt())
            {
                NI = bCommand.get(1).asInt();
                bReply.addInt(f);
            }
        }
    }
*/
    return true;
}



/* Called periodically every getPeriod() seconds */
bool ppsSim::updateModule() {
    cout<<"."<<endl;

    Bottle &p = portOutput.prepare();
    p.clear();
    Bottle &t = portOutputTarget.prepare();
    
    for(int i=0;i<listSize[0]+listSize[1]+listSize[2];i++)
    {
        getCoordinates(objects.get(2*i).asString().c_str(), objects.get(2*i+1).asInt());

    }
    getCoordinates(targets.get(2).asString().c_str(), objects.get(3).asInt());

    p.copy(positions);
    t.copy(tpositions);
    portOutput.write();
    portOutputTarget.write();


    return true;
}

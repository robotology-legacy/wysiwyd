

//   Include declaration file
#include "objectGeneratorSim.h"
//   Include math library

#include <cstdlib>
#include <vector>
using namespace yarp::math;


bool objectGeneratorSim::configure(yarp::os::ResourceFinder &rf)
{

    Matrix iH;
    iH = zeros(4,4);
    iH(0,1)=-1;
    iH(1,2)=1;
    iH(1,3)=0.5976;
    iH(2,0)=-1;
    iH(2,3)=-0.026;
    iH(3,3)=1;
    H=SE3inv(iH);
    elapsedCycles = 0;
    listSize[0]=0;
    listSize[1]=0;
    listSize[2]=0;
    string moduleName = rf.check("name", Value("objectGeneratorSim")).asString().c_str();
    //setName(moduleName.c_str());

    string moduleOutput = rf.check("output", Value("/obstacles:o")).asString().c_str();
    string moduleOutputTarget = rf.check("output", Value("/reachingTarget:o")).asString().c_str();
    string moduleInput = "/pos:i";

    bool    bEveryThingisGood = true;

    string port2icubsim = "/" + moduleName + "/sim:o";
    if (!portSim.open(port2icubsim.c_str())) {
        cout << getName() << ": Unable to open port " << port2icubsim << endl;
        bEveryThingisGood &= false;
    }    

    // create an output port
    std::string port2output = "/" + moduleName + moduleOutput;

    if (!portOutput.open(port2output.c_str())) {
        cout << getName() << ": Unable to open port " << port2output << endl;
        bEveryThingisGood &= false;
    }
    std::string port2outputT = "/" + moduleName + moduleOutputTarget;

    if (!portOutputTarget.open(port2outputT.c_str())) {
        cout << getName() << ": Unable to open port " << port2outputT << endl;
        bEveryThingisGood &= false;
    }

    std::string port2input = "/" + moduleName + moduleInput;

    if (!portInput.open(port2input.c_str())) {
        cout << getName() << ": Unable to open port " << port2input << endl;
        bEveryThingisGood &= false;
    }
    std::string port2world = "/icubSim/world";
    while (!Network::connect(port2icubsim, port2world.c_str()))
    {
        std::cout << "Trying to get input from "<< port2icubsim << "..." << std::endl;
        yarp::os::Time::delay(1.0);
    }

    cout << moduleName << ": finding configuration files..." << endl;

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    Bottle cmd;
    cmd.clear();
    cmd.addString("world");
    cmd.addString("del");
    cmd.addString("all");

    portSim.write(cmd);
    yarp::os::Time::delay(1);   
    spamTable();
    yarp::os::Time::delay(3);
    
    yarp::os::Bottle pos1;

    pos1.clear();
    pos1.addDouble(-0.01);
    pos1.addDouble(0.61);
    pos1.addDouble(0.36);
    objectGeneratorSim::createObject(pos1, true);
    yarp::os::Time::delay(3);

    pos1.clear();
    pos1.addDouble(-0.07);
    pos1.addDouble(0.61);
    pos1.addDouble(0.29);
    objectGeneratorSim::createObject(pos1);
    yarp::os::Time::delay(3);

    cout<<"Configuration done."<<endl;

    return true;
}

void objectGeneratorSim::spamTable()
{
    yarp::os::Bottle size;
    size.clear();
    size.addDouble(0.24);
    size.addDouble(0.60);
    size.addDouble(0.46);

    yarp::os::Bottle pos;
    pos.clear();
    pos.addDouble(0);
    pos.addDouble(0.23);
    pos.addDouble(0.43);

    yarp::os::Bottle colour;
    colour.clear();
    colour.addDouble(0);
    colour.addDouble(0);
    colour.addDouble(0);
    std::string b = "box";

    objectGeneratorSim::createObject(b,size, pos, colour);
    std::cout<< "Table Spammed" << std::endl;
}

void objectGeneratorSim::createObject(std::string ob, yarp::os::Bottle size, yarp::os::Bottle pos, yarp::os::Bottle colour, bool target)
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
    //add radius
    pair.addDouble(size.get(0).asDouble());
    if (target==true){
        targets.append(pair);
    }else{
        objects.append(pair);
    }
}

void objectGeneratorSim::createObject(yarp::os::Bottle pos, bool target)
{
    yarp::os::Bottle s;
    yarp::os::Bottle c;
    s.clear();
    c.clear();
    s.addDouble(0.06);
    s.addDouble(0.06);
    s.addDouble(0.06);
    c.addDouble(255);
    c.addDouble(0);
    if(target==true)
        c.addDouble(255);
    else
        c.addDouble(0);

    objectGeneratorSim::createObject("box",s,pos,c,target);
    
}

void objectGeneratorSim::tf(Bottle* b)
{
    Vector v(4);
    for(int i=0;i<3;i++)
    {
        v[i]=b->get(i).asDouble();
    }
    v[3]=1;
    Vector new_v = H*v;
    for(int i=0;i<3;i++)
    {
        b->get(i)=new_v[i];
    }
}

void objectGeneratorSim::getCoordinates(std::string object, int id,double size, bool target)
{
    cmd.clear();
    cmd.addString("world");
    cmd.addString("get");
    cmd.addString(object);
    cmd.addInt(id);
    yarp::os::Bottle pos;
    pos.clear();
    stringstream ID;
    ID << object << id;
    
    portSim.write(cmd,pos);
    cout << pos.toString() << endl;
    tf(&pos);

    pos.addDouble(size);
    pos.addString(ID.str());

    if (target == false){
        objectGeneratorSim::positions.addList()=pos;
    }else{
        objectGeneratorSim::tpositions.append(pos);
    }
}

bool objectGeneratorSim::close() {
    portOutput.close();
    portInput.close();
    rpc.close();
    return true;
}


/* Respond function */
bool objectGeneratorSim::respond(const yarp::os::Bottle& bCommand, yarp::os::Bottle& bReply)
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
bool objectGeneratorSim::updateModule() {
    cout<<"."<<endl;
    positions.clear();
    tpositions.clear();

    Bottle &p = portOutput.prepare();
    p.clear();
    
    Bottle &t = portOutputTarget.prepare();
    t.clear();


    for(int i=0;i<listSize[0]-1+listSize[1]+listSize[2];i++)
    {
        cout << i << ": "<< objects.get(3*i).asString().c_str() << " " <<objects.get(3*i+1).asInt()<< endl;
        getCoordinates(objects.get(3*i).asString().c_str(), objects.get(3*i+1).asInt(),objects.get(3*i+2).asDouble());

    }
    getCoordinates(targets.get(0).asString().c_str(), targets.get(1).asInt(),targets.get(2).asDouble(),true);
    cout << "Target: " << targets.get(1).asString().c_str() << endl;
    cout << targets.toString() << endl;
    p.addList()=positions;
    t.append(tpositions);
    portOutput.write();
    portOutputTarget.write();


    return true;
}

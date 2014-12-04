

//   Include declaration file
#include "soundGenerator.h"
//   Include math library
#include <cmath>
#include <cstdlib>
#include <vector>


#if WIN32
// apparently not defined in windows
double log2(const double x)
{
    return (log(x)/log(2.0));
}
#endif


bool babbler::configure(yarp::os::ResourceFinder &rf)
{

    elapsedCycles = 0;
    f = 0;
    string moduleName = rf.check("name", Value("tuneBabbler")).asString().c_str();
    setName(moduleName.c_str());

    string moduleOutput = rf.check("output", Value("/microphone")).asString().c_str();

    int seed = rf.check("seed", Value(1)).asInt();

    samples = rf.check("samples", Value(512)).asInt();
    rate = rf.check("rate", Value(8000)).asInt();
    channels = rf.check("channels", Value(1)).asInt();

    bool    bEveryThingisGood = true;

    // create an output port
    port2output = moduleOutput;

    if (!portOutput.open(port2output.c_str())) {
        cout << getName() << ": Unable to open port " << port2output << endl;
        cout << "The microphone might be turned on" << endl;
        bEveryThingisGood &= false;
    }

    cout << moduleName << ": finding configuration files..." << endl;

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    srand(seed);

    return true;
}

//Generates frequencies between 220 and 880 Hz aprox. Separated as clear tones
int babbler::setNewFrequency(int F=220)
{
    return f = F;
}

//Generates frequencies between 220 and 880 Hz aprox. Separated as clear tones
int babbler::newRandomFrequency(int base_frequency = 220)
{
    f = base_frequency * ((int)pow(2, (rand() % 10)*0.25));
    return f;
}

//Defines new sound, having white noise + pure tone
int babbler::getNoisySinus(int wave_intensity=100, int random_offset=10)
{/*
    if (elapsedCycles % 40 == 0)
    {
        f = newRandomFrequency();
    }
    if ((elapsedCycles % 40) - 15 == 0)
    {
        f = 0;
    }
    */
    std::cout << "frequency: " << f << std::endl;
    //Format: ([mat][mo16](2 2048 2 512 2) {nums})(8000)
    //Constants chosen at random
    return ((unsigned int)(wave_intensity * (std::sin(f * 2 * M_PI*elapsedCycles)+1)+50 + rand() % random_offset));

}

int babbler::getValue(std::string soundType="noisySinus")
{
    if (soundType.compare("noisySinus")==0)
    {
        return getNoisySinus();
    }
    else
    {
        std::cout << 123 << std::endl;
        return 123;     // [ugo]: we must return something...
    }
}

void babbler::generateSound()
{
    
}


bool babbler::close() {
    portOutput.close();
    rpc.close();
    return true;
}


/* Respond function */
bool babbler::respond(const yarp::os::Bottle& bCommand, yarp::os::Bottle& bReply)
{

    std::string helpMessage = std::string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "F + (int)Frequency\n" +
        "RF \n" +
        "quit \n";

    bReply.clear();
    std::string keyWord = bCommand.get(0).asString().c_str();

    if (keyWord == "quit") {
        bReply.addString("quitting");
        return false;
    }
    else if (keyWord == "help") {
        cout << helpMessage;
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

    return true;
}



/* Called periodically every getPeriod() seconds */
bool babbler::updateModule() {

    
    yarp::sig::Sound &signal = portOutput.prepare();

    signal.resize(samples);
    signal.setFrequency(rate);
    for (unsigned int i = 0; i < samples; i++)
    {
        signal.set(getValue(), i);
    }

    portOutput.write();
   

    //delete signal;

    elapsedCycles++;


    return true;
}

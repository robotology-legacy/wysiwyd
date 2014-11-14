

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
int babbler::newFrequency(int base_frequency=220)
{
    return base_frequency * ((int)pow(2 ,(rand() % 10)*0.25));
}

//Defines new sound, having white noise + pure tone
int babbler::getNoisySinus(int wave_intensity=75, int random_offset=25)
{
    if (elapsedCycles % 40 == 0)
    {
        f = newFrequency();
    }
    if ((elapsedCycles % 40) - 15 == 0)
    {
        f = 0;
    }
    std::cout << "frequency: " << f << std::endl;
    //Format: ([mat][mo16](2 2048 2 512 2) {nums})(8000)
    //Constants chosen at random
    return ((unsigned int)(wave_intensity * (std::sin(f * 2 * M_PI*elapsedCycles)+1) + rand() % random_offset));

}

int babbler::getValue(std::string soundType="noisySinus")
{
    if (soundType.compare("noisySinus")==0)
    {
        return getNoisySinus();
    }
    else{ std::cout << 123 << std::endl; }
}

void babbler::generateSound()
{
    
}


bool babbler::close() {
    portOutput.close();
    rpc.close();
    return true;
}


bool babbler::respond(const Bottle& command, Bottle& reply) {
    reply.addString("nack");
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


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
    string moduleName = rf.check("name", Value("tuneBabbler")).asString().c_str();
    setName(moduleName.c_str());

    string moduleOutput = rf.check("output", Value("/microphone")).asString().c_str();

    int seed = rf.check("seed", Value(1)).asInt();

    unsigned int samples = rf.check("samples", Value(1)).asInt();
    unsigned int rate = rf.check("rate", Value(1)).asInt();
    unsigned int channels = rf.check("channels", Value(1)).asInt();

    bool    bEveryThingisGood = true;

    // create an output port
    port2output = moduleOutput;

    if (!portOutput.open(port2output.c_str())) {
        cout << getName() << ": Unable to open port " << port2outputNameFreq << endl;
        cout << "The microphone might be turned on" << endl;
        bEveryThingisGood &= false;
    }

    cout << moduleName << ": finding configuration files..." << endl;

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    srand(seed);

    return true;
}

void babbler::generateSound()
{
//Format: ([mat][mo16](2 2048 2 512 2) {nums})(8000)
    yarp::sig::Sound* signal = new yarp::sig::Sound;
    signal();
    signal.resize(samples);
    signal.setFrequency(rate);
    for (unsigned int i = 0, i < samples; i++)
    {
        signal.set(getValue(), i);
    }
}


bool babbler::close() {
    portInput.close();
    portSpectrumOutput.close();
    portOutputFreq.close();
    portOutputGap.close();
    delete[] buffer;
    return true;
}


bool babbler::respond(const Bottle& command, Bottle& reply) {
    reply.addString("nack");
    return true;
}

// Check if it works. Created without debugging...
double babbler::nextpow2(int num) {
    double y = (double)num;
    double x = log10(y) / log10(2.0);
    return pow(2.0, ceil(x));
}


/* Called periodically every getPeriod() seconds */
bool babbler::updateModule() {

    elapsedCycles++;
    // get the input
    yarp::sig::Sound* signal = portInput.read();

    if (signal)
    {
        /* Encoding of the fast Fourier Transform*/

        /* Number of Samples (ideally SAMPLES <= 2024 to ensure musical tone recognition)*/
        int SAMPLES = signal->getSamples();

        /* Sampling Frequency (ideally over 8KHz)*/
        int Fs = signal->getFrequency();

        /* Number of discrimanble frequencies*/
        int NFFT = (int)nextpow2(L); //Number of fast fourier transforms

        std::cout << "Incoming: Samples=" << SAMPLES << "\t SamplingFreq=" << Fs << "\t MaxFreq=" << Fs / 2 << "\t BufferSize=" << L << "\t Resolution=" << Fs / NFFT << std::endl;
     
        int K = (NFFT / 2) + 1;
        vector<double> sig;
        sig.resize(K);
        vector<double> f;
        f.resize(NFFT);// frequencies vector
        complex *pSignal = new complex[L];

        for (int i=0; i<(L-SAMPLES);i++)
        {
            buffer[i] = buffer[i+SAMPLES];
        }
        for (int i = 0; i<SAMPLES; i++)
        {
            buffer[(L - SAMPLES) + i] = signal->get(i);
        }
        //cout << endl << endl << &buffer<<endl<<endl<<endl;
        /*
        for (int i=0;i<SAMPLES;i++)
        {
            buffer.push_back(signal->get(i));
            buffer.pop_back();
        }*/

        for (int i = 0; i < L; i++)
        {
            pSignal[i] = buffer[i]; 
        }

        complex *fftOut = new complex[NFFT];
        CFFT::Forward(pSignal, fftOut, NFFT);
        int maxAmpIdx = 70;

        /*Compute frequency amplitude*/
        for (int i = 1; i < K; i++)
        {
            double x = fabs( fftOut[i].norm() / (double)SAMPLES );
            sig[i] = 2 * x; // Amplitude of the signal

            if (i > 70)
            {
                if (sig[i] > sig[maxAmpIdx]) //stores max amplitude index
                {
                    //cout << "Storing new peak!" << endl;
                    maxAmpIdx = i;
                }    
            }
        }

        /*Compute Frequency vector*/
        for (int i = 1; i < K; i++)
        {
            double j = i / (double)K;
            f[i] = Fs / 2 * j; // This calculates the frequency. j should be multiplied by Fs/2 if everythng is right
        }

        double newMaxFreq = f[maxAmpIdx];
        
        // Print frequency-amplitude
        // cout << "Peak Frequency at = " << newMaxFreq << "\t Amplitude = " << sig[maxAmpIdx] << endl;

        
        double gap = log(newMaxFreq/freqReference)*12/log(2.);
        cout << "Peak Frequency at = " << newMaxFreq << "\t Amplitude = " << sig[maxAmpIdx] << "\t Note gap : " << gap << endl;

        yarp::os::Bottle &treatedFrequency = portOutputFreq.prepare();
        treatedFrequency.clear();

        yarp::os::Bottle &treatedNote = portOutputGap.prepare();
        treatedNote.clear();

        yarp::os::Bottle &SpectralSignal = portSpectrumOutput.prepare();
        SpectralSignal.clear();

        /*
        yarp::os::Bottle &SpectralBool = portSpectBoolOutput.prepare();
        SpectralBool.clear();
        */
        delete[] pSignal;
        pSignal = NULL;

        for (int i = 1; i<K; i++)
            {
                SpectralSignal.addDouble(log10(sig[i]));
                /*
                if (log10(sig[i])>15)
                {
                    SpectralBool.addDouble(1.0);
                }else{
                    SpectralBool.addDouble(0.0);
                }*/
            }

        treatedNote.addDouble(gap);
        treatedFrequency.addDouble(newMaxFreq);

        portOutputFreq.write();
        portOutputGap.write();
        portSpectrumOutput.write();

        delete[] fftOut;
        fftOut = NULL;
        delete signal;

    }

    return true;
}

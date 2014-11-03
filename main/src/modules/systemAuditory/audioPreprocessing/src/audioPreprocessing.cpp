//   CFFT.cpp - impelementation of class
//   of fast Fourier transform - FFT
//
//   The code is property of LIBROW
//   You can use it on your own
//   When utilizing credit LIBROW site

//   Include declaration file
#include "audioPreprocessing.h"
//   Include math library
#include <math.h>
#include <stdlib.h>
#include <vector>



bool CFFT::configure(yarp::os::ResourceFinder &rf)
{
    L = 4096;
    buffer = new complex[L];
    string moduleName = rf.check("name", Value("audioPreprocessing")).asString().c_str();
    setName(moduleName.c_str());


    string moduleInput = rf.check("input", Value("/microphone")).asString().c_str();
    // buffer size is set to 4096 because: 
    //      - It must be a power of 2
    //      - As greater de num of samples (L), greater the resolution (Res)
    //      - Remember that this should be tuned in function of the sampling frequency (Fs) of the microphone
    //      - Res (Hz/sample) = Fs / L
    
    cout << "checking current buffer size..." << endl;
    /*if (buffer.size() < L) 
        {
            cout << "Reallocating memory for buffer ... "<< endl;
            buffer.resize(L);
            for (int i=0;i<L;i++)
            {
                buffer[i] = 0;
            }
        }
    */
    freqReference = rf.check("freqReference", Value(440.)).asDouble();

    bool    bEveryThingisGood = true;


    // Check buffer size as a power of 2
    int L = rf.check("BufferSize", Value(4096)).asInt();
    if ( (10*(int)log2((double)L)%10) != 0)
    {
        cout << getName() << ": ERROR! Buffer size must be a power of 2! Please, change the value (Default: 4096) "<<endl;
                bEveryThingisGood &= false;

    }

    // create one input port from audio
    // Open port2audio
    port2audioName = "/";
    port2audioName += getName() + "/audio:i";

    if (!portInput.open(port2audioName.c_str())) {
        cout << getName() << ": Unable to open port " << port2audioName << endl;
        bEveryThingisGood &= false;
    }


    // create an output port for frequency
    port2outputNameFreq = "/";
    port2outputNameFreq += getName() + "/freq:o";

    if (!portOutputFreq.open(port2outputNameFreq.c_str())) {
        cout << getName() << ": Unable to open port " << port2outputNameFreq << endl;
        bEveryThingisGood &= false;
    }


    //the output for spectrum of frequencies
    port2outputNameSpectrum = "/";
    port2outputNameSpectrum += getName() + "/freqSpectrum:o";

    if (!portSpectrumOutput.open(port2outputNameSpectrum.c_str())) {
        cout << getName() << ": Unable to open port " << port2outputNameSpectrum << endl;
        bEveryThingisGood &= false;
    }

    port2outputNameBool = "/";
    port2outputNameBool += getName() + "/freqSpectrum:o";

    if (!portSpectBoolOutput.open(port2outputNameBool.c_str())) {
        cout << getName() << ": Unable to open port " << port2outputNameBool << endl;
        bEveryThingisGood &= false;
    }

    // create an output port for gap
    port2outputNameGap = "/";
    port2outputNameGap += getName() + "/note:o";

    if (!portOutputGap.open(port2outputNameGap.c_str())) {
        cout << getName() << ": Unable to open port " << port2outputNameGap << endl;
        bEveryThingisGood &= false;
    }

    // connect input port to audio
    while (!Network::connect(moduleInput, port2audioName.c_str()))
    {
        std::cout << "Trying to get input from microphone..." << std::endl;
        yarp::os::Time::delay(1.0);
    }

    cout << moduleName << ": finding configuration files..." << endl;
    //period = rf.check("period", Value(0.1)).asDouble();

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    return true;
}


bool CFFT::close() {
    portInput.close();
    portSpectrumOutput.close();
    portOutputFreq.close();
    portOutputGap.close();
    delete[] buffer;
    return true;
}


bool CFFT::respond(const Bottle& command, Bottle& reply) {
    reply.addString("nack");
    return true;
}

// Check if it works. Created without debugging...
double CFFT::nextpow2(int num) {
    double y = (double)num;
    double x = log10(y) / log10(2.0);
    return pow(2.0, ceil(x));
}


/* Called periodically every getPeriod() seconds */
bool CFFT::updateModule() {
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

//   FORWARD FOURIER TRANSFORM
//     Input  - input data
//     Output - transform result
//     N      - length of both input data and result
bool CFFT::Forward(const complex *const Input, complex *const Output, const unsigned int N)
{
    //   Check input parameters
    if (!Input || !Output || N < 1 || N & (N - 1))
        return false;
    //   Initialize data
    Rearrange(Input, Output, N);
    //   Call FFT implementation
    Perform(Output, N);
    //   Succeeded
    return true;
}

//   FORWARD FOURIER TRANSFORM, INPLACE VERSION
//     Data - both input data and output
//     N    - length of input data
bool CFFT::Forward(complex *const Data, const unsigned int N)
{
    //   Check input parameters
    if (!Data || N < 1 || N & (N - 1))
        return false;
    //   Rearrange
    Rearrange(Data, N);
    //   Call FFT implementation
    Perform(Data, N);
    //   Succeeded
    return true;
}

//   INVERSE FOURIER TRANSFORM
//     Input  - input data
//     Output - transform result
//     N      - length of both input data and result
//     Scale  - if to scale result
bool CFFT::Inverse(const complex *const Input, complex *const Output, const unsigned int N, const bool Scale /* = true */)
{
    //   Check input parameters
    if (!Input || !Output || N < 1 || N & (N - 1))
        return false;
    //   Initialize data
    Rearrange(Input, Output, N);
    //   Call FFT implementation
    Perform(Output, N, true);
    //   Scale if necessary
    if (Scale)
        CFFT::Scale(Output, N);
    //   Succeeded
    return true;
}

//   INVERSE FOURIER TRANSFORM, INPLACE VERSION
//     Data  - both input data and output
//     N     - length of both input data and result
//     Scale - if to scale result
bool CFFT::Inverse(complex *const Data, const unsigned int N, const bool Scale /* = true */)
{
    //   Check input parameters
    if (!Data || N < 1 || N & (N - 1))
        return false;
    //   Rearrange
    Rearrange(Data, N);
    //   Call FFT implementation
    Perform(Data, N, true);
    //   Scale if necessary
    if (Scale)
        CFFT::Scale(Data, N);
    //   Succeeded
    return true;
}

//   Rearrange function
void CFFT::Rearrange(const complex *const Input, complex *const Output, const unsigned int N)
{
    //   Data entry position
    unsigned int Target = 0;
    //   Process all positions of input signal
    for (unsigned int Position = 0; Position < N; ++Position)
    {
        //  Set data entry
        Output[Target] = Input[Position];
        //   Bit mask
        unsigned int Mask = N;
        //   While bit is set
        while (Target & (Mask >>= 1))
            //   Drop bit
            Target &= ~Mask;
        //   The current bit is 0 - set it
        Target |= Mask;
    }
}

//   Inplace version of rearrange function
void CFFT::Rearrange(complex *const Data, const unsigned int N)
{
    //   Swap position
    unsigned int Target = 0;
    //   Process all positions of input signal
    for (unsigned int Position = 0; Position < N; ++Position)
    {
        //   Only for not yet swapped entries
        if (Target > Position)
        {
            //   Swap entries
            const complex Temp(Data[Target]);
            Data[Target] = Data[Position];
            Data[Position] = Temp;
        }
        //   Bit mask
        unsigned int Mask = N;
        //   While bit is set
        while (Target & (Mask >>= 1))
            //   Drop bit
            Target &= ~Mask;
        //   The current bit is 0 - set it
        Target |= Mask;
    }
}

//   FFT implementation
void CFFT::Perform(complex *const Data, const unsigned int N, const bool Inverse /* = false */)
{
    const double pi = Inverse ? 3.14159265358979323846 : -3.14159265358979323846;
    //   Iteration through dyads, quadruples, octads and so on...
    for (unsigned int Step = 1; Step < N; Step <<= 1)
    {
        //   Jump to the next entry of the same transform factor
        const unsigned int Jump = Step << 1;
        //   Angle increment
        const double delta = pi / double(Step);
        //   Auxiliary sin(delta / 2)
        const double Sine = sin(delta * .5);
        //   Multiplier for trigonometric recurrence
        const complex Multiplier(-2. * Sine * Sine, sin(delta));
        //   Start value for transform factor, fi = 0
        complex Factor(1.);
        //   Iteration through groups of different transform factor
        for (unsigned int Group = 0; Group < Step; ++Group)
        {
            //   Iteration within group 
            for (unsigned int Pair = Group; Pair < N; Pair += Jump)
            {
                //   Match position
                const unsigned int Match = Pair + Step;
                //   Second term of two-point transform
                const complex Product(Factor * Data[Match]);
                //   Transform for fi + pi
                Data[Match] = Data[Pair] - Product;
                //   Transform for fi
                Data[Pair] += Product;
            }
            //   Successive transform factor via trigonometric recurrence
            Factor = Multiplier * Factor + Factor;
        }
    }
}

//   Scaling of inverse FFT result
void CFFT::Scale(complex *const Data, const unsigned int N)
{
    const double Factor = 1. / double(N);
    //   Scale all data entries
    for (unsigned int Position = 0; Position < N; ++Position)
        Data[Position] *= Factor;
}

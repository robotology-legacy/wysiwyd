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
    string moduleName = rf.check("name", Value("audioPreprocessing")).asString().c_str();
    setName(moduleName.c_str());

    freqReference = rf.check("freqReference", Value(440.)).asDouble();

    bool    bEveryThingisGood = true;


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

    // create an output port for gap
    port2outputNameGap = "/";
    port2outputNameGap += getName() + "/note:o";

    if (!portOutputGap.open(port2outputNameGap.c_str())) {
        cout << getName() << ": Unable to open port " << port2outputNameGap << endl;
        bEveryThingisGood &= false;
    }

    // connect input port to audio
    while (!Network::connect("/mic", port2audioName.c_str()))
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
        std::cout << "We got a sound ! Samples=" << signal->getSamples() << std::endl;

        /* Encoding of the fast Fourier Transform*/

        /* Number of Samples (ideally SAMPLES <= 2024 to ensure musical tone recognition)*/
        int SAMPLES = signal->getSamples();

        /* Sampling Frequency (ideally over 8KHz)*/
        int Fs = signal->getFrequency();
        std::cout << "Current Sampling Frequency is " << Fs << endl;
        std::cout << "Current Max Frequency is " << Fs/2 << endl;

        /* Number of discrimanble frequencies*/
        int NFFT = (int)nextpow2(SAMPLES); //Number of fast fourier transforms
        
        /* Experiments should be made to ensure that the sampling frequency is good enough for speech recognition */
        std::cout << "Current Frequency Ressolution is " << Fs/NFFT << endl;
        
        int K = (NFFT / 2) + 1;
        vector<double> sig;
        sig.resize(K);
        vector<double> f;
        f.resize(NFFT);// frequencies vector
        complex *pSignal = new complex[SAMPLES];


        for (int i = 0; i < SAMPLES; i++)
        {
            //pSignal[i] = signal->getSafe(i, 0); 
            //pSignal[i] = signal->get(i) / 65535.0;;
            pSignal[i] = signal->get(i);
        }

        complex *fftOut = new complex[NFFT]; //complex[SAMPLES]
        //CFFT::Forward(pSignal, pSignalOut, SAMPLES);
        CFFT::Forward(pSignal, fftOut, NFFT);
        //cout << "Frequency is " <<pSignal;
        //int MaxFreqIdx = 0;
        double MaxAmpIdx = 1;
        /*Compute frequency amplitude*/
        for (int i = 1; i < K; i++)
        {

            double x = fftOut[i].norm() / (double)SAMPLES;
            if (x<0){ x = -x; }
            sig[i] = 2 * x; // Amplitude of the signal
            
            if (sig[i] > sig[MaxAmpIdx]) //stores max amplitude index
            {
                MaxAmpIdx = i;
            }
            
        }

        /*Compute Frequency vector*/
        for (int i = 0; i < K; i++)
        {
            double j = i / (double)K;
            f[i] = Fs / 2 * j; // This calculates the frequency. j should be multiplied by Fs/2 if everythng is right
        }

        double newMaxFreq = f[MaxAmpIdx];
        
        // Print frequency-amplitude
        // cout << "Peak Frequency at = " << newMaxFreq << "\t Amplitude = " << sig[MaxAmpIdx] << endl;

        
        double gap = log(newMaxFreq/freqReference)*12/log(2.);
        cout << "Peak Frequency at = " << newMaxFreq << "\t Amplitude = " << sig[MaxAmpIdx] << "\t Note gap : " << gap << endl;

        yarp::os::Bottle &treatedFrequency = portOutputFreq.prepare();
        treatedFrequency.clear();

        yarp::os::Bottle &treatedNote = portOutputGap.prepare();
        treatedNote.clear();

        yarp::os::Bottle &SpectralSignal = portSpectrumOutput.prepare();
        SpectralSignal.clear();

        /*
        for (int i = 50; i < 100; i++)
        {
        treatedSignal.addDouble(pSignalOut[i].norm() / maxFreq);
        }
        */

       
        /*
        check == 1 ->  send amplitude values to the output
        check == 2 ->  send 1 or 0 if frequency over a threshold
        */
        

        for (int i = 1; i<K; i++)
            {
                SpectralSignal.addDouble(log10(sig[i]));
            }

        treatedNote.addDouble(gap);
        treatedFrequency.addDouble(newMaxFreq);

        portOutputFreq.write();
        portOutputGap.write();
        portSpectrumOutput.write();

        //   Free memory
        delete[] pSignal;
    }

    return true;
}


/* Called periodically every getPeriod() seconds
bool CFFT::updateModule() {

// get the input
yarp::sig::Sound* signal = portInput.read();

if (signal)
{
std::cout << "We got a sound ! (Samples=" <<signal->getSamples()<< std::endl;

int SAMPLES = signal->getSamples();

complex *pSignal = new complex[SAMPLES];

for (int i = 0; i < SAMPLES; i++)
{
//pSignal[i] = signal->getSafe(i, 0);
pSignal[i] = signal->get(i) / 65535.0;;
}

//   Apply FFT
//CFFT::Forward(pSignal, SAMPLES);

complex *pSignalOut = new complex[SAMPLES];
CFFT::Forward(pSignal, pSignalOut, SAMPLES);
//cout << "Frequency is " <<pSignal;

double meanFreq = 0.0;
double maxFreq = 0.0;
int bestIndex = 0;
double weightedMean = 0.0;

for (int i = 50; i < 100; i++)
{
double currentFreqValue = pSignalOut[i].norm();
weightedMean += i * pSignalOut[i].norm();
meanFreq += currentFreqValue;
if (currentFreqValue>maxFreq)
{
bestIndex = i;
maxFreq = currentFreqValue;
}
}
weightedMean /= meanFreq;
meanFreq /= SAMPLES;
cout << "Weighted Average Freq = " << weightedMean << "\t Max Frequency obtained in index=" << bestIndex << endl;

yarp::os::Bottle &treatedSignal = portOutput.prepare();
treatedSignal.clear();
for (int i = 50; i < 100; i++)
{
treatedSignal.addDouble(pSignalOut[i].norm() / maxFreq);
}
portOutput.write();

//   Free memory
delete[] pSignal;
}

return true;
} */







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

//   fft.h - declaration of class
//   of fast Fourier transform - FFT
//
//   The code is property of LIBROW
//   You can use it on your own
//   When utilizing credit LIBROW site

#ifndef _FFT_H_
#define _FFT_H_

//   Include complex numbers header
#include "complex.h"

class CFFT : public RFModule {
private:

    Port        rpc;
    // This is the port, I don't know yet where it should be declared
    yarp::os::BufferedPort<yarp::sig::Sound> portInput;                // a port to communicate with autobiographicalMemory
    // This is the port, I don't know yet where it should be declared
    yarp::os::BufferedPort<yarp::os::Bottle> portOutput;                // a port to communicate with autobiographicalMemory

    string port2audioName;
    string port2outputName;

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule()
    {
        return true;
    }

    bool close();
    void    mainLoop();
    double nextpow2(int num);
    bool updateModule();
    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);

	//   FORWARD FOURIER TRANSFORM
	//     Input  - input data
	//     Output - transform result
	//     N      - length of both input data and result
	static bool Forward(const complex *const Input, complex *const Output, const unsigned int N);

	//   FORWARD FOURIER TRANSFORM, INPLACE VERSION
	//     Data - both input data and output
	//     N    - length of input data
	static bool Forward(complex *const Data, const unsigned int N);

	//   INVERSE FOURIER TRANSFORM
	//     Input  - input data
	//     Output - transform result
	//     N      - length of both input data and result
	//     Scale  - if to scale result
	static bool Inverse(const complex *const Input, complex *const Output, const unsigned int N, const bool Scale = true);

	//   INVERSE FOURIER TRANSFORM, INPLACE VERSION
	//     Data  - both input data and output
	//     N     - length of both input data and result
	//     Scale - if to scale result
	static bool Inverse(complex *const Data, const unsigned int N, const bool Scale = true);

protected:
	//   Rearrange function and its inplace version
	static void Rearrange(const complex *const Input, complex *const Output, const unsigned int N);
	static void Rearrange(complex *const Data, const unsigned int N);

	//   FFT implementation
	static void Perform(complex *const Data, const unsigned int N, const bool Inverse = false);

	//   Scaling of inverse FFT result
	static void Scale(complex *const Data, const unsigned int N);
};

#endif

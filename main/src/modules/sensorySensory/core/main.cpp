
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <yarp/os/all.h>
#include <map>
#include <yarp/sig/all.h>
#include <cv.h>
#include <unistd.h>
#include <stdio.h> // for replacing files

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

static const char *DEFAULT_CONFIG_FILE = "default.ini";
static const double SQ2 = sqrt(2);

//Inicialization functions for Input:

//Gaussian
vector<double>* gaussianInput(vector<double>* x, double sigma, double mu)
{
    for(unsigned int i=0;i<x->size();i++)
    {
        x->at(i) = (double)(1/(sigma*sqrt(2*3.141592)))*exp(-pow(i-mu,2)/(2*pow(sigma,2)));
    }
    return x;
}


//Definition of neuron class
class neuron
{
public:
    double activity;
    double diff_activity;
    uchar* Rpixel;
    uchar* Gpixel;
    uchar* Bpixel;
    bool firing;

    neuron()
    {
        activity = 0;
        diff_activity = 0;
    }
    bool updateActivity(){
        activity += diff_activity;
        diff_activity = 0;
        return true;
    }
    void colorUpdate()
    {
        *Rpixel = (uchar)(180*(activity));
        *Gpixel = (uchar)(180*(activity));
        *Bpixel = (uchar)(180*(activity));
        if (firing)
        {
            *Rpixel = (uchar)(255);
            *Gpixel = (uchar)(0);
            *Gpixel = (uchar)(0);
        }
    }
    void normalizeActivity(double M)
    {
        activity = (activity)/M;
    }

private:

};




//Definition of connected neuron class
class c_neuron: public neuron
{
public:
    unsigned int connection_size;
    vector<int> connections;
    vector<double> weights;

    c_neuron()
    {
        neuron();
        connection_size = 0;

    }
    bool resizeConnections(int n)
    {

        connections.resize(n);
        weights.resize(n);
        connection_size = n;
        return true;
    }
};




//Definition of neural model class
class NeuralModel
{

public:
    double max_activity;
    double total_activity;
    double E_percent;
    double learning_rate;
    int size;
    unsigned int sizes[2];
    IplImage* img;
    vector<c_neuron*> neurons;
    vector<c_neuron*> firing;
    vector<c_neuron*> input;

    bool updateInput(vector<double> in)
    {
        //input.resize(in.size());
        normalizeInput(in);
        if (in.size()!=input.size()){
            cout<<"ERROR: Input dimensions have changed!"<<endl;
            cout<< "Input size: "<< in.size() << endl << "Input neurons: "<< input.size() << endl;
            return false;
        }else{
            for(unsigned int i=0;i<in.size();i++)
            {
                //c_neuron* N = input[i];
                (input[i])->activity = in[i];
            }
            return true;
        }
    }

    bool updateLoop(vector<double> in)
    {

        updateInput(in);
        processActivity();
        updateActivity();
        normalizeAndSelect();
        cout << firing.size()<<endl;
        cout << (firing[0])->activity<<endl;
        updateWeights();
        resetLoop();

        return true;
    }

    NeuralModel( int n_neurons, vector<double> loaded_mean, vector<double> loaded_std, double a = 0.001,double E = 0.95, double LR = 0.1, int input_size = 2048)
    {
        if (loaded_std.size() != (unsigned int)input_size)
        {
            vector<double> New(input_size);
            setSDev(New);
        }else{
            setSDev(loaded_std);
        }
        if (loaded_mean.size() != (unsigned int)input_size)
        {
            vector<double> New(input_size);
            setMean(New);
        }else{
            setMean(loaded_mean);
        }
        size=0;
        alpha = a;
        sizes[0]=(int)input_size;
        sizes[1]=(int)n_neurons/input_size;
        E_percent = E;
        learning_rate = LR;
        generateModel(n_neurons,input_size);

    }

    vector<double> getMean(){return Mean;}
    vector<double> getSDev(){return SDev;}
    void setMean(vector<double> m){Mean = m;}
    void setSDev(vector<double> s){SDev = s;}


private:
    vector<double> SDev;
    vector<double> Mean;
    double alpha;

    vector<double> normalizeInput(vector<double> in)
    {
        for (unsigned int i=0;i<in.size();i++)
        {
            Mean[i] = Mean[i]*(1-alpha) + alpha*in[i];
            SDev[i] = sqrt(SDev[i]*SDev[i] * (1-alpha) + alpha*pow(in[i]-Mean[i],2));
            in[i] = 0.5+0.5*tanh((in[i]-Mean[i])*SDev[i]/SQ2);
        }
        return in;
    }


    bool generateModel(int number_neurons = 8192, int max_neurons = 5, int min_neurons = 1, int input_size = 2048, string input_init = "gaussian")
    {
        img = cvCreateImage(cvSize(sizes[0],sizes[1]),IPL_DEPTH_8U,3);
        //Reallocate neural model size
        neurons.resize(number_neurons);
        //Generate the neurons        
        for (int n = 0; n < number_neurons; n++)
        {
            c_neuron* N = new c_neuron();
            N->Rpixel = (uchar*) (img->imageData + 3*n+0);
            N->Gpixel = (uchar*) (img->imageData + 3*n+1);
            N->Bpixel = (uchar*) (img->imageData + 3*n+2);

            if(n%input_size<1)
            {
                N->resizeConnections(2);
                N->connections[1]=n+1;
                if(number_neurons-n<input_size){
                    N->connections[2]=n+input_size;
                }
            }else if(n%input_size<(input_size-1)){
                N->resizeConnections(3);
                N->connections[1]=n+1;
                N->connections[2]=n-1;
                if(number_neurons-n<input_size){
                    N->connections[3]=n+input_size;
                }
            }else{
                N->resizeConnections(2);
                N->connections[1]=n-1;
                if(number_neurons-n<input_size){
                    N->connections[2]=n+input_size;
                }
            }
            for (unsigned int x = 0; x<N->connection_size;x++)
            {
                N->weights[x]=(rand()%1000)/(double)1000;
            }
            /*
            N->resizeConnections(rand() % max_neurons + min_neurons);
            //Add random weights to the connections. This should be a topology functions
            for (unsigned int i=0; i<N->connection_size;i++)
            {
                N->connections[i] = rand()%number_neurons;
                N->weights[i] = (rand()%1000)/(double)1000.0;
            }
            */
            neurons[n] = N;

        }

        //Reallocate input vector
        input.resize(input_size);
        //Initialize input neurons
        for (int n = 0; n < input_size; n++)
        {
            c_neuron* I = new c_neuron();
            // cout<<"WARNING: Inputs are connected to 1 single neuron" << endl;
            string gaussian = "gaussian";
            //if (input_init.compare(gaussian) == 0)
            if(true)
            {
                I->resizeConnections((input_size));
                for(int z=0;z<input_size;z++){I->connections[z]=z;}
                I->weights = *gaussianInput(&I->weights,(double) 3,(double) n);
            }else{
                I->resizeConnections((int) 1);
                //Add initial connection weights
                if (input_size<number_neurons)
                {
                    for (unsigned int i = 0; i < I->connection_size ;i++)
                    {
                        I->connections[i] = i;
                        cout << "WARNING: Input weights set at random. A proper implementation should be set. "<<endl;
                        I->weights[i] = (rand()%1000)/(double)1000.0;
                    }
                }else{
                    //cout<<"WARNING: More inputs than outputs. Not implemented yet a method to cope with it. "<<endl;
                }
            }
            input[n] = I;

        }

        return true;
    }

    void processActivity()
    {
        //Propagate activity from inputs
        for (unsigned int i = 0; i<input.size();i++)
        {
            //c_neuron* I = input[i];

            for (unsigned int j = 0; j<(input[i])->connection_size;j++)
            {
                //c_neuron* i_neu = neurons[(input[i])->connections[j]];
                (neurons[(input[i])->connections[j]])->diff_activity += (input[i])->weights[j]*(input[i])->activity;
            }
        }

        //Propagate activity in neurons
        for(unsigned int n=1;n < neurons.size();n++)
        {
            //c_neuron* N = neurons[n];
            for (unsigned int c = 1; c < (neurons[n])->connection_size;c++)
            {
                //c_neuron* neu = neurons[(neurons[n])->connections[c]];
                (neurons[(neurons[n])->connections[c]])->diff_activity += ((neurons[n])->weights[c])*((neurons[n])->activity);
            }
        }
    }

    void updateActivity()
    {

        for (unsigned int n = 0; n<neurons.size();n++)
        {
            //c_neuron* N = neurons[n];
            (neurons[n])->updateActivity();
            total_activity += (neurons[n])->activity;
            max_activity = max((neurons[n])->activity,max_activity);
            //min_activity = min((neurons[n])->activity,min_activity);
        }
    }

    // Normalize: Total activity of the network is 1
    // Select: E%-max
    void normalizeAndSelect()
    {
        double Emax = E_percent * max_activity / total_activity;
        for (unsigned int n = 0; n<neurons.size();n++)
        {
            //c_neuron* N = neurons[n];

            /* // old normalize-select version
            (neurons[n])->normalizeActivity(min_activity, max_activity);
            if ((neurons[n])->activity>E_percent)
            {
                firing.push_back(neurons[n]);
                (neurons[n])->firing += 1;
                //cout<<"I pushed something!"<<endl;
            }*/

            (neurons[n])->normalizeActivity(total_activity);
            if ((neurons[n])->activity>Emax)
            {
                firing.push_back(neurons[n]);
                (neurons[n])->firing += 1;
                //cout<<"I pushed something!"<<endl;
            }

            (neurons[n])->colorUpdate();
        }
    }

    void updateWeights()
    {
        //for (unsigned int n = 0; n<firing.size();n++)
        for (unsigned int n = 0; n<neurons.size();n++)
        {
            //c_neuron* N = firing[n];
            //c_neuron* N = neurons[n];
            for (unsigned int c = 1; c < (neurons[n])->connection_size;c++)
            {
                //c_neuron* C = neurons[(neurons[n])->connections[c]];
                // Learning Rule
                //N->weights[c] += learning_rate*C->activity * (N->activity - N->weights[N->connections[c]]*C->activity);
                (neurons[n])->weights[c] += learning_rate*((neurons[(neurons[n])->connections[c]])->diff_activity/max_activity - (neurons[n])->diff_activity/max_activity);
            }

            (neurons[n])->diff_activity=0;
            (neurons[n])->firing &= 0;
        }
        firing.clear();
        firing.resize(0);
        //cout<<firing.size()<<endl;
    }

    void resetLoop()
    {
        total_activity = 0;
        max_activity = 0;
    }



};



//RF Module for Yarp
class NNmodel :public RFModule
{
    NeuralModel* neural_model;
    yarp::os::BufferedPort<yarp::os::Bottle> portFromCoclea;
    BufferedPort<ImageOf <PixelRgb> > imagePort;
    string      portFromCocleaName;
    string imagePortName;
    Port        rpc;
    int pixel_width;
    string      selfpath;
    string      conf_file;


public:
    ofstream    outfile; //soon will be deprecated?
    //ifstream    infile;
    std::vector<double> SDev;
    std::vector<double> Mean;
    bool activity_image;

    //retrieves executable path for saving files
    std::string get_selfpath() {
        char buff[1024];
        ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff)-1);
        if (len != -1) {
          buff[len] = '\0';
          return std::string(buff);
        } else {
            /* handle error condition */
            cout << "ERROR: PATH NOT OBTAINED" << endl;
            return "./";
        }
    }

    // Saves in a file the mean and standard deviation for posterior input normalization
    void saveNormals()
    {
        string inName = selfpath.substr(0,selfpath.find("/NNmodel")) + "/../share/wysiwyd/contexts/cvz/" + conf_file;
        ofstream tmpfile;
        ifstream infile;
        infile.open(inName.c_str());
        tmpfile.open("ini.tmp");

        std::string line;
        string match[2];
        match[0].assign("mean");
        match[1].assign("std");
        bool W = false;

        cout.precision(8);

        while (std::getline(infile, line))
        {
            cout << 1 << endl;
            //string aux;
            //std::istringstream iss(line);
            //cout << Mean[0]<<endl;
            if (line.compare(match[0].c_str())>4)
            {
                tmpfile << "mean\t\t("<< Mean[0];
                for (unsigned int i = 1;i<Mean.size();i++)
                {
                    tmpfile << " " << Mean[i];
                }
                tmpfile << ")"<<endl;

            }else if (line.compare(match[1].c_str())>3)
            {
                tmpfile << "std\t\t("<<SDev[0];
                for (unsigned int j = 1;j<SDev.size();j++)
                {
                    tmpfile << " " << SDev[j];
                }
                tmpfile << ")"<<endl;
                W += 1;
            }else{
                tmpfile << line<<endl;
            }
        }

        if (!W)
        {
            tmpfile << "[Auditory]"<<endl;

            tmpfile << "mean\t\t"<<"("<< Mean[0];
            for (unsigned int i = 1;i<Mean.size();i++)
            {

                cout << " " << Mean[i];
            }
            tmpfile << ")"<<endl;

            tmpfile << "std\t\t("<<SDev[0];
            for (unsigned int j = 1;j<SDev.size();j++)
            {
                tmpfile << " " << SDev[j];
            }
            tmpfile << ")"<<endl;
        }

        tmpfile.close();
        infile.close();
        rename("ini.tmp",inName.c_str());

    }


    bool configure(yarp::os::ResourceFinder &rf)
    {


        pixel_width = 3;

        string moduleName = rf.check("name", Value("NeuralModel")).asString().c_str();
        setName(moduleName.c_str());
        string moduleInput = rf.check("input", Value("/audioPreprocessing/freqSpectrum:o")).asString().c_str();
        string oFileName = rf.check("ofile", Value("out.txt")).asString().c_str();
        //outfile.open(fileName);
        string iFileName = rf.check("ifile", Value("in.ini")).asString().c_str();
        //Mean = rf.check("Means", Value(0)).asDouble();
        conf_file = rf.check("from", Value(DEFAULT_CONFIG_FILE)).asString().c_str();

        activity_image = rf.check("activityImage");

        if (rf.check("std"))
        {
            yarp::os::Bottle* sMask = rf.find("std").asList();
            for (int i = 0; i < sMask->size(); i++)
                SDev.push_back(sMask->get(i).asDouble());
            neural_model->setSDev(SDev);
        }

        if (rf.check("mean"))
        {
            yarp::os::Bottle* mMask = rf.find("mean").asList();
            for (int i = 0; i < mMask->size(); i++)
                Mean.push_back(mMask->get(i).asDouble());
            neural_model->setMean(Mean);
        }

        // Read exe path
        selfpath = get_selfpath();
        std::cout << selfpath << std::endl;

        neural_model = new NeuralModel((int)16384, Mean, SDev);
        outfile.open("out.txt");
        bool    bEveryThingisGood = true;

        //configure input port
        portFromCocleaName = "/";
        portFromCocleaName += getName() + "/coclea:i";

        if(!portFromCoclea.open(portFromCocleaName.c_str()))
        {
            cout << getName() << ": Unable to open port " << portFromCocleaName << endl;
            bEveryThingisGood &= false;
        }

        if (activity_image){
            imagePortName = "/";
            imagePortName += getName() + "/image:o";
            if(!imagePort.open(imagePortName.c_str()))
            {
                cout << getName() << ": Unable to open port " << imagePortName << endl;
                bEveryThingisGood &= false;
            }
        }

        while (!Network::connect(moduleInput, portFromCocleaName.c_str()))
        {
            std::cout << "Trying to get input from FFT..." << std::endl;
            yarp::os::Time::delay(1.0);
        }

        //output port not configured
        cout<<"WARNING: Output port not configured!"<<endl;

        rpc.open(("/" + moduleName + "/rpc").c_str());
        attach(rpc);

        return true;
	}

    bool printActivity()
    {
        for (unsigned int n = 0; n<neural_model->neurons.size();n++)
        {
            if(n!=0){outfile<<";";}
            c_neuron* N = neural_model->neurons[n];
            outfile << N->activity;
        }
        outfile << endl;
        return true;
    }


	bool updateModule()
	{


        Bottle* input_bottle = portFromCoclea.read();
        vector<double> input_vector;
        if (input_bottle->size()!= 2048)
        {
            cout<<"input different than 2048!!"<<endl;
            cout<<"input size" << input_bottle->size()<<endl;
            cout<<"content of bottle" << input_bottle->toString()<<endl;
            cout<<"test"<<input_bottle->get(1).asDouble()<<endl;

        }
        input_vector.resize(input_bottle->size());
        for(int i = 0 ; i < input_bottle->size(); i++)
        {
            input_vector[i] = input_bottle->get(i).asDouble();
        }
        neural_model->updateLoop(input_vector);
        //printActivity();

        if (activity_image){

        ImageOf<PixelRgb> &temp = imagePort.prepare();
        temp.resize(640,480);
        cvResize(neural_model->img,(IplImage*) temp.getIplImage());
        imagePort.write();
        }


		return true;
	}

	bool close()
	{
        Mean = neural_model->getMean();
        SDev = neural_model->getSDev();
        saveNormals();

        if (activity_image){
        imagePort.close();
        }

        outfile.close();
        rpc.close();
        portFromCoclea.close();
        return true;
	}
};

int main(int argc, char * argv[])
{
	Network yarp;
	if (!Network::checkNetwork())
	{
		cout << "yarp network is not available!" << endl;
		return 0;
	}

	ResourceFinder rf;
	rf.setVerbose(true);
    rf.setDefaultContext("cvz");
    rf.setDefaultConfigFile(DEFAULT_CONFIG_FILE); //overridden by --from parameter
	rf.configure(argc, argv);

    NNmodel cvzCore;
	if (cvzCore.configure(rf))
		cvzCore.runModule();
	else
		cout << "Unable to configure the cvz module." << endl;

	return 0;
}

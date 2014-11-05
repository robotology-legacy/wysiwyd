#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>

#include "yarp/os/all.h"

static const double SQ2 = sqrt((double) 2.0);

class NeuralModel
{
public:

    std::vector<double>*                    SDev;
    std::vector<double>*                    Means;
    std::vector<double>*                    activity;
    std::vector<double>*                    diff_activity;
    std::vector<double*>                    input;
    std::vector<double*>                    output;

    std::vector< std::vector< double >* >*    connections;
    double                                  alpha;
    double                                  max_activity;
    double                                  total_activity;
    double                                  E_percent;
    double                                  learning_rate;
    double                                  aux_max;
    unsigned int                            i_size;
    unsigned int                            o_size;
    unsigned int                            size;
    unsigned int                            firing;


    //------------------------------------------------------------------------------------------------------
    //Inicialization functions for Input:

    //Gaussian weight distribution
    std::vector<double>* InputGaussian(std::vector<double>* x, double sigma, double mu)
    {
        for (unsigned int i = 0; i < x->size(); i++)
        {
            x->at(i) = (double)(1 / (sigma*sqrt(2 * 3.141592)))*exp(-pow(i - mu, 2) / (2 * pow(sigma, 2)));
        }
        return x;
    }

    //All2all random weight distribution
    void InputAllRandom()
    {
        for (unsigned int i = 0; i < input.size(); i++)
        {
            for (unsigned int j = input.size(); j < activity->size(); j++)
            {
               ( connections->at(j))->at(i) = (rand() % 1000) / (double)1000;
            }
        }
    }

    //------------------------------------------------------------------------------------------------------
    //Topology generation functions


    //No connections
    void TopologyNone(){}


    //------------------------------------------------------------------------------------------------------
    //Init functions

    NeuralModel(unsigned int n_neurons, std::vector<double>* loaded_mean, std::vector<double>* loaded_std, double a = 0.001, double E = 0.95, double LR = 0.1, unsigned int input_size = 2048, unsigned int output_size = 50)
    {
        //initialize constants
        i_size = input_size;
        o_size = output_size;
        size = n_neurons + i_size;
        alpha = a;
        E_percent = E;
        learning_rate = LR;
        firing = 0;
        aux_max = 0;

        if (size < i_size){ std::cout << "ERROR: More inputs than neurons" << std::endl; }
        if (size < o_size){ std::cout << "ERROR: More outputs than neurons" << std::endl; }

        //reallocate vector sizes
        activity = new std::vector<double>;
        activity->resize(size);
        diff_activity = new std::vector<double>;
        diff_activity->resize(size);
        input.resize(i_size);
        output.resize(o_size);
        /*
        SDev = new std::vector<double>;
        SDev->resize(i_size);
        Means = new std::vector<double>;
        Means->resize(i_size);
*/
        //initialize connections matrix
        connections = new std::vector<std::vector<double>* >;
        connections->resize(size);
        for(unsigned int i = 0;i<size;i++)
        {
            connections->at(i) = new std::vector<double>;
            (connections->at(i))->resize(size);
        }

        //assign pre-loaded data
        if (loaded_std->size() != i_size)
        {
            SDev=loaded_std;
            SDev->resize(i_size);

        }
        else{
            setSDev(loaded_std);
        }
        if (loaded_mean->size() != i_size)
        {
            Means=loaded_mean;
            Means->resize(i_size);

        }
        else{
            setMean(loaded_mean);
        }


        this->generateModel();


    }

    //Init empty topology
    void InitConnections()
    {
        for (unsigned int i = 0; i < (size); i++)
        {
            for (unsigned int j = 0; j < (size); j++)
            {
                (connections->at(j))->at(i) = 0.0;
            }
        }
    }

    void generateModel(std::string input_connections = "allRandom", std::string inner_topology = "none")
    {

        //Generate the neurons        
        for (unsigned int n = 0; n < size; n++)
        {
            activity->at(n) = 0;
            diff_activity->at(n) = 0;
        }

        //Initialize input neurons
        for (unsigned int n = 0; n < i_size; n++)
        {

            input[n] = &((*activity)[n]);
            //std::cout << "Testing: input adress, content and neuron content and adress" << std::endl;
            //std::cout << input[n] << "  "<<*(input[n]) << "  "<< activity->at(n) << "  "<< &activity[n] << std::endl;
        }

        //Initialize input weights
        if (input_connections.compare("allRandom") == 0)
        {
            InputAllRandom();
        }
        else if (input_connections.compare("gaussian") == 0)
        {
            InputGaussian(activity, 0, 5);
        }

        //initialize topology (inner connections)
        if (inner_topology.compare("none") == 0)
        {
            TopologyNone();
        }
    }

    //------------------------------------------------------------------------------------------------------
    //Utilitari functions
    std::vector<double> getMean(){ return *Means; }
    std::vector<double> getSDev(){ return *SDev; }
    void setMean(std::vector<double>* m){ Means = m; }
    void setSDev(std::vector<double>* s){ SDev = s; }

    void normalizeInput(double *in, unsigned int i)
    {

        Means->at(i) = (Means->at(i) * (1 - alpha)) + alpha* (*in);

        SDev->at(i) = sqrt(SDev->at(i) * SDev->at(i) * (1 - alpha) + alpha*pow((*in) - Means->at(i), 2));
        *in = 0.5 + 0.5*tanh(((*in) - Means->at(i))*SDev->at(i) / SQ2);

    }

    //------------------------------------------------------------------------------------------------------
    //Update Functions

    //1: Update Input
    bool updateInput(std::vector<double> in)
    {

        if (in.size() != input.size()){
            std::cout << "ERROR: Input dimensions have changed!" << std::endl;
            std::cout << "Input size: " << in.size() << std::endl << "Input neurons: " << input.size() << std::endl;
            return false;
        }
        else{
            for (unsigned int i = 0; i<in.size(); i++)
            {
                normalizeInput(&in[i], i);
                *(input.at(i)) = in[i];

            }
            return true;
        }
    }

    //2: process Activity
    void processActivity()
    {
        //Propagate activity
        //This could be parallelized!!!
        for (unsigned int j = i_size; j<size; j++) //receiving neuron
        {
            diff_activity->at(j) = (double)0;
            for (unsigned int i = 0; i<size; i++) //signaling neuron
            {
                diff_activity->at(j) += (connections->at(j))->at(i) * activity->at(i);
            }
        }
    }

    //3: Update Activity
    void updateActivity()
    {

        for (unsigned int n = i_size; n<size; n++)
        {
            //update
            activity->at(n) += diff_activity->at(n);

            //store statistics
            total_activity += activity->at(n);
            max_activity = std::max(activity->at(n), max_activity);

        }
    }



    //4: Normalize & select
    void normalizeAndSelect(std::string normalization = "total")
    {
        //Compute current E%max
        double Emax = E_percent;
        aux_max = 1 / max_activity;
        if (normalization.compare("total") == 0)        //Total activity of the network is 1
        {
            double aux_total = 1 / total_activity;
            Emax *= (max_activity * aux_total);

            for (unsigned int n = i_size; n<size; n++)
            {
                activity->at(n) *= aux_total;
            }
        }
        else if (normalization.compare("max") == 0)     //Max activity in the network is 1
        {
            for (unsigned int n = i_size; n<size; n++)
            {
                activity->at(n) /= aux_max;
            }
        }

        for (unsigned int n = i_size; n < size; n++)
        {
            //4.2 Select: E%-max
            if (activity->at(n) < Emax)
            {
                activity->at(n) = 0;
            }
            else
            {
                firing += 1;
            }
        }
    }
    //4: Normalize & select
    void normalizeAndSelect(yarp::os::Bottle port, std::string normalization = "total")
    {
        //Compute current E%max
        double Emax = E_percent;
        aux_max = 1 / max_activity;

        if (normalization.compare("total") == 0)        //Total activity of the network is 1
        {
            double aux_total = 1 / total_activity;
            Emax *= (max_activity * aux_total);

            for (unsigned int n = i_size; n<size; n++)
            {
                activity->at(n) *= aux_total;
            }
        }
        else if (normalization.compare("max") == 0)     //Max activity in the network is 1
        {
            for (unsigned int n = i_size; n<size; n++)
            {
                activity->at(n) /= aux_max;
            }
        }
        for (unsigned int n = i_size; n < size; n++)
        {
            //4.2 Select: E%-max
            if (activity->at(n) < Emax)
            {
                activity->at(n) = 0;
            }
            else
            {
                firing += 1;
            }
            //Write on port
            port.addDouble(activity->at(n));
        }
    }

    void updateWeights()
    {
        for (unsigned int j = i_size; j<size; j++)       //receiving neuron
        {
            for (unsigned int i = 0; i < size; i++)      //spiking neuron
            {
                // Learning Rule
                (connections->at(j))->at(i) += learning_rate * (diff_activity->at(j) * aux_max - diff_activity->at(i) * aux_max);
            }
        }
    }

    //Reset Loop
    void resetLoop()
    {
        total_activity = 0;
        max_activity = 0;
        aux_max = 1;
        firing = 0;
    }


    //------------------------------------------------------------------------------------------------------
};

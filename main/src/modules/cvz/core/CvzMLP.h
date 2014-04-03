#ifndef __CVZ_CVZMLP_H__
#define __CVZ_CVZMLP_H__

#include <map>

#include <string>
#include <sstream>
#include <iostream>
#include "ICvz.h"
#include <time.h>
#include <vector>

#include "helpers.h"
using namespace std;


class CvzMLP : public IConvergenceZone
{
    yarp::os::Semaphore mutex;
    int maxTrainingTrials;
    double eta, momentum;

    IModality* inputModality;
    IModality* outputModality;

    vector< vector<double> > vHiddenLayers; //computes all the activity of the hidden layers
    vector< vector< vector<double> > > vWeights; //stores the inputs for all the weights -> 1) level 2) input 3) target
    vector< vector<double> > vBiases;//store all the biases for HL and output
    vector<double> outputGradients; //output gradients
    vector< vector<double> > hLayerGradients; // stores the gradients for the hidden layers

    double stopMeanError;
    double stopMaxError;


    bool hasLearnt;
    int desiredTrainingSetSize;
    vector < pair< vector<double>, vector<double> > > trainingSet; //the inputs of the training

public:


    //calculate output activity - NEW
    void computOutputActivity(vector<double> &outputs)
    {
        for(int outIt =0; outIt<outputs.size(); outIt++)
        {
            outputs[outIt] = 0.0;
            for(int hidLayAct = 0; hidLayAct<vHiddenLayers[vHiddenLayers.size()-1].size();hidLayAct++)
            {
                outputs[outIt] += vHiddenLayers[vHiddenLayers.size()-1][hidLayAct]*vWeights[vWeights.size()-1][hidLayAct][outIt];
            }
            outputs[outIt] += vBiases[vBiases.size()-1][outIt];
            outputs[outIt] = sigmoidFunction(outputs[outIt]);
        }
    }

    //called once in the beginning to randomize the bias
    void randomizeEverything(vector<int> _hiddenLayers, int in, int out)
    {
        vBiases.resize(_hiddenLayers.size()+1);
        for(int itResize = 0; itResize < vBiases.size()-1; itResize++)
        {
            vBiases[itResize].resize(_hiddenLayers[itResize]);
            for(int it = 0; it<vBiases[itResize].size();it++)
            {
                vBiases[itResize][it] = randomFunction();
            }
        }
        vBiases[_hiddenLayers.size()].resize(out);
        for(int it = 0; it<vBiases[_hiddenLayers.size()].size();it++)
        {
            vBiases[_hiddenLayers.size()][it] = randomFunction();
        }
        cout << "biases initialized" << endl;
        //randomize weights
        vWeights.resize(_hiddenLayers.size()+1);
        for(int itWeightsL = 0; itWeightsL < vWeights.size(); itWeightsL++)
        {
            if(itWeightsL ==0)
                vWeights[itWeightsL].resize(in);
            else
                vWeights[itWeightsL].resize(_hiddenLayers[itWeightsL-1]);
            for(int itNodeInput = 0; itNodeInput<vWeights[itWeightsL].size(); itNodeInput++)
            {
                if(itWeightsL == vWeights.size()-1 )
                    vWeights[itWeightsL][itNodeInput].resize(out);
                else
                    vWeights[itWeightsL][itNodeInput].resize(_hiddenLayers[itWeightsL]);
                for(int endNode = 0; endNode<vWeights[itWeightsL][itNodeInput].size(); endNode++)
                {
                    vWeights[itWeightsL][itNodeInput][endNode] = randomFunction();
                }
            }
        }
        printFuckingWeights();
        cout << "all randomized"<<endl;
        string test;
        cin>> test;
    }

    void printFuckingWeights()
    {
        cout<<"Weights : "<<endl;
        for(int layer = 0; layer < vWeights.size(); layer++)
        {
            cout<<"Layer : "<<layer<<endl;
            for(int inputNode = 0; inputNode<vWeights[layer].size(); inputNode++)
            {
                for(int endNode = 0; endNode<vWeights[layer][inputNode].size(); endNode++)
                {
                    cout << inputNode<<"->"<<endNode<<"="<<vWeights[layer][inputNode][endNode]<<endl;
                }
            }
            cout << endl;
        }
    }

    //returns a random number
    double randomFunction()
    {
        return (double)rand()/(double)RAND_MAX;
    }

    //responsible for intializing everything
    void initializeEverything(int _out, int _in, vector<int> _hiddenLayers)
    {
        //initialize hiddenLayersOutputs
         populateHiddenLayers(_hiddenLayers);
        //initialize weights
         randomizeEverything(_hiddenLayers,_in,_out);
         outputGradients.resize(_out);
    }

    void populateHiddenLayers(vector<int> _hiddenLayers)
    {
        //create the vector of hidden layers where the size of the vector corresponds to the number of hidden layers exist and then each element is a vector of elements that represent the activity of each node
        vHiddenLayers.resize(_hiddenLayers.size());
        hLayerGradients.resize(_hiddenLayers.size());

        for(int itResize = 0; itResize < vHiddenLayers.size(); itResize++)
        {
            vHiddenLayers[itResize].resize(_hiddenLayers[itResize]);
            hLayerGradients[itResize].resize(_hiddenLayers[itResize]);
        }
        cout << "populated HiddenLayers "<<endl;
    }

    //calculates the output errors
    void computeOutputGradients(vector<double> &outputs,  vector<double> &desiredOutputs )
    {
        for(int it = 0; it< outputs.size(); it++)
        {
            outputGradients[it] = outputs[it]*(1-outputs[it])*(desiredOutputs[it] - outputs[it]);
        }
    }

    //calculate the hidden layers activity
    void computeHLActivity(vector<double> &inputs)
    {
        for(int hLayer = 0; hLayer < vHiddenLayers.size(); hLayer++)
        {
            vector<double> currentInputs;
            //this is the first layer so inputs come from inputs
            if(hLayer == 0)
                currentInputs = inputs;
            else
                currentInputs = vHiddenLayers[hLayer-1];
            for(int node = 0; node < vHiddenLayers[hLayer].size(); node++)
            {
                vHiddenLayers[hLayer][node]  = 0.0;
                for(int nodeInput = 0; nodeInput < currentInputs.size(); nodeInput++)
                {
                    vHiddenLayers[hLayer][node] += currentInputs[nodeInput]*vWeights[hLayer][nodeInput][node];
                }
                vHiddenLayers[hLayer][node] += vBiases[hLayer][node];
                vHiddenLayers[hLayer][node] = sigmoidFunction(vHiddenLayers[hLayer][node]);
            }

        }
    }



    //update the rest of the weights
    void updateInputWeights(int layer, vector<double> &inputs)
    {
        //to update the weights w=w+eta*gradientOutput*inputActivity
        //the weights vector is (layer, input, output). So let's first go to the layer
        vector<vector<vector<double> > >::iterator layerIt = vWeights.begin()+layer;
        //now for that level lets iterate through the inputs
        int inputNodeCounter = 0; // indicates at which input we are now
        //lets go now to the level of inputs
        for(vector<vector<double> >::iterator inputNodeIt = (*layerIt).begin(); inputNodeIt != (*layerIt).end(); inputNodeIt++)
        {
            //if the layer is 0 then the input activity comes from the inputs vector. If not it comes from the layer -1 vhiddenLayers
            double nodeInputActivity = 0.0;
            //all this to get the input activity node
            //if 0 get from inputs
            if(layer == 0)
            {
                //this will display the activity of the input node
                int inputActivityNode = 0;
                for(vector<double>::iterator inputNodeActivityIt = inputs.begin();inputNodeActivityIt != inputs.end(); inputNodeActivityIt++)
                {
                    if(inputNodeCounter == inputActivityNode)
                        nodeInputActivity = (*inputNodeActivityIt);
                    inputActivityNode++;
                }

            }
            //if it is not, get from the layer-1 inputs activity
            else
            {
                //get from the previsou layer
                //this will display the activity of the input node
                int inputActivityNode = 0;
                vector<vector<double> >::iterator hiddenLayerActivityIt= vHiddenLayers.begin()+layer-1;
                for(vector<double>::iterator inputNodeActivityIt= (*hiddenLayerActivityIt).begin();inputNodeActivityIt!= (*hiddenLayerActivityIt).end();inputNodeActivityIt++)
                {
                    if(inputNodeCounter == inputActivityNode)
                        nodeInputActivity = (*inputNodeActivityIt);
                    inputActivityNode++;
                }
            }
            double tmpWeight = eta*nodeInputActivity;
            //now iterate through the weights and for each weight find the activity
            int nodeOutputCounter = 0;
            for(vector<double>::iterator targetIt = (*inputNodeIt).begin(); targetIt != (*inputNodeIt).end();targetIt++)
            {
                //deal with the current output gradients
                //cout << "current weight for layer " << layer << " and input " << inputNodeCounter << " for output " << nodeOutputCounter << " is updated from " << (*targetIt);
                (*targetIt) = (*targetIt)+tmpWeight*getCurrentHLGradient(layer,nodeOutputCounter);
                //cout << " to:" << (*targetIt) << endl;

                nodeOutputCounter++;
            }
            inputNodeCounter++;
        }

        //cout << "ok" << endl;
    }

    //stores the updated hidden layer gradients
    void updateCurrentHiddenLayerGradients(int Layer, int node, vector<double> tmp)
    {
        //cout << "updating the hidden Layer gradients! BEFORE" << endl;
       //printHiddenLayerGradients();
        int tmpLayer = 0;
        for(vector<vector<double> >::iterator itLayer = hLayerGradients.begin(); itLayer != hLayerGradients.end(); itLayer++)
        {
            //find the layer for the gradients
            if(tmpLayer == Layer)
            {
                //now for each node
                int tmpNode = 0;
                for(unsigned int i=0; i<tmp.size(); i++)
                {
                    vector<double>::iterator iter = tmp.begin()+tmpNode;
                    vector<double>::iterator itNode = (*itLayer).begin()+tmpNode;
                    (*itNode) = (*iter);
                    tmpNode++;
                }

            }
            tmpLayer++;
        }
        //cout << "AFTER!!!!!" << endl;
        //printHiddenLayerGradients();

    }

    //get the gradient of the hidden layer
    double getCurrentHLGradient(int Layer, int node)
    {
        vector<vector<double> >::iterator it = hLayerGradients.begin()+Layer;
        vector<double>::iterator it1= (*it).begin()+node;
        return (*it1);

    }

    //back propagation - calculate the hidden layers gradients (errors)
    void computeHLGradientsAndWeights(vector<double> &inputs,  vector<double> &outputs)
    {
        for(int itLayer = hLayerGradients.size()-1; itLayer>=0; itLayer--)
        {
            //cout<<"Layer "<<itLayer<<endl;
            //compute gradients
            vector<double> currentError;
            if(itLayer == hLayerGradients.size()-1)
                currentError = outputGradients;
            else
                currentError = hLayerGradients[itLayer+1];

            for (int hlNode = 0; hlNode< hLayerGradients[itLayer].size(); hlNode++)
            {
                hLayerGradients[itLayer][hlNode]  = 0.0;
                for(int nodeInput = 0; nodeInput < currentError.size(); nodeInput++)
                {
                    hLayerGradients[itLayer][hlNode] += currentError[nodeInput]*vWeights[itLayer+1][hlNode][nodeInput];
                }
                hLayerGradients[itLayer][hlNode] *= vHiddenLayers[itLayer][hlNode]*(1-vHiddenLayers[itLayer][hlNode]);
            }
        }
        //cout << "gradient computed" << endl;

        //update weights
        for(int itLayer = vWeights.size()-1; itLayer>=0; itLayer--)
        {
            vector<double> curInputs;
            vector<double> gradients;
            if(itLayer == 0)
                curInputs = inputs;
            else
                curInputs = vHiddenLayers[itLayer-1];

            //printFuckingVector("In",curInputs);

            if(itLayer == vWeights.size()-1)
                gradients = outputGradients;
            else
                gradients = hLayerGradients[itLayer];

            //printFuckingVector("Grad",gradients);

            for (int inputNode = 0; inputNode< vWeights[itLayer].size(); inputNode++)
            {
                for(int nodeOutput = 0; nodeOutput < gradients.size(); nodeOutput++)
                {
                    vWeights[itLayer][inputNode][nodeOutput] += eta*gradients[nodeOutput]*curInputs[inputNode];
                }
            }
        }

    }


    void printFuckingVector(string name, vector<double> v)
    {
        cout<<name<<"   ";
        for(int i=0; i<v.size(); i++)
            cout<< v[i]<<'\t';
        cout<<endl;
    }

    //update the output layer weights with the form: wAa += eta*outputA*errorOutputA
    void updateOutputLayerWeights(  vector<double> &outputs )
    {
        int layer = vWeights.size()-1;
            for(int source=0; source<vWeights[layer].size(); source++)
            {
                for(int destination=0; destination<vWeights[layer][source].size(); destination++)
                {
                    vWeights[layer][source][destination] += eta*outputs[destination]*outputGradients[destination];
                    //cout << vWeights[layer][source][destination] <<" with eta*=" << eta << " *ouput=" << outputs[destination] << " and outputGradient= " << outputGradients[destination]<<endl;
                }
            }
    }

    void backPropagation( vector<double> &inputs,  vector<double> &outputs,  vector<double> &desiredOutputs )
    {
        //compute the gradients
        computeOutputGradients(outputs, desiredOutputs);
        //cout << "a" << endl;
        //update the output layer weights
        //updateOutputLayerWeights(outputs);
        //cout << "b" << endl;
        //calculate hidden layer gradients
        //update the rest of the weights
        computeHLGradientsAndWeights(inputs, outputs);
        //cout << "c" << endl;

        //update biases
        //string test;
        //cin >> test;
    }


    bool configure(yarp::os::ResourceFinder &rf)
	{
		//Call the base class configure
		this->IConvergenceZone::configure(rf);
        //Get additional parameters
        maxTrainingTrials = rf.check("maxTrials", yarp::os::Value(500)).asInt();
        eta = rf.check("eta", yarp::os::Value(0.001)).asDouble();
        momentum = rf.check("momentum", yarp::os::Value(0.04)).asDouble();
        desiredTrainingSetSize = rf.check("desiredTrainingSetSize", yarp::os::Value(4)).asInt();
        stopMeanError = rf.check("epochMeanError", yarp::os::Value(0.01)).asDouble();
        stopMaxError = rf.check("maxError", yarp::os::Value(0.1)).asDouble();
        hasLearnt = false;
        //temporary to check things
        int in,out;
        yarp::os::Bottle* inOut =  rf.find("inputOutput").asList();
        if(inOut && inOut->size()==2){
            cout << "input  Layers:";
            in = inOut->get(0).asInt();
            out = inOut->get(1).asInt();
            cout << in << " output: " << out ;
            cout <<  endl;
        }
        else
        {
            in = 2;
            out = 1;
        }
        //temporary to check things end
        //get hidden layers
        yarp::os::Bottle* hiddenL = rf.find("hiddenLayers").asList();
        if(hiddenL && hiddenL->size() >0)
        {
             vector<int> hiddenLayers;
            for(int i=0; i<hiddenL->size(); i++)
                hiddenLayers.push_back(hiddenL->get(i).asInt());
            cout << "HIDDEN LAYERS VALUES:" << endl;
            for( vector<int>::iterator it=hiddenLayers.begin(); it!=hiddenLayers.end(); it++)
                cout<< " " << (*it);
            cout <<  endl;
            initializeEverything(out,in,hiddenLayers);

        }

        inputModality = modalitiesBottomUp.begin()->second;
        outputModality= modalitiesTopDown.begin()->second;
		return true;
	}





	virtual void ComputePrediction()
	{
		mutex.wait();

         vector<double> inputs;
         vector<double> desiredOutputs;
         vector<double> outputs;

        //Copy input to prediction
        inputs = inputModality->GetValueReal();
        desiredOutputs = outputModality->GetValueReal();
        outputs.resize(desiredOutputs.size());

        if (!hasLearnt)
        {
            if (trainingSet.size()<desiredTrainingSetSize)
            {
                cout << "Filling buffer mode"<< endl;
                if(trainingSet.size()==0)
                    trainingSet.push_back(pair<vector<double>,vector<double> >(inputs,desiredOutputs));
                else
                {
                    pair<vector<double>, vector<double> > current(inputs,desiredOutputs);
                    bool exists= find(trainingSet.begin(),trainingSet.end(), current) != trainingSet.end();
                    //pair<vector<double>, vector<double> > last = trainingSet.back();
                    if (!exists)
                        trainingSet.push_back(pair<vector<double>,vector<double> >(inputs,desiredOutputs));
                    else
                        cout<<"Skip identical"<<endl;
                }
            }
            else
            {
                int test;
                cin>>test;
                //learn
                //Propagate input
                double epochMeanError = 1.0;
                int epochCount =0;
                double maxError = 0.0;
                while(epochMeanError>stopMeanError||maxError>stopMaxError)
                {
                    //system("clear");
                    //cout << "Training epoch"<< endl;
                    epochMeanError = 0.0;

                    maxError = 0.0;
                    for(int sample=0; sample<trainingSet.size();sample++)
                    {
                         vector<double> tmpinputs = trainingSet[sample].first;
                         vector<double> tmpdesiredOutputs = trainingSet[sample].second;
                         vector<double> tmpoutputs(tmpdesiredOutputs.size());
                        //calculate the activity of the hidden layers
                        computeHLActivity(tmpinputs);
                        //calculate the activity of the output layers
                        computOutputActivity(tmpoutputs);
                        //printFuckingVector("Input", tmpinputs);
                        //printFuckingVector("Output", tmpoutputs);
                        //printFuckingVector("Des Output", tmpdesiredOutputs);

                        //Learn
                        //Calculate error + do backpropagation
                        backPropagation(tmpinputs,tmpoutputs,tmpdesiredOutputs);
                        //Print

                        //Calculate error
                        double meanError = 0.0;
                        for(int o=0;o<tmpoutputs.size(); o++)
                        {
                            meanError+=fabs(tmpdesiredOutputs[o]-tmpoutputs[o]);
                        }
                        meanError /= tmpoutputs.size();
                        //cout<<"Step mean error : "<<meanError<<endl;
                        if (meanError>maxError)
                            maxError = meanError;
                        epochMeanError += meanError;
                    }

                    epochMeanError /= trainingSet.size();
                    if (epochCount%100 == 0)
                    {
                        cout<<"Epoch "<<epochCount<<" error mean : "<<epochMeanError<<"\t max: "<<maxError<<endl;
                    }
                    epochCount++;
                }
                hasLearnt = true;
            }
        }
        else
        {
            //Propagate input
            cout << "Prediction mode"<< endl;

            //calculate the activity of the hidden layers
            computeHLActivity(inputs);
            //getHiddenLayersActivity();

            //calculate the activity of the output layers
            computOutputActivity(outputs);
            //until here ok

            //Set the prediction
            outputModality->SetValuePrediction(outputs);

            //Adapt
        }

		mutex.post();
	}

};

#endif

#ifndef __CVZ_CVZMLP_H__
#define __CVZ_CVZMLP_H__

#include <map>

#include <string>
#include <sstream>
#include <iostream>
#include "ICvz.h"
#include <time.h>
#include <vector>

#include "cvz/helpers/helpers.h"

namespace cvz {
	namespace core {

		class CvzMLP : public IConvergenceZone
		{
			yarp::os::Semaphore mutex;
			int maxTrainingTrials;
			double eta, momentum;

			IModality* inputModality;
			IModality* outputModality;

			std::vector< std::vector<double> > vHiddenLayers; //computes all the activity of the hidden layers
			std::vector< std::vector< std::vector<double> > > vWeights; //stores the inputs for all the weights -> 1) level 2) input 3) target
			std::vector< std::vector<double> > vBiases;//store all the biases for HL and output
			std::vector<double> outputGradients; //output gradients
			std::vector< std::vector<double> > hLayerGradients; // stores the gradients for the hidden layers

			double stopMeanError;
			double stopMaxError;


			bool hasLearnt;
			int desiredTrainingSetSize;
			std::vector < std::pair< std::vector<double>, std::vector<double> > > trainingSet; //the inputs of the training

		public:


			//calculate output activity - NEW
			void computOutputActivity(std::vector<double> &outputs)
			{
				for (unsigned int outIt = 0; outIt < outputs.size(); outIt++)
				{
					outputs[outIt] = 0.0;
					for (unsigned int hidLayAct = 0; hidLayAct < vHiddenLayers[vHiddenLayers.size() - 1].size(); hidLayAct++)
					{
						outputs[outIt] += vHiddenLayers[vHiddenLayers.size() - 1][hidLayAct] * vWeights[vWeights.size() - 1][hidLayAct][outIt];
					}
					outputs[outIt] += vBiases[vBiases.size() - 1][outIt];
					outputs[outIt] = helpers::sigmoidFunction(outputs[outIt]);
				}
			}

			//called once in the beginning to randomize the bias
			void randomizeEverything(std::vector<int> _hiddenLayers, int in, int out)
			{
				vBiases.resize(_hiddenLayers.size() + 1);
				for (unsigned int itResize = 0; itResize < vBiases.size() - 1; itResize++)
				{
					vBiases[itResize].resize(_hiddenLayers[itResize]);
					for (unsigned int it = 0; it < vBiases[itResize].size(); it++)
					{
						vBiases[itResize][it] = randomFunction();
					}
				}
				vBiases[_hiddenLayers.size()].resize(out);
				for (unsigned int it = 0; it < vBiases[_hiddenLayers.size()].size(); it++)
				{
					vBiases[_hiddenLayers.size()][it] = randomFunction();
				}
				std::cout << "biases initialized" << std::endl;
				//randomize weights
				vWeights.resize(_hiddenLayers.size() + 1);
				for (unsigned int itWeightsL = 0; itWeightsL < vWeights.size(); itWeightsL++)
				{
					if (itWeightsL == 0)
						vWeights[itWeightsL].resize(in);
					else
						vWeights[itWeightsL].resize(_hiddenLayers[itWeightsL - 1]);
					for (unsigned int itNodeInput = 0; itNodeInput < vWeights[itWeightsL].size(); itNodeInput++)
					{
						if (itWeightsL == vWeights.size() - 1)
							vWeights[itWeightsL][itNodeInput].resize(out);
						else
							vWeights[itWeightsL][itNodeInput].resize(_hiddenLayers[itWeightsL]);
						for (unsigned int endNode = 0; endNode < vWeights[itWeightsL][itNodeInput].size(); endNode++)
						{
							vWeights[itWeightsL][itNodeInput][endNode] = randomFunction();
						}
					}
				}
				printFuckingWeights();
				std::cout << "all randomized" << std::endl;
				std::string test;
				std::cin >> test;
			}

			void printFuckingWeights()
			{
				std::cout << "Weights : " << std::endl;
				for (unsigned int layer = 0; layer < vWeights.size(); layer++)
				{
					std::cout << "Layer : " << layer << std::endl;
					for (unsigned int inputNode = 0; inputNode < vWeights[layer].size(); inputNode++)
					{
						for (unsigned int endNode = 0; endNode < vWeights[layer][inputNode].size(); endNode++)
						{
							std::cout << inputNode << "->" << endNode << "=" << vWeights[layer][inputNode][endNode] << std::endl;
						}
					}
					std::cout << std::endl;
				}
			}

			//returns a random number
			double randomFunction()
			{
				return (double)rand() / (double)RAND_MAX;
			}

			//responsible for intializing everything
			void initializeEverything(int _out, int _in, std::vector<int> _hiddenLayers)
			{
				//initialize hiddenLayersOutputs
				populateHiddenLayers(_hiddenLayers);
				//initialize weights
				randomizeEverything(_hiddenLayers, _in, _out);
				outputGradients.resize(_out);
			}

			void populateHiddenLayers(std::vector<int> _hiddenLayers)
			{
				//create the std::vector of hidden layers where the size of the std::vector corresponds to the number of hidden layers exist and then each element is a std::vector of elements that represent the activity of each node
				vHiddenLayers.resize(_hiddenLayers.size());
				hLayerGradients.resize(_hiddenLayers.size());

				for (unsigned int itResize = 0; itResize < vHiddenLayers.size(); itResize++)
				{
					vHiddenLayers[itResize].resize(_hiddenLayers[itResize]);
					hLayerGradients[itResize].resize(_hiddenLayers[itResize]);
				}
				std::cout << "populated HiddenLayers " << std::endl;
			}

			//calculates the output errors
			void computeOutputGradients(std::vector<double> &outputs, std::vector<double> &desiredOutputs)
			{
				for (unsigned int it = 0; it < outputs.size(); it++)
				{
					outputGradients[it] = outputs[it] * (1 - outputs[it])*(desiredOutputs[it] - outputs[it]);
				}
			}

			//calculate the hidden layers activity
			void computeHLActivity(std::vector<double> &inputs)
			{
				for (unsigned int hLayer = 0; hLayer < vHiddenLayers.size(); hLayer++)
				{
					std::vector<double> currentInputs;
					//this is the first layer so inputs come from inputs
					if (hLayer == 0)
						currentInputs = inputs;
					else
						currentInputs = vHiddenLayers[hLayer - 1];
					for (unsigned int node = 0; node < vHiddenLayers[hLayer].size(); node++)
					{
						vHiddenLayers[hLayer][node] = 0.0;
						for (unsigned int nodeInput = 0; nodeInput < currentInputs.size(); nodeInput++)
						{
							vHiddenLayers[hLayer][node] += currentInputs[nodeInput] * vWeights[hLayer][nodeInput][node];
						}
						vHiddenLayers[hLayer][node] += vBiases[hLayer][node];
						vHiddenLayers[hLayer][node] = helpers::sigmoidFunction(vHiddenLayers[hLayer][node]);
					}

				}
			}



			//update the rest of the weights
			void updateInputWeights(int layer, std::vector<double> &inputs)
			{
				//to update the weights w=w+eta*gradientOutput*inputActivity
				//the weights std::vector is (layer, input, output). So let's first go to the layer
				std::vector<std::vector<std::vector<double> > >::iterator layerIt = vWeights.begin() + layer;
				//now for that level lets iterate through the inputs
				int inputNodeCounter = 0; // indicates at which input we are now
				//lets go now to the level of inputs
				for (std::vector<std::vector<double> >::iterator inputNodeIt = (*layerIt).begin(); inputNodeIt != (*layerIt).end(); inputNodeIt++)
				{
					//if the layer is 0 then the input activity comes from the inputs std::vector. If not it comes from the layer -1 vhiddenLayers
					double nodeInputActivity = 0.0;
					//all this to get the input activity node
					//if 0 get from inputs
					if (layer == 0)
					{
						//this will display the activity of the input node
						int inputActivityNode = 0;
						for (std::vector<double>::iterator inputNodeActivityIt = inputs.begin(); inputNodeActivityIt != inputs.end(); inputNodeActivityIt++)
						{
							if (inputNodeCounter == inputActivityNode)
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
						std::vector<std::vector<double> >::iterator hiddenLayerActivityIt = vHiddenLayers.begin() + layer - 1;
						for (std::vector<double>::iterator inputNodeActivityIt = (*hiddenLayerActivityIt).begin(); inputNodeActivityIt != (*hiddenLayerActivityIt).end(); inputNodeActivityIt++)
						{
							if (inputNodeCounter == inputActivityNode)
								nodeInputActivity = (*inputNodeActivityIt);
							inputActivityNode++;
						}
					}
					double tmpWeight = eta*nodeInputActivity;
					//now iterate through the weights and for each weight find the activity
					int nodeOutputCounter = 0;
					for (std::vector<double>::iterator targetIt = (*inputNodeIt).begin(); targetIt != (*inputNodeIt).end(); targetIt++)
					{
						//deal with the current output gradients
						//std::cout << "current weight for layer " << layer << " and input " << inputNodeCounter << " for output " << nodeOutputCounter << " is updated from " << (*targetIt);
						(*targetIt) = (*targetIt) + tmpWeight*getCurrentHLGradient(layer, nodeOutputCounter);
						//std::cout << " to:" << (*targetIt) << std::endl;

						nodeOutputCounter++;
					}
					inputNodeCounter++;
				}

				//std::cout << "ok" << std::endl;
			}

			//stores the updated hidden layer gradients
			void updateCurrentHiddenLayerGradients(int Layer, int node, std::vector<double> tmp)
			{
				//std::cout << "updating the hidden Layer gradients! BEFORE" << std::endl;
				//printHiddenLayerGradients();
				int tmpLayer = 0;
				for (std::vector<std::vector<double> >::iterator itLayer = hLayerGradients.begin(); itLayer != hLayerGradients.end(); itLayer++)
				{
					//find the layer for the gradients
					if (tmpLayer == Layer)
					{
						//now for each node
						int tmpNode = 0;
						for (unsigned int i = 0; i < tmp.size(); i++)
						{
							std::vector<double>::iterator iter = tmp.begin() + tmpNode;
							std::vector<double>::iterator itNode = (*itLayer).begin() + tmpNode;
							(*itNode) = (*iter);
							tmpNode++;
						}

					}
					tmpLayer++;
				}
				//std::cout << "AFTER!!!!!" << std::endl;
				//printHiddenLayerGradients();

			}

			//get the gradient of the hidden layer
			double getCurrentHLGradient(int Layer, int node)
			{
				std::vector<std::vector<double> >::iterator it = hLayerGradients.begin() + Layer;
				std::vector<double>::iterator it1 = (*it).begin() + node;
				return (*it1);

			}

			//back propagation - calculate the hidden layers gradients (errors)
			void computeHLGradientsAndWeights(std::vector<double> &inputs, std::vector<double> &outputs)
			{
				for (int itLayer = hLayerGradients.size() - 1; itLayer >= 0; itLayer--)
				{
					//std::cout<<"Layer "<<itLayer<<std::endl;
					//compute gradients
					std::vector<double> currentError;
					if (itLayer == hLayerGradients.size() - 1)
						currentError = outputGradients;
					else
						currentError = hLayerGradients[itLayer + 1];

					for (unsigned int hlNode = 0; hlNode < hLayerGradients[itLayer].size(); hlNode++)
					{
						hLayerGradients[itLayer][hlNode] = 0.0;
						for (unsigned int nodeInput = 0; nodeInput < currentError.size(); nodeInput++)
						{
							hLayerGradients[itLayer][hlNode] += currentError[nodeInput] * vWeights[itLayer + 1][hlNode][nodeInput];
						}
						hLayerGradients[itLayer][hlNode] *= vHiddenLayers[itLayer][hlNode] * (1 - vHiddenLayers[itLayer][hlNode]);
					}
				}
				//std::cout << "gradient computed" << std::endl;

				//update weights
				for (int itLayer = vWeights.size() - 1; itLayer >= 0; itLayer--)
				{
					std::vector<double> curInputs;
					std::vector<double> gradients;
					if (itLayer == 0)
						curInputs = inputs;
					else
						curInputs = vHiddenLayers[itLayer - 1];

					//printFuckingVector("In",curInputs);

					if (itLayer == vWeights.size() - 1)
						gradients = outputGradients;
					else
						gradients = hLayerGradients[itLayer];

					//printFuckingVector("Grad",gradients);

					for (unsigned int inputNode = 0; inputNode < vWeights[itLayer].size(); inputNode++)
					{
						for (unsigned int nodeOutput = 0; nodeOutput < gradients.size(); nodeOutput++)
						{
							vWeights[itLayer][inputNode][nodeOutput] += eta*gradients[nodeOutput] * curInputs[inputNode];
						}
					}
				}

			}


			void printFuckingVector(std::string name, std::vector<double> v)
			{
				std::cout << name << "   ";
				for (unsigned int i = 0; i < v.size(); i++)
					std::cout << v[i] << '\t';
				std::cout << std::endl;
			}

			//update the output layer weights with the form: wAa += eta*outputA*errorOutputA
			void updateOutputLayerWeights(std::vector<double> &outputs)
			{
				int layer = vWeights.size() - 1;
				for (unsigned int source = 0; source < vWeights[layer].size(); source++)
				{
					for (unsigned int destination = 0; destination < vWeights[layer][source].size(); destination++)
					{
						vWeights[layer][source][destination] += eta*outputs[destination] * outputGradients[destination];
						//std::cout << vWeights[layer][source][destination] <<" with eta*=" << eta << " *ouput=" << outputs[destination] << " and outputGradient= " << outputGradients[destination]<<std::endl;
					}
				}
			}

			void backPropagation(std::vector<double> &inputs, std::vector<double> &outputs, std::vector<double> &desiredOutputs)
			{
				//compute the gradients
				computeOutputGradients(outputs, desiredOutputs);
				//std::cout << "a" << std::endl;
				//update the output layer weights
				//updateOutputLayerWeights(outputs);
				//std::cout << "b" << std::endl;
				//calculate hidden layer gradients
				//update the rest of the weights
				computeHLGradientsAndWeights(inputs, outputs);
				//std::cout << "c" << std::endl;

				//update biases
				//string test;
				//cin >> test;
			}


			bool configure(yarp::os::Property &rf)
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
				int in, out;
				yarp::os::Bottle* inOut = rf.find("inputOutput").asList();
				if (inOut && inOut->size() == 2){
					std::cout << "input  Layers:";
					in = inOut->get(0).asInt();
					out = inOut->get(1).asInt();
					std::cout << in << " output: " << out;
					std::cout << std::endl;
				}
				else
				{
					in = 2;
					out = 1;
				}
				//temporary to check things end
				//get hidden layers
				yarp::os::Bottle* hiddenL = rf.find("hiddenLayers").asList();
				if (hiddenL && hiddenL->size() > 0)
				{
					std::vector<int> hiddenLayers;
					for (int i = 0; i < hiddenL->size(); i++)
						hiddenLayers.push_back(hiddenL->get(i).asInt());
					std::cout << "HIDDEN LAYERS VALUES:" << std::endl;
					for (std::vector<int>::iterator it = hiddenLayers.begin(); it != hiddenLayers.end(); it++)
						std::cout << " " << (*it);
					std::cout << std::endl;
					initializeEverything(out, in, hiddenLayers);

				}

				inputModality = modalitiesBottomUp.begin()->second;
				outputModality = modalitiesTopDown.begin()->second;
				return true;
			}





			virtual void ComputePrediction()
			{
				mutex.wait();

				std::vector<double> inputs;
				std::vector<double> desiredOutputs;
				std::vector<double> outputs;

				//Copy input to prediction
				inputs = inputModality->GetValueReal();
				desiredOutputs = outputModality->GetValueReal();
				outputs.resize(desiredOutputs.size());

				if (!hasLearnt)
				{
					if (trainingSet.size() < (unsigned int) desiredTrainingSetSize)
					{
						std::cout << "Filling buffer mode" << std::endl;
						if (trainingSet.size() == 0)
							trainingSet.push_back(std::pair<std::vector<double>, std::vector<double> >(inputs, desiredOutputs));
						else
						{
							std::pair<std::vector<double>, std::vector<double> > current(inputs, desiredOutputs);
							bool exists = find(trainingSet.begin(), trainingSet.end(), current) != trainingSet.end();
							//pair<std::vector<double>, std::vector<double> > last = trainingSet.back();
							if (!exists)
								trainingSet.push_back(std::pair<std::vector<double>, std::vector<double> >(inputs, desiredOutputs));
							else
								std::cout << "Skip identical" << std::endl;
						}
					}
					else
					{
						int test;
						std::cin >> test;
						//learn
						//Propagate input
						double epochMeanError = 1.0;
						int epochCount = 0;
						double maxError = 0.0;
						while (epochMeanError > stopMeanError || maxError > stopMaxError)
						{
							//system("clear");
							//std::cout << "Training epoch"<< std::endl;
							epochMeanError = 0.0;

							maxError = 0.0;
							for (unsigned int sample = 0; sample < trainingSet.size(); sample++)
							{
								std::vector<double> tmpinputs = trainingSet[sample].first;
								std::vector<double> tmpdesiredOutputs = trainingSet[sample].second;
								std::vector<double> tmpoutputs(tmpdesiredOutputs.size());
								//calculate the activity of the hidden layers
								computeHLActivity(tmpinputs);
								//calculate the activity of the output layers
								computOutputActivity(tmpoutputs);
								//printFuckingVector("Input", tmpinputs);
								//printFuckingVector("Output", tmpoutputs);
								//printFuckingVector("Des Output", tmpdesiredOutputs);

								//Learn
								//Calculate error + do backpropagation
								backPropagation(tmpinputs, tmpoutputs, tmpdesiredOutputs);
								//Print

								//Calculate error
								double meanError = 0.0;
								for (unsigned int o = 0; o<tmpoutputs.size(); o++)
								{
									meanError += fabs(tmpdesiredOutputs[o] - tmpoutputs[o]);
								}
								meanError /= tmpoutputs.size();
								//std::cout<<"Step mean error : "<<meanError<<std::endl;
								if (meanError>maxError)
									maxError = meanError;
								epochMeanError += meanError;
							}

							epochMeanError /= trainingSet.size();
							if (epochCount % 100 == 0)
							{
								std::cout << "Epoch " << epochCount << " error mean : " << epochMeanError << "\t max: " << maxError << std::endl;
							}
							epochCount++;
						}
						hasLearnt = true;
					}
				}
				else
				{
					//Propagate input
					std::cout << "Prediction mode" << std::endl;

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
	}
}
#endif

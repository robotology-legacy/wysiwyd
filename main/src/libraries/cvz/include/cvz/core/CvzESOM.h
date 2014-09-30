#ifndef __CVZ_CVZ_ESOM_H__
#define __CVZ_CVZ_ESOM_H__

#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>
#include <float.h>
#include <list>

#include "ICvz.h"
#include "cvzMmcm_IDL.h"
#include "cvz/helpers/helpers.h"

namespace cvz {
    namespace core {

        class ESOMNode
        {
        public:
            double activity;
            std::map<IModality*, std::vector< double > > prototype;

            ESOMNode(){};

            double distanceTo(ESOMNode* n)
            {
                double d = 0.0;
                for (std::map<IModality*, std::vector<double> >::iterator itMod = prototype.begin(); itMod != prototype.end(); itMod++)
                {
                    if (n->prototype.find(itMod->first) == n->prototype.end())
                        return -1;
                    
                    double dMod = 0.0;
                    for (int i = 0; i < itMod->second.size(); i++)
                        d += fabs(prototype[itMod->first][i] - n->prototype[itMod->first][i]);
                    d += dMod / itMod->second.size();
                }
                d /= (double)prototype.size();
                return d;
            }
        };

        class CvzESOM : public IConvergenceZone
        {
            std::list< ESOMNode* > nodes;
            std::map<ESOMNode*, std::map< ESOMNode *, double > > connections;

            int learnStepCnt;
            bool useSoftMaxPrediction;
            yarp::os::Semaphore mutex;

        public:

            virtual std::string getType() { return cvz::core::TYPE_ESOM; };
            int getNodesCount() { return nodes.size(); }
            int getConnectionsCount() {
                int cnt = 0;
                for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                {
                    for (std::list<ESOMNode*>::iterator itNode2 = nodes.begin(); itNode2 != nodes.end(); itNode2++)
                    {
                        if (connections[*itNode][*itNode2] != -1.0)
                        {
                            cnt++;
                        }
                    }
                }
                return cnt;
            }

            /*IDL methods*/
            /***************************************************************/

            void start()
            {
                moduleStart();
            }

            void pause()
            {
                modulePause();
            }

            virtual bool configure(yarp::os::Property &rf)
            {
                //Call the base class configure
                this->IConvergenceZone::configure(rf);

                //Get additional parameters
                double default_dtreshold = 0.01;
                if (!parametersRuntime.check("dtreshold"))
                    parametersRuntime.put("dtreshold", yarp::os::Value(default_dtreshold));
                if (!parametersRuntime.check("learningRate"))
                    parametersRuntime.put("learningRate", yarp::os::Value(0.01));
                if (!parametersRuntime.check("sigma"))
                    parametersRuntime.put("sigma", yarp::os::Value(default_dtreshold));
                if (!parametersRuntime.check("stepsBeforePrunning"))
                    parametersRuntime.put("stepsBeforePrunning", yarp::os::Value(10));
                if (!parametersRuntime.check("prunningStrenghtTreshold"))
                    parametersRuntime.put("prunningStrenghtTreshold", yarp::os::Value(1.0));

                useSoftMaxPrediction = !rf.check("noSoftMax");
                learnStepCnt = 0;

                //Good to go!
                std::cout << std::endl << "ESOM Model:" << std::endl
                    << "Distance treshold = " << parametersRuntime.find("dtreshold").asDouble() <<std::endl;

                return true;
            }

            virtual void ComputePrediction()
            {
                mutex.wait();

                //Compute the sum of all the modalities influence
                double influenceTotal = 0;
                for (std::map< IModality*, double>::iterator it = modalitiesInfluence.begin(); it != modalitiesInfluence.end(); it++)
                    influenceTotal += it->second;

                //Reset all nodes activity
                for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                    (*itNode)->activity = 0.0;

                //------------------------------------------------------------------------------------------------------------------------
                //Calculate the distance to all nodes
                ESOMNode* bestNode = NULL;

                for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                {
                    for (std::map< IModality*, double>::iterator itMod = modalitiesInfluence.begin(); itMod != modalitiesInfluence.end(); itMod++)
                    {
                        //In case of 0 influence we just skip the modality
                        if (itMod->second == 0.0)
                            continue;

                        std::vector<double> vReal = itMod->first->GetValueReal();
                        double dMod = 0.0;
                        for (int i = 0; i < itMod->first->Size(); i++)
                        {
                            dMod += fabs(((*itNode)->prototype[itMod->first][i] - vReal[i]));
                        }
                        dMod /= itMod->first->Size();
                        (*itNode)->activity += dMod * itMod->second;
                    }
                    (*itNode)->activity /= influenceTotal;
                    if (bestNode == NULL || bestNode->activity >(*itNode)->activity)
                        bestNode = (*itNode);
                }

                //Check if we need to create a new node
                if (bestNode == NULL || bestNode->activity > parametersRuntime.find("dtreshold").asDouble())
                {
                    bestNode = addNewNode();
                }

                //Learning
                bool allModLearningZero = true;
                for (std::map<IModality*, double>::iterator itL = modalitiesLearning.begin(); itL != modalitiesLearning.end(); itL++)
                    allModLearningZero &= (itL->second == 0.0);
                if (parametersRuntime.find("learningRate").asDouble() > 0.0 && !allModLearningZero)
                    adaptWeights(bestNode);

                //feedback
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                    predictModality(it->second, bestNode);
                //feedforward
                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                    predictModality(it->second, bestNode);

                mutex.post();
            }

            double getDistanceOf(ESOMNode* n)
            {
                double d = 0.0;
                for (std::map< IModality*, double>::iterator itMod = modalitiesInfluence.begin(); itMod != modalitiesInfluence.end(); itMod++)
                {
                    std::vector<double> vReal = itMod->first->GetValueReal();
                    double dMod = 0.0;
                    for (int i = 0; i < itMod->first->Size(); i++)
                    {
                        dMod += fabs((n->prototype[itMod->first][i] - vReal[i]));
                    }
                    dMod /= itMod->first->Size();
                    d += dMod;
                }
                return d;
            }

            ESOMNode* addNewNode()
            {
                std::cout << "Adding a new node. Node count=" << nodes.size() + 1 << std::endl;
                ESOMNode* newNode = new ESOMNode();
                for (std::map< IModality*, double>::iterator itMod = modalitiesInfluence.begin(); itMod != modalitiesInfluence.end(); itMod++)
                {
                    newNode->prototype[itMod->first] = itMod->first->GetValueReal();
                }

                //Identify the 2 nearest neighbhoors
                ESOMNode* near1 = NULL;
                ESOMNode* near2 = NULL;
                double near1d = 999.99;
                double near2d = 999.99;
                for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                {
                    double d = newNode->distanceTo(*itNode);
                    if (d <= near2d)
                    {
                        if (d <= near1d)
                        {
                            near2 = near1;
                            near2d = near1d;
                            near1 = *itNode;
                            near1d = d;
                        }
                        else
                        {
                            near2 = *itNode;
                            near2d = d;
                        }
                    }
                }

                nodes.push_back(newNode);
                //Update the connectivity matrix with this new node
                for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                {
                    connections[newNode][*itNode] = -1.0;
                    connections[*itNode][newNode] = -1.0;
                }

                if (near1 != NULL)
                {
                    connections[newNode][near1] = 1.0;
                    connections[near1][newNode] = 1.0;
                }
                if (near2 != NULL)
                {
                    connections[newNode][near2] = 1.0;
                    connections[near2][newNode] = 1.0;
                }
                if (near1 != NULL && near2 != NULL && connections[near1][near2] == -1.0)
                {
                    connections[near1][near2] = 1.0;
                    connections[near2][near1] = 1.0;
                }

                connections[newNode][newNode] = 1.0;
                return newNode;
            }


            void adaptWeights(ESOMNode* winner)
            {
                for (std::map< IModality*, double>::iterator itMod = modalitiesInfluence.begin(); itMod != modalitiesInfluence.end(); itMod++)
                {
                    std::vector<double> vReal = itMod->first->GetValueReal();
                    for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                    {
                        if (connections[winner][*itNode] != -1.0)
                        {
                            for (int i = 0; i < itMod->first->Size(); i++)
                            {
                                double error = vReal[i] - (*itNode)->prototype[itMod->first][i];
                                double dW = parametersRuntime.find("learningRate").asDouble() * exp(-pow((*itNode)->activity, 2.0) / (2 * pow(parametersRuntime.find("sigma").asDouble(),2.0))) * error;
                                (*itNode)->prototype[itMod->first][i] += dW;
                            }
                        }
                    }
                }

                //Reset connection strenght
                for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                {
                    if (connections[winner][*itNode] != -1.0)
                    {
                        connections[winner][*itNode] = parametersRuntime.find("dtreshold").asDouble() / getDistanceOf(*itNode);
                        connections[*itNode][winner] = connections[winner][*itNode];
                    }
                }

                //Increase the learning counter for prunning
                learnStepCnt++;
                if (learnStepCnt>parametersRuntime.find("stepsBeforePrunning").asInt())
                {
                    std::cout << "Prunning connection...";
                    learnStepCnt = 0;
                    std::list<ESOMNode*>::iterator a = nodes.end();
                    std::list<ESOMNode*>::iterator b = nodes.end();;
                    for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                    {
                        for (std::list<ESOMNode*>::iterator itNode2 = nodes.begin(); itNode2 != nodes.end(); itNode2++)
                        {
                            if (connections[*itNode][*itNode2] != -1.0)
                            {

                                if (a == nodes.end() || connections[*itNode][*itNode2] < connections[*a][*b])
                                {
                                    a = itNode;
                                    b = itNode2;
                                }
                            }
                        }
                    }

                    if (nodes.size() <= 3 || connections[*a][*b] > parametersRuntime.find("prunningStrenghtTreshold").asDouble())
                    {
                        std::cout << "cancelled. (Only one node or connection too strong = " << connections[*a][*b] <<")." << std::endl;
                        return;
                    }

                    //Prune the connection
                    connections[*a][*b] = -1;
                    connections[*b][*a] = -1;
                    std::cout << "Done" << std::endl;

                    //check if a should be pruned
                    std::cout << "Prunning node...";
                    bool isConnectedA = false;
                    bool isConnectedB = false;
                    for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                    {
                        if (connections[*a][*itNode] != -1.0)
                            isConnectedA = true;
                        if (connections[*b][*itNode] != -1.0)
                            isConnectedB = true;
                    }

                    ESOMNode* ptra = *a;
                    ESOMNode* ptrb = *b;

                    if (!isConnectedA)
                    {
                        std::cout << "Erasing one node...";
                        for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                        {
                            connections[*itNode].erase(ptra);
                        }
                        connections.erase(ptra);
                        nodes.erase(a);
                    }
                    if (!isConnectedB)
                    {
                        std::cout << "Erasing one node...";
                        for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                        {
                            connections[*itNode].erase(ptrb);
                        }
                        connections.erase(ptrb);
                        nodes.erase(b);
                    }
                    std::cout << "Done" << std::endl;
                    std::cout << "Prunning over. Node count=" << nodes.size() << std::endl;
                }
            }

            void predictModality(IModality* mod, ESOMNode* winner)
            {
                //SINGLE WINNER PREDICTION
                if (!useSoftMaxPrediction)
                {
                    mod->SetValuePrediction(winner->prototype[mod]);
                    return;
                }
                //SOFT MAX
                //Get all the nodes connected to the winner
                std::vector<double> softMax(winner->prototype[mod].size(), 0.0);
                double sum = 0.0;
                for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                {
                    if (connections[winner][*itNode] != -1.0)
                    {
                        sum += exp(1 - (*itNode)->activity);
                    }
                }

                for (std::list<ESOMNode*>::iterator itNode = nodes.begin(); itNode != nodes.end(); itNode++)
                {
                    if (connections[winner][*itNode] != -1.0)
                    {
                        for (int c = 0; c < (*itNode)->prototype[mod].size(); c++)
                        {
                            softMax[c] += (*itNode)->prototype[mod][c] * exp((1 - (*itNode)->activity)) / sum;
                        }
                    }
                }

                mod->SetValuePrediction(winner->prototype[mod]);
            }
        };
    }
}

#endif

#ifndef __CVZ_CVZNEURALMAP_H__
#define __CVZ_CVZNEURALMAP_H__

#include <vector>
#include <list>
#include <algorithm>
#include "ICvz.h"
#include "cvz/helpers/helpers.h"
#include "cvz/core/CvzSheet.h"

namespace cvz {
    namespace core {

#define MAGIC_NUMBER_INTERNAL_MODALITY_SIZE 3

            class CvzFiber
            {
            public:
                std::vector<CvzSheet> layers;
                std::map<IModality*, IModality*> connections;

                CvzFiber()
                {

                }

                bool configure(yarp::os::Property &prop)
                {

                    yarp::os::Bottle* layersStructure = prop.find("layersStructure").asList();

                    //The size comes in the form :
                    //  ( 
                    //      (2 ( (type mmcm) (height 10) (width 10) (layers 10) ) ) 
                    //      (1 ( (type mmcm) (height 10) (width 10) (layers 10) ) ) 
                    //      (5 ( (type mmcm) (height 10) (width 10) (layers 10) ) ) 
                    //  ) 
                    //for a 3 layered fiber with a 2x2, a single and a 5x5 sheets of cvz using their respective config file
                    //
                    int layersCount = layersStructure->size();
                    std::cout << "Creating a fiber of " << layersCount << " layers " << layersStructure->toString() << std::endl;
                    layers.resize(layersCount);

                    for (int l = 0; l < layersCount; l++)
                    {
                        int sqrSize = layersStructure->get(l).asList()->get(0).asInt();
                        yarp::os::Bottle* mapStructure = layersStructure->get(l).asList()->get(1).asList();
                        yarp::os::Property p;
                        if (mapStructure != NULL)
                            mapStructure->write(p);
                        else
                        {
                            std::cout << "[CvzFiber] Warning: no map structure provided for layer " << l << std::endl;
                        }

                        //Deal with the layer map names
                        std::string nameRoot = p.check("name", yarp::os::Value("default")).asString();
                        p.unput("name");
                        std::stringstream nameTotal;
                        nameTotal << nameRoot << "_"<<l;
                        p.put("name", nameTotal.str());

                        int modalityCounter = 0;
                        //Automatic generation of the input modalities
                        if (l != 0)
                        {
                            int previousSqrSize = layersStructure->get(l - 1).asList()->get(0).asInt();
                            //int previousOutputModalitiesCount = 1;
                            int inputModalitiesCount = 1;
                            if (previousSqrSize >= sqrSize)
                            {
                                std::cout << "[CvzFiber] " << l - 1 << "->" << l << " is convergent" << std::endl;
                                inputModalitiesCount = (int)pow(previousSqrSize / sqrSize, 2.0);
                            }
                            else
                            {
                                std::cout << "[CvzFiber] " << l-1 << "->" << l << " is divergent" << std::endl;
                                //inputModalitiesCount = (int) pow(previousSqrSize / sqrSize, 2.0);
                            }

                            for (int iMod = 0; iMod < inputModalitiesCount; iMod++)
                            {
                                std::stringstream ssModGroup;
                                ssModGroup << "modality_" << modalityCounter;
                                yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
                                std::stringstream ssModName;
                                ssModName << "input_" << iMod;
                                pMod.put("name", ssModName.str());
                                pMod.put("size", MAGIC_NUMBER_INTERNAL_MODALITY_SIZE);
                                modalityCounter++;
                            }
                        }
                        else //first layer, we generate the out-of-fiber inputs here
                        {
                            yarp::os::Bottle* inputModalityPrototype = prop.find("inputModalityPrototype").asList();
                            std::stringstream ssModGroup;
                            ssModGroup << "modality_" << modalityCounter;
                            yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
                            inputModalityPrototype->write(pMod);
                            modalityCounter++;
                        }

                        //Automatic generation of the output modalities
                        if (l != layersStructure->size() - 1)
                        {
                            int nextSqrSize = layersStructure->get(l + 1).asList()->get(0).asInt();
                            int outputModalitiesCount = 1;
                            if (nextSqrSize <= sqrSize)
                            {
                                std::cout << "[CvzFiber] " << l << "->" << l + 1 << " is convergent" << std::endl;
                            }
                            else
                            {
                                std::cout << "[CvzFiber] " << l << "->" << l + 1 << " is divergent" << std::endl;
                                outputModalitiesCount = (int)pow(nextSqrSize / sqrSize, 2.0);
                            }
                            
                            for (int iMod = 0; iMod < outputModalitiesCount; iMod++)
                            {
                                std::stringstream ssModGroup;
                                ssModGroup << "modality_" << modalityCounter;
                                yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
                                std::stringstream ssModName;
                                ssModName << "output_" << iMod;
                                pMod.put("name", ssModName.str());
                                pMod.put("isTopDown", yarp::os::Value(1));
                                pMod.put("size", MAGIC_NUMBER_INTERNAL_MODALITY_SIZE);
                                modalityCounter++;
                            }        
                        }
                        
                        if (l==(int)layers.size()-1)//last layer, we create the potential out-of-fiber convergence at this level
                        {
                            yarp::os::Bottle* outputModalityPrototype = prop.find("outputModalityPrototype").asList();
                            std::stringstream ssModGroup;
                            ssModGroup << "modality_" << modalityCounter;
                            yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
                            outputModalityPrototype->write(pMod);
                            modalityCounter++;
                        }
                        
                        bool isFine = layers[l].configure(sqrSize, sqrSize, p);
                        if (!isFine)
                        {
                            std::cout << "Fiber : Problems during configuration... Aborting." << std::endl
                                << "***********************************" << std::endl;
                        }
                    }

                    //Compute the connectivity pattern
                    createConnections();

                    //Autoconnect to the input prototype
                    std::string autoConnectStem = prop.check("autoConnectInputStem",yarp::os::Value("")).asString();
                    if (autoConnectStem != "")
                    {
                        std::cout << "Trying autoconnection of first layer on the stem name : " << autoConnectStem << std::endl;
                        
                        for (size_t x = 0; x < layers[0].size(); x++)
                        {
                            for (size_t y = 0; y < layers[0][x].size(); y++)
                            {
                                std::string inputModalityName = layers[0][x][y]->modalitiesBottomUp.begin()->second->GetFullNameReal();
                                std::stringstream ssPortExt;
                                ssPortExt << autoConnectStem << x << "_" << y << ":o";
                                std::cout << "Trying to get input from " << ssPortExt.str() << " to " << inputModalityName << std::endl;
                                yarp::os::Network::connect(ssPortExt.str(), inputModalityName);
                            }
                        }
                    }

                    return true;
                }

                bool cycle()
                {
                    //double t0 = yarp::os::Time::now();
                    for (size_t l = 0; l < layers.size(); l++)
                    {
                        //double t1 = yarp::os::Time::now();
                        //Refresh from bottom up
                        layers[l].cycle();
                        //std::cout << "Layer " << l << " cycle time = " << yarp::os::Time::now() - t1 << std::endl;
                        if (l != layers.size() - 1)
                        {
                            //Propagate to next layer
                            for (size_t x1 = 0; x1 < layers[l].size(); x1++)
                            {
                                for (size_t y1 = 0; y1 < layers[l][x1].size(); y1++)
                                {
                                    for (std::map<std::string, IModality*>::iterator itSrc = layers[l][x1][y1]->modalitiesTopDown.begin(); itSrc != layers[l][x1][y1]->modalitiesTopDown.end(); itSrc++)
                                    {
                                        IModality* src = itSrc->second;
                                        IModality* target = connections[src];

                                        std::vector<double> srcPred = src->GetValuePrediction();
                                        target->SetValueReal(srcPred);
                                        std::vector<double> targetPred = target->GetValuePrediction();
                                        src->SetValueReal(targetPred);
                                    }
                                }
                            }
                        }
                    }
                    //std::cout << "Whole fiber cycle time = " << yarp::os::Time::now() - t0 << std::endl;
                    return true;
                }



                //Search for the first not connected modality on the src cvz, connect it to the first available modality on the target cvz
                bool connectFreeModalities(IConvergenceZone* src, IConvergenceZone* dest, std::list<IModality*>& usedModalities)
                {
                    for (std::map<std::string, IModality*>::iterator itSrcMod = src->modalitiesTopDown.begin(); itSrcMod != src->modalitiesTopDown.end(); itSrcMod++)
                    {
                        if (find(usedModalities.begin(), usedModalities.end(), itSrcMod->second) == usedModalities.end())
                        {
                            IModality* srcModality = itSrcMod->second;
                            for (std::map<std::string, IModality*>::iterator itDestMod = dest->modalitiesBottomUp.begin(); itDestMod != dest->modalitiesBottomUp.end(); itDestMod++)
                            {
                                if (find(usedModalities.begin(), usedModalities.end(), itDestMod->second) == usedModalities.end())
                                {
                                    IModality* destModality = itDestMod->second;
                                    connections[srcModality] = destModality;
                                    usedModalities.push_back(srcModality);
                                    usedModalities.push_back(destModality);
                                    std::cout << "Established a connection from " << src->getName() << " to " << dest->getName() << " using " << srcModality->GetFullName() << "-->" << destModality->GetFullName() << std::endl;
                                    return true;
                                }
                            }
                        }
                    }
                    std::cerr << "[CvzFiber] warning in the connectivity pattern" << std::endl;
                    return false;
                }

                //Create the connectivity matrix according to the convergence/divergence pattern between each layer of the fiber
                void createConnections()
                {
                    std::list<IModality*> usedModalities;

                    //Create the connections
                    for (size_t l = 0; l < layers.size(); l++)
                    {
                        //Refresh from bottom up
                        //layers[l].cycle();

                        //Propagate the activity upward
                        if (l < layers.size() - 1)
                        {
                            int srcSheetSize = layers[l].size();
                            int destSheetSize = layers[l + 1].size();
                            double convergenceRatio = srcSheetSize / destSheetSize;

                            for (size_t x1 = 0; x1 < layers[l].size(); x1++)
                            {
                                for (size_t y1 = 0; y1 < layers[l][x1].size(); y1++)
                                {
                                    IConvergenceZone* srcCvz = layers[l][x1][y1];
                                    for (size_t x2 = 0; x2 < layers[l+1].size(); x2++)
                                    {
                                        for (size_t y2 = 0; y2 < layers[l+1][x2].size(); y2++)
                                        {
                                            IConvergenceZone* destCvz = layers[l+1][x2][y2];

                                            if (convergenceRatio>1) //convergence : 1 to many
                                            {
                                                //Check if those two are connected
                                                int modulo = x1 % (srcSheetSize / destSheetSize);
                                                int base = x1 / (srcSheetSize / destSheetSize);
                                                bool isConnected = ((int)x2 == base);
                                                
                                                modulo = y1 % (srcSheetSize / destSheetSize);
                                                base = y1 / (srcSheetSize / destSheetSize);
                                                isConnected &= ((int)y2 == base);
                                                if (isConnected)
                                                    connectFreeModalities(srcCvz, destCvz, usedModalities);
                                            }
                                            else //divergence many to 1
                                            {
                                                //Check if those two are connected
                                                int minX1 = x1 * (destSheetSize / srcSheetSize);
                                                int maxX1 = minX1 + (destSheetSize / srcSheetSize);
                                                int minY1 = y1 * (destSheetSize / srcSheetSize);
                                                int maxY1 = minY1 + (destSheetSize / srcSheetSize);
                                                bool isConnected = (((int)x2 >= minX1) && ((int)x2 < maxX1));
                                                isConnected &= (((int)y2 >= minY1) && ((int)y2 < maxY1));
                                                if (isConnected)
                                                    connectFreeModalities(srcCvz, destCvz, usedModalities);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

            };
    }
}
#endif

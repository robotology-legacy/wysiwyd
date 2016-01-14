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
                std::map<IModality*, IModality*> inverted_connections;
                
                CvzFiber()
                {

                }
                double getDistance(double x1, double y1, double x2, double y2)
                {
                    double d = 0.0;
                    double dX = std::abs(x1 - x2);
                    double dY = std::abs(y1 - y2);
                    double tdX = std::abs(x1 + (1.0 - x2));
                    double tdY = std::abs(y1 + (1.0 - y2));
                    d = sqrt(pow(std::min(dX, tdX), 2.0) + pow(std::min(dY, tdY), 2.0));
                    return d;
                }

                int countInputModalities(const int &sizePrevious, const int &size, const double &radius)
                {

                    double spacing = 1.0 / (double)(1.0 + size);
                    double spacingPrevious = 1.0 / (double)(1.0 + sizePrevious);

                    //Count incoming
                    int input = 0;
                    for (int aX = 1; aX <= sizePrevious; aX++)
                    {
                        for (int aY = 1; aY <= sizePrevious; aY++)
                        {
                            double ab = getDistance(aX*spacingPrevious, aY*spacingPrevious, spacing, spacing);
                            if (ab <= radius)
                            {
                                input++;
                            }
                        }
                    }
                    return input;
                }

                int countOutputModalities(const int &size, const int &sizeNext, const double &radius)
                {

                    double spacingNext = 1.0 / (double)(1.0 + sizeNext);
                    double spacing = 1.0 / (double)(1.0 + size);

                    //Count outgoing
                    int output = 0;
                    for (int bX = 1; bX <= sizeNext; bX++)
                    {
                        for (int bY = 1; bY <= sizeNext; bY++)
                        {
                            double ab = getDistance(spacing, spacing, bX*spacingNext, bY*spacingNext);
                            if (ab <= radius)
                            {
                                output++;
                            }
                        }
                    }
                    return output;
                }
                bool configure(yarp::os::Property &prop)
                {
                    double arborisationRadius = prop.check("arborisationRadius", 0.75).asDouble();
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
                    std::cout << "Creating a fiber of " << layersCount << " layers " << layersStructure->toString() << " with an arborisationRadius of " << arborisationRadius<< std::endl;
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
                        nameTotal << nameRoot << "_" << l;
                        p.put("name", nameTotal.str());

                        int modalityCounter = 0;
                        //Automatic generation of the input modalities

                        int inputModalitiesCount = 0;
                        int outputModalitiesCount = 0;
                        int previousSqrSize = 0;
                        int nextSize = 0;
                        if (l != 0)
                        {
                            previousSqrSize = layersStructure->get(l - 1).asList()->get(0).asInt();
                            inputModalitiesCount = countInputModalities(previousSqrSize, sqrSize, arborisationRadius);
                        }
                        if (l != (int)layers.size() - 1)
                        {
                            nextSize = layersStructure->get(l + 1).asList()->get(0).asInt();
                            outputModalitiesCount = countOutputModalities(sqrSize, nextSize, arborisationRadius);
                        }

                        //Input
                        if (l != 0)
                        {
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
                        else
                        {
                            std::string debugFck = prop.toString();
                            yarp::os::Bottle* inputModalityPrototype = prop.find("inputModalityPrototype").asList();
                            if (inputModalityPrototype)
                            {
                                std::stringstream ssModGroup;
                                ssModGroup << "modality_" << modalityCounter;
                                yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
                                inputModalityPrototype->write(pMod);
                                modalityCounter++;
                            }
                        }

                        //Output
                        if (l != (int)layers.size() - 1)
                        {
                            for (int iMod = 0; iMod < outputModalitiesCount; iMod++)
                            {
                                std::stringstream ssModGroup;
                                ssModGroup << "modality_" << modalityCounter;
                                yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
                                std::stringstream ssModName;
                                ssModName << "output_" << iMod;
                                pMod.put("name", ssModName.str());
                                pMod.put("isTopDown", yarp::os::Value(1));
                                pMod.put("learningRate", yarp::os::Value(0.0));
                                pMod.put("size", MAGIC_NUMBER_INTERNAL_MODALITY_SIZE);
                                modalityCounter++;
                            }
                        }
                        else
                        {
                            yarp::os::Bottle* outputModalityPrototype = prop.find("outputModalityPrototype").asList();
                            if (outputModalityPrototype)
                            {
                                std::stringstream ssModGroup;
                                ssModGroup << "modality_" << modalityCounter;
                                yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
                                outputModalityPrototype->write(pMod);
                                modalityCounter++;
                            }
                        }
                        
                        bool isFine = layers[l].configure(sqrSize, sqrSize, p);
                        if (!isFine)
                        {
                            std::cout << "Fiber : Problems during configuration... Aborting." << std::endl
                                << "***********************************" << std::endl;
                        }
                    }

                    //Compute the connectivity pattern
                    createConnections(arborisationRadius);

                    //Autoconnect to the input prototype to a matrix of ports (e.g output of ImageSplitter)
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
                    double t0 = yarp::os::Time::now();
                    for (size_t l = 0; l < layers.size(); l++)
                    {
                        double t1 = yarp::os::Time::now();
                        //Refresh from bottom up
                        layers[l].cycle();
                        std::cout << "Layer " << l << " cycle time = " << yarp::os::Time::now() - t1 << std::endl;
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
                    std::cout << "Whole fiber cycle time = " << yarp::os::Time::now() - t0 << std::endl;
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
                                    inverted_connections[destModality] = srcModality;
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
                void createConnections(double radius)
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

                            double spacingA = 1.0 / (double)(1.0 + srcSheetSize);
                            double spacingB = 1.0 / (double)(1.0 + destSheetSize);

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
                                            
                                            double ab = getDistance((1+x1)*spacingA, (1+y1)*spacingA, (1+x2)*spacingB, (1+y2)*spacingB);
                                            if (ab <= radius)
                                                connectFreeModalities(srcCvz, destCvz, usedModalities);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                void close()
                {
                    for (size_t l = 0; l < layers.size(); l++)
                    {
                        layers[l].close();
                    }
                }

                //Retrieve the receptive field of a given neuron, on a given map by backpropagating it until it reaches the bottom of the fiber.
                //Recursive function. Will go down the fiber until it cannot anymore
                IplImage* getRFforNeuron(CvzMMCM* map, int x, int y, int z)
                {
                    std::list<IplImage*> modImgs;
                    for (std::map<std::string, IModality*>::iterator itBot = map->modalitiesBottomUp.begin(); itBot != map->modalitiesBottomUp.end(); itBot++)
                    {
                        IplImage* modalityRF;
                        //Check something is sending input to this modality
                        if (inverted_connections.find(itBot->second) != inverted_connections.end())
                        {
                            //Get the receptive field of this neuron
                            std::vector<double> rf = map->getReceptiveFieldWeights(x, y, z, itBot->second);
                            int bmuX, bmuY, bmuZ;

                            //Get the best matching unit for the projecting map
                            CvzMMCM* projectingMap = (CvzMMCM*)inverted_connections[itBot->second]->GetOwnerCvz();
                            projectingMap->getBestMatchingUnit(rf, inverted_connections[itBot->second], bmuX, bmuY, bmuZ);
                            modalityRF = getRFforNeuron(projectingMap, bmuX, bmuY, bmuZ);
                        }
                        else
                        {
                            yarp::sig::ImageOf<yarp::sig::PixelRgb> imgrf = map->getReceptiveFieldRepresentation(x, y, z, itBot->second);
                            modalityRF = cvCreateImage(cvSize(imgrf.width(), imgrf.height()), 8, 3);
                            cvCopyImage(imgrf.getIplImage(), modalityRF);
                        }
                        modImgs.push_back(modalityRF);
                    }

                    //Add up all the modalities
                    //1. calculate the size of the full image
                    int totalW = 0;
                    int totalH = 0;
                    for (std::list<IplImage*>::iterator it = modImgs.begin(); it != modImgs.end(); it++)
                    {
                        totalW += (*it)->width;
                        totalH = std::max(totalH, (*it)->height);
                    }

                    //2. Create the image
                    IplImage* fullImg = cvCreateImage(cvSize(totalW, totalH), 8, 3);
                    
                    //3. copy all sub parts in the right place && free memory
                    int xOffset = 0;
                    for (std::list<IplImage*>::iterator it = modImgs.begin(); it != modImgs.end(); it++)
                    {
                        cvSetImageROI(fullImg, cvRect(xOffset, 0, (*it)->width, (*it)->height));
                        cvCopyImage((*it), fullImg);
                        cvResetImageROI(fullImg);
                        xOffset += (*it)->width;
                        cvReleaseImage(&(*it));
                    }

                    return fullImg;
                }

                //Get the receptive field of a layer by going down to the first layer
                IplImage* getRecursiveRFLayer(int layer)
                {
                    bool isMMCMFiber = true;
                    for (std::vector<CvzSheet>::iterator it = layers.begin(); it != layers.end(); it++)
                        isMMCMFiber &= layers[layer].isFullMMCM();
                    if (!isMMCMFiber)
                    {
                        std::cerr << "Not full mmcm returning empty image." << std::endl;
                        IplImage* fullImg = cvCreateImage(cvSize(1, 1), 8, 3);
                        return fullImg;
                    }
                    
                    IplImage* fullLayerImage;
                    std::vector< std::vector< IplImage* > > subpartsFullLayer;
                    subpartsFullLayer.resize(layers[layer].Width());

                    for (int xMap = 0; xMap < layers[layer].Width(); xMap++)
                    {
                        subpartsFullLayer[xMap].resize(layers[layer].Height());

                        for (int yMap = 0; yMap < layers[layer].Height(); yMap++)
                        {
                            CvzMMCM* map = (CvzMMCM*)layers[layer][xMap][yMap];

                            std::vector< std::vector< IplImage* > > subparts;
                            subparts.resize(map->W());
                            for (int x = 0; x < map->W(); x++)
                            {
                                subparts[x].resize(map->H());
                                for (int y = 0; y < map->H(); y++)
                                {
                                    IplImage* tmpForZ = NULL;
                                    for (int z = 0; z < map->L(); z++)
                                    {
                                        IplImage* rf = getRFforNeuron(map, x, y, z);

                                        if (tmpForZ == NULL)
                                            tmpForZ = cvCreateImage(cvSize(rf->width*map->L(), rf->height), 8, 3);

                                        cvSetImageROI(tmpForZ, cvRect(z*rf->width, 0, rf->width, rf->height));
                                        cvCopyImage(rf, tmpForZ);
                                        cvResetImageROI(tmpForZ);
                                        cvReleaseImage(&rf);
                                    }
                                    subparts[x][y] = tmpForZ;
                                }
                            }

                            //Add up all the modalities
                            //1. calculate the size of the full image
                            int totalW = subparts[0][0]->width * map->W();
                            int totalH = subparts[0][0]->height * map->H();


                            //2. Create the image
                            IplImage* fullImg = cvCreateImage(cvSize(totalW, totalH), 8, 3);

                            //3. copy all sub parts in the right place && free memory
                            for (int x = 0; x < map->W(); x++)
                            {
                                for (int y = 0; y < map->H(); y++)
                                {
                                    cvSetImageROI(fullImg, cvRect(x*subparts[x][y]->width, y*subparts[x][y]->height, subparts[x][y]->width, subparts[x][y]->height));
                                    cvCopyImage(subparts[x][y], fullImg);
                                    cvResetImageROI(fullImg);
                                    cvReleaseImage(&subparts[x][y]);
                                }
                            }

                            subpartsFullLayer[xMap][yMap] = fullImg;
                        }
                    }

                    //Add up all the modalities
                    //1. calculate the size of the full image
                            //HERE THERE MAY BE A BUG
                            //I assumed that all the subparts would have the same size, which is not guaranted... 
                    int totalW = subpartsFullLayer[0][0]->width * layers[layer].Width();
                    int totalH = subpartsFullLayer[0][0]->height * layers[layer].Height();;

                    //2. Create the image
                    fullLayerImage = cvCreateImage(cvSize(totalW, totalH), 8, 3);

                    //3. copy all sub parts in the right place && free memory
                    for (int xMap = 0; xMap < layers[layer].Width(); xMap++)
                    {
                        for (int yMap = 0; yMap < layers[layer].Height(); yMap++)
                        {
                            cvSetImageROI(fullLayerImage, cvRect(xMap*subpartsFullLayer[xMap][yMap]->width, yMap*subpartsFullLayer[xMap][yMap]->height, subpartsFullLayer[xMap][yMap]->width, subpartsFullLayer[xMap][yMap]->height));
                            cvCopyImage(subpartsFullLayer[xMap][yMap], fullLayerImage);
                            cvResetImageROI(fullLayerImage);
                            cvReleaseImage(&subpartsFullLayer[xMap][yMap]);
                        }
                    }
                    return fullLayerImage;
                }
            };
    }
}
#endif

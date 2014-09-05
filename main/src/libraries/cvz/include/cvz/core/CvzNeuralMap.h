#ifndef __CVZ_CVZNEURALMAPH__
#define __CVZ_CVZNEURALMAP_H__

#include <vector>
#include <list>
#include <algorithm>
#include "ICvz.h"
#include "cvz/helpers/helpers.h"


namespace cvz {
	namespace core {

#define MAGIC_NUMBER_INTERNAL_MODALITY_SIZE 3

			class CvzSheet:public std::vector<std::vector< IConvergenceZone* > >
			{
			public:
				bool configure(int w, int h, yarp::os::Property &prop)
				{
					std::cout << "***********************************"<<std::endl
						<<"Configuring CvzSheet (" << w << "x" << h << ")";

					this->resize(w);
					for (int x = 0; x < w; x++)
					{
						this->operator[](x).resize(h);
						for (int y = 0; y < h; y++)
						{
							yarp::os::Property prop2 = prop;
							std::string nameRoot = prop2.check("name", yarp::os::Value("default")).asString();
							prop2.unput("name");
							std::stringstream nameTotal; 
							nameTotal << nameRoot << "_" << x << "_" << y;
							prop2.put("name", nameTotal.str());
							this->operator[](x)[y] = new IConvergenceZone();
							bool isFine = this->operator[](x)[y]->configure(prop2);
							if (!isFine)
							{
								std::cout << "Problems during configuration... Aborting." << std::endl
									<< "***********************************" << std::endl;
								return false;
							}
						}
					}
					std::cout << "Configured CvzSheet" << std::endl
						<< "***********************************" << std::endl;
					return true;
						
				}

				void cycle()
				{
					for (int x = 0; x < this->size(); x++)
					{
						for (int y = 0; y < this->operator[](x).size(); y++)
						{
							bool isFine = this->operator[](x)[y]->cycle();
						}
					}
				}
			};

			class CvzFiber
			{
				std::vector<CvzSheet> layers;
				std::map<IModality*, IModality*> connections;

				bool configure(yarp::os::Property &prop)
				{
					yarp::os::Bottle* layersStructure = prop.find("layersStructure").asList();

					//The size comes in the form :
					//	( 
					//		(2 ( (type mmcm) (height 10) (width 10) (layers 10) ) ) 
					//		(1 ( (type mmcm) (height 10) (width 10) (layers 10) ) ) 
					//		(5 ( (type mmcm) (height 10) (width 10) (layers 10) ) ) 
					//	) 
					//for a 3 layered fiber with a 2x2, a single and a 5x5 sheets of cvz using their respective config file
					//
					std::cout << "Creating a fiber of " << layersStructure->size() << " layers " << layersStructure->toString() << std::endl;
					layers.resize(layersStructure->size());
					for (int l = 0; l < layersStructure->size(); l++)
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

						int modalityCounter = 0;
						//Automatic generation of the input modalities
						if (l != 0)
						{
							int previousSqrSize = layersStructure->get(l - 1).asList()->get(0).asInt();
							int previousOutputModalitiesCount = 1;
							int inputModalitiesCount = 1;
							if (previousSqrSize >= sqrSize)
							{
								std::cout << "[CvzFiber] " << l-1 << "->" << l << " is convergent" << std::endl;
								previousOutputModalitiesCount = pow(previousSqrSize/sqrSize, 2.0);
							}
							else
							{
								std::cout << "[CvzFiber] " << l-1 << "->" << l << " is divergent" << std::endl;
								inputModalitiesCount = pow(previousSqrSize / sqrSize, 2.0);
							}

							for (int iMod = 0; iMod < inputModalitiesCount; iMod++)
							{
								std::stringstream ssModGroup;
								ssModGroup << "[modality_" << modalityCounter << "]";
								yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
								std::stringstream ssModName;
								ssModName << "input_" << iMod;
								pMod.put("name", ssModName.str());
								pMod.put("size", MAGIC_NUMBER_INTERNAL_MODALITY_SIZE);
								modalityCounter++;
							}
						}
						else //first layer
						{
							yarp::os::Bottle* inputModalityPrototype = prop.find("inputModalityPrototype").asList();
							std::stringstream ssModGroup;
							ssModGroup << "[modality_" << modalityCounter << "]";
							yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
							inputModalityPrototype->write(pMod);
							modalityCounter++;
						}

						//Automatic generation of the output modalities
						if (l != layersStructure->size() - 1)
						{
							int nextSqrSize = layersStructure->get(l + 1).asList()->get(0).asInt();
							int outputModalitiesCount = 1;
							int nextInputModalitiesCount = 1;
							if (nextSqrSize >= sqrSize)
							{
								std::cout << "[CvzFiber] " << l << "->" << l + 1 << " is convergent" << std::endl;
								nextInputModalitiesCount = pow(sqrSize / nextSqrSize,2.0);
							}
							else
							{
								std::cout << "[CvzFiber] " << l << "->" << l + 1 << " is divergent" << std::endl;
								outputModalitiesCount = pow(sqrSize / nextSqrSize, 2.0);
							}

							for (int iMod = 0; iMod < outputModalitiesCount; iMod++)
							{
								std::stringstream ssModGroup;
								ssModGroup << "[modality_" << modalityCounter << "]";
								yarp::os::Property &pMod = p.addGroup(ssModGroup.str());
								std::stringstream ssModName;
								ssModName << "output_" << iMod;
								pMod.put("name", ssModName.str());
								pMod.put("isTopDown", NULL);
								pMod.put("size", MAGIC_NUMBER_INTERNAL_MODALITY_SIZE);
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
					createConnections();
				}

				bool cycle()
				{
					for (int l = 0; l < layers.size(); l++)
					{
						//Refresh from bottom up
						layers[l].cycle();

						//Propagate to next layer
						for (int x1 = 0; x1 < layers[l].size(); x1++)
						{
							for (int y1 = 0; y1 < layers[l][x1].size(); y1++)
							{
								for (std::map<std::string, IModality*>::iterator itSrc = layers[l][x1][y1]->modalitiesTopDown.begin(); itSrc != layers[l][x1][y1]->modalitiesTopDown.end(); itSrc++)
								{
									IModality* src = itSrc->second;
									IModality* target = connections[src];

									target->SetValueReal(src->GetValuePrediction());
									src->SetValueReal(target->GetValuePrediction());
								}
							}
						}
					}
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
								}
								else
								{
									std::cerr << "[CvzFiber] problem in the connectivity pattern (no free target modality)" << std::endl;
									return false;
								}
							}
						}
						else
						{
							std::cerr << "[CvzFiber] warning in the connectivity pattern (no free source modality)" << std::endl;
							return false;
						}
					}
					return true;
				}

				//Create the connectivity matrix according to the convergence/divergence pattern between each layer of the fiber
				void createConnections()
				{
					std::list<IModality*> usedModalities;

					//Create the connections
					for (int l = 0; l < layers.size(); l++)
					{
						//Refresh from bottom up
						//layers[l].cycle();

						//Propagate the activity upward
						if (l < layers.size() - 1)
						{
							int srcSheetSize = layers[l].size();
							int destSheetSize = layers[l + 1].size();
							double convergenceRatio = srcSheetSize / destSheetSize;

							for (int x1 = 0; x1 < layers[l].size(); x1++)
							{
								for (int y1 = 0; y1 < layers[l][x1].size(); y1++)
								{
									IConvergenceZone* srcCvz = layers[l][x1][y1];
									for (int x2 = 0; x2 < layers[l+1].size(); x2++)
									{
										for (int y2 = 0; y2 < layers[l+1][x2].size(); y2++)
										{
											IConvergenceZone* destCvz = layers[l][x2][y2];

											if (convergenceRatio>1) //convergence : 1 to many
											{
												//Check if those two are connected
												bool isConnected = (x2 == x1 % (srcSheetSize / destSheetSize) + (int)(x1 / (srcSheetSize / destSheetSize)));
												isConnected &= (y2 == y1 % (srcSheetSize / destSheetSize) + (int)(y1 / (srcSheetSize / destSheetSize)));
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
												bool isConnected = (x2 >= minX1 && x2 < maxX1);
												isConnected &= (y2 >= minY1 && y2 < maxY1);
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

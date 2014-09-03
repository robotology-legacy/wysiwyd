#ifndef __CVZ_CVZNEURALMAPH__
#define __CVZ_CVZNEURALMAP_H__

#include <vector>
#include "ICvz.h"
#include "cvz/helpers/helpers.h"


namespace cvz {
	namespace core {

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
						for (int y = 0; y < w; y++)
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
			};

			class CvzFiber
			{
				std::vector<CvzSheet> layers;

				bool configure(yarp::os::Property &prop)
				{
					yarp::os::Bottle* layersStructure = prop.find("layersStructure").asList();

					//The size comes in the form ( (2 2 retina.ini) (1 1 v1.ini) (5 5 v2.ini) ) for a 3 layered fiber with a 2x2, a single and a 5x5 sheets of cvz using their respective config file
					std::cout << "Creating a fiber of " << layersStructure->size() << " layers " << layersStructure->toString() << std::endl;
					layers.resize(layersStructure->size());
					for (int l = 0; l < layersStructure->size(); l++)
					{
						int w = layersStructure->get(l).asList()->get(0).asInt();
						int h = layersStructure->get(l).asList()->get(1).asInt();
						std::string configFile = layersStructure->get(l).asList()->get(2).asString();
						yarp::os::Property p;
						p.fromConfigFile(configFile);
						bool isFine = layers[l].configure(w, h, p);
						if (!isFine)
						{
							std::cout << "Fiber : Problems during configuration... Aborting." << std::endl
								<< "***********************************" << std::endl;
						}
					}
				}
			};
	}
}
#endif

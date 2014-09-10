#ifndef __CVZ_CVZSHEET_H__
#define __CVZ_CVZSHEET_H__

#include <vector>
#include <list>
#include <algorithm>
#include "ICvz.h"
#include "cvz/helpers/helpers.h"


namespace cvz {
    namespace core {

            class CvzSheet:public std::vector<std::vector< IConvergenceZone* > >
            {
            public:
                CvzSheet()
                {

                }

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
                            //std::string debug = prop.toString();
                            yarp::os::Property prop2 = prop;
                            std::string nameRoot = prop2.check("name", yarp::os::Value("default")).asString();
                            prop2.unput("name");
                            std::stringstream nameTotal; 
                            nameTotal << nameRoot << "_" << x << "_" << y;
                            prop2.put("name", nameTotal.str());
                            CvzBuilder::allocate(&this->operator[](x)[y], prop2.check("type", yarp::os::Value(cvz::core::TYPE_ICVZ)).asString());
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
                    for (size_t x = 0; x < this->size(); x++)
                    {
                        for (size_t y = 0; y < this->operator[](x).size(); y++)
                        {
                            bool isFine = this->operator[](x)[y]->cycle();
                        }
                    }
                }
            };

    }
}
#endif

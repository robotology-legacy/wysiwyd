#ifndef __CVZ_CVZSHEET_H__
#define __CVZ_CVZSHEET_H__

#include <vector>
#include <list>
#include <algorithm>
#include "ICvz.h"
#include "cvz/helpers/helpers.h"

//OpenCV
#include <cv.h>
#include <cvaux.h>
#include "highgui.h"


namespace cvz {
    namespace core {

            void replace_all(std::string & in, const std::string & plain, const std::string & tok)
            {
                std::string::size_type n = 0;
                const std::string::size_type l = plain.length();
                while (1){
                    n = in.find(plain, n);
                    if (n != std::string::npos){
                        in.replace(n, l, tok);
                    }
                    else{
                        break;
                    }
                }
            }

            class CvzSheet:public std::vector<std::vector< IConvergenceZone* > >
            {
            private:
                int width;
                int height;
            public:
                int Width() { return width; }
                int Height() { return height; }

                CvzSheet()
                {
                    width = 0;
                    height = 0;
                }

                bool configure(int w, int h, yarp::os::Property &prop)
                {
                    std::cout << "***********************************"<<std::endl
                        <<"Configuring CvzSheet (" << w << "x" << h << ")";
                    width = w;
                    height = h;
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
                    for (size_t x = 0; x < width; x++)
                    {
                        for (size_t y = 0; y < height; y++)
                        {
                            this->operator[](x)[y]->cycle();
                        }
                    }
                }

                void close()
                {
                    for (size_t x = 0; x < width; x++)
                    {
                        for (size_t y = 0; y < height; y++)
                        {
                            this->operator[](x)[y]->close();
                        }
                    }
                }

                bool isFullMMCM()
                {
                    for (size_t x = 0; x < width; x++)
                    {
                        for (size_t y = 0; y < height; y++)
                        {
                            IConvergenceZone* cvzPtr = this->operator[](x)[y];
                            if (cvzPtr->getType() != TYPE_MMCM)
                            {
                                return false;
                            }
                        }
                    }
                    return true;
                }

                IplImage* getReceptiveField(int x, int y)
                {
                    IConvergenceZone* cvzPtr = this->operator[](x)[y];
                    if (cvzPtr->getType() != TYPE_MMCM)
                    {
                        std::cerr << "getReceptiveField(x,y) only implemented for MMCM type. Aborting..." << std::endl;
                        IplImage* fullImg = cvCreateImage(cvSize(1, 1), 8, 3);
                        return fullImg;
                    }

                    CvzMMCM * mmcmPtr = (CvzMMCM *)this->operator[](x)[y];
                    //First we probe the size of one RF.
                    int singlelW = 0; //We display the potential layers next to each other
                    int singleH = 0;
                    for (std::map<std::string, cvz::core::IModality* >::iterator mod = mmcmPtr->modalitiesBottomUp.begin(); mod != mmcmPtr->modalitiesBottomUp.end(); mod++)
                    {
                        yarp::sig::ImageOf<yarp::sig::PixelRgb> probe = mmcmPtr->getReceptiveFieldRepresentation(0, 0, 0, mod->second);
                        singlelW += probe.width();
                        singleH = std::max(singleH, probe.height());
                    }
                    int totalW = singlelW * (mmcmPtr->W() * mmcmPtr->L()); //We display the potential layers next to each other
                    int totalH = singleH * mmcmPtr->H();


                    IplImage* fullImg = cvCreateImage(cvSize(totalW, totalH), 8, 3);
                    //here we should fill image with black
                    for (int x = 0; x < mmcmPtr->W(); x++)
                    {
                        for (int y = 0; y < mmcmPtr->H(); y++)
                        {
                            for (int z = 0; z < mmcmPtr->L(); z++)
                            {
                                int xOffset = x*singlelW + z*singlelW;
                                int yOffset = y*singleH;
                                for (std::map<std::string, cvz::core::IModality* >::iterator mod = mmcmPtr->modalitiesBottomUp.begin(); mod != mmcmPtr->modalitiesBottomUp.end(); mod++)
                                {
                                    yarp::sig::ImageOf<yarp::sig::PixelRgb> thumbnail = mmcmPtr->getReceptiveFieldRepresentation(x, y, z, mod->second);
                                     
                                    cvSetImageROI(fullImg, cvRect(xOffset, yOffset, thumbnail.width(), thumbnail.height()));
                                    cvCopyImage(thumbnail.getIplImage(), fullImg);
                                    cvResetImageROI(fullImg);

                                    xOffset += thumbnail.width();
                                }
                            }
                        }
                    }
                    //cvReleaseImage(&fullImg);
                    return fullImg;
                }
                
                IplImage* getReceptiveField()
                {
                    if (!isFullMMCM())
                    {
                        std::cerr << "getReceptiveField() only implemented for MMCM type. Aborting..." << std::endl;
                        IplImage* fullImg = cvCreateImage(cvSize(1, 1), 8, 3);
                        return fullImg;
                    }

                     //First we probe the size of one RF. In theory only one bottom up modality
                    IplImage* probe = getReceptiveField(0, 0);
                    int totalW = probe->width * width;
                    int totalH = probe->height * height;
                    IplImage* fullImg = cvCreateImage(cvSize(totalW, totalH), 8, 3);

                    for (size_t x = 0; x < width; x++)
                    {
                        for (size_t y = 0; y < height; y++)
                        {
                            IplImage* thumbnail = getReceptiveField(x, y);
                            cvSetImageROI(fullImg, cvRect(x*probe->width, y*probe->height, probe->width, probe->height));
                            cvCopyImage(thumbnail, fullImg);
                            cvResetImageROI(fullImg);
                            cvReleaseImage(&thumbnail);
                        }
                    }
                    cvReleaseImage(&probe);
                    return fullImg;
                }
            };

    }
}
#endif

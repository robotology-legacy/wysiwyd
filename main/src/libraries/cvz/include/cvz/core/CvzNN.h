#ifndef __CVZ_CVZNN_H__
#define __CVZ_CVZNN_H__

#include <map>
#include <string>
#include <stdlib.h>
#include <vector>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>
#include <float.h>

#include "ICvz.h"
#include "cvzMmcm_IDL.h"

#include "cvz/helpers/helpers.h"
#include <cvz/helpers/NN_Helpers.h>


namespace cvz {
    namespace core {

#define MMCM_CONNECTIVITY_SHEET "sheet"


        class CvzNN : public IConvergenceZone, public cvzMmcm_IDL
        {
            yarp::os::Semaphore mutex;

        public:
            yarp::os::BufferedPort<yarp::os::Bottle>    portFromCoclea;
            yarp::os::BufferedPort<yarp::os::Bottle>    portActivity;

            yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelRgb> >           imagePort;

            std::string              imagePortName;
            std::string         portFromCocleaName;
            std::string         actPortName;

            NeuralModel*         NN;

            std::string         topology;
            int                 size;
            int                 input_size;
            int                 output_size;
            int                 height;
            int                 width;
            int                 layers;
            //double              emax;
            std::vector<double> SDev;
            std::vector<double> Mean;

            std::ofstream       actPath;
            std::ofstream       weiPath;
            yarp::os::Time      t;

            int H() { return height;}
            int W() { return width; }
            int L() { return layers; }
            void sL(int l) { layers = l; }
            void sH(int h) { height = h; }
            void sW(int w) { width = w; }
            

            /*IDL methods*/
            /***************************************************************/
            virtual bool attach(yarp::os::RpcServer &source)
            {
                return this->yarp().attachAsServer(source);
            }

            void start()
            {
                moduleStart();
            }
            void pause()
            {
                modulePause();
            }


            void setLearningRate(const double l) { std::cout << "Learning rate set to : " << l << std::endl; parametersRuntime.put("learningRate", l); }
            double getLearningRate() { return parametersRuntime.find("learningRate").asDouble(); }
            void setEMax(const double emax) { std::cout << "E(%)max set to : " << emax << std::endl; parametersRuntime.put("emax", emax); }
            double getEMax() { return parametersRuntime.find("emax").asDouble(); }
            void setAlpha(const double s) { std::cout << "Alpha set to : " << s << std::endl; parametersRuntime.put("alpha",s); }
            double getAlpha() { return parametersRuntime.find("alpha").asDouble(); }
            double getActivity(const int32_t x) { return (NN->activity)->at(x); }
            double getActivity(const int32_t x, const int32_t y) { return (NN->activity)->at(y*width+x); }
            bool saveWeightsToFile(const std::string &path)
            {
                std::cout << "Trying to save weights to " << path << std::endl;
                return saveWeights(path);
            }
            bool loadWeightsFromFile(const std::string &path)
            {
                std::cout << "Trying to load weights with name: " << path << std::endl;
                yarp::os::ResourceFinder rf;
                std::string fullPath = rf.findFileByName(path);
                std::cout << "Trying to load weights from : " << fullPath << std::endl;
                return loadWeights(fullPath);
            }
            /*I should not need this:
            bool saveRF(const std::string &path)
            {
                bool globalError = true;
                std::cout << "Trying to save the receptive fields to " << path << std::endl;
                for (std::map<cvz::core::IModality*, double>::iterator itMod = modalitiesInfluence.begin(); itMod != modalitiesInfluence.end(); itMod++)
                {
                    IplImage* imgRF = this->getReceptiveFieldRepresentation(itMod->first);
                    std::stringstream fileName;
                    fileName << path << itMod->first->Name() << ".jpg";
                    int errorCode = cvSaveImage(fileName.str().c_str(), imgRF);
                    std::cout << "Saving receptive fields of " << fileName.str() << " --> " << cvErrorStr(errorCode) << std::endl;
                    if (errorCode != 1)
                        globalError = false;
                    cvReleaseImage(&imgRF);
                }
                std::cout << "Done. " << std::endl;
                return globalError;
            }*/

            virtual std::string getType() { return cvz::core::TYPE_NN; };

            yarp::sig::ImageOf<yarp::sig::PixelRgb> getLayerActivity(int i)
            {
                mutex.wait();



                yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
                //yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = imagePort.prepare();

                /*double maxActivity = NN->activity[xWin][yWin][zWin];
                double minActivity = activity[xLoose][yLoose][zLoose];*/
                img.resize(width, size);
                unsigned int k = 0;
                std::cout<<width<<std::endl;
                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        /*
                        double normalisedValue = activity[x][y][i];
                        if (maxActivity - minActivity != 0)
                            normalisedValue = (normalisedValue - minActivity) / (maxActivity - minActivity);*/
                        double normalisedValue = (NN->activity)->at(k);
                        img.pixel(x, y) = helpers::double2RGB( normalisedValue);
                        k++;
                        /*
                        img.pixel(x, y).r = (int)(255*(NN->activity)->at(y));
                        img.pixel(x, y).g = 0;
                        img.pixel(x, y).b = (int)(255*(NN->activity)->at(y));
    */
                        /*
                        //Paint the whole collection of neurons equally activated as the winner
                        if (activity[x][y][i] == maxActivity)
                        {
                            img.pixel(x, y).r = (int)(255*(NN->activity)->at(y));
                            img.pixel(x, y).g = 0;
                            img.pixel(x, y).b = 255;
                        }
                        if (x == xWin && y == yWin && i == zWin)
                        {
                            img.pixel(x, y).r = 0;
                            img.pixel(x, y).g = 0;
                            img.pixel(x, y).b = 0;
                        }*/
                        //IplImage* iplimg = img.getIplImage();

                        yarp::sig::ImageOf<yarp::sig::PixelRgb> &temp = imagePort.prepare();
                        temp.resize(640,480);
                        cvResize((IplImage*) img.getIplImage(),(IplImage*) temp.getIplImage());
                        imagePort.write();
                    }
                }
                mutex.post();

                //Console.WriteLine("Visualization computed in " + (time2 - time1).ToString());
                return img;
            }



            virtual bool close()
            {
                bool ok = this->IConvergenceZone::close();
                actPath.close();
                weiPath.close();
                portActivity.close();
                portFromCoclea.close();
                return ok;
            }

            virtual bool configure(yarp::os::Property &rf)
            {
                std::cout<<121212<<std::endl;
                //Call the base class configure
                this->IConvergenceZone::configure(rf);
                //Get additional parameters
                if (!parametersStartTime.check("size"))
                    parametersStartTime.put("size", yarp::os::Value(100));
                //if (!parametersStartTime.check("emax"))
                //    parametersStartTime.put("emax", yarp::os::Value(0.95));
                if (!parametersStartTime.check("width"))
                    parametersStartTime.put("width", yarp::os::Value(10));
                if (!parametersStartTime.check("height"))
                    parametersStartTime.put("height", yarp::os::Value(10));
                if (!parametersStartTime.check("inputSize"))
                    parametersStartTime.put("inputSize", yarp::os::Value(2048));
                if (!parametersStartTime.check("layers"))
                    parametersStartTime.put("layers", yarp::os::Value(1));
                if (!parametersStartTime.check("topology"))
                    parametersStartTime.put("topology", yarp::os::Value(MMCM_CONNECTIVITY_SHEET));

                //Starttime parameters
                size = parametersStartTime.find("size").asInt();
                //emax = parametersStartTime.find("emax").asDouble();
                height = parametersStartTime.find("height").asInt();
                width = parametersStartTime.find("width").asInt();
                input_size = parametersStartTime.find("inputSize").asInt();
                output_size = parametersStartTime.find("outputSize").asInt();
                layers = parametersStartTime.find("layers").asInt();
                topology = parametersStartTime.find("topology").asString();


                //Runtime parameters
                if (!parametersRuntime.check("learningRate"))
                    parametersRuntime.put("learningRate", yarp::os::Value(0.01));
                if (!parametersRuntime.check("alpha"))
                    parametersRuntime.put("alpha", yarp::os::Value(1/3600));
                if (!parametersRuntime.check("emax"))
                    parametersRuntime.put("emax", yarp::os::Value(0.95));


                //Load previous normalization parameters
                yarp::os::Bottle bGroup = rf.findGroup("Auditory");

                if (bGroup.check("std"))
                {
                    yarp::os::Bottle* sMask = bGroup.find("std").asList();
                    for (int i = 0; i < sMask->size(); i++)
                        SDev.push_back(sMask->get(i).asDouble());
                }

                if (bGroup.check("mean"))
                {
                    yarp::os::Bottle* mMask = bGroup.find("mean").asList();
                    for (int i = 0; i < mMask->size(); i++)
                        Mean.push_back(mMask->get(i).asDouble());
                }

                bool    bEveryThingisGood = true;


                //Auditory input
                
                std::string moduleInput = rf.check("input", yarp::os::Value("/audioPreprocessing/freqSpectrum:o")).asString().c_str();
                
                //configure input port
                portFromCocleaName = "/";
                portFromCocleaName += getName() + "/coclea:i";

                if (!portFromCoclea.open(portFromCocleaName.c_str()))
                {
                    std::cout << getName() << ": Unable to open port " << portFromCocleaName << std::endl;
                    bEveryThingisGood &= false;
                }
                while (!yarp::os::Network::connect(moduleInput, portFromCocleaName.c_str()))
                {
                    std::cout << "Trying to get input from FFT..." << std::endl;
                    yarp::os::Time::delay(1.0);
                }


                //Zero everything
                actPortName = "/";
                actPortName += getName();
                actPortName += "/activity:o";
                portActivity.open(actPortName);

                //imagePort
                imagePortName = "/";
                imagePortName += getName() + "/image:o";
                if(!imagePort.open(imagePortName.c_str()))
                {
                    std::cout << getName() << ": Unable to open port " << imagePortName << std::endl;
                    bEveryThingisGood &= false;
                }


                //Configure network
                NN = new NeuralModel(size, &Mean, &SDev, getAlpha(), getEMax(), getLearningRate(),input_size,output_size);

                //Save files
                actPath.open("activity.csv");
                weiPath.open("weights.csv");

                return true;
            }

            virtual void performPeriodicAction(const int &cyclesElapsed)
            {
                this->IConvergenceZone::performPeriodicAction(cyclesElapsed);
                if (cyclesElapsed % 10 == 0)
                {
                    //std::cout << "Do something on 500 step" << std::endl;
                    std::cout << cyclesElapsed << std::endl;

                    printActivity(actPath,cyclesElapsed);
                    printWeights(weiPath,cyclesElapsed);
                }
            }

            bool printActivity(std::ofstream &outfile, int cycles)
                {
                    outfile<< cycles <<";"<<t.now()<<";";
                    for (unsigned int n = 0; n<NN->activity->size();n++)
                    {
                        if(n!=0){outfile<<";";}
                        outfile << NN->activity->at(n);
                    }
                    outfile << std::endl;
                    return true;
                }

            bool printWeights(std::ofstream &outfile, int cycles)
                {
                    outfile<< cycles <<";"<<t.now()<<";";
                    for (unsigned int j = 0; j<NN->connections->size();j++)
                    {
                        for (unsigned int i = 0; i<NN->connections->size();i++)
                        {
                            outfile << (NN->connections->at(j))->at(i);
                            if(i==NN->connections->size()-1){if(j!=NN->connections->size()-1){outfile<<";";}}else{outfile<<",";}
                        }
                    }
                    outfile << std::endl;
                    return true;
                }

            virtual void ComputePrediction()
            {
                mutex.wait();

                yarp::os::Bottle* input_bottle = portFromCoclea.read();
                yarp::os::Bottle &botActivity = portActivity.prepare();
                botActivity.clear();

                std::vector<double> input_vector;
                if (input_bottle->size() != input_size)
                {
                    std::cout << "input different than "<< input_size <<"!!" << std::endl;
                    std::cout << "input size" << input_bottle->size() << std::endl;
                    std::cout << "content of bottle" << input_bottle->toString() << std::endl;
                    std::cout << "test" << input_bottle->get(1).asDouble() << std::endl;

                }
                input_vector.resize(input_bottle->size());
                for (int i = 0; i < input_bottle->size(); i++)
                {
                    input_vector[i] = input_bottle->get(i).asDouble();
                }
                NN->updateInput(input_vector);
                NN->processActivity();
                NN->updateActivity();
                //NN->normalizeAndSelect();
                NN->normalizeAndSelect("max");
                for (int i=0;i<size;i++)
                {botActivity.addDouble((NN->activity)->at(i+input_size));}

                portActivity.write();
                //update Weights from LR!!!
                NN->updateWeights();
                //getLayerActivity(0);
                NN->resetLoop();


                //------------------------------------------------------------------------------------------------------------------------
                //Set the predicted values 

                ////feedback
                //for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                //    predictModality(it->second, algorithm);
                ////feedforward
                //for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                //    predictModality(it->second, algorithm);

                // I ignore preditction because my system is for unsupervised learning. I have nothing to predict. I only map concurrent activity.


                mutex.post();
            }

            bool saveWeights(std::string path)
            {
                std::ofstream file;
                file.open(path.c_str());
                file << "width" << '\t' << width << std::endl;
                file << "height" << '\t' << height << std::endl;
                file << "layers" << '\t' << layers << std::endl;

                file << std::endl;

                int modCount = 0;
                //for (std::map<IModality*, std::vector<helpers::Cube> >::iterator wModIt = weights.begin(); wModIt != weights.end(); wModIt++)
                //{
                file << "[modality_" << modCount << "]" << std::endl;
                //file << "name" << '\t' << wModIt->first->Name() << std::endl;
                //file << "size" << '\t' << wModIt->first->Size() << std::endl;

                //Hauria de guardar les dades de la modality
                //file << wModIt->first->getModalityConfiguration();

                yarp::os::Bottle b;

                for (unsigned int x = 0; x < NN->size; x++)
                {
                    for (unsigned int y = 0; y < NN->size; y++)
                    {
                            b.addDouble((NN->connections->at(y))->at(x));
                    }
                }

                file << "weights (" << '\t' << b.toString() << ")" << std::endl;
                file << std::endl;

                file.close();
                return true;
            }


            bool loadWeights(std::string path)
            {
                yarp::os::Property file; file.fromConfigFile(path.c_str());
                int tmpW = file.find("width").asInt();
                int tmpH = file.find("height").asInt();
                int tmpL = file.find("layers").asInt();

                if (tmpW != width || tmpH != height || tmpL != layers)
                {
                    std::cerr << "Error while loading the weights. The map size is not matching." << std::endl;
                    return false;
                }

                //int modCount = 0;
                yarp::os::Bottle bGroup = file.findGroup("modality_0");
                while (!bGroup.isNull())
                {
                    //Check that modality has loaded from right file!!


                    yarp::os::Bottle *bWeights = bGroup.find("weights").asList();
                    NN->setConnections(bWeights);
                    /*
                    int wCtr = 0;
                    for (unsigned int x = 0; x < size; x++)
                    {
                        for (unsigned int y = 0; y < size; y++)
                        {
                            connections->at(y)->at(x) = bWeights->get(wCtr).asDouble();
                            wCtr++;
                        }
                    }*/

                }
                return true;
            }



        };
    }
}
#endif

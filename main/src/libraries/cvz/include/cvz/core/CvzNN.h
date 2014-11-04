#ifndef __CVZ_CVZNN_H__
#define __CVZ_CVZNN_H__

#include <map>
#include <string>
#include <stdlib.h>
#include <vector>
#include <yarp/os/all.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>
#include <float.h>

#include "ICvz.h"
#include "cvz/helpers/helpers.h"
#include <cvz/helpers/NN_Helpers.h>

namespace cvz {
    namespace core {

        class CvzNN : public IConvergenceZone
        {
            yarp::os::Semaphore mutex;

        public:
            yarp::os::BufferedPort<yarp::os::Bottle> portFromCoclea;
            yarp::os::BufferedPort<yarp::os::Bottle> portActivity;
            std::string         portFromCocleaName;
            std::string         actPortName;

            NeuralModel*         NN;

            std::string         topology;
            int                 size;
            int                 input_size;
            int                 layers;
            std::vector<double> SDev;
            std::vector<double> Mean;

        


            virtual std::string getType() { return cvz::core::TYPE_NN; };


            virtual bool close()
            {
                bool ok = this->IConvergenceZone::close();
                portActivity.close();
                portFromCoclea.close();
                return ok;
            }

            virtual bool configure(yarp::os::Property &rf)
            {
                //Call the base class configure
                this->IConvergenceZone::configure(rf);

                //Get additional parameters
                if (!parametersStartTime.check("size"))
                    parametersStartTime.put("size", yarp::os::Value(50));
                if (!parametersStartTime.check("inputSize"))
                    parametersStartTime.put("inputSize", yarp::os::Value(2048));
                if (!parametersStartTime.check("layers"))
                    parametersStartTime.put("layers", yarp::os::Value(1));
                if (!parametersStartTime.check("topology"))
                    parametersStartTime.put("topology", yarp::os::Value(MMCM_CONNECTIVITY_SHEET));
                
                //Starttime parameters
                size = parametersStartTime.find("size").asInt();
                input_size = parametersStartTime.find("inputSize").asInt();
                layers = parametersStartTime.find("layers").asInt();
                topology = parametersStartTime.find("topology").asString();

                //Runtime parameters
                if (!parametersRuntime.check("learningRate"))
                    parametersRuntime.put("learningRate", yarp::os::Value(0.05));
                if (!parametersRuntime.check("alpha"))
                    parametersRuntime.put("alpha", yarp::os::Value(3600));
                if (!parametersRuntime.check("SDev"))
                    parametersRuntime.put("SDev", yarp::os::Value(2.0));


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

                //Configure network
                NN = new NeuralModel(50, Mean, SDev);


                return true;
            }

            virtual void performPeriodicAction(const int &cyclesElapsed)
            {
                this->IConvergenceZone::performPeriodicAction(cyclesElapsed);
                if (cyclesElapsed % 500 == 0)
                {
                    std::cout << "Do something on 500 step" << std::endl;
                }
            }

            virtual void ComputePrediction()
            {
                mutex.wait();

                yarp::os::Bottle* input_bottle = portFromCoclea.read();
                yarp::os::Bottle &botActivity = portActivity.prepare();
                botActivity.clear();

                std::vector<double> input_vector;
                if (input_bottle->size() != 2048)
                {
                    std::cout << "input different than 2048!!" << std::endl;
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
                NN->normalizeAndSelect(botActivity);
                std::cout << NN->firing << std::endl;
                portActivity.write();
                NN->updateWeights();
                NN->resetLoop();


                //------------------------------------------------------------------------------------------------------------------------
                //Set the predicted values 

                ////feedback
                //for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                //    predictModality(it->second, algorithm);
                ////feedforward
                //for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                //    predictModality(it->second, algorithm);

                mutex.post();
            }
        };
    }
}
#endif

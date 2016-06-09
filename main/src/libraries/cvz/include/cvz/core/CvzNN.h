#ifndef __CVZ_CVZNN_H__
#define __CVZ_CVZNN_H__

#include <map>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>
#include <float.h>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "ICvz.h"
#include "cvzMmcm_IDL.h"

#include "cvz/helpers/helpers.h"
#include <cvz/helpers/NN_Helpers.h>

#ifdef _WIN32
    #include <Windows.h>

    std::string get_selfpath() {
        char buffer[MAX_PATH];//always use MAX_PATH for filepaths
        int len = GetModuleFileName(NULL, buffer, sizeof(buffer));
        if (len != -1) {
            buffer[len] = '\0';
            return std::string(buffer);
        }
        else {
            /* handle error condition */
            std::cout << "ERROR: PATH NOT OBTAINED" << std::endl;
            return "./";
        }
    }
    const std::string SELFPATH = (get_selfpath()).substr(0, (get_selfpath()).find("\\Release\\cvzCore.exe")) + "/../share/wysiwyd/contexts/cvz/";
   
#else
    #include <unistd.h>
    //retrieves executable path for saving files
    std::string get_selfpath() {
        char buff[1024];
        ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff)-1);
        if (len != -1) {
            buff[len] = '\0';
            return std::string(buff);
        }
        else {
            /* handle error condition */
            std::cout << "ERROR: PATH NOT OBTAINED" << std::endl;
            return "./";
        }
    }
    const std::string SELFPATH = (get_selfpath()).substr(0, (get_selfpath()).find("/cvzCore")) + "/../share/wysiwyd/contexts/cvz/";

#endif


static const char *DEFAULT_CONFIG_FILE = "normals_NN.ini";

namespace cvz {
    namespace core {

#define MMCM_CONNECTIVITY_SHEET "sheet"


        class CvzNN : public IConvergenceZone, public cvzMmcm_IDL
        {
            yarp::os::Semaphore mutex;

        public:
            //yarp::os::BufferedPort<yarp::os::Bottle>    portFromCoclea;
            yarp::os::BufferedPort<yarp::os::Bottle>    portActivity;

            //yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelRgb> >           imagePort;

            yarp::os::Port toSoundGenerator; // port to sound generator, if testing

            yarp::os::Bottle cmd;  //bottle to contain the command for the soundGenerator

            std::string         imagePortName;
            std::string         portFromCocleaName;
            std::string         actPortName;

            NeuralModel*        NN;

            std::string         topology;
            int                 size;
            unsigned int        input_size;
            int                 output_size;
            int                 height;
            int                 width;
            int                 layers;

            std::vector<double> SDev;
            std::vector<double> Mean;

            std::ofstream       actPath;
            std::ofstream       weiPath;
            yarp::os::Time      t;
            std::string         selfpath;
            std::string         conf_file;
            int                 maxCycles;
            int                 testing_frequency;
            bool                testing;
            int                 experiment_number;
            //std::string         activity_save_file;

            yarp::os::Property  rf_save;


            int H() { return height;}
            int W() { return width; }
            int L() { return layers; }

            //Those are setup at start time. Are you sure you want to be able to modify them during simulation ?
            //void sL(int l) { layers = l; }
            //void sH(int h) { height = h; }
            //void sW(int w) { width = w; }
            

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
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif
            double getActivity(const int &x) { return (NN->activity)->at(x); }
            double getActivity(const int &x, const int &y) { return (NN->activity)->at(y*width+x); }
#if __clang__
#pragma clang diagnostic pop
#endif
            void setMaxCycles(const int mc) { std::cout << "Max Cycles set to : " << mc << std::endl; parametersRuntime.put("maxCycles", mc); }
            int getMaxCycles() { return parametersRuntime.find("maxCycles").asInt(); }
            void setNumberExperiments(const int mc) { std::cout << "Max Cycles set to : " << mc << std::endl; parametersRuntime.put("nExp", mc); }
            int getNumberExperiments() { return parametersRuntime.find("nExp").asInt(); }
            void setExperimentCount(const int mc) { std::cout << "Max Cycles set to : " << mc << std::endl; parametersRuntime.put("expCount", mc); }
            int getExperimentCount() { return parametersRuntime.find("expCount").asInt(); }

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

            virtual std::string getType() { return cvz::core::TYPE_NN; };

            yarp::sig::ImageOf<yarp::sig::PixelRgb> getNetworkActivity()
            {
                mutex.wait();
                yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
                //yarp::sig::ImageOf<yarp::sig::PixelRgb> &img = imagePort.prepare();

                /*double maxActivity = NN->activity[xWin][yWin][zWin];
                double minActivity = activity[xLoose][yLoose][zLoose];*/
                img.resize(width, height);

                if (width*height != size)
                {
                    std::cerr << "getNetworkActivity: incongruency between w,h and size" << std::endl;
                }
                //unsigned int k = 0;
                std::cout << "width: "<<width<<std::endl;
                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        double normalisedValue = getActivity(x, y);
                        img.pixel(x, y) = helpers::double2RGB( normalisedValue);
                    }
                }

                mutex.post();

                //Console.WriteLine("Visualization computed in " + (time2 - time1).ToString());
                return img;
            }


            

            // Saves in a file the mean and standard deviation for posterior input normalization
            void saveNormals()
            {
                std::cout << "Saving means and STD of inputs..." << std::endl;
                std::string inName = SELFPATH + conf_file;
                std::ofstream tmpfile;
                std::ifstream infile;
                infile.open(inName.c_str());
                tmpfile.open("ini.tmp");

                std::cout << inName << std::endl;

                std::string line;
                std::string match[2];
                match[0].assign("mean");
                match[1].assign("std");
                bool W = false;

                std::cout.precision(8);

                std::cout << "saving normals..." << std::endl;
                while (std::getline(infile, line))
                {
                    
                    //string aux;
                    //std::istringstream iss(line);
                    //cout << Mean[0]<<endl;
                    if (line.compare(match[0].c_str())>4)
                    {
                        tmpfile << "mean\t\t(" << Mean[0];
                        for (unsigned int i = 1; i<Mean.size(); i++)
                        {
                            tmpfile << " " << Mean[i];
                        }
                        tmpfile << ")" << std::endl;

                    }
                    else if (line.compare(match[1].c_str())>3)
                    {
                        tmpfile << "std\t\t(" << SDev[0];
                        for (unsigned int j = 1; j<SDev.size(); j++)
                        {
                            tmpfile << " " << SDev[j];
                        }
                        tmpfile << ")" << std::endl;
                        W = 1;
                    }
                    else{
                        tmpfile << line << std::endl;
                    }
                }

                if (!W)
                {
                    tmpfile << "[Auditory]" << std::endl;

                    tmpfile << "mean\t\t" << "(" << Mean[0];
                    for (unsigned int i = 1; i<Mean.size(); i++)
                    {

                        tmpfile << " " << Mean[i];
                    }
                    tmpfile << ")" << std::endl;

                    tmpfile << "std\t\t(" << SDev[0];
                    for (unsigned int j = 1; j<SDev.size(); j++)
                    {
                        tmpfile << " " << SDev[j];
                    }
                    tmpfile << ")" << std::endl;
                }

                //close files
                tmpfile.close();
                infile.close();

                //Save changes and keep old version
                std::string old = inName + "_old";
                remove(old.c_str());
                rename(inName.c_str(), old.c_str());
                remove(inName.c_str());
                rename("ini.tmp", inName.c_str());

            }


            virtual bool close()
            {
                std::cout << "closing..." << std::endl;
                saveNormals();
                bool ok = this->IConvergenceZone::close();
                //actPath.close();
                //weiPath.close();
                portActivity.close();
                toSoundGenerator.close();
                //portFromCoclea.close();
                return ok;
            }

            
            virtual bool reset()
            {
                saveNormals();
                bool ok = this->IConvergenceZone::close();
                //actPath.close();
                //renameExperiment();
                //weiPath.close();
                portActivity.close();
                toSoundGenerator.close();
                //portFromCoclea.close();
                if (getExperimentCount() < getNumberExperiments())
                {
                    int old_e_n = getExperimentCount();
                    int oldmaxXycles = maxCycles;
                    configure(rf_save);
                    setExperimentCount(old_e_n + 1);
                    setMaxCycles(oldmaxXycles);
                }
                else if (maxCycles < 10000)
                {
                    configure(rf_save);
                    setExperimentCount(1);
                    maxCycles = 10000;
                    setMaxCycles(maxCycles);
                }
                else
                {
                    close();
                }

                return ok;
            }
            
            bool renameExperiment()
            {
                /*std::string prev = activity_save_file;
                activity_save_file += std::to_string((long double)getExperimentCount());
                activity_save_file += "_" + std::to_string((long double)getMaxCycles());
                activity_save_file += ".csv";
                std::cout << "save file for activity is: " << activity_save_file << std::endl;
                //actPath.open(prev);
                rename(prev.c_str(), activity_save_file.c_str());
                yarp::os::Time::delay(3);
                remove(prev.c_str());
                */
                return true;
            }

            virtual bool configure(yarp::os::Property &rf)
            {

                

                //Call the base class configure
                this->IConvergenceZone::configure(rf);

                rf_save = rf;
                experiment_number = 0;

                testing_frequency = 0;


                conf_file = rf.check("from", yarp::os::Value(DEFAULT_CONFIG_FILE)).asString().c_str();

                testing = rf.check("test");

                if (testing)
                {
                    std::cout << "True" << std::endl;
                }
                else{ std::cout << "false" << std::endl; }

                //Get additional parameters
                if (!parametersStartTime.check("size"))
                    parametersStartTime.put("size", yarp::os::Value(100));
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
                height = parametersStartTime.find("height").asInt();
                width = parametersStartTime.find("width").asInt();
                input_size = parametersStartTime.find("inputSize").asInt();
                output_size = parametersStartTime.find("outputSize").asInt(); //careful you do not have a default value for this
                layers = parametersStartTime.find("layers").asInt();
                topology = parametersStartTime.find("topology").asString();


                //Runtime parameters
                if (!parametersRuntime.check("learningRate"))
                    parametersRuntime.put("learningRate", yarp::os::Value(0.01));
                if (!parametersRuntime.check("alpha"))
                    parametersRuntime.put("alpha", yarp::os::Value(1 / 3600));
                if (!parametersRuntime.check("emax"))
                    parametersRuntime.put("emax", yarp::os::Value(0.95));
                
                //Testing things
                if (!parametersRuntime.check("maxCycles"))
                    parametersRuntime.put("maxCycles", yarp::os::Value(0));
                
                if (!parametersRuntime.check("nExp"))
                    parametersRuntime.put("nExp", yarp::os::Value(15));
                if (!parametersRuntime.check("expCount"))
                    parametersRuntime.put("expCount", yarp::os::Value(1));
                maxCycles = getMaxCycles();

                //I think you forgot to upload the configuration file you are using in app, the auditory group does not contains std or mean
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

                //Open activity port
                actPortName = "/";
                actPortName += getName();
                actPortName += "/activity:o";
                portActivity.open(actPortName);

                //Open imagePort
                /*imagePortName = "/";
                imagePortName += getName() + "/image:o";
                if (!imagePort.open(imagePortName.c_str()))
                {
                    std::cout << getName() << ": Unable to open port " << imagePortName << std::endl;
                    bEveryThingisGood &= false;
                }*/

                if (true){
                std::string portName = "/";
                portName += getName();
                portName += "/frequency/rpc:o";
                toSoundGenerator.open(portName.c_str());
                connect();
                }

                //dataDumpers!
                std::string targetPort = rf.check("dumps", yarp::os::Value("/dump/noname")).asString().c_str();
                yarp::os::Network::connect(actPortName, targetPort);
                
                //Configure network
                NN = new NeuralModel(size, &Mean, &SDev, getAlpha(), getEMax(), getLearningRate(),input_size,output_size);

                /*
                //Save files
                activity_save_file = "activity_";
                std::cout << "save file for activity is: " << activity_save_file << std::endl;
                
                actPath.open(activity_save_file);
                weiPath.open("weights.csv");
                */

                // Read exe path
                selfpath = get_selfpath();
                std::cout << selfpath << std::endl;


                return true;
            }


            bool connect(const std::string &targetPort = "/tuneBabbler/rpc")
            {
                return yarp::os::Network::connect(toSoundGenerator.getName(), targetPort.c_str());
            }

            virtual void performPeriodicAction(const int &cyclesElapsed)
            {
                this->IConvergenceZone::performPeriodicAction(cyclesElapsed);
                if (cyclesElapsed % 10 == 0)
                {
                    
                    std::cout << cyclesElapsed << std::endl;

                    //printActivity(actPath,cyclesElapsed);
                    //printWeights(weiPath,cyclesElapsed);
                }
                if (true)
                {
                    if (cyclesElapsed % 50 == 0)
                    {
                        changeFrequency();
                    }
                    if ((cyclesElapsed % 50)-25 == 0)
                    {
                        setFrequency(0);
                    }
                }

                if (maxCycles > 0)
                {

                    if (cyclesElapsed % (maxCycles+1) == maxCycles)
                    {
                        std::cout << maxCycles << " cycles elapsed. Exiting..." << std::endl;
                        yarp::os::Time::delay(1.0);
                        if (false)
                        {
                            reset();
                        }
                        else
                        {
                            close();
                        }
                    }
                }
            }

            void changeFrequency()
            {
                cmd.clear();
                cmd.addString("RF");
                testing_frequency = toSoundGenerator.write(cmd);
            }
            void setFrequency(int F=0)
            {
                cmd.clear();
                cmd.addString("F");
                cmd.addInt(F);
                testing_frequency = F;
                toSoundGenerator.write(cmd);
            }

            bool printActivity(std::ofstream &outfile, int cycles)
                {
                    outfile<< cycles <<";"<<t.now()<<";";
                    if (testing)
                    {
                        outfile << testing_frequency << ";";
                    }
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
                    if (testing)
                    {
                        outfile << testing_frequency << ";";
                    }
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

                //yarp::os::Bottle* input_bottle = portFromCoclea.read();

                //Here you are assuming that you network only has one input modality...
                std::vector<double> input_vector = modalitiesBottomUp["coclea"]->GetValueReal();
                if (input_vector.size() != input_size)
                {
                    std::cout << "input different than " << input_size << "!!" << std::endl;
                    std::cout << "input size for modality " << modalitiesBottomUp["coclea"]->Name() << std::endl;
                    std::cout << "input size" << modalitiesBottomUp["coclea"]->Size() << std::endl;
                    //std::cout << "content of bottle" << input_bottle->toString() << std::endl;
                    //std::cout << "test" << input_bottle->get(1).asDouble() << std::endl;

                }

                NN->updateInput(input_vector);
                NN->processActivity();
                NN->updateActivity();
                //NN->normalizeAndSelect();
                NN->normalizeAndSelect("max");

                yarp::os::Bottle &botActivity = portActivity.prepare();
                botActivity.clear();
                botActivity.addInt(testing_frequency);
                botActivity.addInt(0);

                for (int i=0;i<size;i++)

                {
                    botActivity.addDouble((NN->activity)->at(i+input_size));
                }

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

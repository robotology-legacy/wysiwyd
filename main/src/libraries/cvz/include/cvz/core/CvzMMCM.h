#ifndef __CVZ_CVZMMCM_H__
#define __CVZ_CVZMMCM_H__

#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>
#include <float.h>

#include "ICvz.h"
#include "cvzMmcm_IDL.h"
#include "cvz/helpers/helpers.h"

namespace cvz {
    namespace core {
        
#define MMCM_CONNECTIVITY_SHEET "sheet"
#define MMCM_CONNECTIVITY_TORUS "torus"

#define MMCM_ALGORITHM_SOM          "som"
#define MMCM_ALGORITHM_DSOM         "dsom"
#define MMCM_ALGORITHM_POPULATION   "population"

        class CvzMMCM : public IConvergenceZone, public cvzMmcm_IDL
        {
            int height, width, layers;
            int xWin, yWin, zWin, xLoose, yLoose, zLoose;
            helpers::Cube activity;
            std::map<IModality*, std::vector< helpers::Cube > > weights;
            yarp::os::BufferedPort<yarp::os::Bottle> portActivity;
            int xBuff, yBuff, zBuff;
            yarp::os::Semaphore mutex;

            std::string connectivityPatern;
            std::string algorithm;

        public:

            virtual std::string getType() { return cvz::core::TYPE_MMCM; };
            int H() { return height; }
            int W() { return width; }
            int L() { return layers; }
            std::string ConnectivityPattern() { return connectivityPatern; }
            std::string Algorithm() { return algorithm; }

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
            void setSigma(const double s) { std::cout << "Sigma set to : " << s << std::endl; parametersRuntime.put("sigma",s); }
            double getSigma() { return parametersRuntime.find("sigma").asDouble(); }
            double getActivity(const int32_t x, const int32_t y, const int32_t z) { return activity[x][y][z]; }
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
            }
            //virtual yarp::os::Bottle getParametersForBroadcast()
            //{
            //    yarp::os::Bottle b = this->IConvergenceZone::getParametersForBroadcast();
            //    yarp::os::Bottle &bLearning = b.addList();
            //    bLearning.addString("learningRate");
            //    bLearning.addDouble(getLearningRate());
            //    return b;
            //}

            virtual bool close()
            {
                bool ok = this->IConvergenceZone::close();

                portActivity.interrupt();
                portActivity.close();
                return ok;
            }

            virtual bool configure(yarp::os::Property &rf)
            {
                //Call the base class configure
                this->IConvergenceZone::configure(rf);

                //Get additional parameters
                if (!parametersStartTime.check("height"))
                    parametersStartTime.put("height", yarp::os::Value(10));
                if (!parametersStartTime.check("width"))
                    parametersStartTime.put("width", yarp::os::Value(10));
                if (!parametersStartTime.check("layers"))
                    parametersStartTime.put("layers", yarp::os::Value(1));
                if (!parametersStartTime.check("connectivityPattern"))
                    parametersStartTime.put("connectivityPattern", yarp::os::Value(MMCM_CONNECTIVITY_SHEET));
                if (!parametersStartTime.check("algorithm"))
                    parametersStartTime.put("algorithm", yarp::os::Value(MMCM_ALGORITHM_DSOM));

                //Starttime parameters
                height = parametersStartTime.find("height").asInt();
                width = parametersStartTime.find("width").asInt();
                layers = parametersStartTime.find("layers").asInt();
                connectivityPatern = parametersStartTime.find("connectivityPattern").asString();
                algorithm = parametersStartTime.find("algorithm").asString();

                //Runtime parameters
                if (!parametersRuntime.check("learningRate"))
                    parametersRuntime.put("learningRate", yarp::os::Value(0.05));
                if (!parametersRuntime.check("sigma"))
                    parametersRuntime.put("sigma", yarp::os::Value((1.0 / 4.0) * (height + width) / 2.0));
                if (!parametersRuntime.check("elasticity"))
                    parametersRuntime.put("elasticity", yarp::os::Value(2.0));

                //Allocate the map
                activity.allocate(width, height, layers);

                //Allocate the weights
                for (std::map<IModality*, double>::iterator it = modalitiesInfluence.begin(); it != modalitiesInfluence.end(); it++)
                {
                    std::vector< helpers::Cube > w;
                    w.resize(it->first->Size());
                    for (int i = 0; i < it->first->Size(); i++)
                    {
                        w[i].allocate(width, height, layers);
                        w[i].randomize(0.0, 1.0);
                    }
                    weights[it->first] = w;
                }

                //Zero everything
                activity = 0.0;
                xBuff = yBuff = zBuff = xWin = yWin = zWin = xLoose = yLoose = zLoose = 0;
                std::string actPortName = "/";
                actPortName += getName();
                actPortName += "/activity:o";
                portActivity.open(actPortName);

                //Good to go!
                std::cout << std::endl << "Multi Modal Convergence Map configured:" << std::endl
                    << "\t Width :  " << width << std::endl
                    << "\t Height : " << height << std::endl
                    << "\t Layers : " << layers << std::endl
                    << "\t Algorithm : " << algorithm << std::endl
                    << "\t Connectivity : " << connectivityPatern << std::endl;

                return true;
            }

            virtual void performPeriodicAction(const int &cyclesElapsed)
            {
                this->IConvergenceZone::performPeriodicAction(cyclesElapsed);
                if (cyclesElapsed % 500 == 0)
                {
                    std::stringstream spath;
                    spath << this->getName() << "_" << cyclesElapsed << "_";
                    saveRF(spath.str().c_str());
                }
            }

            virtual void ComputePrediction()
            {
                mutex.wait();
                //Reset the cube activity
                activity = 0.0;

                xWin = yWin = zWin = xLoose = yLoose = zLoose = 0;

                double influenceTotal = 0;
                for (std::map< IModality*, double>::iterator it = modalitiesInfluence.begin(); it != modalitiesInfluence.end(); it++)
                    influenceTotal += it->second;

                yarp::os::Bottle &botActivity = portActivity.prepare();
                botActivity.clear();

                //------------------------------------------------------------------------------------------------------------------------
                //Compute map activity 
                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        for (int z = 0; z < layers; z++)
                        {
                            //from bottom up input
                            for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                            {
                                std::vector<double> valueReal = it->second->GetValueReal();
                                double modalityMeanError = 0.0;
                                for (int i = 0; i < it->second->Size(); i++)
                                {
                                    modalityMeanError += fabs(weights[it->second][i][x][y][z] - valueReal[i]);
                                }
                                modalityMeanError /= it->second->Size();
                                activity[x][y][z] += 1.0 - (modalityMeanError * modalitiesInfluence[it->second]);
                            }

                            //from top down feedback
                            for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                            {
                                std::vector<double> valueReal = it->second->GetValueReal();
                                double modalityMeanError = 0.0;
                                for (int i = 0; i < it->second->Size(); i++)
                                {
                                    modalityMeanError += fabs(weights[it->second][i][x][y][z] - valueReal[i]);
                                }
                                modalityMeanError /= it->second->Size();
                                activity[x][y][z] += 1.0 - (modalityMeanError * modalitiesInfluence[it->second]);
                            }

                            //Get the whole activity in [0,1]
                            activity[x][y][z] /= influenceTotal;// modalitiesBottomUp.size();

                            botActivity.addDouble(activity[x][y][z]);

                            //keep track of winner neuron & error
                            if (activity[x][y][z] > activity[xWin][yWin][zWin])
                            {
                                xWin = x;
                                yWin = y;
                                zWin = z;
                            }
                            if (activity[x][y][z] < activity[xLoose][yLoose][zLoose])
                            {
                                xLoose = x;
                                yLoose = y;
                                zLoose = z;
                            }
                        }
                    }
                }


                //Do an E%MAX
                if (true /*algorithm == MMCM_ALGORITHM_POPULATION*/)
                {
                    for (int x = 0; x < width; x++)
                    {
                        for (int y = 0; y < height; y++)
                        {
                            for (int z = 0; z < layers; z++)
                            {
                                if (fabs(activity[xWin][yWin][zWin] - activity[x][y][z]) > fabs(0.05*activity[xWin][yWin][zWin]))
                                {
                                    activity[x][y][z] = 0.0;
                                    xLoose = x;
                                    yLoose = y;
                                    zLoose = z;
                                }
                            }
                        }
                    }
                }

                //------------------------------------------------------------------------------------------------------------------------
                //Send the output activity
                portActivity.write();

                //------------------------------------------------------------------------------------------------------------------------
                //Learning
                //check if all modalities learning is off
                bool allModLearningZero = true;
                for (std::map<IModality*, double>::iterator itL = modalitiesLearning.begin(); itL != modalitiesLearning.end(); itL++)
                    allModLearningZero &= (itL->second == 0.0);
                if (parametersRuntime.find("learningRate").asDouble() > 0.0 && !allModLearningZero)
                    adaptWeights();


                //------------------------------------------------------------------------------------------------------------------------
                //Set the predicted values 

                //feedback
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                    predictModality(it->second, algorithm);
                //feedforward
                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                    predictModality(it->second, algorithm);

                mutex.post();
            }

            void predictModality(IModality* mod, std::string algorithmUsed)
            {
                //SOFT MAX
                /*
                double totalActivity = 0.0;
                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        for (int z = 0; z < layers; z++)
                        {
                            totalActivity += exp(activity[x][y][z]);
                        }
                    }
                }
                */

                std::vector<double> valuePrediction;
                valuePrediction.resize(mod->Size(), 0.0);

                std::vector<double> valuePredictionWinner;
                valuePredictionWinner.resize(mod->Size(), 0.0);
                for (int i = 0; i < mod->Size(); i++)
                {
                    valuePredictionWinner[i] = weights[mod][i][xWin][yWin][zWin];

                    int contributingNeurons = 0;
                    if (true /*algorithmUsed == MMCM_ALGORITHM_POPULATION*/)
                    {
                        for (int x = 0; x < width; x++)
                        {
                            for (int y = 0; y < height; y++)
                            {
                                for (int z = 0; z < layers; z++)
                                {  
                                    //double contribution = activity[x][y][z];//SOFTMAX exp(activity[x][y][z]) / totalActivity;// / activity[xWin][yWin][zWin];
                                    if (activity[x][y][z]>0)
                                    {
                                        contributingNeurons++;
                                        valuePrediction[i] += weights[mod][i][x][y][z];
                                    }
                                }
                            }
                        }
                        valuePrediction[i] /= (double)contributingNeurons;
                    }
                }
                if (true /*algorithmUsed == MMCM_ALGORITHM_POPULATION*/)
                {
                    mod->SetValuePrediction(valuePrediction);
                }
                else
                {
                    mod->SetValuePrediction(valuePredictionWinner);
                }
            }

            double getDistance(int x1, int y1, int z1, int x2, int y2, int z2, std::string connectivity)
            {
                double d = 0.0;
                double dX = abs(x1 - x2);
                double dY = abs(y1 - y2);
                double dZ = abs(z1 - z2);

                double euclideanDistance = sqrt(pow(dX, 2.0) + pow(dY, 2.0) + pow(dZ, 2.0));
                if (connectivity == MMCM_CONNECTIVITY_SHEET)
                    d = euclideanDistance;
                else if (connectivity == MMCM_CONNECTIVITY_TORUS)
                {
                    double tdX = abs(x1 + (width - x2));
                    double tdY = abs(y1 + (height - y2));
                    double tdZ = abs(z1 + (layers - z2));
                    d = sqrt(pow(std::min(dX, tdX), 2.0) + pow(std::min(dY, tdY), 2.0) + pow(std::min(dZ, tdZ), 2.0));
                }
                else
                    std::cerr << "Error : Unknown connectivity pattern. Probably no learning will occur." << std::endl;
                return d;
            }


            double getDWCoefficient(double distance2winner, double neuronError, double winnerError, std::string algorithmUsed)
            {
                double dW = 1.0;

                if (algorithmUsed == MMCM_ALGORITHM_SOM)
                    dW = helpers::GaussianBell(distance2winner, parametersRuntime.find("sigma").asDouble());
                else if (algorithmUsed == MMCM_ALGORITHM_DSOM)
                {
                    if (activity[xWin][yWin][zWin] != 0.0)
                        dW = expf(-(1 / pow(parametersRuntime.find("elasticity").asDouble(), 2)) * (distance2winner / winnerError));
                    else
                        dW = 0.0;
                }
                else if (algorithmUsed == MMCM_ALGORITHM_POPULATION)
                {
                    return fabs(neuronError / winnerError);
                }
                else
                    std::cerr << "Error : Unknown algorithm. All neurons will learn equally." << std::endl;
                return dW;
            }

            void adaptWeights()
            {
                double winnerError = activity[xWin][yWin][zWin];

                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        for (int z = 0; z < layers; z++)
                        {
                            float distance2winner = getDistance(x,y,z,xWin,yWin,zWin, connectivityPatern);
                            double dWCoefficient = getDWCoefficient(distance2winner, activity[x][y][z], winnerError, algorithm);
                          
                            //from bottom up input
                            for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                            {
                                std::vector<double> valueReal = it->second->GetValueReal();
                                for (int i = 0; i < it->second->Size(); i++)
                                {
                                    //double currentW = weights[it->second][i][x][y][z];
                                    //double desiredW = valueReal[i];
                                    double error = (valueReal[i] - weights[it->second][i][x][y][z]);
                                    double dW = error * dWCoefficient * getLearningRate() * modalitiesLearning[it->second];
                                    weights[it->second][i][x][y][z] += dW;
                                    helpers::Clamp(weights[it->second][i][x][y][z], 0.0, 1.0);
                                }
                            }

                            //from topdown input
                            for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                            {
                                std::vector<double> valueReal = it->second->GetValueReal();
                                for (int i = 0; i < it->second->Size(); i++)
                                {
                                    double error = (valueReal[i] - weights[it->second][i][x][y][z]);
                                    double dW = error * dWCoefficient * getLearningRate() * modalitiesLearning[it->second];
                                    weights[it->second][i][x][y][z] += dW;
                                    helpers::Clamp(weights[it->second][i][x][y][z], 0.0, 1.0);
                                }
                            }
                        }
                    }
                }
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
                for (std::map<IModality*, std::vector<helpers::Cube> >::iterator wModIt = weights.begin(); wModIt != weights.end(); wModIt++)
                {
                    file << "[modality_" << modCount << "]" << std::endl;
                    //file << "name" << '\t' << wModIt->first->Name() << std::endl;
                    //file << "size" << '\t' << wModIt->first->Size() << std::endl;
                    file << wModIt->first->getModalityConfiguration();
                    yarp::os::Bottle b;
                    for (unsigned int comp = 0; comp < wModIt->second.size(); comp++)
                    {
                        for (unsigned int x = 0; x < wModIt->second[comp].size(); x++)
                        {
                            for (unsigned int y = 0; y < wModIt->second[comp][x].size(); y++)
                            {
                                for (unsigned int z = 0; z < wModIt->second[comp][x][y].size(); z++)
                                {
                                    b.addDouble(wModIt->second[comp][x][y][z]);
                                }
                            }
                        }
                    }
                    file << "weights (" << '\t' << b.toString() << ")" << std::endl;
                    file << std::endl;
                    modCount++;
                }
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

                int modCount = 0;
                yarp::os::Bottle bGroup = file.findGroup("modality_0");
                while (!bGroup.isNull())
                {

                    std::string mName = bGroup.find("name").asString();
                    
                    //Get the right modality
                    IModality* m = NULL;
                    for (std::map<IModality*, std::vector<helpers::Cube> >::iterator wModIt = weights.begin(); wModIt != weights.end(); wModIt++)
                    {
                        if (wModIt->first->Name() == mName)
                        {
                            m = wModIt->first;
                            break;
                        }
                    }

                    if (m == NULL)
                    {
                        std::cerr << "Error while loading the weights. The modality " << mName << " was not found in the current map" << std::endl;
                        return false;
                    }

                    int mSize = m->Size();

                    yarp::os::Bottle *bWeights = bGroup.find("weights").asList();
                    int wCtr = 0;
                    for (int comp = 0; comp < mSize; comp++)
                    {
                        for (unsigned int x = 0; x < weights[m][comp].size(); x++)
                        {
                            for (unsigned int y = 0; y < weights[m][comp][x].size(); y++)
                            {
                                for (unsigned int z = 0; z < weights[m][comp][x][y].size(); z++)
                                {
                                    weights[m][comp][x][y][z] = bWeights->get(wCtr).asDouble();
                                    wCtr++;
                                }
                            }
                        }
                    }

                    modCount++;
                    std::stringstream ss;
                    ss << "modality_" << modCount;
                    bGroup = file.findGroup(ss.str());
                }
                return true;
            }

            yarp::sig::ImageOf<yarp::sig::PixelRgb> getLayerActivity(int i)
            {
                mutex.wait();
                yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
                double maxActivity = activity[xWin][yWin][zWin];
                double minActivity = activity[xLoose][yLoose][zLoose];
                img.resize(width, height);
                for (int x = 0; x < width; x++)
                for (int y = 0; y < height; y++)
                {
                    double normalisedValue = activity[x][y][i];
                    if (maxActivity - minActivity != 0)
                        normalisedValue = (normalisedValue - minActivity) / (maxActivity - minActivity);
                    img.pixel(x, y) = helpers::double2RGB(normalisedValue);

                    //Paint the whole collection of neurons equally activated as the winner
                    if (activity[x][y][i] == maxActivity)
                    {
                        img.pixel(x, y).r = 255;
                        img.pixel(x, y).g = 0;
                        img.pixel(x, y).b = 255;
                    }
                    if (x == xWin && y == yWin && i == zWin)
                    {
                        img.pixel(x, y).r = 0;
                        img.pixel(x, y).g = 0;
                        img.pixel(x, y).b = 0;
                    }
                }
                mutex.post();

                //Console.WriteLine("Visualization computed in " + (time2 - time1).ToString());
                return img;
            }

            /**
            * Returns as an image the receptive field for a given modality of a given neuron of the map.
            * @param x x coordinate of the neuron to plot
            * @param y y coordinate of the neuron to plot
            * @param z z coordinate of the neuron to plot
            * @param modalityToPlot Pointer to the modality you want to plot
            * @return An image representing the receptive field of the whole map.
            */
            IplImage* getReceptiveFieldRepresentation(IModality* modalityToPlot)
            {
                //First we probe the size of one RF.
                int singlelW = 0; //We display the potential layers next to each other
                int singleH = 0;
                yarp::sig::ImageOf<yarp::sig::PixelRgb> probe = getReceptiveFieldRepresentation(0, 0, 0, modalityToPlot);
                singlelW += probe.width();
                singleH = std::max(singleH, probe.height());
   
                int totalW = singlelW * (W() * L()); //We display the potential layers next to each other
                int totalH = singleH * H();

                IplImage* fullImg = cvCreateImage(cvSize(totalW, totalH), 8, 3);
                //here we should fill image with black
                for (int x = 0; x < W(); x++)
                {
                    for (int y = 0; y < H(); y++)
                    {
                        for (int z = 0; z < L(); z++)
                        {
                            int xOffset = x*singlelW + z*singlelW;
                            int yOffset = y*singleH;
                            yarp::sig::ImageOf<yarp::sig::PixelRgb> thumbnail = getReceptiveFieldRepresentation(x, y, z, modalityToPlot);
                            cvSetImageROI(fullImg, cvRect(xOffset, yOffset, thumbnail.width(), thumbnail.height()));
                            cvCopyImage(thumbnail.getIplImage(), fullImg);
                            cvResetImageROI(fullImg);
                            xOffset += thumbnail.width();
                        }
                    }
                }
                return fullImg;
            }

            /**
            * Returns as an image the receptive field for a given modality of a given neuron of the map.
            * @param x x coordinate of the neuron to plot
            * @param y y coordinate of the neuron to plot
            * @param z z coordinate of the neuron to plot
            * @param modalityToPlot Pointer to the modality you want to plot
            * @return An image representing the receptive field of the neuron x,y,z.
            */
            yarp::sig::ImageOf<yarp::sig::PixelRgb> getReceptiveFieldRepresentation(int x, int y, int z, IModality* modalityToPlot)
            {
                mutex.wait();
                yarp::sig::ImageOf<yarp::sig::PixelRgb> img;

                //Buffer the current activity
                std::vector<double> rf(modalityToPlot->Size());
                for (int i = 0; i < modalityToPlot->Size(); i++)
                {
                    rf[i] = weights[modalityToPlot][i][x][y][z];
                }
                img = modalityToPlot->getVisualizationFromVector(rf);
                mutex.post();

                //Console.WriteLine("Visualization computed in " + (time2 - time1).ToString());
                return img;
            }

            /**
            * Returns the weights of a specific neuron for a specific modality (raw receptive field).
            * @param x x coordinate of the neuron to plot
            * @param y y coordinate of the neuron to plot
            * @param z z coordinate of the neuron to plot
            * @param modalityToPlot Pointer to the modality you want to plot
            * @return A vector of double representing the weights of the modality for this specific neuron.
            */
            std::vector<double> getReceptiveFieldWeights(int x, int y, int z, IModality* modality)
            {
                std::vector<double> buff(modality->Size());
                for (int i = 0; i < modality->Size(); i++)
                {
                    buff[i] = weights[modality][i][x][y][z];
                }
                return buff;
            }

            /**
            * Get the best matching unit coordinates for a given activity on a given modality.
            * @param values vector of double to match against the map receptives fields
            * @param modalityToPlot Pointer to the modality to use as the source
            * @param bx reference to x coordinate of the neuron to plot
            * @param by reference to y coordinate of the neuron to plot
            * @param bz reference to z coordinate of the neuron to plot
            * @return A boolean true in case of success, false if the values/modailty size do not match.
            */
            bool getBestMatchingUnit(std::vector<double> values, IModality* modalitySource, int& bx, int& by, int& bz)
            {
                mutex.wait();
                bx = by = bz = 0;
                if (values.size() != (unsigned int) modalitySource->Size())
                    return false;

                double bestError = DBL_MAX;
                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        for (int z = 0; z < layers; z++)
                        {
                            double unitError = 0;
                            for (unsigned int i = 0; i < values.size(); i++)
                            {
                                unitError += fabs(weights[modalitySource][i][x][y][z] - values[i]);
                            }
                            if (bestError>unitError)
                            {
                                bestError = unitError;
                                bx = x;
                                by = y;
                                bz = z;
                            }
                        }
                    }
                }
                mutex.post();
                return true;
            }
        };
    }
}
#endif

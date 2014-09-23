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
            std::queue< std::vector< int > > winnersBuffer;
            int recurrenceDelay;
            int xBuff, yBuff, zBuff;
            yarp::os::Semaphore mutex;

            std::string connectivityPatern;
            std::string algorithm;

        public:

            virtual std::string getType() { return cvz::core::TYPE_MMCM; };
            double lRate, sigma, elasticity;
            int H() { return height; }
            int W() { return width; }
            int L() { return layers; }
            std::string ConnectivityPattern() { return connectivityPatern; }
            std::string Algorithm() { return algorithm; }

            IModality* recurrentModality;

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
            void setLearningRate(const double l) { std::cout << "Learning rate set to : " << l << std::endl; lRate = l; }
            double getLearningRate() { return lRate; }
            void setSigma(const double s) { std::cout << "Sigma set to : " << s << std::endl; sigma = s; }
            double getSigma() { return sigma; }
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

            virtual yarp::os::Bottle getParametersForBroadcast()
            {
                yarp::os::Bottle b = this->IConvergenceZone::getParametersForBroadcast();
                yarp::os::Bottle &bLearning = b.addList();
                bLearning.addString("learningRate");
                bLearning.addDouble(getLearningRate());
                return b;
            }
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
                height = rf.check("height", yarp::os::Value(10)).asInt();
                width = rf.check("width", yarp::os::Value(10)).asInt();
                layers = rf.check("layers", yarp::os::Value(1)).asInt();
                lRate = rf.check("learningRate", yarp::os::Value(0.05)).asDouble();
                connectivityPatern = rf.check("connectivityPattern", yarp::os::Value(MMCM_CONNECTIVITY_SHEET)).asString();
                algorithm = rf.check("algorithm", yarp::os::Value(MMCM_ALGORITHM_DSOM)).asString();

                int recModalitySize = rf.check("recurrentModality", yarp::os::Value(0)).asInt();
                recurrenceDelay = rf.check("recurrentDelay", yarp::os::Value(10)).asInt();
                if (recModalitySize > 0)
                {
                    std::string recModName = "/";
                    recModName += getName();
                    recModName += "/recurrent";
                    yarp::os::Property propTmp;
                    propTmp.unput("name");
                    propTmp.put("name", recModName);
                    propTmp.unput("size");
                    propTmp.put("size", recModalitySize);

                    yarp::os::Bottle bMinBounds;
                    yarp::os::Bottle bMaxBounds;
                    for (int i = 0; i < recModalitySize; i++)
                    {
                        bMinBounds.addDouble(0);
                        bMaxBounds.addDouble(1);
                    }
                    yarp::os::Value pMin;
                    bMinBounds.write(pMin);
                    yarp::os::Value pMax;
                    bMaxBounds.write(pMax);
                    propTmp.unput("minBounds");
                    propTmp.put("minBounds", pMin);
                    propTmp.unput("maxBounds");
                    propTmp.put("maxBounds", pMax);

                    yarp::os::Bottle bTmp;
                    bTmp.read(propTmp);
                    recurrentModality = new IModality(recModName, bTmp, this);
                    modalitiesInfluence[recurrentModality] = rf.check("recurrentInfluence", yarp::os::Value(1.0)).asDouble();
                }
                else
                    recurrentModality = NULL;

                double sigmaFactor = rf.check("sigmaFactor", yarp::os::Value(1.0)).asDouble();
                sigma = sigmaFactor * (1.0 / 4.0) * (height + width) / 2.0;
                elasticity = rf.check("elasticity", yarp::os::Value(2.0)).asDouble();

                //Allocate the map
                activity.allocate(width, height, layers);

                //Allocate the weights
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                {
                    //Check if MMCM specifics parameters where added into the modality

                    std::vector< helpers::Cube > w;
                    w.resize(it->second->Size());
                    for (int i = 0; i < it->second->Size(); i++)
                    {
                        w[i].allocate(width, height, layers);
                        w[i].randomize(0.0, 1.0);
                    }
                    weights[it->second] = w;
                }

                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                {
                    std::vector< helpers::Cube > w;
                    w.resize(it->second->Size());
                    for (int i = 0; i < it->second->Size(); i++)
                    {
                        w[i].allocate(width, height, layers);
                        w[i].randomize(0.0, 1.0);
                    }
                    weights[it->second] = w;
                }

                if (recurrentModality != NULL)
                {
                    std::vector< helpers::Cube > w;
                    w.resize(recurrentModality->Size());
                    for (int i = 0; i < recurrentModality->Size(); i++)
                    {
                        w[i].allocate(width, height, layers);
                        w[i].randomize(0.0, 1.0);
                    }
                    weights[recurrentModality] = w;
                }

                //Set modalitiesInfluence and modalitiesLearning to 1.0 by default
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                {
                    modalitiesInfluence[it->second] = 1.0;
                    modalitiesLearning[it->second] = 1.0;
                }
                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                {
                    modalitiesInfluence[it->second] = 1.0;
                    modalitiesLearning[it->second] = 1.0;
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

                            //Reccurent modality
                            if (recurrentModality != NULL)
                            {
                                std::vector<double> recVReal = recurrentModality->GetValueReal();
                                double modalityMeanError = 0.0;
                                for (int i = 0; i < recurrentModality->Size(); i++)
                                {
                                    modalityMeanError += fabs(weights[recurrentModality][i][x][y][z] - recVReal[i]);
                                }
                                modalityMeanError /= recurrentModality->Size();
                                activity[x][y][z] += 1.0 - (modalityMeanError * modalitiesInfluence[recurrentModality]);
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

                //------------------------------------------------------------------------------------------------------------------------
                //Send the output activity
                portActivity.write();

                //------------------------------------------------------------------------------------------------------------------------
                //Keep track of the past winners in a buffer (not really used yet)
                if (winnersBuffer.size() == (unsigned int) recurrenceDelay)
                    winnersBuffer.pop();
                std::vector<int> currentWinner(3);
                currentWinner[0] = xWin;
                currentWinner[1] = yWin;
                currentWinner[2] = zWin;
                winnersBuffer.push(currentWinner);

                if (cyclesElapsed % recurrenceDelay == 0)
                {
                    xBuff = xWin;
                    yBuff = yBuff;
                    zBuff = zBuff;
                }


                //------------------------------------------------------------------------------------------------------------------------
                //Learning
                //check if all modalities learning is off
                bool allModLearningZero = true;
                for (std::map<IModality*, double>::iterator itL = modalitiesLearning.begin(); itL != modalitiesLearning.end(); itL++)
                    allModLearningZero &= (itL->second == 0.0);
                if (lRate > 0.0 && !allModLearningZero)
                    adaptWeights();


                //------------------------------------------------------------------------------------------------------------------------
                //Set the predicted values 

                //feedback
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                    predictModality(it->second, algorithm);
                //feedforward
                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                    predictModality(it->second, algorithm);

                //Recurrent
                if (recurrentModality != NULL)
                {
                    std::vector<double> valuePrediction;
                    valuePrediction.resize(recurrentModality->Size());
                    for (int i = 0; i < recurrentModality->Size(); i++)
                    {
                        valuePrediction[i] = weights[recurrentModality][i][xWin][yWin][zWin];
                    }
                    recurrentModality->SetValuePrediction(valuePrediction);
                    recurrentModality->SetValueReal(valuePrediction); //we set the next input of this modality to be the last prediction
                }
                mutex.post();
            }

            void predictModality(IModality* mod, std::string algorithmUsed)
            {
                std::vector<double> valuePrediction;
                valuePrediction.resize(mod->Size(), 0.0);

                std::vector<double> valuePredictionWinner;
                valuePredictionWinner.resize(mod->Size(), 0.0);
                for (int i = 0; i < mod->Size(); i++)
                {
                    valuePredictionWinner[i] = weights[mod][i][xWin][yWin][zWin];

                    if (algorithmUsed == MMCM_ALGORITHM_POPULATION)
                    {
                        double totalContribution = 0.0;
                        for (int x = 0; x < width; x++)
                        {
                            for (int y = 0; y < height; y++)
                            {
                                for (int z = 0; z < layers; z++)
                                {
                                    double contribution = activity[x][y][z];// / activity[xWin][yWin][zWin];
                                    valuePrediction[i] += (weights[mod][i][x][y][z] * contribution);
                                    totalContribution += contribution;
                                }
                            }
                        }
                        valuePrediction[i] /= totalContribution;
                    }
                }
                if (algorithmUsed == MMCM_ALGORITHM_POPULATION)
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
                double euclideanDistance = sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0) + pow(z1 - z2, 2.0));
                if (connectivity == MMCM_CONNECTIVITY_SHEET)
                    d = euclideanDistance;
                else if (connectivity == MMCM_CONNECTIVITY_TORUS)
                    d = sqrt (pow(std::min(x1 - x2, x1 + (width - x2)), 2.0) + pow(std::min(y1 - y2, y1 + (height - y2)), 2.0) + pow(std::min(z1 - z2, z1 + (layers - z2)), 2.0));

                return d;
            }


            double getDWCoefficient(double distance2winner, double neuronError, double winnerError, std::string algorithmUsed)
            {
                double dW = 1.0;

                if (algorithmUsed == MMCM_ALGORITHM_SOM)
                    dW = helpers::GaussianBell(distance2winner, sigma);
                else if (algorithmUsed == MMCM_ALGORITHM_DSOM)
                {
                    if (activity[xWin][yWin][zWin] != 0.0)
                        dW = expf(-(1 / pow(elasticity, 2)) * (distance2winner / winnerError));
                    else
                        dW = 0.0;
                }
                else if (algorithmUsed == MMCM_ALGORITHM_POPULATION)
                {
                    dW = helpers::sigmoidFunction(fabs(neuronError), 0.4, 10);
                }
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
                                    double dW = error;
                                    dW = dW * dWCoefficient * lRate * modalitiesLearning[it->second];
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
                                    double dW = error * modalitiesLearning[it->second] * lRate * modalitiesLearning[it->second];
                                    dW = dW * dWCoefficient * lRate * modalitiesLearning[it->second];
                                    weights[it->second][i][x][y][z] += dW;
                                    helpers::Clamp(weights[it->second][i][x][y][z], 0.0, 1.0);
                                }
                            }

                            //recurrent connection is buffered
                            if (recurrentModality)
                            {

                                //float pastDistanceH = sqrt(pow(x - winnersBuffer.back()[0], 2.0) + pow(y - winnersBuffer.back()[1], 2.0));
                                //float pastDistanceV = sqrt(pow(z - winnersBuffer.back()[2], 2.0));
                                //float pdHCoef = helpers::GaussianBell(pastDistanceH, sigmaH);
                                //float pdVCoef = helpers::GaussianBell(pastDistanceV, sigmaV);
                                std::vector<double> valueReal = recurrentModality->GetValueReal();
                                for (int i = 0; i < recurrentModality->Size(); i++)
                                {
                                    //double currentW = weights[recurrentModality][i][x][y][z];
                                    //double desiredW = valueReal[i];
                                    double error = (weights[recurrentModality][i][xWin][yWin][zWin] - weights[recurrentModality][i][x][y][z]);
                                    double dW = error * lRate * modalitiesLearning[recurrentModality];
                                    dW = dW * dWCoefficient * lRate * modalitiesLearning[recurrentModality];

                                    weights[recurrentModality][i][x][y][z] += dW;
                                    helpers::Clamp(weights[recurrentModality][i][x][y][z], 0.0, 1.0);
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

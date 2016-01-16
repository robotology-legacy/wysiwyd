#ifndef __CVZ_ICVZ_H__
#define __CVZ_ICVZ_H__

#include "CvzTags.h"
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include "IModality.h"
#include "cvz/helpers/helpers.h"

//OpenCV
#include <opencv2/opencv.hpp>

namespace cvz {
    namespace core {
        class IConvergenceZone //: public cvz_IDL
        {

        private:

            std::string name;
            int cyclesElapsed;
            double period;
            bool isPaused;
            bool logResultsEnabled;
            double logStartTime;
            void broadcastParameters()
            {
                yarp::os::Bottle b = getParametersForBroadcast();
                parametersPortOut.write(b);
            }

        protected:
            //Parameters defined when the CVZ is instantiated and read only after that
            yarp::os::Property parametersStartTime;

        public:
            yarp::os::RpcServer rpcPort;
            yarp::os::Port  parametersPortOut;

            //Parameters accesible in read/write mode at runtime
            yarp::os::Property parametersRuntime;
            yarp::os::Property getParametersStartTime() const { return parametersStartTime; }

            std::string getRpcPortName(){ return rpcPort.getName(); }
            std::string getName(){ return name; }
            void setName(std::string _name){ name = _name; }
            
            //Thrifted methods
            void moduleStart() { std::cout << "Started." << std::endl; isPaused = false; }
            void modulePause() { std::cout << "Paused." << std::endl; isPaused = true; }            


            virtual std::string getType() { return cvz::core::TYPE_ICVZ; };

            std::map<std::string, IModality*> modalitiesBottomUp;
            std::map<std::string, IModality*> modalitiesTopDown;
            std::map<IModality*, double > modalitiesInfluence;
            std::map<IModality*, double > modalitiesLearning;

            virtual bool configure(yarp::os::Property &prop)
            {
                //For legacy config files and easier configuration, everything that is not in a group is appended to startime parameters
                yarp::os::Bottle bFull;
                bFull.read(prop);
                std::string debug = bFull.toString();
                yarp::os::Property outOfGroupProps;
                for (int i = 0; i < bFull.size(); i++)
                {
                    std::string key = bFull.get(i).asList()->get(0).asString();
                    if (key.find("modality_") == std::string::npos && key != "Parameters_RunTime" && key != "Parameters_StartTime")
                    {
                        outOfGroupProps.put(key, prop.find(key));
                    }
                }
                bFull.clear();
                bFull.read(outOfGroupProps);
                debug = bFull.toString();

                yarp::os::Bottle& bParamsFixed = prop.findGroup("Parameters_StartTime");
                if (!bParamsFixed.isNull())
                {
                    bParamsFixed.append(bFull);
                    bParamsFixed.write(parametersStartTime);
                }
                else
                {
                    bFull.write(parametersStartTime);
                }

                //Add the required parameters if they are not specified
                if (!parametersStartTime.check("name"))
                    parametersStartTime.put("name", yarp::os::Value("defaultCvz"));
                if (!parametersStartTime.check("period"))
                    parametersStartTime.put("period", yarp::os::Value(0.01));

                std::string name = parametersStartTime.find("name").asString();
                period = parametersStartTime.find("period").asDouble();;
                logResultsEnabled = parametersStartTime.check("enableLog");

                yarp::os::Bottle& bParamsVariable = prop.findGroup("Parameters_RunTime");
                bParamsVariable.write(parametersRuntime);
                //std::string debug = parametersRuntime.toString();
                //std::string debug2 = bParamsVariable.toString();

                setName(name.c_str());
                std::string modPortPrefix = "/";
                modPortPrefix += getName() + "/";

                int modalityCount = 0;

                yarp::os::Bottle bMod; bMod.addDouble(0);
                bool reachedLastModality = false;
                while (!reachedLastModality)
                {
                    std::string ss = "modality_";
                    ss += helpers::int2str(modalityCount);

                    bMod = prop.findGroup(ss.c_str());
                    if (bMod.size() == 0)
                    {
                        reachedLastModality = true;
                        break;
                    }

                    std::cout << name << " is configuring " << ss << std::endl;
                    std::string modName = bMod.find("name").asString();
                    bool isTopDown = bMod.check("isTopDown");
                    double modLearning = bMod.check("learningRate", yarp::os::Value(1.0)).asDouble();
                    double modInf = bMod.check("influence", yarp::os::Value(1.0)).asDouble();
                    //Get the type and any additional parameters from the property
                    std::string modType = bMod.find("type").asString();
                    IModality* mod = NULL;
                    if (modType == "yarpVector")
                        mod = new ModalityBufferedPort<yarp::os::Bottle>(modPortPrefix, bMod,this);
                    else if (modType == "yarpSound")
                        mod = new ModalityBufferedPort<yarp::sig::Sound>(modPortPrefix, bMod, this);
                    else if (modType == "yarpImageFloat")
                        mod = new ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >(modPortPrefix, bMod, this);
                    else if (modType == "yarpImageRgb")
                        mod = new ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >(modPortPrefix, bMod, this);
                    else
                    {
                        std::cout << "Warning, this modality type does not exist. Using the IModality class...." << std::endl;
                        mod = new IModality(modPortPrefix, bMod, this);
                    }

                    if (mod != NULL)
                    {
                        if (isTopDown)
                            modalitiesTopDown[mod->Name()] = mod;
                        else
                            modalitiesBottomUp[mod->Name()] = mod;

                        modalitiesInfluence[mod] = modInf;
                        modalitiesLearning[mod] = modLearning;
                    }

                    modalityCount++;
                    ss.clear();
                }

                isPaused = false;
                std::string paraName = "/";
                paraName += getName() + "/parameters:o";
                parametersPortOut.open(paraName.c_str());

                std::string rpcName = "/";
                rpcName += getName() + "/rpc";
                rpcPort.open(rpcName.c_str());
                attach(rpcPort);

                if (logResultsEnabled)
                    logFillHeaders();
                std::cout << std::endl << "Modalities added. Starting the CVZ process with " << period << "s period" << std::endl;
                cyclesElapsed = 0;

                return true;
            }
            

            /***************************************************************/
            virtual yarp::os::Bottle getParametersForBroadcast()
            {
                yarp::os::Bottle b;
                for (std::map<IModality*, double >::iterator itInf = modalitiesInfluence.begin(); itInf != modalitiesInfluence.end(); itInf++)
                {
                    yarp::os::Bottle &bSub = b.addList();
                    bSub.addString(itInf->first->Name()); //Mod name
                    bSub.addDouble(itInf->second); //Influence
                    bSub.addDouble(modalitiesLearning[itInf->first]); //Learning
                }

                yarp::os::Bottle btmp;
                btmp.read(parametersRuntime);
                b.append(btmp);

                yarp::os::Bottle btmp2;
                btmp2.read(parametersStartTime);
                b.append(btmp2);

                return b;
            }

            /***************************************************************/
            virtual bool attach(yarp::os::RpcServer &source)
            {
                std::cerr << "Error: You are using the abstract CVZ attached method. Thrift interface will not be available. Implement attach in your daughter class." << std::endl;
                return false;
                //return this->yarp().attachAsServer(source);
            }

            bool interruptModule()
            {
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                {
                    it->second->Interrupt();
                }
                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                {
                    it->second->Interrupt();
                }
                return true;
            }

            virtual bool close()
            {
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                {
                    it->second->Close();
                }
                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                {
                    it->second->Close();
                }

                return true;
            }



            bool cycle()
            {
                if (isPaused)
                    return true;

                //Read the modalities
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                {
                    it->second->Input();
                }
                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                {
                    it->second->Input();
                }
                
                if (logResultsEnabled)
                    logFillData();

                //Do some computation
                ComputePrediction();

                //Write the modalities
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                {
                    it->second->Output();
                }
                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                {
                    it->second->Output();
                }

                //Send parameters
                broadcastParameters();

                //if (cyclesElapsed % 500 == 0)
                //    std::cout << getName() << "\t t=" << cyclesElapsed << std::endl;
                performPeriodicAction(cyclesElapsed);
                cyclesElapsed++;
                return true;
            }

            virtual void performPeriodicAction(const int &cyclesElapsed)
            {
                if (cyclesElapsed % 500 == 0)
                    std::cout << getName() << "\t t=" << cyclesElapsed << std::endl;
            }

            virtual void ComputePrediction()
            {
                std::cout << "Warning: You are using a dummy class! The base class of CVZ just send back the input." << std::endl;

                //Copy input to prediction
                for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                {
                    it->second->SetValuePrediction(it->second->GetValueReal());
                }
                for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                {
                    it->second->SetValuePrediction(it->second->GetValueReal());
                }
            }

            void logFillHeaders()
            {
                std::ofstream file((getName() + ".log").c_str(), std::ofstream::app);

                file << "time;";
                for (std::map<IModality*, double>::iterator it = modalitiesInfluence.begin(); it != modalitiesInfluence.end(); it++)
                {
                    file << it->first->GetFullName() << "_realValue" << ';';
                    file << it->first->GetFullName() << "_fullInputPrediction" << ';';
                    file << it->first->GetFullName() << "_partialInputPrediction" << ';';
                    file << it->first->GetFullName() << "_fullMeanError" << ';';
                    file << it->first->GetFullName() << "_partialMeanError" << ';';
                }
                file << std::endl;
                file.close();
                logStartTime = yarp::os::Time::now();
            }

            //Given the current input, this function will generate all the possible modalities subsets and log the predictions
            //May slow down the 
            void logFillData()
            {
                //double t0 = yarp::os::Time::now();

                std::map<IModality*, double> previousInfluences = modalitiesInfluence;
                std::map<IModality*, double> previousLearning = modalitiesLearning;

                std::map < IModality*, std::vector< std::vector<double> > > entries; //[mod [real, fullPred, partialPred] ]

                //Set all learning rates to 0, influence to 1 and create the entries
                for (std::map<IModality*, double>::iterator it = modalitiesInfluence.begin(); it != modalitiesInfluence.end(); it++)
                {
                    modalitiesLearning[it->first] = 0.0;
                    modalitiesInfluence[it->first] = 1.0;

                    std::vector<double> vTmp(it->first->Size());
                    entries[it->first].resize(3, vTmp);
                    entries[it->first][0] = it->first->GetValueReal();
                }

                //Test prediction given the complete input
                ComputePrediction();
                for (std::map<IModality*, double>::iterator it = modalitiesInfluence.begin(); it != modalitiesInfluence.end(); it++)
                {
                    entries[it->first][1] = it->first->GetValuePrediction();
                }

                //Test prediction for all modalities given the others
                for (std::map<IModality*, double>::iterator it = modalitiesInfluence.begin(); it != modalitiesInfluence.end(); it++)
                {
                    //Set all influences to 1.0 and the tested one to 0.0
                    for (std::map<IModality*, double>::iterator it2 = modalitiesInfluence.begin(); it2 != modalitiesInfluence.end(); it2++)
                    {
                        if (it->first == it2->first)
                            modalitiesInfluence[it2->first] = 0.0;
                        else
                            modalitiesInfluence[it2->first] = 1.0;
                    }

                    ComputePrediction();
                    entries[it->first][2] = it->first->GetValuePrediction();
                }

                //Compute errors && Write down the results
                std::ofstream file((getName() + ".log").c_str(), std::ofstream::app);

                file << yarp::os::Time::now() - logStartTime << ';';

                for (std::map<IModality*, double>::iterator it = modalitiesInfluence.begin(); it != modalitiesInfluence.end(); it++)
                {
                    for (int c = 0; c < it->first->Size(); c++)
                        file << entries[it->first][0][c] << ' ';
                    file << ';';

                    for (int c = 0; c < it->first->Size(); c++)
                        file << entries[it->first][1][c] << ' ';
                    file << ';';

                    for (int c = 0; c < it->first->Size(); c++)
                        file << entries[it->first][2][c] << ' ';
                    file << ';';

                    double fullMeanError = 0.0;
                    double partialMeanError = 0.0;
                    for (int c = 0; c < it->first->Size(); c++)
                    {
                        fullMeanError += fabs(entries[it->first][0][c] - entries[it->first][1][c]);
                        partialMeanError += fabs(entries[it->first][0][c] - entries[it->first][2][c]);
                    }
                    file << fullMeanError / (double)it->first->Size() << ';';
                    file << partialMeanError / (double)it->first->Size() << ';';
                }
                file << std::endl;
                file.close();

                //Reset learning and influence as they were before
                modalitiesInfluence = previousInfluences;
                modalitiesLearning = previousLearning;

                //std::cout << "Logging time : " << yarp::os::Time::now() - t0 << std::endl;
            }

        };
    }
}
#endif

#ifndef __CVZ_ICVZ_H__
#define __CVZ_ICVZ_H__

#include "CvzTags.h"
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include "IModality.h"
#include "cvz/helpers/helpers.h"

namespace cvz {
	namespace core {
		class IConvergenceZone //: public cvz_IDL
		{

		private:
			void broadcastParameters()
			{
				yarp::os::Bottle b = getParametersForBroadcast();
				parametersPortOut.write(b);
			}

		public:
			yarp::os::RpcServer rpcPort;
			yarp::os::Port	parametersPortOut;

			std::string getRpcPortName(){ return rpcPort.getName(); }
			std::string getName(){ return name; }
			void setName(std::string _name){ name = _name; }
			
			//Thrifted methods
			void moduleStart() { std::cout << "Started." << std::endl; isPaused = false; }
			void modulePause() { std::cout << "Paused." << std::endl; isPaused = true; }			

		protected:
			std::string name;
			int cyclesElapsed;
			double period;
			bool isPaused;

		public:

            virtual std::string getType() { return cvz::core::TYPE_ICVZ; };

			std::map<std::string, IModality*> modalitiesBottomUp;
			std::map<std::string, IModality*> modalitiesTopDown;
			std::map<IModality*, double > modalitiesInfluence;
			std::map<IModality*, double > modalitiesLearning;

			virtual bool configure(yarp::os::Property &prop)
			{

				std::string name = prop.check("name", yarp::os::Value("defaultCvz")).asString();
				period = prop.check("period", yarp::os::Value(0.01)).asDouble();
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

				if (cyclesElapsed % 500 == 0)
					std::cout << getName() << "\t t=" << cyclesElapsed << std::endl;
				cyclesElapsed++;
				return true;
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


		};
	}
}
#endif

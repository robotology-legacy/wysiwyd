#ifndef __CVZ_ICVZ_H__
#define __CVZ_ICVZ_H__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include "IModality.h"
#include "cvz/helpers/helpers.h"
#include "cvz_IDL.h"

namespace cvz {
	namespace core {
		class IConvergenceZone : public cvz_IDL
		{

		public:
			yarp::os::RpcServer rpcPort;

			std::string getRpcPortName(){ return rpcPort.getName(); }
			std::string getName(){ return name; }
			void setName(std::string _name){ name = _name; }
			void start() { std::cout << "Started." << std::endl; isPaused = false; }
			void pause() { std::cout << "Paused." << std::endl; isPaused = true; }

		protected:
			std::string name;
			int cyclesElapsed;
			double period;
			bool isPaused;

		public:

			std::map<std::string, IModality*> modalitiesBottomUp;
			std::map<std::string, IModality*> modalitiesTopDown;
			std::map<IModality*, double > modalitiesInfluence;
			std::map<IModality*, double > modalitiesLearning;

			virtual bool configure(yarp::os::Property &prop)
			{
				std::string name = prop.check("name", yarp::os::Value("defaultCvz")).asString();
				period = prop.check("period", yarp::os::Value(0.01)).asDouble();
				setName(name.c_str());

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
					//Get the generic parameters (name, size, minBounds, maxBounds, isTopDown)
					std::string modName = bMod.find("name").asString();
					int modSize = bMod.find("size").asInt();
					double modInf = bMod.check("influence", yarp::os::Value(1.0)).asDouble();
					double modLearning= bMod.check("learningRate", yarp::os::Value(1.0)).asDouble();

					std::vector<double> minBounds;
					if (bMod.check("minBounds"))
					{
						yarp::os::Bottle* bMask = bMod.find("minBounds").asList();
						for (int i = 0; i < bMask->size(); i++)
							minBounds.push_back(bMask->get(i).asDouble());
					}
					else
						minBounds.resize(modSize, 0.0);

					std::vector<double> maxBounds;
					if (bMod.check("maxBounds"))
					{
						yarp::os::Bottle* bMask = bMod.find("maxBounds").asList();
						for (int i = 0; i < bMask->size(); i++)
							maxBounds.push_back(bMask->get(i).asDouble());
					}
					else
						maxBounds.resize(modSize, 1.0);

					bool isTopDown = bMod.check("isTopDown");

					//Get the type and any additional parameters
					std::string modType = bMod.find("type").asString();
					IModality* mod = NULL;
					if (modType == "yarpVector")
					{
						std::vector<bool> mask;
						if (bMod.check("mask"))
						{
							yarp::os::Bottle* bMask = bMod.find("mask").asList();
							for (int i = 0; i < bMask->size(); i++)
								mask.push_back(bMask->get(i).asDouble());
						}
						bool isBlocking = bMod.check("isBlocking");
						std::string modPortPrefix = "/";
						modPortPrefix += getName() + "/";
						modPortPrefix += modName;
						mod = new ModalityBufferedPort<yarp::os::Bottle>(modPortPrefix, modSize, minBounds, maxBounds, mask, isBlocking);

						std::string autoConnect = bMod.check("autoconnect", yarp::os::Value("")).asString();
						if (autoConnect != "")
							((ModalityBufferedPort<yarp::os::Bottle>*)mod)->ConnectInput(autoConnect);
					}
					else if (modType == "yarpSound")
					{
						std::vector<bool> mask;
						if (bMod.check("mask"))
						{
							yarp::os::Bottle* bMask = bMod.find("mask").asList();
							for (int i = 0; i < bMask->size(); i++)
								mask.push_back(bMask->get(i).asDouble());
						}
						bool isBlocking = bMod.check("isBlocking");
						std::string modPortPrefix = "/";
						modPortPrefix += getName() + "/";
						modPortPrefix += modName;
						mod = new ModalityBufferedPort<yarp::sig::Sound>(modPortPrefix, modSize, minBounds, maxBounds, mask, isBlocking);

						std::string autoConnect = bMod.check("autoconnect", yarp::os::Value("")).asString();
						if (autoConnect != "")
							((ModalityBufferedPort<yarp::sig::Sound>*)mod)->ConnectInput(autoConnect);
					}
					else if (modType == "yarpImageFloat")
					{
						std::vector<bool> mask;
						if (prop.check("mask"))
						{
							yarp::os::Bottle* bMask = prop.find("mask").asList();
							for (int i = 0; i < bMask->size(); i++)
								mask.push_back(bMask->get(i).asDouble());
						}
						bool isBlocking = bMod.check("isBlocking");
						std::string modPortPrefix = "/";
						modPortPrefix += getName() + "/";
						modPortPrefix += modName;
						mod = new ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >(modPortPrefix, modSize, minBounds, maxBounds, mask, isBlocking);

						std::string autoConnect = bMod.check("autoconnect", yarp::os::Value("")).asString();
						if (autoConnect != "")
							((ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >*)mod)->ConnectInput(autoConnect);
					}
					else if (modType == "yarpImageRgb")
					{
						std::vector<bool> mask;
						if (prop.check("mask"))
						{
							yarp::os::Bottle* bMask = prop.find("mask").asList();
							for (int i = 0; i < bMask->size(); i++)
								mask.push_back(bMask->get(i).asDouble());
						}
						bool isBlocking = bMod.check("isBlocking");
						std::string modPortPrefix = "/";
						modPortPrefix += getName() + "/";
						modPortPrefix += modName;
						mod = new ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >(modPortPrefix, modSize, minBounds, maxBounds, mask, isBlocking);

						std::string autoConnect = bMod.check("autoconnect", yarp::os::Value("")).asString();
						if (autoConnect != "")
							((ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*)mod)->ConnectInput(autoConnect);
					}
					else
					{
						std::cout << "Warning, this modality type does not exist. Discarded." << std::endl;
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
				std::string rpcName = "/";
				rpcName += getName() + "/rpc";
				rpcPort.open(rpcName.c_str());
				attach(rpcPort);

				std::cout << std::endl << "Modalities added. Starting the CVZ process with " << period << "s period" << std::endl;
				cyclesElapsed = 0;

				return true;
			}
			
			/***************************************************************/
			bool attach(yarp::os::RpcServer &source)
			{
				return this->yarp().attachAsServer(source);
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

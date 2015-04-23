/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Gr?goire Pointeau
* email:   gregoire.pointeau@inserm.fr
* website: http://efaa.upf.edu/
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* wysiwyd/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#ifndef __EFAA_SUBSYSTEM_RECOG_H__
#define __EFAA_SUBSYSTEM_RECOG_H__

#define SUBSYSTEM_RECOG        "recog"

#include "wrdac/subsystems/subSystem.h"
#include <iostream>

namespace wysiwyd{
	namespace wrdac{

		/**
		* \ingroup wrdac_clients
		*
		* Abstract subSystem for speech recognizer
		*/
		class SubSystem_Recog : public SubSystem
		{
		protected:
			bool ABMconnected;
			virtual bool connect() {
				// paste master name of 
				SubABM = new SubSystem_ABM("from_recog");
				ABMconnected = (SubABM->Connect());
				std::cout << ((ABMconnected) ? "Recog connected to ABM" : "Recog didn't connect to ABM") << std::endl;
				return yarp::os::Network::connect(portRPC.getName(), "/speechRecognizer/rpc");
			}
			SubSystem_ABM* SubABM;
		public:

			yarp::os::Port portRPC;
			SubSystem_Recog(const std::string &masterName) :SubSystem(masterName){
				portRPC.open(("/" + m_masterName + "/recog:rpc").c_str());
                m_type = SUBSYSTEM_RECOG;
			}


			virtual void Close() { portRPC.interrupt(); portRPC.close(); };


			/**
			* From one grxml grammar, return the sentence recognized for one timeout
			*
			*/
			yarp::os::Bottle recogFromGrammar(std::string &sInput)
			{
				yarp::os::Bottle bMessenger;
				yarp::os::Bottle bReply;
				bMessenger.addString("recog");
				bMessenger.addString("grammarXML");
				bMessenger.addString(sInput);
				portRPC.write(bMessenger, bReply);
				return bReply;
			}


			/**
			*   From one grxml grammar, return the first sentence non-empty recognized
			*   can last for several timeout (by default 50
			*/
			yarp::os::Bottle recogFromGrammarLoop(std::string sInput, int iLoop = 50)
			{
				std::ostringstream osError;
				bool fGetaReply = false;
				yarp::os::Bottle bMessenger, //to be send TO speech recog
					bAnswer,
					bReply, //response from speech recog without transfer information, including raw sentence
					bOutput; // semantic information of the content of the recognition

				bMessenger.addString("recog");
				bMessenger.addString("grammarXML");
				bMessenger.addString(sInput);

				int loop = 0;

				while (!fGetaReply && loop < iLoop)
				{
					portRPC.write(bMessenger, bReply);

					std::cout << "Reply from Speech Recog : " << bReply.toString() << std::endl;

					if (bReply.toString() == "NACK" || bReply.size() != 2)
					{
						bOutput.addInt(0);
						osError << "Check grammar";
						bOutput.addString(osError.str());
						std::cout << osError.str() << std::endl;
						return bOutput;
					}

					if (bReply.get(0).toString() == "0")
					{
						bOutput.addInt(0);
						osError << "Grammar not recognized";
						bOutput.addString(osError.str());
						std::cout << osError.str() << std::endl;
						return bOutput;
					}

					bAnswer = *bReply.get(1).asList();

					if (bAnswer.toString() != "" && !bAnswer.isNull())
					{
						fGetaReply = true;
						bOutput.addInt(1);
						bOutput.addList() = bAnswer;


						// send the result of recognition to the ABM
						if (ABMconnected)
						{
							std::list<std::pair<std::string, std::string> > lArgument;
							lArgument.push_back(std::pair<std::string, std::string>(bAnswer.get(0).toString(), "sentence"));
							lArgument.push_back(std::pair<std::string, std::string>(bAnswer.get(1).toString(), "semantic"));
							lArgument.push_back(std::pair<std::string, std::string>(m_masterName, "provider"));
							SubABM->sendActivity("action",
								"sentence",
								"recog",
								lArgument,
								true);
						}

					}
					loop++;
				}

				if (!fGetaReply)
				{
					bOutput.addInt(0);
					osError << "no vocal input";
					bOutput.addString(osError.str());
					std::cout << osError.str() << std::endl;
				}

				return bOutput;
			}

		};


	}
}//Namespace
#endif


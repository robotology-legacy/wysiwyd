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
			* Recursive method to extract the semantic cues of each word recognized
            * the bRecogBottle is the sentence bottle sent by speechRecog (so without the sentence at first)
            * e.g.  from Recgo : sentence ((temporal "before you") (actionX (action1 ((verb1 point) (object "the circle")))) (actionX (action2 ((verb2 push) (object "the ball")))))
            * bRecogBottle     : (temporal "before you") (actionX (action1 ((verb1 point) (object "the circle")))) (actionX (action2 ((verb2 push) (object "the ball"))))
			*
			*/
			void recogFromGrammarSemantic(yarp::os::Bottle bRecogBottle, std::string s_deep, int i_deep)
			{
				yarp::os::Bottle bReply;

                //TODO : list of string for the deepness, no need for the int in that case
                //TODO : careful, may have to copy each time because of recursive

                std::string currentWord = "" ;
                std::string currentRole = "" ;

              //  yInfo() << "bRecogBottle = " << bRecogBottle.toString() ;

                //case 1 : string string -> end of the recursive
                if(bRecogBottle.get(0).isString() && bRecogBottle.get(1).isString()){

                    //yInfo() << "===== case 1 : string/string =====" ;

                    currentRole = bRecogBottle.get(0).asString() ;
                    currentWord = bRecogBottle.get(1).asString() ;
					std::cout << std::endl;
                    //SQL insert
                    //std::cout << "=== s_deep = " << s_deep << " and i_deep = " << i_deep << "===" << std::endl;
                    std::cout << "C1 : -------> role = " << currentRole << " and word = " << currentWord << " and level " << i_deep << std::endl ;
                } 

                //case 2 : string list -> sub-sentence, sub-part
                else if (bRecogBottle.get(0).isString() && bRecogBottle.get(1).isList()){
                    
                  //  yInfo() << "===== case 2 : string/List =====" ;

                    s_deep = bRecogBottle.get(0).asString() ; //TODO : increase the list
					std::cout << "C2 : -------> role = " << "semantic" << " and word = " << s_deep << " and level = " << i_deep << std::endl;

                    i_deep = i_deep*10 + 1 ;
					int i_deep_cp = i_deep;
                    recogFromGrammarSemantic(*bRecogBottle.get(1).asList(), s_deep, i_deep_cp) ;
                    

                } 
                
                //case 3 : it is not case 1 or 2, so we should have reach the "end" of a semantic, and having group of pairs (role1 arg1) (role2 arg2) (role3 arg3) 
                //list -> list of word
                else if (bRecogBottle.size() > 1){
              //       yInfo() << "===== case 3 : List =====" ;

                    for(unsigned int i = 0 ; i < bRecogBottle.size() ; i++) {

                     //   yInfo() << " --> i = " << i ;
						
						int i_deep_cp = i_deep;
						if (i != 0)
						{
							i_deep += 1;
							i_deep_cp = i_deep;
							//std::cout << "C3 : -------> role = " << "semantic" << " and word = " << bRecogBottle.get(i).toString() << " and level = " << i_deep << std::endl;
						}
						recogFromGrammarSemantic(*bRecogBottle.get(i).asList(), s_deep, i_deep_cp);


                     }

                } else {
                    yError() << "None possible case in recogFronGrammarSemantic : something is wrong (Bottle from SpeechRecog?)" ;

                }

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


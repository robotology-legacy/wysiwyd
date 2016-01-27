/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Gregoire Pointeau, Maxime Petit
* email:   gregoire.pointeau@inserm.fr, m.petit@imperial.ac.uk
* website: http://wysiwyd.upf.edu/
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
#include "wrdac/subsystems/subSystem_ABM.h"

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
                ABMconnected = (SubABM->Connect());
                yInfo() << ((ABMconnected) ? "Recog connected to ABM" : "Recog didn't connect to ABM");
                if (yarp::os::Network::connect(ears_port.getName(), "/ears/rpc")) {
                    yInfo() << "Recog connected to ears";
                }  else {
                    yWarning() << "Recog didn't connect to ears";
                }
                return yarp::os::Network::connect(portRPC.getName(), "/speechRecognizer/rpc");
            }
            SubSystem_ABM* SubABM;
            std::string speakerName_;
            yarp::os::Port ears_port;

        public:

            yarp::os::Port portRPC;
            SubSystem_Recog(const std::string &masterName) : SubSystem(masterName){
                portRPC.open(("/" + m_masterName + "/recog:rpc").c_str());
                ears_port.open("/" + m_masterName + "/ears:o");
                m_type = SUBSYSTEM_RECOG;
                SubABM = new SubSystem_ABM(m_masterName+"/from_recog");
            }


            virtual void Close() {
                portRPC.interrupt();
                portRPC.close();
                ears_port.interrupt();
                ears_port.close();
                SubABM->Close();
            };

            /**
            * Set the speaker name to be sent as argument to abm when snapshot
            *
            */
            bool setSpeakerName(std::string speaker)
            {
                speakerName_ = speaker ;
                yInfo() << " [subSystem_Recog] : speaker is now " << speakerName_ ;
                return true;
            }

            void listen(bool on) {
                if (!yarp::os::Network::isConnected(ears_port.getName(), "/ears/rpc")){
                    yarp::os::Network::connect(ears_port.getName(), "/ears/rpc");
                }
                if (yarp::os::Network::isConnected(ears_port.getName(), "/ears/rpc")) {
                    yarp::os::Bottle cmd, reply;
                    cmd.addString("listen");
                    if (on) {
                        cmd.addString("on");
                    } else {
                        cmd.addString("off");
                    }
                    yDebug() << "Listen sending command " << cmd.toString();
                    ears_port.write(cmd, reply);                 
                    yDebug() << "Listen got reply" << reply.toString();
                }
            }

            /**
            * From one grxml grammar, return the sentence recognized for one timeout
            *
            */
            yarp::os::Bottle recogFromGrammar(std::string &sInput)
            {
                if (!yarp::os::Network::isConnected(portRPC.getName(), "/speechRecognizer/rpc")){
                    if (!yarp::os::Network::connect(portRPC.getName(), "/speechRecognizer/rpc")){
                        yarp::os::Bottle bReply;
                        bReply.addInt(0);
                        bReply.addString("recog not connected");
                        yWarning(" recog not connected");
                        return bReply;
                    }
                }
                // turn on the main grammar through ears
                listen(false);

                yarp::os::Bottle bMessenger;
                yarp::os::Bottle bReply;
                bMessenger.addString("recog");
                bMessenger.addString("grammarXML");
                bMessenger.addString(sInput);
                portRPC.write(bMessenger, bReply);

                listen(true);

                return bReply;
                // turn off the main grammar through ears
            }

            /**
            *   From one grxml grammar, return the first sentence non-empty recognized
            *   can last for several timeout (by default 50
            */
            yarp::os::Bottle recogFromGrammarLoop(std::string sInput, int iLoop = 50, bool isEars = false)
            {
                if (!yarp::os::Network::isConnected(portRPC.getName(), "/speechRecognizer/rpc")){
                    if (!yarp::os::Network::connect(portRPC.getName(), "/speechRecognizer/rpc")){
                        yarp::os::Bottle bReply;
                        bReply.addInt(0);
                        bReply.addString("recog not connected");
                        yWarning(" recog not connected");
                        return bReply;
                    }
                }
                std::ostringstream osError;
                bool fGetaReply = false;
                yarp::os::Bottle bMessenger, //to be send TO speech recog
                    bAnswer,
                    bReply, //response from speech recog without transfer information, including raw sentence
                    bOutput; // semantic information of the content of the recognition

                bMessenger.addString("recog");
                bMessenger.addString("grammarXML");
                bMessenger.addString(sInput);

                int loop;
                (iLoop == -1) ? loop = -3 : loop = 0;
                
                // listen off

                while (!fGetaReply && loop < iLoop)
                {
                    // turn on the main grammar through ears
                    if (!isEars) 
                        listen(false);  
                    
                    // send the message
                    portRPC.write(bMessenger, bReply);

                    // turn on the main grammar through ears
                      

                    yInfo() << " Reply from Speech Recog : " << bReply.toString();

                    if (bReply.toString() == "NACK" || bReply.size() < 2)
                    {
                        bOutput.addInt(0);
                        osError << "Check grammar";
                        bOutput.addString(osError.str());
                        yError() << " " << osError.str();
                        if (!isEars) listen(true);
                        return bOutput;
                    }else if (bReply.get(0).toString() == "0")
                    {
                        bOutput.addInt(0);
                        osError << "Grammar not recognized";
                        bOutput.addString(osError.str());
                        yInfo() << " " << osError.str();
                        if (!isEars) listen(true);
                        return bOutput;
                    }else if (bReply.get(0).toString()=="ACK")
                    {
                        if (bReply.get(1).toString() == "-1")
                        {
                            yInfo()<< "Only found Garbage...";
                            yDebug()<< "Check Why this happens";
                        }else{
                            yInfo()<< "Sentence Acknowledged";
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
                                    //add speaker name. name should be sent through fonction before
                                    if(speakerName_.empty()){
                                        speakerName_ = "partner";
                                        yWarning() << " [subSystem_Recog] " << "name of the speaker has been assigned to the default value : " << speakerName_ ;
                                    }
                                    lArgument.push_back(std::pair<std::string, std::string>(speakerName_, "speaker"));
                                    SubABM->sendActivity("action",
                                        "sentence",
                                        "recog",
                                        lArgument,
                                        true);
                                }
                            }
                        }

                    }
                    if (iLoop != -1)
                        loop++;
                }

                if (!fGetaReply)
                {
                    bOutput.addInt(0);
                    osError << "no vocal input";
                    bOutput.addString(osError.str());
                    yError() << " " << osError.str();
                }
                if (!isEars) listen(true);
                return bOutput;
            }

        };


    }
}//Namespace
#endif


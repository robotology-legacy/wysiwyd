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
            virtual bool connect();
            SubSystem_ABM* SubABM;
            std::string speakerName_;
            yarp::os::Port ears_port;
            yarp::os::Port portRPC;

        public:
            SubSystem_Recog(const std::string &masterName);

            virtual void Close();

            /**
            * Set the speaker name to be sent as argument to abm when snapshot
            *
            */
            bool setSpeakerName(std::string speaker);

            void listen(bool on);

            void waitForEars();

            bool interruptSpeechRecognizer();

            /**
            * From one grxml grammar, return the sentence recognized for one timeout
            *
            */
            yarp::os::Bottle recogFromGrammar(std::string &sInput);

            /**
            *   From one grxml grammar, return the first sentence non-empty recognized
            *   can last for several timeout (by default 50
            */
            yarp::os::Bottle recogFromGrammarLoop(std::string sInput, int iLoop = 50, bool isEars = false, bool forwardABM = true);

        };


    }
}//Namespace
#endif


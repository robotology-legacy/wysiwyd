/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Stéphane Lallée
* email:   stephane.lallee@gmail.com
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

#ifndef __EFAA_SUBSYSTEM_SPEECH_H__
#define __EFAA_SUBSYSTEM_SPEECH_H__

#define SUBSYSTEM_SPEECH        "speech"
#define SUBSYSTEM_SPEECH_ESPEAK "speech_espeak"

#include "wrdac/subsystems/subSystem.h"
#include "wrdac/subsystems/subSystem_ABM.h"
#include "wrdac/clients/opcClient.h"
#include "wrdac/functions.h"
#include <iostream>
#include <iterator>
#include <algorithm>

namespace wysiwyd{
    namespace wrdac{

        /**
        * \ingroup wrdac_clients
        *
        * Abstract subSystem for speech management (both TTS and STT )
        */
        class SubSystem_Speech : public SubSystem
        {
        protected:
            yarp::os::Port tts;
            yarp::os::Port ttsRpc;
            yarp::os::BufferedPort<yarp::os::Bottle> stt;
            yarp::os::Port sttRpc;


            bool ABMconnected;
            SubSystem_ABM* SubABM;
            OPCClient *opc;

        public:

            SubSystem_Speech(const std::string &masterName);
            virtual bool connect();

            unsigned int countWordsInString(std::string const& str);

            /**
            * Produce text to speech output
            * @param text The text to be said.
            * @param shouldWait Is the function blocking until the end of the sentence or not.
            */
            virtual void TTS(const std::string &text, bool shouldWait = true, bool recordABM = true, std::string addressee = "none");

            /**
            * (todo) Recognize a specific sentence or a grammar through a blocking call
            * @param grammar The grammar to be recognized.
            * @param timeout Timeout for recognition (<0 value means wait until something is recognized).
            * @return The sentence recognized
            */
            virtual yarp::os::Bottle* STT(const std::string &grammar, double timeout = -1);

            /**
            * Read input from the speech recognizer runtime grammar
            * @param isBlocking Should we wait for a sentence?
            * @return The sentence recognized
            */
            virtual yarp::os::Bottle* STT(bool isBlocking);

            /**
            * Flush the pending reads by consuming all of them.
            */
            void STTflush();

            /**
            * Add a word to a given vocabulory
            * @param vocabuloryName The name of the vocabulory to expand
            * @param word The word to be added to this vocabulory
            */
            virtual void STT_ExpandVocabulory(const std::string &vocabuloryName, const std::string &word);;

            /**
            * Set the command line options sent by iSpeak
            * @param custom The options as a string
            */
            void SetOptions(const std::string &custom);

            /**
            * Check if iSpeak is currently speaking
            */
            bool isSpeaking();

            virtual void Close();
        };

        //--------------------------------------------------------------------------------------------

        /**
        * \ingroup wrdac_clients
        *
        * SubSystem for speech synthesis with emotional tuning of speed and pitch using eSpeak
        */
        class SubSystem_Speech_eSpeak : public SubSystem_Speech
        {
        protected:
            int m_speed;
            int m_pitch;

        public:

            SubSystem_Speech_eSpeak(std::string &masterName) :SubSystem_Speech(masterName){

                m_type = SUBSYSTEM_SPEECH_ESPEAK;
                m_speed = 100;
                m_pitch = 50;
            }

            void SetVoiceParameters(int speed = 100, int pitch = 50, std::string voice = "en") {
                speed = (std::max)(80, speed);
                speed = (std::min)(450, speed);
                pitch = (std::max)(0, pitch);
                pitch = (std::min)(99, pitch);
                m_speed = speed;
                m_pitch = pitch;
                std::stringstream ss;
                ss << "-s " << speed << " -p " << pitch << " -v " << voice;
                yarp::os::Bottle param;
                param.addString("set");
                param.addString("opt");
                param.addString(ss.str().c_str());
                ttsRpc.write(param);
            }
        };

    }
}//Namespace
#endif



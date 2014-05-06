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
#include <iostream>
#include <iterator>
#include <algorithm>

namespace wysiwyd{namespace wrdac{

/**
* \ingroup wrdac_clients
*
* Abstract subSystem for speech management (both TTS and STT )
*/
class SubSystem_Speech: public SubSystem
{
protected:
    yarp::os::Port tts;
    yarp::os::Port ttsRpc;
    yarp::os::BufferedPort<yarp::os::Bottle> stt;
    yarp::os::Port sttRpc;

public:
      
    SubSystem_Speech(std::string masterName):SubSystem(masterName)
    {
        tts.open( ("/" + m_masterName + "/tts:o").c_str());
        ttsRpc.open( ("/" + m_masterName + "/tts:rpc").c_str());
        stt.open(("/" + m_masterName + "/stt:i").c_str());
        sttRpc.open(("/" + m_masterName + "/stt:rpc").c_str());
        m_type = SUBSYSTEM_SPEECH;
    }  
    virtual bool connect() 
    { 
        yarp::os::Network::connect("/iSpeak/emotions:o", "/icub/face/emotions/in"); 
        bool connected = yarp::os::Network::connect(tts.getName(), "/iSpeak");
        connected &= yarp::os::Network::connect(ttsRpc.getName(), "/iSpeak/rpc");
        connected &= yarp::os::Network::connect("/speechRecognizer/recog/continuousGrammar:o", stt.getName().c_str());
        connected &= yarp::os::Network::connect(sttRpc.getName().c_str(), "/speechRecognizer/rpc");
        return connected;
    }
 
    unsigned int countWordsInString(std::string const& str)
    {
        std::stringstream stream(str);
        return std::distance(std::istream_iterator<std::string>(stream), std::istream_iterator<std::string>());
    }

    /**
    * Produce text to speech output
    * @param text The text to be said.
    * @param shouldWait Is the function blocking until the end of the sentence or not.
    */ 
    virtual void TTS(std::string text, bool shouldWait=true) {
        yarp::os::Bottle txt; txt.addString(text.c_str());
        tts.write(txt);            
        //int words = countWordsInString(text);
        //double durationMn =  words / (double)m_speed;
        //double durationS = durationMn *60.0;
        //yarp::os::Time::delay(durationS);
        yarp::os::Bottle cmd,reply;
        cmd.addVocab(VOCAB('s','t','a','t'));
        std::string status = "speaking";
        bool speechStarted = false;
        while(shouldWait&&(!speechStarted ||status=="speaking"))
        {
            ttsRpc.write(cmd,reply);
            status = reply.get(0).asString();
            if (!speechStarted && status != "quiet")
            {
                speechStarted = true;
            }
            yarp::os::Time::delay(0.2);
        }
    }
        
    /**
    * (todo) Recognize a specific sentence or a grammar through a blocking call 
    * @param grammar The grammar to be recognized.
    * @param timeout Timeout for recognition (<0 value means wait until something is recognized).
    * @return The sentence recognized
    */ 
    virtual yarp::os::Bottle* STT(std::string grammar, double timeout=-1)
    {
        //todo
        return NULL;
    }

    /**
    * Read input from the speech recognizer runtime grammar
    * @param isBlocking Should we wait for a sentence?
    * @return The sentence recognized
    */ 
    virtual yarp::os::Bottle* STT(bool isBlocking)
    {
        return stt.read(isBlocking);
    }

    /**
    * Add a word to a given vocabulory
    * @param vocabuloryName The name of the vocabulory to expand
    * @param word The word to be added to this vocabulory
    */ 
   virtual void STT_ExpandVocabulory(std::string vocabuloryName, std::string word)
   {
        yarp::os::Bottle bAugmentVocab;
        bAugmentVocab.addString("rgm");
        bAugmentVocab.addString("vocabulory");
        bAugmentVocab.addString("add");
        std::string sVocabTemp = "#";
        sVocabTemp += vocabuloryName.c_str();
        bAugmentVocab.addString(sVocabTemp.c_str());
        bAugmentVocab.addString(word.c_str());
        sttRpc.write(bAugmentVocab);       
   };

    /**
    * Set the command line options sent by iSpeak
    * @param custom The options as a string
    */  
    void SetOptions(std::string custom) {
        yarp::os::Bottle param; 
        param.addString("set");
        param.addString("opt");
        param.addString(custom.c_str());
        ttsRpc.write(param);
    }

    /**
    * Check if iSpeak is currently speaking
    */  
    bool isSpeaking() {
        yarp::os::Bottle cmd,reply;
        cmd.addVocab(VOCAB('s','t','a','t'));            
        ttsRpc.write(cmd,reply);
        return (reply.get(0).asString()!="quiet");
    }

    virtual void Close()
    {
        tts.interrupt();
        tts.close();
        ttsRpc.interrupt();
        ttsRpc.close();
        stt.interrupt();
        stt.close();
        sttRpc.interrupt();
        sttRpc.close();
    }
};

//--------------------------------------------------------------------------------------------

/**
* \ingroup wrdac_clients
*
* SubSystem for speech synthesis with emotional tuning of speed and pitch using eSpeak
*/
class SubSystem_Speech_eSpeak: public SubSystem_Speech
{
protected:
    int m_speed;
    int m_pitch;

public:

    SubSystem_Speech_eSpeak(std::string masterName):SubSystem_Speech(masterName){

        m_type = SUBSYSTEM_SPEECH_ESPEAK;
        m_speed = 100;
        m_pitch = 50;
    }

    void SetVoiceParameters(int speed = 100, int pitch = 50, std::string voice = "en") {
        speed = (std::max)(80,speed);
        speed = (std::min)(450,speed);
        pitch = (std::max)(0,pitch);
        pitch = (std::min)(99,pitch);
        m_speed = speed;
        m_pitch = pitch;
        std::stringstream ss;
        ss<<"-s "<<speed<<" -p "<<pitch<<" -v "<<voice;
        yarp::os::Bottle param; 
        param.addString("set");
        param.addString("opt");
        param.addString(ss.str().c_str());
        ttsRpc.write(param);
    }
};

}}//Namespace
#endif



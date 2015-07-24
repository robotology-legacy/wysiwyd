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

#ifndef __EFAA_FACECODE_H__
#define __EFAA_FACECODE_H__
#include <vector>
#include <algorithm>
#include <map>
#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <sstream>
#include <iostream>
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/subsystems/subSystem_speech.h"

namespace wysiwyd{namespace wrdac{
#define SUBSYSTEM_EXPRESSION "expression"

/**
 * @defgroup facialExpression EFAA Helper Facial Expression
 *
 * @ingroup wrdac
 *
 * A simple structure and a client to manage the LEDs on the iCub face
 *
 * @author Stéphane Lallée
 */

/**
* \ingroup facialExpression
*
* Represent a LED configuration for the face.
*/
class FaceLED
{
private:
    static int getLEDDecimalValue(int i)
    {
        return 0x01<<i;
    }

    static std::string getHexCode(char startLetter, int decimalSum)
    {
        std::ostringstream codeValue;
        codeValue<< startLetter;
        if (decimalSum<16)
            codeValue<<'0';
        codeValue<<std::hex << decimalSum;
        return codeValue.str();
    }

    std::vector<bool> mouth;
    std::vector<bool> lEye;
    std::vector<bool> rEye;
    double            eyeLids;
    int               eyeLidsMin, eyeLidsMax;
public:

    FaceLED()
    {
        mouth.resize(6,false);
        lEye.resize(4,false);
        rEye.resize(4,false);
        eyeLids = 1.0;
        eyeLidsMin = 40;
        eyeLidsMax = 60;
    }
        
    /**
    * Define the robot specific range for the eyelids
    */
    void DefineEyelidsRange(int min, int max)
    {
        eyeLidsMin = min;
        eyeLidsMax = max;
    }

    /**
    * Define all the mouth LED status
    */
    void SetMouthLED(bool L1, bool L2, bool L3, bool L4, bool L5, bool L6)
    {
        mouth[0] = L1;
        mouth[1] = L2;
        mouth[2] = L3;
        mouth[3] = L4;
        mouth[4] = L5;
        mouth[5] = L6;
    }

    /**
    * Define the status of a single LED of the mouth
    */
    void SetMouthLED(int i, bool status)
    {
        mouth[i] = status;
    }

    /**
    * Define all the left eyebrow LED status
    */
    void SetLEyeLED(bool L1, bool L2, bool L3, bool L4)
    {
        lEye[0] = L1;
        lEye[1] = L2;
        lEye[2] = L3;
        lEye[3] = L4;
    }

    /**
    * Define the status of a single LED of the left eyebrow
    */
    void SetLEyeLED(int i, bool status)
    {
        lEye[i] = status;
    }

    /**
    * Define all the right eyebrow LED status
    */
    void SetREyeLED(bool L1, bool L2, bool L3, bool L4)
    {
        rEye[0] = L1;
        rEye[1] = L2;
        rEye[2] = L3;
        rEye[3] = L4;
    }

    /**
    * Define the status of a single LED of the left eyebrow
    */
    void SetREyeLED(int i, bool status)
    {
        rEye[i] = status;
    }

    /**
    * Define the status of a single LED of the left eyebrow
    */
    void SetEyeOpening(double i)
    {
        eyeLids = (std::max)(0.0,(std::min)(1.0,i));
    }

    /**
    * return the raw hexadecimal code for the mouth
    */
    std::string mouthCode()
    {
        int hexSum = 0;
        for(int i=0;i<6;i++)
        {
            if (mouth[i])
                hexSum += getLEDDecimalValue(i);
        }
        std::string code = getHexCode('M',hexSum);
        std::transform(code.begin(), code.end(),code.begin(), ::toupper);
        return code;
    }

    /**
    * return the raw hexadecimal code for the right eye
    */
    std::string rightEyeCode()
    {
        int hexSum = 0;
        for(int i=0;i<4;i++)
        {
            if (rEye[i])
                hexSum += getLEDDecimalValue(i);
        }
        std::string code = getHexCode('R',hexSum);
        std::transform(code.begin(), code.end(),code.begin(), ::toupper);
        return code;
    }

    /**
    * return the raw hexadecimal code for the left eye
    */
    std::string leftEyeCode()
    {
        int hexSum = 0;
        for(int i=0;i<4;i++)
        {
            if (lEye[i])
                hexSum += getLEDDecimalValue(i);
        }

        std::string code = getHexCode('L',hexSum);
        std::transform(code.begin(), code.end(),code.begin(), ::toupper);
        return code;
    }
    /**
    * return the raw hexadecimal code for the eyes opening
    */
    std::string eyesOpeningCode()
    {   int maxValue = eyeLidsMax;
        int minValue = eyeLidsMin;
        int scaledValue = (int)(minValue + (maxValue - minValue) * eyeLids);
        std::stringstream strstr;
        strstr<<'S'<<scaledValue;
        std::string code = strstr.str(); //getHexCode('S',scaledValue);
        //std::transform(code.begin(), code.end(),code.begin(), ::toupper);
        return code;
    }
};

/**
* \ingroup facialExpression
*
* Client to send FaceLED configurations to the robot.
*/
class FaceLEDClient
{
private:
    yarp::os::Port toEmotionInterface;

public:
    FaceLEDClient(yarp::os::ResourceFinder &rf)
    {
        std::string moduleName = rf.check("name",yarp::os::Value("faceLED")).asString().c_str();
        std::string portName = "/";
        portName +=moduleName.c_str();
        portName +="/emotions:o";
        toEmotionInterface.open(portName.c_str());
        connect();
    }

    FaceLEDClient(const std::string &moduleName)
    {
        std::string portName = "/";
        portName +=moduleName.c_str();
        portName +="/emotions:o";
        toEmotionInterface.open(portName.c_str());
    }

    /**
    * Interrupt port communication
    */
    void interrupt()
    {
        toEmotionInterface.interrupt();
    }

    /**
    * Close port
    */
    void close()
    {
        toEmotionInterface.close();
    }
    /**
    * Try to connect to the emotion interface port ("/icub/face/emotions/in")
    */
    bool connect(const std::string &targetPort = "/icub/face/emotions/in")
    {
        return yarp::os::Network::connect(toEmotionInterface.getName(),targetPort.c_str());
    }

    /**
    * Make the iCub blink
    */
    void blink(double blink_time = 0.0750)
    {
        FaceLED led;
        led.SetEyeOpening(0);
        send(led, false, false, false , true);
        yarp::os::Time::delay(blink_time);
        led.SetEyeOpening(1);
        send(led, false, false, false , true);
    }

    /**
    * Send a facial expression to be displayed
    * @param face The FaceLED configuration to be displayed
    * @param sendMouth Should the mouth be updated (default=true)
    * @param sendEyeBrowR Should the right eybrow be updated (default=true)
    * @param sendEyeBrowL Should the left eybrow be updated (default=true)
    * @param sendEyeOpening Should the eyelids be updated (default=true)
    */
    void send(FaceLED face, bool sendMouth = true, bool sendEyeBrowR = true, bool sendEyeBrowL = true, bool sendEyeOpening = true)
    {
        yarp::os::Bottle cmd;

        //Set the mouth
        if (sendMouth)
        {
            cmd.addString("set");
            cmd.addString("raw");
            cmd.addString(face.mouthCode().c_str());
            toEmotionInterface.write(cmd);
            //cout<<"Sending..."<<cmd.toString().c_str()<<endl;
        }

        //lEye
        if (sendEyeBrowL)
        {
            cmd.clear();
            cmd.addString("set");
            cmd.addString("raw");
            cmd.addString(face.leftEyeCode().c_str());
            toEmotionInterface.write(cmd);
            //cout<<"Sending..."<<cmd.toString().c_str()<<endl;
        }

        //rEye
        if (sendEyeBrowR)
        {
            cmd.clear();
            cmd.addString("set");
            cmd.addString("raw");
            cmd.addString(face.rightEyeCode().c_str());
            toEmotionInterface.write(cmd);
            //cout<<"Sending..."<<cmd.toString().c_str()<<endl;
        }

        //opening
        if (sendEyeOpening)
        {
            cmd.clear();
            cmd.addString("set");
            cmd.addString("raw");
            cmd.addString(face.eyesOpeningCode().c_str());
            toEmotionInterface.write(cmd);
            //cout<<"Sending..."<<cmd.toString().c_str()<<endl;
        }
    }
};

/**
* \ingroup facialExpression
*
* Structure to store multiple facial expressions corresponding to various levels
* of an emotion.
*/
struct FacialEmotion
{
    std::map<double,FaceLED> intensity;

    /**
    * Return the closest template for a given intensity
    * @param value The current emotional intensity.
    */
    FaceLED getClosestFace(double value)
    {
        std::map<double,FaceLED>::iterator bestChoice = intensity.begin();
        for(std::map<double,FaceLED>::iterator it = intensity.begin(); it!=intensity.end(); it++)
        {
            if ( fabs(value - it->first) < fabs(value - bestChoice->first) )
                bestChoice = it;
        }
        return bestChoice->second;
    }
};

/**
* \ingroup facialExpression
*
* Structure to store a voice pattern
*/
struct VoiceParameters
{
    int pitch;
    int speed;
};

/**
* \ingroup facialExpression
*
* Structure to store multiple voice parameters corresponding to various levels
* of an emotion.
*/
struct VoiceEmotion
{
    std::map<double,VoiceParameters> intensity;

    /**
    * Return the closest template for a given intensity
    * @param value The current emotional intensity.
    */
    VoiceParameters getClosestVoice(double value)
    {
        std::map<double,VoiceParameters>::iterator bestChoice = intensity.begin();
        for(std::map<double,VoiceParameters>::iterator it = intensity.begin(); it!=intensity.end(); it++)
        {
            if ( fabs(value - it->first) < fabs(value - bestChoice->first) )
                bestChoice = it;
        }
        return bestChoice->second;
    }
};

/**
* \ingroup wrdac_clients
*
* A client to control the face expression using an emotion name and its intensity.
*/
class SubSystem_Expression :public FaceLEDClient, public SubSystem
{
private:
    std::map<std::string, FacialEmotion> emotions;
    std::map<std::string, VoiceEmotion> voices;
    std::string voiceFileUsed;

public:
    SubSystem_Expression(const std::string &masterName, yarp::os::ResourceFinder &rf) :FaceLEDClient(rf), SubSystem(masterName)
    {
        LoadExpressions(rf);
    }

    SubSystem_Expression(const std::string &moduleName, bool isRFVerbose = false) :FaceLEDClient(moduleName), SubSystem(moduleName)
    {
        yarp::os::ResourceFinder rf;
        rf.setVerbose(isRFVerbose);
        rf.setDefaultContext("facialEmotions");
        rf.setDefaultConfigFile("defaultFaces.ini");
        rf.configure(0,NULL);
        LoadExpressions(rf);
    }
      
    /**
    * Get the list of loaded emotions
    */
    std::list<std::string> knownEmotions()
    {
        std::list<std::string> ems; 
        for(std::map<std::string, FacialEmotion>::iterator it = emotions.begin(); it != emotions.end(); it++)
        {
            ems.push_back(it->first);
        }
        return ems;
    }
    /**
    * Express a given emotion to a given intensity
    */
    bool express(const std::string &emotion, double intensity, SubSystem_Speech_eSpeak* voice = NULL, const std::string &overrideVoice = "default")
    {
        if (emotions.find(emotion) != emotions.end())
        {
            //send face
            std::cout<<"Expressing : "<<emotion<<" ("<<intensity<<")"<<std::endl;
            FaceLED face = emotions[emotion].getClosestFace(intensity);
            this->FaceLEDClient::send(face);

            //set voice
            if (voice != NULL)
            {
                VoiceParameters v = voices[emotion].getClosestVoice(intensity);
                if (overrideVoice != "default")
                    voice->SetVoiceParameters(v.speed,v.pitch,overrideVoice);
                else
                    voice->SetVoiceParameters(v.speed,v.pitch,voiceFileUsed);
            }

            return true;
        }
        else
        {
            std::cerr<<"Asked emotion ("<<emotion<<") does not exist."<<std::endl;
            return false;
        }
    }

    /**
    * Load predefined set of expressions assigned to an emotional value.
    */
    void LoadExpressions(yarp::os::ResourceFinder &rf)
    {
        int emotionCount = rf.check("emotionsCount",yarp::os::Value(0)).asInt();
        int minEyeOpening = rf.check("minEyeOpening",yarp::os::Value(64)).asInt();
        int maxEyeOpening = rf.check("maxEyeOpening",yarp::os::Value(90)).asInt();
        voiceFileUsed = rf.check("voice",yarp::os::Value("en")).asString().c_str();
        
        for(int i=1;i<=emotionCount;i++)
        {
            std::ostringstream groupName;
            groupName << "emotion_"<<i;
            yarp::os::Bottle &bEmotion = rf.findGroup( groupName.str().c_str() );
            std::string emotionName = bEmotion.find("emoName").asString().c_str();
            //std::cout<<"Loading emotion "<<emotionName<<std::endl;
            int checkPoints = bEmotion.find("checkPointCount").asInt();
            for(int c=1;c<=checkPoints;c++)
            {
                std::ostringstream sValue; sValue << c << "_value";
                std::ostringstream sMouth; sMouth << c << "_mouth";
                std::ostringstream slEye; slEye << c << "_lEye";
                std::ostringstream srEye; srEye << c << "_rEye";
                std::ostringstream sEyeOpening;sEyeOpening << c << "_eyeOpening";
                std::ostringstream sVoiceSpeed;sVoiceSpeed << c << "_voiceSpeed";
                std::ostringstream sVoicePitch;sVoicePitch << c << "_voicePitch";

                double cValue = bEmotion.find(sValue.str().c_str()).asDouble();
                yarp::os::Bottle* cMouth = bEmotion.find(sMouth.str().c_str()).asList();
                yarp::os::Bottle* crEye = bEmotion.find(srEye.str().c_str()).asList();
                yarp::os::Bottle* clEye = bEmotion.find(slEye.str().c_str()).asList();
                double cEyeOpening = bEmotion.find(sEyeOpening.str().c_str()).asDouble();
                int cVoiceSpeed = bEmotion.find(sVoiceSpeed.str().c_str()).asInt();
                int cVoicePitch = bEmotion.find(sVoicePitch.str().c_str()).asInt();

                //std::cout<<"\t Mouth : \t"<<cMouth->toString().c_str()<<std::endl;
                //std::cout<<"\t R Eyebrow : \t"<<crEye->toString().c_str()<<std::endl;
                //std::cout<<"\t L Eyebrow : \t"<<clEye->toString().c_str()<<std::endl;
                //std::cout<<"\t Eye Opening : \t"<<cEyeOpening<<std::endl;
                //std::cout<<"\t Voice Speed : \t"<<cVoiceSpeed<<std::endl;
                //std::cout<<"\t Voice Pitch : \t"<<cVoicePitch<<std::endl;

                //voice
                voices[emotionName.c_str()].intensity[cValue].speed = cVoiceSpeed;
                voices[emotionName.c_str()].intensity[cValue].pitch = cVoicePitch;

                //Set the mouth values
                for(int m=0;m<6;m++)
                    emotions[emotionName.c_str()].intensity[cValue].SetMouthLED(m, cMouth->get(m).asInt() == 1);

                //Set Eyebrows
                for(int m=0;m<4;m++)
                {
                    emotions[emotionName.c_str()].intensity[cValue].SetLEyeLED(m, clEye->get(m).asInt() == 1);
                    emotions[emotionName.c_str()].intensity[cValue].SetREyeLED(m, crEye->get(m).asInt() == 1);
                }
                //Eye opening
                emotions[emotionName.c_str()].intensity[cValue].DefineEyelidsRange(minEyeOpening,maxEyeOpening);
                emotions[emotionName.c_str()].intensity[cValue].SetEyeOpening(cEyeOpening);
            }
        }
    }

    virtual void Close() { this->FaceLEDClient::interrupt(); this->FaceLEDClient::close(); }
    virtual bool connect() { return this->FaceLEDClient::connect(); }

};

}}
#endif


#include <yarp/os/all.h>
#include "wrdac/subsystems/subSystem_facialExpression.h"

wysiwyd::wrdac::SubSystem_Expression::SubSystem_Expression(const std::string &masterName, yarp::os::ResourceFinder &rf) :FaceLEDClient(rf), SubSystem(masterName)
{
    LoadExpressions(rf);
}

wysiwyd::wrdac::SubSystem_Expression::SubSystem_Expression(const std::string &moduleName, bool isRFVerbose) :FaceLEDClient(moduleName), SubSystem(moduleName)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(isRFVerbose);
    rf.setDefaultContext("facialEmotions");
    rf.setDefaultConfigFile("defaultFaces.ini");
    rf.configure(0,NULL);
    LoadExpressions(rf);
}

std::list<std::string> wysiwyd::wrdac::SubSystem_Expression::knownEmotions()
{
    std::list<std::string> ems;
    for(std::map<std::string, FacialEmotion>::iterator it = emotions.begin(); it != emotions.end(); it++)
    {
        ems.push_back(it->first);
    }
    return ems;
}

bool wysiwyd::wrdac::SubSystem_Expression::express(const std::string &emotion, double intensity, wysiwyd::wrdac::SubSystem_Speech_eSpeak *voice, const std::string &overrideVoice)
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

void wysiwyd::wrdac::SubSystem_Expression::LoadExpressions(yarp::os::ResourceFinder &rf)
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

void wysiwyd::wrdac::SubSystem_Expression::Close() { this->FaceLEDClient::interrupt(); this->FaceLEDClient::close(); }

bool wysiwyd::wrdac::SubSystem_Expression::connect() { return this->FaceLEDClient::connect(); }

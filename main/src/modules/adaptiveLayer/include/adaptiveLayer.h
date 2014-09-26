#include "wrdac/clients/icubClient.h"
//#include <std::map>

struct StimulusEmotionalResponse
{
    std::vector<std::string> m_sentences;
    std::vector<std::string> m_choregraphies;
    std::map<std::string, double> m_emotionalEffect;
    std::string getRandomSentence() { if (m_sentences.size()==0) return ""; return m_sentences[yarp::os::Random::uniform(0,m_sentences.size()-1)];}
    std::string getRandomChoregraphy() { if(m_choregraphies.size()==0) return "default"; return m_choregraphies[yarp::os::Random::uniform(0,m_choregraphies.size()-1)];}
};

class AdaptiveLayer : public yarp::os::RFModule
{
private:
    wysiwyd::wrdac::ICubClient *iCub;

    yarp::os::Port pSpeechRecognizerKeywordOut;

    double period;
    std::map<std::string, StimulusEmotionalResponse> gestureEffects;
    yarp::os::Port    rpc;

    //Configuration
    void populateSpeechRecognizerVocabulary();
    void configureOPC(yarp::os::ResourceFinder &rf);
    void configureSpeech(yarp::os::ResourceFinder &rf);
    void configureGestures(yarp::os::ResourceFinder &rf);


public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule()
    {
        return true;
    }

    bool close()
    {
        iCub->close();
        delete iCub;
        return true;
    }

    double getPeriod()
    {
        return period;
    }

    bool updateModule();


    //Retrieve and treat the speech input
    bool handleSpeech();
    wysiwyd::wrdac::Relation getRelationFromSemantic(yarp::os::Bottle b);
    std::string getEntityFromWordGroup(yarp::os::Bottle *b);

    //Retrieve and treat the gesture information input
    bool handleGesture();

    //RPC & scenarios
    bool respond(const yarp::os::Bottle &cmd, yarp::os::Bottle &reply);

};

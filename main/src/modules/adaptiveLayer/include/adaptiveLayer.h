#include "wrdac/clients/icubClient.h"
#include <map>

using namespace std;
using namespace yarp::os;
using namespace yarp::math;
using namespace wysiwyd::wrdac;

struct StimulusEmotionalResponse
{
    vector<string> m_sentences;
    vector<string> m_choregraphies;
    map<string, double> m_emotionalEffect;
    string getRandomSentence() { if (m_sentences.size()==0) return ""; return m_sentences[yarp::os::Random::uniform(0,m_sentences.size()-1)];}
    string getRandomChoregraphy() { if(m_choregraphies.size()==0) return "default"; return m_choregraphies[yarp::os::Random::uniform(0,m_choregraphies.size()-1)];}
};

class AdaptiveLayer : public RFModule
{
private:
    ICubClient *iCub;

    Port pSpeechRecognizerKeywordOut;

    double period;
    map<string, StimulusEmotionalResponse> gestureEffects;
    Port    rpc;

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
    Relation getRelationFromSemantic(Bottle b);
    string getEntityFromWordGroup(Bottle *b);

    //Retrieve and treat the gesture information input
    bool handleGesture();

    //RPC & scenarios
    bool respond(const Bottle& cmd, Bottle& reply);

};

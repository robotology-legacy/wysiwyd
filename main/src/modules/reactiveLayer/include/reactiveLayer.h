#include <string>
#include <iostream>
#include <iomanip>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/SVD.h>
#include "wrdac/clients/icubClient.h"
#include <map>
#include "internalVariablesDecay.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
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

class ReactiveLayer: public RFModule
{
private:
    ICubClient *iCub;

    double period;
    double salutationLifetime, lastResponseTime;
    double faceUpdatePeriod, lastFaceUpdate;

	InternalVariablesDecay* decayThread;

	//Drive triggers
	bool physicalInteraction;
	bool someonePresent;

	//Reflexes
	map<string, StimulusEmotionalResponse> salutationEffects;
	map<string, StimulusEmotionalResponse> tactileEffects;
	map<string, StimulusEmotionalResponse> homeostaticOverEffects;
	map<string, StimulusEmotionalResponse> homeostaticUnderEffects;

    Port    rpc;

    //Configuration
	void configureOPC(yarp::os::ResourceFinder &rf);
	void configureAllostatic(yarp::os::ResourceFinder &rf);
	void configureTactile(yarp::os::ResourceFinder &rf);
	void configureSalutation(yarp::os::ResourceFinder &rf);

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

	//Check for newcomers and salute them if required
	bool handleSalutation(bool& someoneIsPresent);

    //Retrieve and treat the tactile information input
    bool handleTactile();

    //Retrieve and treat the gesture information input
    bool handleGesture();

	//Update the drives accordingly to the stimuli
	bool updateAllostatic();

	//Express emotions
	bool updateEmotions();

	//RPC & scenarios
	bool respond(const Bottle& cmd, Bottle& reply);
};

#include "wrdac/clients/icubClient.h"
//#include <std::map>

enum Role
{
	Subject = 0,
	Verb = 1,
	Object = 2,
	Place = 3,
	Time = 4,
	Manner = 5,
	Undefined = 6
};

class GameCluedo : public yarp::os::RFModule
{
private:
    wysiwyd::wrdac::ICubClient *iCub;

    yarp::os::Port pSpeechRecognizerKeywordOut;

    double period;
    yarp::os::Port    rpc;

    //Configuration
    void populateSpeechRecognizerVocabulary();
    void configureOPC(yarp::os::ResourceFinder &rf);
    void configureSpeech(yarp::os::ResourceFinder &rf);

	bool gameStarted;
	bool isMyTurn;

	std::list< wysiwyd::wrdac::Relation > myGrid;
	std::list< wysiwyd::wrdac::Relation > hisGrid;

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

	bool findPartialRelation(wysiwyd::wrdac::Relation* incompleteRelation, Role &missingRole)
	{
		incompleteRelation = NULL;
		missingRole = Role::Undefined;

		for (std::list<wysiwyd::wrdac::Relation>::iterator it = myGrid.begin(); it != myGrid.end(); it++)
		{
			if (it->subject() == "?")
			{
				incompleteRelation = &(*it);
				missingRole = Role::Subject;
				return true;
			}
			if (it->verb() == "?")
			{
				incompleteRelation = &(*it);
				missingRole = Role::Verb;
				return true;
			}
			if (it->object() == "?")
			{
				incompleteRelation = &(*it);
				missingRole = Role::Object;
				return true;
			}
			if (it->complement_place() == "?")
			{
				incompleteRelation = &(*it);
				missingRole = Role::Place;
				return true;
			}
			if (it->complement_time() == "?")
			{
				incompleteRelation = &(*it);
				missingRole = Role::Time;
				return true;
			}
			if (it->complement_manner() == "?")
			{
				incompleteRelation = &(*it);
				missingRole = Role::Manner;
				return true;
			}
		}
		return false;
	}

	std::string formQuestionFromRelation(wysiwyd::wrdac::Relation* r, Role roleToQuestion)
	{
		std::string question = "";
		switch (roleToQuestion)
		{
		case Role::Subject:
			question += "Who did ";
			question += r->verb();
			question += " the ";
			question += r->object();
			question += " in the ";
			question += r->complement_place();
			question += " during the ";
			question += r->complement_time();
			question += "?";
			break;
		case Role::Verb:
			question += "What did ";		
			question += r->subject();
			question += " do with the ";
			question += r->object();
			question += " in the ";
			question += r->complement_place();
			question += " during the ";
			question += r->complement_time();
			question += "?";
			break;
		case Role::Object:			
			question += "What did ";
			question += r->subject();
			question += r->verb();
			question += " in the ";
			question += r->complement_place();
			question += " during the ";
			question += r->complement_time();
			question += "?";
			break;
		case Role::Place:			
			question += "Where did ";
			question += r->subject();
			question += r->verb();
			question += " the ";
			question += r->object();
			question += " during the ";
			question += r->complement_time();
			question += "?";
			break;
		case Role::Time:
			question += "When did ";
			question += r->subject();
			question += r->verb();
			question += " the ";
			question += r->object();
			question += " in the ";
			question += r->complement_place();
			question += "?";
			break;
		case Role::Manner:
			question += "How did ";
			question += r->subject();
			question += r->verb();
			question += " the ";
			question += r->object();
			question += " in the ";
			question += r->complement_place();
			question += " during the ";
			question += r->complement_time();
			question += "?";
			break;
		case Role::Undefined:
			question += "What the hell?!";
			break;
		default:
			question += "What the fuck?!";
			break;
		}
		return question;
	}

    //Retrieve and treat the speech input
    bool handleSpeech(bool expectAffirmation);
    wysiwyd::wrdac::Relation getRelationFromSemantic(yarp::os::Bottle b);
    std::string getEntityFromWordGroup(yarp::os::Bottle *b);

    //RPC
    bool respond(const yarp::os::Bottle &cmd, yarp::os::Bottle &reply);

};

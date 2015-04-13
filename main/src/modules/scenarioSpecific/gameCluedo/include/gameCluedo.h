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
const double PERIOD_REENGAGE = 2.0;

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

    double timeReengagment;


    bool isMyTurn;

    std::list< wysiwyd::wrdac::Relation > myIncompleteBeliefs;
    std::ofstream logFile;
    double startTime;

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

    void log(std::string msg)
    {
        std::stringstream fMsg;
        fMsg << yarp::os::Time::now() - startTime << '\t' << msg << std::endl;
        std::cout << fMsg.str();
        logFile << fMsg.str();
        logFile.flush();
    }

    bool findPartialRelation(wysiwyd::wrdac::Relation* &incompleteRelation, Role &missingRole)
    {
        incompleteRelation = NULL;
        missingRole = Undefined;

        for (std::list<wysiwyd::wrdac::Relation>::iterator it = myIncompleteBeliefs.begin(); it != myIncompleteBeliefs.end(); it++)
        {
            if (it->subject() == "?")
            {
                incompleteRelation = &(*it);
                missingRole = Subject;
                return true;
            }
            if (it->verb() == "?")
            {
                incompleteRelation = &(*it);
                missingRole = Verb;
                return true;
            }
            if (it->object() == "?")
            {
                incompleteRelation = &(*it);
                missingRole = Object;
                return true;
            }
            if (it->complement_place() == "?")
            {
                incompleteRelation = &(*it);
                missingRole = Place;
                return true;
            }
            if (it->complement_time() == "?")
            {
                incompleteRelation = &(*it);
                missingRole = Time;
                return true;
            }
            if (it->complement_manner() == "?")
            {
                incompleteRelation = &(*it);
                missingRole = Manner;
                return true;
            }
        }
        return false;
    }
    std::string formAffirmationFromRelation(wysiwyd::wrdac::Relation* r)
    {
        std::string s;
        s += r->subject();
        s += " did ";
        s += r->verb();
        s += " the ";
        s += r->object();
        s += " in the ";
        s += r->complement_place();
        s += " during the ";
        s += r->complement_time();
        s += ".";
        return s;
    }

    std::string formQuestionFromRelation(wysiwyd::wrdac::Relation* r, Role roleToQuestion)
    {
        std::string question = "";
        switch (roleToQuestion)
        {
        case Subject:
            question += "Who did ";
            question += r->verb();
            question += " the ";
            question += r->object();
            //question += " in the ";
            //question += r->complement_place();
            //question += " during the ";
            //question += r->complement_time();
            question += "?";
            break;
        case Verb:
            question += "What did ";
            question += r->subject();
            question += " do with the ";
            question += r->object();
            //question += " in the ";
            //question += r->complement_place();
            //question += " during the ";
            //question += r->complement_time();
            question += "?";
            break;
        case Object:
            question += "What did ";
            question += r->subject();
            question += " ";
            question += r->verb();
            question += " in the ";
            question += r->complement_place();
            //question += " during the ";
            //question += r->complement_time();
            question += "?";
            break;
        case Place:
            question += "Where did ";
            question += r->subject();
            question += " ";
            question += r->verb();
            question += " the ";
            question += r->object();
            question += " during the ";
            question += r->complement_time();
            question += "?";
            break;
        case Time:
            question += "When did ";
            question += r->subject();
            question += " ";
            question += r->verb();
            question += " the ";
            question += r->object();
            question += " in the ";
            question += r->complement_place();
            question += "?";
            break;
        case Manner:
            question += "How did ";
            question += r->subject();
            question += " ";
            question += r->verb();
            question += " the ";
            question += r->object();
            question += " in the ";
            question += r->complement_place();
            question += " during the ";
            question += r->complement_time();
            question += "?";
            break;
        case Undefined:
            question += "What the hell?!";
            break;
        default:
            question += "What the fuck?!";
            break;
        }
        return question;
    }
    wysiwyd::wrdac::Relation completePartialRelation(wysiwyd::wrdac::Relation* partialRelation, wysiwyd::wrdac::Relation answer)
    {
        std::string s, v, o, p, t, m;
        s = (partialRelation->subject() == "?") ? answer.subject() : partialRelation->subject();
        v = (partialRelation->verb() == "?") ? answer.verb() : partialRelation->verb();
        o = (partialRelation->object() == "?") ? answer.object() : partialRelation->object();
        p = (partialRelation->complement_place() == "?") ? answer.complement_place() : partialRelation->complement_place();
        t = (partialRelation->complement_time() == "?") ? answer.complement_time() : partialRelation->complement_time();
        m = (partialRelation->complement_manner() == "?") ? answer.complement_manner() : partialRelation->complement_manner();
        return(wysiwyd::wrdac::Relation(s, v, o, p, t, m));
    }

    //Retrieve and treat the speech input
    bool handleSpeech(bool expectAffirmation, wysiwyd::wrdac::Relation* queriedRelation, Role queriedRole);
    wysiwyd::wrdac::Relation getRelationFromSemantic(yarp::os::Bottle b);
    std::string getEntityFromWordGroup(yarp::os::Bottle *b);

    //RPC
    bool respond(const yarp::os::Bottle &cmd, yarp::os::Bottle &reply);

};

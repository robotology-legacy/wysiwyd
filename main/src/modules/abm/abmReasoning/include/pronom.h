#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


class pronom
{   
public:
    string                              sSubject;       // or pronom

    matrix3D                            m3Data;

    pair<int, int>                      pSubject_Is_Agent;
    pair<int, int>                      pSubject_Is_Speaker;
    pair<int, int>                      pSubject_Is_Addressee;

    bool        AddInstance(Bottle bInput);         //  Add an instance to the stats


    // TODO : TO BE REMOVE
    pair<int, int>                      pSpeaker_Is_Addressee;
    pair<int, int>                      pSpeaker_Is_Subject;
    pair<int, int>                      pSpeaker_Is_Agent;

    pair<int, int>                      pAddressee_Is_Subject;
    pair<int, int>                      pAddressee_Is_Agent;

    vector<pair<string, int> >          vAddresseeIs;   //  Vector with the name of addressee and interances
    vector<pair<string, int> >          vAgentIs;   //  Vector with the name of addressee and interances
    vector<pair<string, int> >          vSpeakerIs; //  Vector with the name of addressee and interances

    
};

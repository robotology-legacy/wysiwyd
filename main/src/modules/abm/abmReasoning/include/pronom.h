#include <abmReasoningFunction.h>


class pronom
{   
public:
    std::string                              sSubject;       // or pronom

    matrix3D                            m3Data;

    std::pair<int, int>                      pSubject_Is_Agent;
    std::pair<int, int>                      pSubject_Is_Speaker;
    std::pair<int, int>                      pSubject_Is_Addressee;

    bool        AddInstance(yarp::os::Bottle bInput);         //  Add an instance to the stats


    // TODO : TO BE REMOVE
    std::pair<int, int>                      pSpeaker_Is_Addressee;
    std::pair<int, int>                      pSpeaker_Is_Subject;
    std::pair<int, int>                      pSpeaker_Is_Agent;

    std::pair<int, int>                      pAddressee_Is_Subject;
    std::pair<int, int>                      pAddressee_Is_Agent;

    std::vector<std::pair<std::string, int> >          vAddresseeIs;   //  Vector with the name of addressee and interances
    std::vector<std::pair<std::string, int> >          vAgentIs;   //  Vector with the name of addressee and interances
    std::vector<std::pair<std::string, int> >          vSpeakerIs; //  Vector with the name of addressee and interances
    

};

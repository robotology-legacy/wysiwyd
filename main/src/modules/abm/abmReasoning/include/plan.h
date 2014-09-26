#include <abmReasoningFunction.h>


class plan
{   
public:

    std::string                                          sName;
    std::string                                          sManner;
    std::vector<std::string>                                  vActivitytype;
    std::vector<std::string>                                  vActivityname;
    std::vector< std::list < std::pair < std::string , std::string > > >     vActivityArguments;         // argument of the current activity (expl : icub-agent1 circle-object1 east-spatial1)
    std::vector< std::pair <std::string, std::string> >                 vArguments;                 // std::pair of argument + role of the plan (expl : circle-object1 , icub-agent2 ... )
};


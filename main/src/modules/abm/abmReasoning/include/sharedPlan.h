#include <abmReasoningFunction.h>
#include <plan.h>

class sharedPlan
{   
public:

    std::vector< std::pair <plan, int > >                 listPlanPossible; // list of the different plan possible for the execution of a shared plan, with their number of apperances.
    std::string                                          sName;
    std::string                                          sManner;
    std::vector< std::pair < std::string , std::string > >              vArguments; // argument of the shared plan
};


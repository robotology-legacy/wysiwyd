#include <efaa/abmReasoningFunction.h>
#include <efaa/plan.h>

using namespace yarp::os;
using namespace efaa::helpers;
using namespace std;



class sharedPlan
{	
public:

	vector< pair <plan, int > >					listPlanPossible; // list of the different plan possible for the execution of a shared plan, with their number of apperances.
	string											sName;
	string											sManner;
	vector< pair < string , string > >				vArguments; // argument of the shared plan
};


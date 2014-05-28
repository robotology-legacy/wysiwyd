#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;



class plan
{	
public:
	
	string											sName;
	string											sManner;
	vector<string>									vActivitytype;
	vector<string>									vActivityname;
	vector< list < pair < string , string > > >		vActivityArguments;			// argument of the current activity (expl : icub-agent1 circle-object1 east-spatial1)
	vector< pair <string, string> >					vArguments;					// pair of argument + role of the plan (expl : circle-object1 , icub-agent2 ... )
};


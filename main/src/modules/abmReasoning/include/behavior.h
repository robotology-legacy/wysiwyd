#include <abmReasoningFunction.h>

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;



class behavior
{	
public:
	string											sName;
	string											sArgument;
	vector < vector< pair <string, double> > >		vEffect;
	Bottle						getConsequence();
	Bottle						getConsequence(int last);

};


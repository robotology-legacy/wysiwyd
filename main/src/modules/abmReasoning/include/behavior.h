#include <efaa/abmReasoningFunction.h>

using namespace yarp::os;
using namespace efaa::helpers;
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


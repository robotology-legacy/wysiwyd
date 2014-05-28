#include <efaa/abmReasoningFunction.h>

using namespace yarp::os;
using namespace efaa::helpers;
using namespace std;



class knownInteraction
{	
public:
	string											sSubject;
	vector<	tuple<string, int, string, string> >			listInteraction;		// an interaction : argument (name of the "object", number of time, type of "interaction", role of the interaction "spatial..."
	void	addInteraction(tuple<string, int, string, string>);

};


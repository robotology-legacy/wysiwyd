#include "cvz/core/all.h"
#include "cvz/gui/all.h"
#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

class CvzFiberModule :public RFModule
{
	cvz::core::CvzFiber* fiber;

public:
	bool configure(yarp::os::ResourceFinder &rf)
	{
		fiber = new cvz::core::CvzFiber();
		yarp::os::Property prop; prop.fromConfigFile(rf.findFile("from"));
		return fiber->configure(prop);
	}

	bool updateModule()
	{
		fiber->cycle();
		return true;
	}

	bool close()
	{
		delete fiber;
		return true;
	}
};

int main(int argc, char * argv[])
{
	Network yarp;
	if (!Network::checkNetwork())
	{
		cout << "yarp network is not available!" << endl;
		return 0;
	}

	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultContext("cvz");
	rf.setDefaultConfigFile("defaultFiber.ini"); //overridden by --from parameter
	rf.configure(argc, argv);

	CvzFiberModule cvzFiber;
	if (cvzFiber.configure(rf))
		cvzFiber.runModule();
	else
		cout << "Unable to configure the cvz fiber module." << endl;

	return 0;
}
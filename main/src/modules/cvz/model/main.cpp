#include "cvz/core/all.h"
#include "cvz/gui/all.h"

#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

int main(int argc, char * argv[])
{
	Network yarp;
	bool b1, b2;
	cout << "Server name is : " << yarp.getNameServerName() << endl;
	//if (!yarp.setNameServerName("/test"))
	//	cout << "Impossible to force name server!" << endl;
	//else
	//	cout << "Forced name server to be /test" << endl;

	yarp.detectNameServer(true, b1, b2);

	if (!Network::checkNetwork())
	{
		cout << "yarp network is not available!" << endl;
		return 0;
	}

	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultContext("cvz");
	rf.setDefaultConfigFile("default.ini"); //overridden by --from parameter
	rf.configure(argc, argv);

	cvz::core::CvzStack stack;
	int retinaX = 2;
	int retinaY = 2;

	//Add V1
	//stack.addCvz("v1_3x3.ini", "v1");
	stack.addCvz("v1.ini", "v1");

	//Add the head proprioception
	stack.addCvz("icub_head.ini", "gaze");

	//Connect the head to v1 -- I know this is non plausible
	stack.connectModalities("/gaze/v1", "/v1/gaze");

	//Instantiate the retina maps
	for (int x = 0; x < retinaX; x++)
	{
		for (int y = 0; y < retinaY; y++)
		{
			std::stringstream ssName;
			ssName << "retina/" << x << "_" << y;
			stack.addCvz("retinaCell.ini", ssName.str().c_str());

			std::string upModalityName = "/";
			upModalityName += ssName.str().c_str();
			upModalityName += "/v1";

			std::stringstream ssV1Name;
			ssV1Name<< "/v1/in_" <<x<<"_"<<y;
			std::string downModalityName = ssV1Name.str();
			std::cout << "*****************|" << downModalityName << endl;
			stack.connectModalities(upModalityName, ssV1Name.str().c_str());
		}
	}

	//Make sure the graph is completed and all the connections are estblished
	stack.connectModalities();

	stack.configure(rf);
	stack.runModule();
	return 0;
}
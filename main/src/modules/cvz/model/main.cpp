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
	//cvz::gui::GuiICvz* gui = NULL;
	//cvz::gui::GuiThread* guiThread = NULL;
	//std::string cvzType = rf.check("type", yarp::os::Value(cvz::core::TYPE_ICVZ)).asString();
	//bool displayGui = rf.check("displayGui");
	
	for (int x = 0; x < 2; x++)
	{
		for (int y = 0; y < 2; y++)
		{
			std::stringstream ssName;
			ssName << "retina/" << x << "_" << y;
			stack.addCvz("retinaCell.ini", ssName.str().c_str());
		}
	}

	stack.configure(rf);
	stack.runModule();
	return 0;
}
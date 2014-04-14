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

	cvz::core::IConvergenceZone* mod = NULL;
	cvz::gui::GuiICvz* gui = NULL;
	cvz::gui::GuiThread* guiThread = NULL;
	std::string cvzType = rf.check("type", yarp::os::Value(cvz::core::TYPE_ICVZ)).asString();

	bool displayGui = rf.check("displayGui");
	if (cvz::core::CvzBuilder::allocate(&mod, cvzType))
	{
		yarp::os::Property prop; prop.fromConfigFile(rf.findFile("from"));
		if (rf.check("name"))
		{
			prop.unput("name");
			prop.put("name", Value(rf.find("name")));
		}

		mod->configure(prop);
		if (displayGui)
		{
			if (cvz::gui::CvzGuiBuilder::allocate(&gui, cvzType, mod))
			{
				gui->start();
				guiThread = new cvz::gui::GuiThread(gui, 10);
				guiThread->start();
			}
			else
				cout << "This cvz type (" << cvzType << ") is not handled by the GUI builder." << endl;
		}
		mod->runModule();
		if (gui != NULL)
		{
			guiThread->stop();
			delete guiThread;
			gui->stop();
			delete gui;

		}
		delete mod;
	}
	else
	{
		cout << "This cvz type (" << cvzType << ") is not handled by the builder." << endl
			<< cvz::core::CvzBuilder::helpMessage() << endl;
	}
	return 0;
}
#include "cvz/core/all.h"
#include "cvz/gui/all.h"

#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

class CvzModule :public RFModule
{
	cvz::core::ThreadedCvz * thread;
	cvz::gui::GuiICvz* gui;
	cvz::gui::GuiThread* guiThread;

public:
	bool configure(yarp::os::ResourceFinder &rf)
	{
		thread = NULL;
		gui = NULL;
		guiThread = NULL;

		std::string cvzType = rf.check("type", yarp::os::Value(cvz::core::TYPE_ICVZ)).asString();
		yarp::os::Property prop; prop.fromConfigFile(rf.findFile("from"));

		if (prop.check("name") && rf.check("name"))
		{
			prop.unput("name");
			prop.put("name", rf.find("name").asString());
		}
		thread = new cvz::core::ThreadedCvz(prop, rf.check("period", yarp::os::Value(100)).asInt());
		bool success = thread->start();

		if (success && rf.check("displayGui"))
		{
			if (cvz::gui::CvzGuiBuilder::allocate(&gui, cvzType, thread->cvz))
			{
				gui->start();
				guiThread = new cvz::gui::GuiThread(gui, 10);
				guiThread->start();
			}
			else
				cout << "This cvz type (" << cvzType << ") is not handled by the GUI builder." << endl;
		}
		return success;
	}

	bool updateModule()
	{
		return true;
	}

	bool close()
	{
		if (gui != NULL)
		{
			guiThread->stop();
			delete guiThread;
			gui->stop();
			delete gui;
		}

		thread->stop();
		delete thread;
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
	rf.setDefaultConfigFile("default.ini"); //overridden by --from parameter
	rf.configure(argc, argv);

	CvzModule cvzCore;
	if (cvzCore.configure(rf))
		cvzCore.runModule();
	else
		cout << "Unable to configure the cvz module." << endl;

	return 0;
}
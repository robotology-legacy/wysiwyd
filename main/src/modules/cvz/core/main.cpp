#include"IModality.h"
#include"CvzBuilder.h"
#include"GuiICvz.h"
#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

int main(int argc, char * argv[])
{
	Network::init();
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("default.ini"); //overridden by --from parameter
	rf.setDefaultContext("cvz/conf");   //overridden by --context parameter
	rf.configure("EFAA_ROOT", argc, argv);

	IConvergenceZone* mod = NULL;
	GuiICvz* gui = NULL;
	GuiThread* guiThread = NULL;
	std::string cvzType = rf.check("type", yarp::os::Value(TYPE_ICVZ)).asString();

	bool displayGui = rf.check("displayGui");
	if (CvzBuilder::allocate(&mod, cvzType))
	{
		mod->configure(rf);
		if (displayGui)
		{
			if (CvzGuiBuilder::allocate(&gui, cvzType, mod))
			{
				gui->start();
				guiThread = new GuiThread(gui, 10);
				guiThread->start();
			}
			else
				cout << "This cvz type (" << cvzType << ") is not handled by the GUI builder." << endl;
		}
		mod->runModule();
		guiThread->stop();
		gui->stop();
		delete mod;
		delete gui;
		delete guiThread;
	}
	else
	{
		cout << "This cvz type (" << cvzType << ") is not handled by the builder." << endl
			<< CvzBuilder::helpMessage() << endl;
	}
	return 0;
}
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
	rf.setDefaultContext("visualSystem");
	rf.setDefaultConfigFile("visualSystem.ini"); //overridden by --from parameter
	rf.configure(argc, argv);

	cvz::core::CvzStack stack;
	int retinaX = rf.check("retinaW",Value(3)).asInt();
	int retinaY = rf.check("retinaH", Value(3)).asInt();

	//Add V1
	//stack.addCvz("v1_3x3.ini", "v1");
	//stack.addCvz("v1.ini", "v1");

	stringstream configV1;
	configV1
		<< "type" << '\t' << cvz::core::TYPE_MMCM << endl
		<< "name" << '\t' << "v1" << endl
		<< "width" << '\t' << 25 << endl
		<< "height" << '\t' << 25 << endl
		<< "layers" << '\t' << 5 << endl <<endl;

	//Add the proprioception
	configV1
		<< "[modality_0]" << endl
		<< "name" << '\t' << "gaze"<< endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	int modalitiesCount = 1;
	for (int x = 0; x < retinaX; x++)
	{
		for (int y = 0; y < retinaY; y++)
		{
			configV1
				<< "[modality_" << modalitiesCount << "]" << endl
				<< "name" << '\t' << "in_" << x << "_" << y << endl
				<< "type" << '\t' << "yarpVector" << endl
				<< "size" << '\t' << 3 << endl << endl;
			modalitiesCount++;
		}
	}
	Property propV1;
	propV1.fromConfig(configV1.str().c_str());
	stack.addCvzFromProperty(propV1,"v1");

	//Add the head proprioception
	stack.addCvzFromConfigFile(std::string("icub_head.ini"), "gaze");

	//Connect the head to v1 -- I know this is non plausible
	stack.connectModalities("/gaze/v1", "/v1/gaze");

	//Instantiate the retina maps
	for (int x = 0; x < retinaX; x++)
	{
		for (int y = 0; y < retinaY; y++)
		{
			std::stringstream ssName;
			ssName << "retina/" << x << "_" << y;
			stack.addCvzFromConfigFile("retinaCell.ini", ssName.str().c_str());

			//Connect camera input
			std::string inputModName = "/";
			inputModName += ssName.str().c_str();
			inputModName += "/retina";
			std::stringstream inputName;
			inputName << "/imageSplitter/split/" << x << "_" << y<<":o";
			stack.connectExternalInput(inputName.str(), inputModName);

			//Connect topdown
			std::string upModalityName = "/";
			upModalityName += ssName.str().c_str();
			upModalityName += "/v1";
			std::stringstream ssV1Name;
			ssV1Name<< "/v1/in_" <<x<<"_"<<y;
			std::string downModalityName = ssV1Name.str();
			std::cout << "*****************|" << downModalityName << endl;
			stack.connectModalities(upModalityName, downModalityName);
		}
	}
	//Connect the inputs

	//Instantiate the retina maps

	//Make sure the graph is completed and all the connections are estblished
	stack.connectModalities();

	stack.configure(rf);
	stack.runModule();
	return 0;
}
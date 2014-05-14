#include "cvz/core/all.h"
#include "cvz/gui/all.h"

#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

void configureV1Retina(cvz::core::CvzStack* stack, int retinaX, int retinaY)
{
	stringstream configV1Retina;

	//V1 Retina
	configV1Retina
		<< "type" << '\t' << cvz::core::TYPE_MMCM << endl
		<< "name" << '\t' << "v1Retina" << endl
		<< "width" << '\t' << 10 << endl
		<< "height" << '\t' << 10 << endl
		<< "layers" << '\t' << 6 << endl << endl;

	int modalitiesCount = 0;
	for (int x = 0; x < retinaX; x++)
	{
		for (int y = 0; y < retinaY; y++)
		{
			configV1Retina
				<< "[modality_" << modalitiesCount << "]" << endl
				<< "name" << '\t' << "in_" << x << "_" << y << endl
				<< "type" << '\t' << "yarpVector" << endl
				<< "size" << '\t' << 3 << endl << endl;
			modalitiesCount++;
		}
	}

	configV1Retina
		<< "[modality_" << modalitiesCount << "]" << endl
		<< "name" << '\t' << "out" << endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "isTopDown" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	Property propV1;
	propV1.fromConfig(configV1Retina.str().c_str());
	stack->addCvzFromProperty(propV1, "v1Retina");

	//Instantiate the retina maps
	for (int x = 0; x < retinaX; x++)
	{
		for (int y = 0; y < retinaY; y++)
		{
			std::stringstream ssName;
			ssName << "retina/" << x << "_" << y;
			stack->addCvzFromConfigFile("retinaCell.ini", ssName.str().c_str());

			//Connect camera input
			std::string inputModName = "/";
			inputModName += ssName.str().c_str();
			inputModName += "/retina";
			std::stringstream inputName;
			inputName << "/imageSplitter/split/" << x << "_" << y << ":o";
			stack->connectExternalInput(inputName.str(), inputModName);

			//Connect topdown
			std::string upModalityName = "/";
			upModalityName += ssName.str().c_str();
			upModalityName += "/v1";
			std::stringstream ssV1Name;
			ssV1Name << "/v1Retina/in_" << x << "_" << y;
			std::string downModalityName = ssV1Name.str();
			stack->connectModalities(upModalityName, downModalityName);
		}
	}
}

/*******************************************************************************/
void configureV1Fovea(cvz::core::CvzStack* stack, int foveaX, int foveaY)
{
	stringstream configV1Retina;

	//V1 Retina
	configV1Retina
		<< "type" << '\t' << cvz::core::TYPE_MMCM << endl
		<< "name" << '\t' << "v1Fovea" << endl
		<< "width" << '\t' << 10 << endl
		<< "height" << '\t' << 10 << endl
		<< "layers" << '\t' << 6 << endl << endl;

	int modalitiesCount = 0;
	for (int x = 0; x < foveaX; x++)
	{
		for (int y = 0; y < foveaY; y++)
		{
			configV1Retina
				<< "[modality_" << modalitiesCount << "]" << endl
				<< "name" << '\t' << "in_" << x << "_" << y << endl
				<< "type" << '\t' << "yarpVector" << endl
				<< "size" << '\t' << 3 << endl << endl;
			modalitiesCount++;
		}
	}

	configV1Retina
		<< "[modality_" << modalitiesCount << "]" << endl
		<< "name" << '\t' << "out"<< endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "isTopDown" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	Property propV1;
	propV1.fromConfig(configV1Retina.str().c_str());
	stack->addCvzFromProperty(propV1, "v1Fovea");

	//Instantiate the retina maps
	for (int x = 0; x < foveaX; x++)
	{
		for (int y = 0; y < foveaY; y++)
		{
			std::stringstream ssName;
			ssName << "fovea/" << x << "_" << y;
			stack->addCvzFromConfigFile("foveaCell.ini", ssName.str().c_str());

			//Connect camera input
			std::string inputModName = "/";
			inputModName += ssName.str().c_str();
			inputModName += "/retina";
			std::stringstream inputName;
			inputName << "/imageSplitter/fovea/split/" << x << "_" << y << ":o";
			stack->connectExternalInput(inputName.str(), inputModName);

			//Connect topdown
			std::string upModalityName = "/";
			upModalityName += ssName.str().c_str();
			upModalityName += "/v1";
			std::stringstream ssV1Name;
			ssV1Name << "/v1Fovea/in_" << x << "_" << y;
			std::string downModalityName = ssV1Name.str();
			stack->connectModalities(upModalityName, downModalityName);
		}
	}
}

int main(int argc, char * argv[])
{
	Network yarp;
	bool b1, b2;
	cout << "Server name is : " << yarp.getNameServerName() << endl;

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

	//Get the parameters of the model
	cvz::core::CvzStack stack;
	int retinaX = rf.check("retinaW",Value(3)).asInt();
	int retinaY = rf.check("retinaH", Value(3)).asInt();
	int foveaX = rf.check("foveaW", Value(3)).asInt();
	int foveaY = rf.check("foveaH", Value(3)).asInt();

	/*************************************************/
	//Add V1
	configureV1Retina(&stack, retinaX, retinaY);
	configureV1Fovea(&stack, foveaX, foveaY);

	/*************************************************/
	//Add v2
	stringstream configV2;
	configV2
		<< "type" << '\t' << cvz::core::TYPE_MMCM << endl
		<< "name" << '\t' << "v2" << endl
		<< "width" << '\t' << 5 << endl
		<< "height" << '\t' << 5 << endl
		<< "layers" << '\t' << 6 << endl << endl;

	//Add the proprioception
	configV2
		<< "[modality_0]" << endl
		<< "name" << '\t' << "gaze" << endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	//Add the v1Retina
	configV2
		<< "[modality_1]" << endl
		<< "name" << '\t' << "v1Retina" << endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	configV2
		<< "[modality_2]" << endl
		<< "name" << '\t' << "v1Fovea" << endl
		<< "type" << '\t' << "yarpVector" << endl
		<< "size" << '\t' << 3 << endl << endl;

	Property propV2;
	propV2.fromConfig(configV2.str().c_str());
	stack.addCvzFromProperty(propV2, "v2");

	//Add the head proprioception
	stack.addCvzFromConfigFile(std::string("icub_head.ini"), "gaze");

	//Connect the head to v2 -- I know this is non plausible
	stack.connectModalities("/gaze/v1", "/v2/gaze");
	stack.connectModalities("/v1Retina/out", "/v2/v1Retina");
	stack.connectModalities("/v1Fovea/out", "/v2/v1Fovea");

	//Make sure the graph is completed and all the connections are established
	stack.connectModalities();

	stack.configure(rf);
	stack.runModule();
	return 0;
}
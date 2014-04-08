
#include <yarp/os/all.h>
#include <cvz/core/all.h>
#include "Cvz2Xml.h"
using namespace std;
using namespace yarp::os;
int main(int argc, char * argv[])
{
	Network yarp;
	if (!yarp.checkNetwork())
		return -1;

	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultContext("cvz");
	rf.setDefaultConfigFile("stackDefault.ini"); //overridden by --from parameter
	rf.configure(argc, argv);

	Bottle* cvzList = rf.find("zones").asList();
	if (cvzList == NULL)
	{
		std::cout << "Please, provide --zones \"(cvz1.ini  \"nameZone1\" ... cvzN.ini \"nameZoneN\")\" " << endl;
		return -1;
	}
	std::string outputFile = rf.check("outputFile", Value("output.xml")).asString().c_str();
	std::string applicationName = rf.check("applicationName", Value("Default CVZ hierarchy")).asString().c_str();

	//Create the cvz stack in memory
	std::vector<cvz::core::IConvergenceZone* > zones;
	std::map<cvz::core::IConvergenceZone*, std::string > zonesFiles;
	std::map<cvz::core::IConvergenceZone*, std::string > zonesNames;
	for (int m = 0; m < cvzList->size(); m += 2)
	{
		cvz::core::IConvergenceZone* mod = NULL;
		std::string fileName = cvzList->get(m).asString().c_str();
		std::string mapName = cvzList->get(m + 1).asString().c_str();
		Property prop;
		prop.fromConfigFile(rf.findFileByName(fileName));
		prop.unput("name");
		prop.put("name", Value(mapName));

		std::string cvzType = prop.check("type", yarp::os::Value(cvz::core::TYPE_ICVZ)).asString().c_str();

		if (cvz::core::CvzBuilder::allocate(&mod, cvzType))
		{
			mod->configure(prop);
		}

		zones.push_back(mod);
		zonesFiles[mod] = fileName;
		zonesNames[mod] = mapName;
	}

	//Dump it to an xml format
	std::ofstream xml(outputFile.c_str());
	xml << "<application>" << '\n';
	xml << '\t' + getBalise("name", applicationName);

	std::map<cvz::core::IConvergenceZone*, std::string> configFilesModulesDescription;
	//Modules
	for (int m = 0; m < zones.size(); m++)
	{
		std::stringstream params;
		params << "--from " + zonesFiles[zones[m]] + " --name " << zonesNames[zones[m]];

		//Create the module xml description
		configFilesModulesDescription[zones[m]] = cvz2xml(zones[m], zonesFiles[zones[m]]);

		//Create the module balise
		xml <<
			'\t' + getBalise("module",
			"\t\t" + getBalise("name", "cvzCore ("+zones[m]->getName()+")") +
			"\t\t" + getBalise("parameters", params.str()) +
			"\t\t" + getBalise("node", "icubsrv"));

		//Create the connections balise
		std::stringstream dummyInReal;
		std::stringstream dummyOutReal;
		std::stringstream dummyInPrediction;
		std::stringstream dummyOutPrediction;
		dummyInReal << "/dummyInReal_" << m;
		dummyOutReal << "/dummyOutReal_" << m;
		dummyInPrediction << "/dummyInPrediction_" << m;
		dummyOutPrediction << "/dummyOutPrediction_" << m;

		for (std::map<std::string, cvz::core::IModality*>::iterator itMod = zones[m]->modalitiesBottomUp.begin(); itMod != zones[m]->modalitiesBottomUp.end(); itMod++)
		{
			//RealInput
			xml <<
				'\t' + getBalise("connection",
				"\t\t" + getBalise("source", dummyInReal.str()) +
				"\t\t" + getBalise("to", itMod->second->GetFullNameReal()) +
				"\t\t" + getBalise("protocol", "tcp"));

			//PredictionInput
			xml <<
				'\t' + getBalise("connection",
				"\t\t" + getBalise("source", itMod->second->GetFullNamePrediction()) +
				"\t\t" + getBalise("to", dummyInPrediction.str()) +
				"\t\t" + getBalise("protocol", "tcp"));
		}

		for (std::map<std::string, cvz::core::IModality*>::iterator itMod = zones[m]->modalitiesTopDown.begin(); itMod != zones[m]->modalitiesTopDown.end(); itMod++)
		{
			//RealInput
			xml <<
				'\t' + getBalise("connection",
				"\t\t" + getBalise("source", dummyOutReal.str()) +
				"\t\t" + getBalise("to", itMod->second->GetFullNameReal()) +
				"\t\t" + getBalise("protocol", "tcp"));

			//PredictionInput
			xml <<
				'\t' + getBalise("connection",
				"\t\t" + getBalise("source", itMod->second->GetFullNamePrediction()) +
				"\t\t" + getBalise("to", dummyOutPrediction.str()) +
				"\t\t" + getBalise("protocol", "tcp"));
		}
	}
	xml << "</application>";
	xml.close();

	//Export the modules descriptions
	for (std::map < cvz::core::IConvergenceZone*, std::string >::iterator it = configFilesModulesDescription.begin(); it != configFilesModulesDescription.end(); it++)
	{
		std::ofstream xml( (it->first->getName()+ ".xml").c_str() );
		xml << it->second;
		xml.close();
	}

	//Close the zones
	for (int m = 0; m < zones.size(); m++)
	{
		zones[m]->stopModule();
		zones[m]->interruptModule();
		zones[m]->close();
	}
	return 0;
}
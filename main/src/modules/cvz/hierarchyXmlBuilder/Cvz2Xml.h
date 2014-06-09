
#include <yarp/os/all.h>
#include <cvz/core/all.h>
#include <list>

using namespace yarp::os;

std::string getBalise(std::string bName, std::string bContent)
{
    return ("<" + bName + "> " + bContent + "  </" + bName + "> \n");
}

std::string cvz2xml(cvz::core::IConvergenceZone* z, std::string configFile)
{
    std::list<std::string> inputs;
    std::list<std::string> outputs;
    for (std::map<std::string, cvz::core::IModality*>::iterator itMod = z->modalitiesBottomUp.begin(); itMod != z->modalitiesBottomUp.end(); itMod++)
    {
        inputs.push_back(itMod->second->GetFullNameReal());
        outputs.push_back(itMod->second->GetFullNamePrediction());
    }

    for (std::map<std::string, cvz::core::IModality*>::iterator itMod = z->modalitiesTopDown.begin(); itMod != z->modalitiesTopDown.end(); itMod++)
    {
        inputs.push_back(itMod->second->GetFullNameReal());
        outputs.push_back(itMod->second->GetFullNamePrediction());
    }
    std::stringstream sInputs;
    for (std::list<std::string>::iterator it = inputs.begin(); it != inputs.end(); it++)
    {
        sInputs <<
            '\t' + getBalise("input",
            "\t\t" + getBalise("type", "Bottle") +
            "\t\t" + getBalise("port", (*it)) +
            "\t\t" + getBalise("required", "no") +
            "\t\t" + getBalise("priority", "no") +
            "\t\t" + getBalise("description", "An input to the CVZ"));
    }
    std::stringstream sOutputs;
    for (std::list<std::string>::iterator it = outputs.begin(); it != outputs.end(); it++)
    {
        sInputs <<
            '\t' + getBalise("output",
            "\t\t" + getBalise("type", "Bottle") +
            "\t\t" + getBalise("port", (*it))+
            "\t\t" + getBalise("description", "An output of the CVZ"));
    }

    sInputs <<
        '\t' + getBalise("input",
        "\t\t" + getBalise("type", "Rpc") +
        "\t\t" + getBalise("port", z->getRpcPortName()) +
        "\t\t" + getBalise("required", "no") +
        "\t\t" + getBalise("priority", "no") +
        "\t\t" + getBalise("description", "Rpc port accepting commands such as... (todo generate from the IConvergenceZone class)"));

    std::stringstream xml;
    std::string name=z->getName().c_str();
    xml << 
        getBalise("module",
        getBalise("name", "cvzCore ("+name+")") +
        getBalise("description", "Cvz from "+configFile) +
        getBalise("arguments", 
            std::string("<param default = \""+configFile+"\" required = \"no\" desc = \"config file describing the CVZ caracteristics.\">from</param> \n") + 
            std::string("<param required = \"no\" desc = \"For overriding the name from the config file\">name</param>\n")) +
        getBalise("authors", "<author email = \"stephane.lallee@gmail.com\"> Stephane Lallee </author>") +
        getBalise("data", sInputs.str() + sOutputs.str()) +
        getBalise("deployer", "cvzCore") +
        getBalise("dependencies", getBalise("include", "")) +
        getBalise("development", getBalise("library", "YARP") + getBalise("library", "cvz")));

    return xml.str();
}

std::string stack2xml(std::string applicationName, const cvz::core::CvzStack& s)
{
	return "Not implemented yet.";
//	std::stringstream xml;
//	xml << "<application>" << '\n';
//	xml << '\t' + getBalise("name", applicationName);
//
//	std::map<cvz::core::IConvergenceZone*, std::string> configFilesModulesDescription;
//	//Modules
//	for (int m = 0; m < zones.size(); m++)
//	{
//		std::stringstream params;
//		params << "--from " + zonesFiles[zones[m]] + " --name " << zonesNames[zones[m]];
//
//		//Create the module xml description
//		configFilesModulesDescription[zones[m]] = cvz2xml(zones[m], zonesFiles[zones[m]]);
//
//		//Create the module balise
//		string name = zones[m]->getName().c_str();
//		xml <<
//			'\t' + getBalise("module",
//			"\t\t" + getBalise("name", "cvzCore (" + name + ")") +
//			"\t\t" + getBalise("parameters", params.str()) +
//			"\t\t" + getBalise("node", "icubsrv"));
//
//		//Create the connections balise
//		std::stringstream dummyInReal;
//		std::stringstream dummyOutReal;
//		std::stringstream dummyInPrediction;
//		std::stringstream dummyOutPrediction;
//		dummyInReal << "/dummyInReal_" << m;
//		dummyOutReal << "/dummyOutReal_" << m;
//		dummyInPrediction << "/dummyInPrediction_" << m;
//		dummyOutPrediction << "/dummyOutPrediction_" << m;
//
//		for (std::map<std::string, cvz::core::IModality*>::iterator itMod = zones[m]->modalitiesBottomUp.begin(); itMod != zones[m]->modalitiesBottomUp.end(); itMod++)
//		{
//			//RealInput
//			xml <<
//				'\t' + getBalise("connection",
//				"\t\t" + getBalise("source", dummyInReal.str()) +
//				"\t\t" + getBalise("to", itMod->second->GetFullNameReal()) +
//				"\t\t" + getBalise("protocol", "tcp"));
//
//			//PredictionInput
//			xml <<
//				'\t' + getBalise("connection",
//				"\t\t" + getBalise("source", itMod->second->GetFullNamePrediction()) +
//				"\t\t" + getBalise("to", dummyInPrediction.str()) +
//				"\t\t" + getBalise("protocol", "tcp"));
//		}
//
//		for (std::map<std::string, cvz::core::IModality*>::iterator itMod = zones[m]->modalitiesTopDown.begin(); itMod != zones[m]->modalitiesTopDown.end(); itMod++)
//		{
//			//RealInput
//			xml <<
//				'\t' + getBalise("connection",
//				"\t\t" + getBalise("source", dummyOutReal.str()) +
//				"\t\t" + getBalise("to", itMod->second->GetFullNameReal()) +
//				"\t\t" + getBalise("protocol", "tcp"));
//
//			//PredictionInput
//			xml <<
//				'\t' + getBalise("connection",
//				"\t\t" + getBalise("source", itMod->second->GetFullNamePrediction()) +
//				"\t\t" + getBalise("to", dummyOutPrediction.str()) +
//				"\t\t" + getBalise("protocol", "tcp"));
//		}
//	}
//	xml << "</application>";
//	xml.close();
//
//	//Export the modules descriptions
//	for (std::map < cvz::core::IConvergenceZone*, std::string >::iterator it = configFilesModulesDescription.begin(); it != configFilesModulesDescription.end(); it++)
//	{
//		std::string name = it->first->getName().c_str();
//		std::ofstream xml((name + ".xml").c_str());
//		xml << it->second;
//		xml.close();
//	}
//}

}

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


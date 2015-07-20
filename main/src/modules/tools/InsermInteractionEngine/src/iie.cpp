#include "iie.h"


/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string IIE::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        yInfo() << " Error in iie::grammarToString. Couldn't open file : " << sPath << ".";
        return "Error in iie::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

bool IIE::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("iie")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << ": finding configuration files...";
    period = rf.check("period", Value(1.)).asDouble();

    bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "iie", "client.ini", isRFVerbose);

    iCub->opc->isVerbose &= false;
    if (!iCub->connect())
    {
        yInfo() << " iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }

    string test;

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    // Open port2reasoning
    Port2SupervisorName = "/";
    Port2SupervisorName += getName() + "/toSupervisor";

    if (!Port2Supervisor.open(Port2SupervisorName.c_str())) {
        cout << getName() << ": Unable to open port " << Port2SupervisorName << endl;
        bEveryThingisGood &= false;
    }
    bEveryThingisGood &= Network::connect(Port2SupervisorName.c_str(), "/qRM/rpc");


    if (!iCub->getABMClient())
    {
        yInfo() << " WARNING ABM NOT CONNECTED";
    }

    return true;
}


bool IIE::close() {
    iCub->close();
    delete iCub;

    return true;
}


bool IIE::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "help") {
        yInfo() << helpMessage;
        reply.addString("ok");
    }


    return true;
}

/* Called periodically every getPeriod() seconds */
bool IIE::updateModule() {

    if (iCub->opc->isConnected())
    {
        iCub->opc->checkout();
        list<Entity*> lEntities = iCub->opc->EntitiesCacheCopy();

        for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
        {
            string sName = (*itEnt)->name();
            string sNameCut = sName;
            string delimiter = "_";
            size_t pos = 0;
            std::string token;
            while ((pos = sName.find(delimiter)) != std::string::npos) {
                token = sName.substr(0, pos);
                std::cout << token << std::endl;
                sName.erase(0, pos + delimiter.length());
                sNameCut = token;
            }
            yInfo() << " sNameCut is:" << sNameCut;
            // check is label is known

            if (sNameCut == "unknown" || sNameCut == "partner")
            {
                // label is unknown send information to qRM
                Bottle b2Supervisor;
                b2Supervisor.addString("exploreUnknownEntity");
                b2Supervisor.addString((*itEnt)->entity_type());
                b2Supervisor.addString((*itEnt)->name());
                Bottle bReplyFromQRM;
                Port2Supervisor.write(b2Supervisor, bReplyFromQRM);
                yInfo() << " ACHTUNG! UNKNOWN OBJECT MODAKUKA !";

                yInfo() << " bReplyFromQRM: " << bReplyFromQRM.toString();
            }
        }
    }
    else
    {
        yWarning() << " in IIE: OPC not Connected";
    }

    //  mainLoop();
    return true;
}



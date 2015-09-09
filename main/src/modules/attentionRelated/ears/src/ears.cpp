#include "ears.h"

bool ears::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("ears")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << moduleName << " : finding configuration files...";
    period = rf.check("period", Value(0.1)).asDouble();

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "ears", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= false;
    if (!iCub->connect())
    {
        yInfo() << " iCubClient : Some dependencies are not running...";
        Time::delay(1.0);
    }
    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    portToReactive.open(("/" + moduleName + "/reactive:o").c_str());
    if (!Network::connect(portToReactive.getName().c_str(),"/reactiveLayer/rpc")) {
        yWarning() << " reactive Layer is not reachable";
    }

    MainGrammar = rf.findFileByName(rf.check("MainGrammar", Value("MainGrammar.xml")).toString());

    bListen = false;

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";
    
    return true;
}


bool ears::close() {
    iCub->close();
    rpc.close();
    portToReactive.close();
    delete iCub;

    return true;
}


bool ears::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "quit \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "listen")
    {
        if (command.size() == 2)
        {
            if (command.get(1).asString() == "on")
            {
                bListen = true;
            }
            else if (command.get(1).asString() == "off")
            {
                bListen = false;
            }
        }
    }
    else {
        yInfo() << helpMessage;
        reply.addString("wrong command");
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool ears::updateModule() {

    if (bListen)
    {
        Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
            bAnswer, //response from speech recog without transfer information, including raw sentence
            bSemantic; // semantic information of the content of the recognition
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(MainGrammar), 1);

        if (bRecognized.get(0).asInt() == 0)
        {
            yWarning() << " error in ears::updateModule | Error in speechRecog";
            return true;
        }

        bAnswer = *bRecognized.get(1).asList();

        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        bSemantic = *(*bAnswer.get(1).asList()).get(1).asList();
        cout << bSemantic.toString() << endl;
        string sPredicate = bSemantic.check("predicate", Value("none")).asString();
        string sObject    = bSemantic.check("object", Value("none")).asString();

        iCub->opc->update();
        list<Entity*> entities = iCub->opc->EntitiesCacheCopy();

        vector<Bottle> vListAction;

        bool bFoundObject = false;
        for (list<Entity*>::iterator itEnt = entities.begin() ; itEnt != entities.end() ; itEnt++)
        {
            if ((*itEnt)->name() == sObject)
            {
                if ((*itEnt)->entity_type() == EFAA_OPC_ENTITY_OBJECT || (*itEnt)->entity_type() == EFAA_OPC_ENTITY_RTOBJECT || (*itEnt)->entity_type() == EFAA_OPC_ENTITY_AGENT)
                {
                    Object *obj = dynamic_cast<Object*>(*itEnt);
                    if (obj->m_present)
                    {
                        bFoundObject = true;
                    }
                }
            }
        }
        if (!bFoundObject)
        {
            Bottle bCondition;
            bCondition.addString("need");
            bCondition.addString("macro");
            bCondition.addString("search");
            bCondition.addString(sObject);
            vListAction.push_back(bCondition);
        }

        Bottle bAction;
        bAction.addString("need");
        bAction.addString("primitive");
        bAction.addString(sPredicate);
        bAction.addString(sObject);

        vListAction.push_back(bAction);

        for (vector<Bottle>::iterator itBo = vListAction.begin() ; itBo != vListAction.end() ; itBo++)
        {
            portToReactive.write(*itBo);
        }
    }




    return true;
}


/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string ears::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    if (!isGrammar)
    {
        yInfo() << "Error in ears::grammarToString. Couldn't open file : " << sPath << ".";
        return "Error in ears::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

#include <abmReasoning.h>


using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


#define THRESHOLD_CONFIDENCE 5.0


/*   -------------------------   MODULE RELATED FUNCTIONS  ------------------------------   */

abmReasoning::abmReasoning(ResourceFinder &rf)
{
    iFunction = new abmReasoningFunction(rf);
    savefile = rf.findFileByName("saveRequest.txt");
    opcNameTable.push_back(EFAA_OPC_ENTITY_TAG);
    opcNameTable.push_back(EFAA_OPC_ENTITY_RELATION);
    opcNameTable.push_back(EFAA_OPC_ENTITY_OBJECT);
    opcNameTable.push_back(EFAA_OPC_ENTITY_RTOBJECT);
    opcNameTable.push_back(EFAA_OPC_ENTITY_AGENT);
    opcNameTable.push_back(EFAA_OPC_ENTITY_ADJECTIVE);
    bReady = false;
}


double abmReasoning::getPeriod()
{
    return 1.0;
}

/* Update loop, check for the behavior allowed */
bool abmReasoning::updateModule()
{
    return true;
}

/* configure the module */
bool abmReasoning::configure(ResourceFinder &rf)
{
    resfind = rf;
    moduleName = rf.check("name", Value("abmReasoning"), "module name (string)").asString();

    setName(moduleName.c_str());

    string portHandlerName = "/";
    portHandlerName += getName() + "/rpc";
    handlerPort.open(portHandlerName.c_str());

    string portSenderName = "/";
    portSenderName += getName() + "/request:o";
    senderPort.open(portSenderName.c_str());
    bDreaming = false;


    iCub = new ICubClient(moduleName, "abmReasoning", "client.ini", false);
    iCub->opc->isVerbose = false;

    Network::connect(senderPort.getName(), "/autobiographicalMemory/rpc");

    port_to_OPCManager.open("/abmReasoning/toOPCManager");
    if (!Network::connect(port_to_OPCManager.getName(), "/opcManager/rpc"))
    {
        yInfo() << " Warning in abmReasoning::Interlocutor::Initialize | connection problem with OPCManager";
    }


    initialisePlanner(rf);
    Bottle bConnectOpc;
    bConnectOpc.addString("connect");
    bConnectOpc.addString(abmReasoningFunction::s_realOPC);
    connectOPC(bConnectOpc);
    iCub->connect();

    attach(handlerPort);
    bPopulateOPC = !(rf.check("noPopulate"));

    if (!(rf.check("noKnowledge"))) getKnowledge();
    //remove all the previous pddl files
    pddlSolDelete(1, pddlNb);

    bestSol = -1;

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << "abmReasoning ready ! \n \n ";

    bReady = true;


    //    adjKnowledge test;
    //    test.determineTimingInfluence();

    //findAllActionsV2(1074);

    // bool bTestLanguage = false;

    //// WordKnowledge.simulateData();

    // if (bTestLanguage)
    // {
    //     //  if (!rf.check("noSentences"))   findAllSentence();
    //     if (!rf.check("noSentences"))   findAllSentence();
    //     listGrammarKnowledge.testModel(100,2);
    //     for (int i = 1 ; i < 8 ; i++)
    //     {
    //         if (i !=2)
    //         {
    //             listGrammarKnowledge.clear();
    //             listGrammarKnowledge.simulateLearning(i,5);
    //             listGrammarKnowledge.testModel(100,i);
    //         }
    //     }
    // }
    return true;
}


/*
*   Initialise the varialbes of the Planner
*   input : ressource finder
*/
void abmReasoning::initialisePlanner(ResourceFinder & rf)
{

    // TODO : use config file !!
    Bottle &bPlanner = rf.findGroup("planner");

    //configure PDDL planner variables
    plannerPath = (bPlanner.check("plannerPath",
        Value("C:/Robot/planner/LPG-td-1.0/"),
        "planner path (string)")).asString();

    plannerExec = (bPlanner.check("plannerExec1",
        Value("lpg-td-1.0"),
        "planner exec (string)")).asString();
    plannerExec += " ";

    plannerOptDom = (bPlanner.check("plannerOptDom",
        Value("-o"),
        "planner optDom (string)")).asString();
    plannerOptDom += " ";

    plannerOptProb = (bPlanner.check("plannerOptProb",
        Value("-f"),
        "planner optProb (string)")).asString();
    plannerOptProb += " ";

    plannerOptOut = (bPlanner.check("plannerOptOut",
        Value("-out"),
        "planner optOut (string)")).asString();
    plannerOptOut += " ";

    plannerOptNb = (bPlanner.check("plannerOptNb",
        Value("-n"),
        "planner optNb (int)")).asString();
    plannerOptNb += " ";

    plannerOptCpu = (bPlanner.check("plannerOptCpu",
        Value("-cputime"),
        "planner optCpu (int)")).asString();
    plannerOptCpu += " ";

    pddlDomain = (bPlanner.check("pddlDomain", Value("domainEFAA.pddl"))).asString();  //name of the domain pddl file
    pddlProblem = (bPlanner.check("pddlProblem", Value("problemEFAA.pddl"))).asString();  //name of the problem pddl file
    pddlOut = (bPlanner.check("pddlOut", Value("solutionEFAA"))).asString();  //name of the solution plan file -> will be called solutionEFAA_1.SOL, solutionEFAA_2.SOL, ...
    pddlNb = (bPlanner.check("pddlNb", Value(30))).asInt();  //nb max of solution files produced
    pddlCpu = (bPlanner.check("pddlCpu", Value(2))).asInt();  //nb max of solution files produced


    //pddlDomain = "domainEFAA.pddl";       //name of the domain pddl file
    //pddlProblem = "problemEFAA.pddl" ;      //name of the problem pddl file
    //pddlOut = "solutionEFAA" ;      //name of the solution plan file -> will be called solutionEFAA_1.SOL

    //there is no last Action => used to update vCurrentActions in findAllSharedPlan
    saveEndLastAction = -1;
}


/* Respond function */
bool abmReasoning::respond(const yarp::os::Bottle& bCommand, yarp::os::Bottle& bReply)
{

    bReply.clear();
    if (!bReady)
    {
        bReply.addString("module not initialised yet, please wait.");
        yInfo() << "\t" << "reply : " << bReply.toString().c_str() << "\n";
        handlerPort.reply(bReply);
        return true;
    }

    if (bCommand.get(0).asString() == "quit") {
        bReply.addString("ack");
        bReply.addString("quitting");
        return false;
    }

    else if (bCommand.get(0).asString() == "sqlQueryTest") {
        bReply.addString("ack");
        //      bReply.addList() = sqlQueryTest();
    }


    // FIND ACTIVITY

    else if (bCommand.get(0).asString() == "findActivity") {

        yInfo() << "\t" << "bCommand.size() = " << bCommand.size();

        if (bCommand.size() <= 2) {
            bReply.addString("nack");
            bReply.addString("Usage : findActivity <actionName> 'begin'|'end'|'both' [columns,to,select]");
        }
        else if (bCommand.get(2).asString() != "begin" && bCommand.get(2).asString() != "end" && bCommand.get(2).asString() != "both") {
            bReply.addString("nack");
            bReply.addString("Usage : findOPC <actionName> 'begin'|'end'|'both' [<colums,to,select>] => check your begin/end/both");
        }
        else {
            bReply.addString("ack");

            string actionName = bCommand.get(1).asString().c_str();
            string beginOrEnd = bCommand.get(2).asString().c_str();
            if (bCommand.size() == 3) {
                bReply.addList() = findActivity(actionName, beginOrEnd);
            }
            else {
                string select = bCommand.get(3).asString().c_str();
                bReply.addList() = findActivity(actionName, beginOrEnd, select);
            }
        }
    }


    // FIND OPC

    else if (bCommand.get(0).asString() == "findOPC") {

        if (bCommand.size() <= 2) {
            bReply.addString("nack");
            bReply.addString("Usage : findOPC <actionName> 'begin'|'end'|'both' [<colums,to,select>]");
        }
        else if (bCommand.get(2).asString() != "begin" && bCommand.get(2).asString() != "end" && bCommand.get(2).asString() != "both") {
            bReply.addString("nack");
            bReply.addString("Usage : findOPC <actionName> 'begin'|'end'|'both' [<colums,to,select>] => check your begin/end/both");
        }
        else {
            bReply.addString("ack");
            string actionName = bCommand.get(1).asString().c_str();
            string beginOrEnd = bCommand.get(2).asString().c_str();
            //          bReply.addList() = findOPC(actionName, beginOrEnd);
        }
    }


    // GET CONSEQUENCES DRIVES

    else if (bCommand.get(0).asString() == "getActionConsequenceDrives") {

        if (bCommand.size() <= 2) {
            bReply.addString("nack");
            bReply.addString("Usage : getActionConsequenceDrives 'actionName' 'arg'");
        }

        else {
            bReply.addString("ack");
            pair<string, string> pAction(bCommand.get(1).asString().c_str(), bCommand.get(2).asString().c_str());
            bReply.addList() = getActionConsequenceDrives(pAction);
        }
    }

    // PRINT PDDL DOMAIN

    else if (bCommand.get(0).asString() == "printPDDLContextualKnowledgeDomain") {

        if (bCommand.size() >= 2) {
            bReply.addString("nack");
            bReply.addString("Usage : printPDDLContextualKnowledge");
        }

        else {
            bReply.addString("ack");
            bReply.addList() = printPDDLContextualKnowledgeDomain();
        }
    }



    // PRINT PDDL PROBLEM

    else if (bCommand.get(0).asString() == "printPDDLContextualKnowledgeProblem") {

        if (bCommand.size() != 2) {
            bReply.addString("nack");
            bReply.addString("Usage : printPDDLContextualKnowledgeProblem ( (condition1) (condition2) )");
        }

        else {

            if (!bCommand.get(1).isList()) {
                bReply.addString("nack");
                bReply.addString("Usage : printPDDLContextualKnowledgeProblem ( (condition1) (condition2) )");
            }

            bReply.addString("ack");
            Bottle bGoal;
            bGoal = *bCommand.get(1).asList();
            bReply.addList() = printPDDLContextualKnowledgeProblem(bGoal);
        }
    }

    // HELP

    else if (bCommand.get(0).asString() == "help") {
        bReply.addString("ack");
        bReply.addString("commands are: findActivity <actionName> <begin|end|both> [<colums,to,select>]| findSharedPlan <sharedPlanName> <begin|end|both> [<colums,to,select>] | help | sqlQueryTest | quit ");
    }


    // ASK LAST ACTION

    else if (bCommand.get(0).asString() == "askLastAction") {
        bReply.addString("ack");
        bReply.addList() = askLastAction();
    }


    // ASK LAST ACTIVITY

    else if (bCommand.get(0).asString() == "askLastActivity") {
        bReply.addString("ack");
        bReply.addList() = askLastActivity(bCommand);
    }


    // ADD LAST ACTIVITY

    else if (bCommand.get(0).asString() == "addLastActivity") {
        bReply.addString("ack");
        bReply.addList() = addLastActivity(bCommand);
    }

    //TEST : to be removed when done
    else if (bCommand.get(0).asString() == "findPossibleSharedPlan") {
        bReply.addString("ack");
        bReply.addList() = findPossibleSharedPlan(bCommand.get(1).asInt(), bCommand.get(2).asInt());
    }

    //TEST : to be removed when done
    else if (bCommand.get(0).asString() == "availableSharedPlan") {
        bReply.addString("ack");
        bReply.addList() = availableSharedPlan(bCommand.get(1).asInt(), bCommand.get(2).asInt());
    }


    //TEST : to be removed when done
    else if (bCommand.get(0).asString() == "findAllSentence") {
        bReply.addString("ack");
        bReply.addList() = findAllSentence();
    }

    else if (bCommand.get(0).asString() == "ago") {
        bReply.addString("ack");
        getIDfromTime(bCommand);
        bReply.addList() = bCommand;
    }



    // DISCRIMINATE ACTION

    else if (bCommand.get(0).asString() == "discriminateAction") {
        bReply.addString("ack");
        bReply.addList() = discriminateLastAction();
    }


    // IMAGINE INSTANCE

    else if (bCommand.get(0).asString() == "imagine") {
        bReply.addString("ack");
        bReply.addList() = imagineOPC(atoi(bCommand.get(1).toString().c_str()));
    }


    // GET ACTION 
    else if (bCommand.get(0).asString() == "getAction") {
        bReply.addString("ack");
        bReply.addList() = askActionFromId(atoi(bCommand.get(1).toString().c_str()));
    }


    // QUERY BEHAVIOR

    else if (bCommand.get(0).asString() == "queryBehavior") {
        bReply.addString("ack");
        bReply.addList() = queryBehavior(bCommand);
    }


    // SAVE KNOWLEDGE

    else if (bCommand.get(0).asString() == "saveKnowledge") {
        bReply.addString("ack");
        bReply.addList() = saveKnowledge();
    }


    // RESET KNOWLEDGE

    else if (bCommand.get(0).asString() == "resetKnowledge") {
        bReply.addString("ack");
        if (bCommand.size() == 2)
        {
            bReply.addList() = resetKnowledge(atoi(bCommand.get(1).toString().c_str()));
            //bReply.addList() = retroReasoning(atoi(bCommand.get(1).toString().c_str()));
        }
        else
        {
            bReply.addList() = resetKnowledge();
            //	bReply.addList() = retroReasoning();
        }
    }
    // RETRO REASONING


    else if (bCommand.get(0).asString() == "retroReasoning") {
        bReply.addString("ack");
        if (bCommand.size() == 2)
        {
            bReply.addList() = retroReasoning(atoi(bCommand.get(1).toString().c_str()));
        }
        else
        {
            bReply.addList() = retroReasoning();
        }
    }

    else if (bCommand.get(0).asString() == "level3") {
        bReply.addString("ack");
        if (bCommand.size() == 2)
        {
            bReply.addList() = level3Reasoning(atoi(bCommand.get(1).toString().c_str()));
        }
        else
        {
            bReply.addList() = level3Reasoning();
        }
    }

    // DISCRIMINATE UNKNOWN ACTIONS

    else if (bCommand.get(0).asString() == "discriminateUnknownActions") {
        bReply.addString("ack");
        bReply.addList() = discriminateUnknownActions();
    }


    // CONNECT OPC

    else if (bCommand.get(0).asString() == "connectOPC") {
        bReply.addString("ack");
        bReply.addList() = connectOPC(bCommand);
    }


    // EXECUTE ACTION

    else if (bCommand.get(0).asString() == "executeAction") {
        bReply.addString("ack");
        bReply.addList() = executeAction(bCommand);
    }

    else if (bCommand.get(0).asString() == "executeActionFromAdv") {
        bReply.addString("ack");
        bReply.addList() = executeActionFromAdv(bCommand);
    }

    // EXECUTE ACTIVITY

    else if (bCommand.get(0).asString() == "executeActivity") {
        bReply.addString("ack");
        bReply.addList() = executeActivity(bCommand);
    }

    // EXECUTE ACTIVITY

    else if (bCommand.get(0).asString() == "whatIs") {
        bReply.addString("ack");
        if (bCommand.size() == 2)
        {
            bReply.addList() = whatIs(bCommand.get(1).asString());
        }
        else
        {
            yWarning("Error in abmReasoning::respond wrong size of input | (whatIs input)");
            bReply.addString("Error in abmReasoning::respond wrong size of input | (whatIs input)");
        }
    }


    // EXECUTE SHARED PLAN

    else if (bCommand.get(0).asString() == "executeSharedPlan") {
        bReply.addString("ack");
        bReply.addList() = executeSharedPlan(bCommand);
    }

    // GRAMMAR

    else if (bCommand.get(0).asString() == "askGrammar") {
        bReply.addString("ack");
        bReply.addList() = askGrammar(bCommand);
    }

    // WORD KNOWLEDGE

    else if (bCommand.get(0).asString() == "askWordKnowledge") {
        bReply.addString("ack");

        // Get Context

        bReply.addList() = askWordKnowledge(bCommand);
    }


    // UPDATE LOCATION

    else if (bCommand.get(0).asString() == "updateLocation") {
        bReply.addString("ack");
        bReply.addList() = updateKnownLocations();
    }

    else if (bCommand.get(0).asString() == "displayContextual") {
        bReply.addString("ack");
        if (bCommand.size() == 3)
        {
            displayContextual(bCommand.get(1).toString(), bCommand.get(2).toString ());
        }
        else
        {

            yInfo() << " displaying " << listContextualKnowledge.size() << " contextual knowledge";
            for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin();
                itCK != listContextualKnowledge.end();
                itCK++)
            {
                displayContextual(itCK->sName, itCK->sArgument);
            }
        }
    }



    // UPDATE OBJECT LOCATION

    else if (bCommand.get(0).asString() == "updateObjectLocation") {
        bReply.addString("ack");
        bReply.addList() = updateOpcObjectLocation(bCommand.get(1).toString().c_str());
    }

    //no command recognize
    else {
        bReply.addString("nack");
        bReply.addString("wrong commands. Available are : findActivity <actionName> <begin|end|both> [<colums,to,select>] | findSharedPlan <sharedPlanName> <begin|end|both> [<colums,to,select>] | help | sqlQueryTest | quit ");
    }

    yInfo() << "\t" << "reply : " << bReply.toString().c_str() << "\n";
    handlerPort.reply(bReply);

    return true;
}


bool abmReasoning::interruptModule()
{
    yInfo() << "\t" << "Interrupting your module, for port cleanup";

    handlerPort.close();
    senderPort.close();

    realOPC->interrupt();
    realOPC->close();
    mentalOPC->interrupt();
    mentalOPC->close();

    return true;
}


bool abmReasoning::close()
{
    yInfo() << "\t" << "Calling close function\n";
    //  yInfo() << "\t" << saveKnowledge().toString()  ;

    if (realOPC->isConnected()) delete realOPC;
    if (mentalOPC->isConnected()) delete mentalOPC;

    return true;
}


Bottle abmReasoning::connectOPC(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 2)
    {
        bOutput.addString("Error in connect, wrong number of input");
    }

    if (!bInput.get(1).isString())
    {
        bOutput.addString("Error in connect, wrong format of input");
    }

    string sPortTemp = moduleName + "/" + abmReasoningFunction::s_realOPC;
    realOPC = new OPCClient(sPortTemp.c_str());
    int iTry = 0;
    while (!realOPC->isConnected())
    {
        yInfo() << "\t" << "abmReasoning Connecting to " << abmReasoningFunction::s_realOPC << "..." << realOPC->connect(abmReasoningFunction::s_realOPC);
        if (!realOPC->isConnected())
            Time::delay(0.5);
        iTry++;
        if (iTry > 1)
        {
            yInfo() << "\t" << "abmReasoning failed to connect to " << abmReasoningFunction::s_realOPC;
            bOutput.addString("Connection failed, please check your port");
            break;
        }
    }

    if (realOPC->isConnected())
    {
        realOPC->checkout();
        realOPC->update();
    }

    sPortTemp = moduleName + "/" + abmReasoningFunction::s_mentalOPC;
    mentalOPC = new OPCClient(sPortTemp.c_str());
    iTry = 0;
    while (!mentalOPC->isConnected())
    {
        yInfo() << "\t" << "abmReasoning Connecting to " << abmReasoningFunction::s_mentalOPC << "..." << mentalOPC->connect(abmReasoningFunction::s_mentalOPC);
        if (!mentalOPC->isConnected())
            Time::delay(0.5);
        iTry++;
        if (iTry > 1)
        {
            yInfo() << "\t" << "abmReasoning failed to connect to " << abmReasoningFunction::s_mentalOPC;
            bOutput.addString("Connection failed, please check your port");
            return bOutput;
        }
        mentalOPC->isVerbose = false;
    }
    mentalOPC->checkout();
    mentalOPC->update();

    bOutput.addString("Connection done");
    return bOutput;
}


abmReasoning::~abmReasoning()
{
    delete iCub;
    if (iFunction != NULL)
        delete iFunction;
}


void abmReasoning::changeDreaming()
{
    bDreaming = !bDreaming;
    yInfo() << "\t" << "bDreaming changed to : " << bDreaming;
}



#include <abmReasoning.h>
#include <functional>
 
#define THRESHOLD_CONFIDENCE 5.0


/*   -------------------------   MODULE RELATED FUNCTIONS  ------------------------------   */

abmReasoning::abmReasoning(ResourceFinder &rf)
{
    iFunction = new abmReasoningFunction(rf);
    path = rf.getContextPath();
    savefile = (rf.getContextPath()+"/saveRequest.txt").c_str();
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
    return true ;
}

/* configure the module */
bool abmReasoning::configure(ResourceFinder &rf)
{       
    moduleName = rf.check("name", 
        Value("abmReasoning"), 
        "module name (string)").asString();

    setName(moduleName.c_str());

    string portHandlerName = "/";
    portHandlerName +=  getName() + "/rpc";
    handlerPort.open(portHandlerName.c_str());

    string portSenderName = "/";
    portSenderName +=  getName() + "/request:o";
    senderPort.open(portSenderName.c_str());
    bDreaming = false;

    
    iCub = new ICubClient(moduleName,"abmReasoning/conf","client.ini",false);
    iCub->opc->isVerbose = false;

    Network::connect(senderPort.getName(), "/autobiographicalMemory/request:i");

    Interlocutor.initialize();
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
    pddlSolDelete(1,pddlNb);

    bestSol = -1;

    std::cout << endl << endl << "----------------------------------------------" << endl << endl << "abmReasoning ready !" << endl << endl;

    bReady = true;
    bool bTestLanguage = false;

    if (bTestLanguage)
    {
        //  if (!rf.check("noSentences"))   findAllSentence();
        if (!rf.check("noSentences"))   findAllSentence();
        listGrammarKnowledge.testModel(100,2);
        for (int i = 1 ; i < 8 ; i++)
        {
            if (i !=2)
            {
                listGrammarKnowledge.clear();
                listGrammarKnowledge.simulateLearning(i,5);
                listGrammarKnowledge.testModel(100,i);
            }
        }
    }
    return false;
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
    plannerOptDom += " " ;

    plannerOptProb = (bPlanner.check("plannerOptProb", 
        Value("-f"), 
        "planner optProb (string)")).asString();
    plannerOptProb += " " ;

    plannerOptOut = (bPlanner.check("plannerOptOut", 
        Value("-out"), 
        "planner optOut (string)")).asString();
    plannerOptOut += " " ;

    plannerOptNb = (bPlanner.check("plannerOptNb", 
        Value("-n"), 
        "planner optNb (int)")).asString();
    plannerOptNb += " " ;

    plannerOptCpu = (bPlanner.check("plannerOptCpu", 
        Value("-cputime"), 
        "planner optCpu (int)")).asString();
    plannerOptCpu += " " ;

    pddlDomain = (bPlanner.check("pddlDomain",  Value("domainEFAA.pddl"))).asString();  //name of the domain pddl file
    pddlProblem = (bPlanner.check("pddlProblem",    Value("problemEFAA.pddl"))).asString();  //name of the problem pddl file
    pddlOut = (bPlanner.check("pddlOut",    Value("solutionEFAA"))).asString();  //name of the solution plan file -> will be called solutionEFAA_1.SOL, solutionEFAA_2.SOL, ...
    pddlNb = (bPlanner.check("pddlNb",  Value(30))).asInt();  //nb max of solution files produced
    pddlCpu = (bPlanner.check("pddlCpu",    Value(2))).asInt();  //nb max of solution files produced


    //pddlDomain = "domainEFAA.pddl";       //name of the domain pddl file
    //pddlProblem = "problemEFAA.pddl" ;      //name of the problem pddl file
    //pddlOut = "solutionEFAA" ;      //name of the solution plan file -> will be called solutionEFAA_1.SOL
    
    //there is no last Action => used to update vCurrentActions in findAllSharedPlan
    saveEndLastAction = -1 ;
}


/* Respond function */
bool abmReasoning::respond(const yarp::os::Bottle& bCommand, yarp::os::Bottle& bReply)
{  

    bReply.clear(); 
    if (!bReady)
    {
        bReply.addString("module not initialised yet, please wait.");
        std::cout << "reply : " << bReply.toString().c_str() << "\n" << endl; 
        handlerPort.reply(bReply);
        return true;
    }

    if (bCommand.get(0).asString()=="quit") {
        bReply.addString("ack");
        bReply.addString("quitting");
        return false;     
    }

    else if (bCommand.get(0).asString()=="sqlQueryTest") {
        bReply.addString("ack");
//      bReply.addList() = sqlQueryTest();
    }


    // FIND ACTIVITY

    else if (bCommand.get(0).asString()=="findActivity") {

        std::cout << "bCommand.size() = " << bCommand.size() << endl ;

        if (bCommand.size() <= 2) {
            bReply.addString("nack");
            bReply.addString("Usage : findActivity <actionName> 'begin'|'end'|'both' [columns,to,select]");
        } else if(bCommand.get(2).asString() != "begin" && bCommand.get(2).asString() != "end" && bCommand.get(2).asString() != "both") {
            bReply.addString("nack");
            bReply.addString("Usage : findOPC <actionName> 'begin'|'end'|'both' [<colums,to,select>] => check your begin/end/both");
        } else {
            bReply.addString("ack");

            string actionName = bCommand.get(1).asString().c_str() ;
            string beginOrEnd = bCommand.get(2).asString().c_str() ;
            if (bCommand.size() == 3) {
                bReply.addList() = findActivity(actionName, beginOrEnd);
            } else {
                string select = bCommand.get(3).asString().c_str();
                bReply.addList() = findActivity(actionName, beginOrEnd, select);
            }
        }
    }


    // FIND OPC

    else if (bCommand.get(0).asString()=="findOPC") {

        if (bCommand.size() <= 2 ) {
            bReply.addString("nack");
            bReply.addString("Usage : findOPC <actionName> 'begin'|'end'|'both' [<colums,to,select>]");
        } else if(bCommand.get(2).asString() != "begin" && bCommand.get(2).asString() != "end" && bCommand.get(2).asString() != "both") {
            bReply.addString("nack");
            bReply.addString("Usage : findOPC <actionName> 'begin'|'end'|'both' [<colums,to,select>] => check your begin/end/both");
        }   else {
            bReply.addString("ack");
            string actionName = bCommand.get(1).asString().c_str() ;
            string beginOrEnd = bCommand.get(2).asString().c_str();
//          bReply.addList() = findOPC(actionName, beginOrEnd);
        }
    }


    // GET CONSEQUENCES DRIVES

    else if (bCommand.get(0).asString()=="getActionConsequenceDrives") {

        if (bCommand.size() <= 2) {
            bReply.addString("nack");
            bReply.addString("Usage : getActionConsequenceDrives 'actionName' 'arg'");
        }

        else {
            bReply.addString("ack");
            pair<string, string> pAction ( bCommand.get(1).asString().c_str() , bCommand.get(2).asString().c_str() ) ;
            bReply.addList() = getActionConsequenceDrives(pAction);
        }
    }

    // PRINT PDDL DOMAIN

    else if (bCommand.get(0).asString()=="printPDDLContextualKnowledgeDomain") {

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

    else if (bCommand.get(0).asString()=="printPDDLContextualKnowledgeProblem") {

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
            Bottle bGoal ;
            bGoal = *bCommand.get(1).asList();
            bReply.addList() = printPDDLContextualKnowledgeProblem(bGoal);
        }
    }

    // HELP

    else if (bCommand.get(0).asString()=="help") {
        bReply.addString("ack");
        bReply.addString("commands are: findActivity <actionName> <begin|end|both> [<colums,to,select>]| findSharedPlan <sharedPlanName> <begin|end|both> [<colums,to,select>] | help | sqlQueryTest | quit ");
    }


    // ASK LAST ACTION

    else if (bCommand.get(0).asString() == "askLastAction") {
        bReply.addString("ack");
        bReply.addList() = Interlocutor.askLastAction();
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
        bReply.addList() = Interlocutor.imagineOPC(atoi(bCommand.get(1).toString().c_str()));
    }

    
    // GET ACTION 
    else if (bCommand.get(0).asString() == "getAction") {
        bReply.addString("ack");
        bReply.addList() = Interlocutor.askActionFromId(atoi(bCommand.get(1).toString().c_str()));
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
            bReply.addList() = retroReasoning(atoi(bCommand.get(1).toString().c_str()));
        }
        else
        {
            bReply.addList() = resetKnowledge();
            bReply.addList() = retroReasoning();
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


    // EXECUTE ACTIVITY

    else if (bCommand.get(0).asString() == "executeActivity") {
        bReply.addString("ack");
        bReply.addList() = executeActivity(bCommand);
    }


    // EXECUTE SHARED PLAN

    else if (bCommand.get(0).asString() == "executeSharedPlan") {
        bReply.addString("ack");
        bReply.addList() = executeSharedPlan(bCommand);
    }

    // EXECUTE SHARED PLAN

    else if (bCommand.get(0).asString() == "askGrammar") {
        bReply.addString("ack");
        bReply.addList() = askGrammar(bCommand);
    }


    // UPDATE LOCATION

    else if (bCommand.get(0).asString() == "updateLocation") {
        bReply.addString("ack");
        bReply.addList() = updateKnownLocations();
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

    std::cout << "reply : " << bReply.toString().c_str() << "\n" << endl; 
    handlerPort.reply(bReply);

    return true;
}


bool abmReasoning::interruptModule()
{
    std::cout<<"Interrupting your module, for port cleanup"<<endl;

    Interlocutor.close();
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
    std::cout<<"Calling close function\n";
//  std::cout << saveKnowledge().toString() << endl;

    if (realOPC->isConnected()) delete realOPC;
    if (mentalOPC->isConnected()) delete mentalOPC;

    return true;
}


Bottle abmReasoning::connectOPC(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() !=2)
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
    while(!realOPC->isConnected())
    {  
        std::cout<<"abmReasoning Connecting to " << abmReasoningFunction::s_realOPC << "..." << realOPC->connect(abmReasoningFunction::s_realOPC) <<endl;
        if (!realOPC->isConnected())
            Time::delay(0.5);
        iTry++;
        if (iTry > 1)
        {
            std::cout << "abmReasoning failed to connect to " << abmReasoningFunction::s_realOPC << endl;
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
    while(!mentalOPC->isConnected())
    {  
        std::cout<<"abmReasoning Connecting to " << abmReasoningFunction::s_mentalOPC << "..." << mentalOPC->connect(abmReasoningFunction::s_mentalOPC) <<endl;
        if (!mentalOPC->isConnected())
            Time::delay(0.5);
        iTry++;
        if (iTry > 1)
        {
            std::cout << "abmReasoning failed to connect to " << abmReasoningFunction::s_mentalOPC << endl;
            bOutput.addString("Connection failed, please check your port");
            return bOutput;
        }
    }
    mentalOPC->checkout();
    mentalOPC->update();

    bOutput.addString("Connection done");
    return bOutput;
}


abmReasoning::~abmReasoning()
{
    if (iFunction != NULL)
        delete iFunction;
}


void abmReasoning::changeDreaming()
{
    bDreaming = !bDreaming;
    std::cout << "bDreaming changed to : " << bDreaming << endl;
}



/*   -------------------------   METHODS CALLED VIA HANDLERPORT  ------------------------------   */



// Send a SQL query (within a bottle) to AutobiographicalMemory. bRequest must be the complete request
Bottle abmReasoning::request(Bottle bRequest)
{
    Bottle bReplyRequest;
    senderPort.write(bRequest, bReplyRequest);
    return bReplyRequest;
}


Bottle abmReasoning::requestFromStream(string sInput)
{
    Bottle bReplyRequest,
        bQuery;
    bQuery.addString("request");
    bQuery.addString(sInput.c_str());
    senderPort.write(bQuery, bReplyRequest);
    return bReplyRequest;
}

/* TODO */
Bottle abmReasoning::save(Bottle bInput)
{
    //TODO
    Bottle bOutput;
    bOutput = bInput;
    //  output.addString("file saved");
    return bOutput;
}


Bottle abmReasoning::sqlQueryTest()
{
    Bottle bOutput;

    //check : simple object query :
    Bottle bTest;
    bTest.addString("request");
    bTest.addString("SELECT * FROM object WHERE instance <= 3") ;
    bOutput = request(bTest);

    return bOutput;
}


/*   -------------------------   QUERY FROM OUTSIDE  ------------------------------   */


Bottle abmReasoning::findActivity(string actionName, string beginOrEnd, string select)
{
    //build the SQL query
    ostringstream os;
    os << "SELECT " << select << " FROM main WHERE activityname = '" << actionName << "'" ;

    //complete sqlQuery if want begin or end
    if (beginOrEnd != "both") {

        //begin
        string isBegin = "true" ;
        if (beginOrEnd == "end") {
            //end
            isBegin = "false" ;
        }
        os << " AND begin = " << isBegin ;
    }

    return requestFromStream(os.str().c_str());
}


Bottle abmReasoning::findActivityById(int id, string select)
{
    //build the SQL query
    ostringstream os;
    os << "SELECT " << select << " FROM main WHERE instance = '" << id << "'" ;

    return requestFromStream(os.str().c_str());
}

/*
* Return the consequence on the drive of a given behavior
* input format : queryBehavior number name argument
* if no argument given all the behavior are asked
*/
Bottle abmReasoning::queryBehavior(Bottle bInput)
{
    Bottle bOutput;
    string sName, sArgument;
    int last;
    string sError = "Error in abmReasoning::queryBehavior | ";
    if (bInput.size() < 3)
    {
        sError += "Wrong number of input (<2).";
        std::cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    sName = bInput.get(2).toString();
    last = atoi(bInput.get(1).toString().c_str());
    if (last < 1)
    {
        sError += "number of last action to low";
        std::cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (sName == "all")
    {
        for (unsigned int b = 0 ; b < listBehaviors.size() ; b++)
        {
            bOutput.addList() = listBehaviors[b].getConsequence(last);
        }
        return bOutput;
    }

    bOutput.addString(sName.c_str());
    // if the argument is specified :
    if (bInput.size() == 4)
    {
        sArgument = bInput.get(3).toString();
        for (unsigned int b = 0 ; b < listBehaviors.size() ; b++)
        {
            if (listBehaviors[b].sName == sName && listBehaviors[b].sArgument == sArgument)
            {
                return listBehaviors[b].getConsequence(last);
            }
        }
    }


    //if the argument is not specified
    vector <pair <string, vector <double> > >   vDrive;

    // for every behavior
    for (unsigned int b = 0 ; b < listBehaviors.size() ; b++)
    {
        if (listBehaviors[b].sName == sName)
        {
            int loop = 0,
                legal = listBehaviors[b].vEffect.size() - last;

            // for each time the behavior has been seen
            for (vector < vector< pair <string, double> > >::iterator it_occurence = listBehaviors[b].vEffect.begin() ; it_occurence != listBehaviors[b].vEffect.end() ; it_occurence++ )
            {
                if (loop >= legal)
                {
                    // for each drive changed
                    for (vector< pair <string, double> >::iterator it_drive = it_occurence->begin() ; it_drive != it_occurence->end() ; it_drive++)
                    {

                        bool driveKnown = false;

                        // for each known effect
                        for (vector <pair <string, vector <double> > >::iterator it_effect = vDrive.begin() ; it_effect != vDrive.end() ; it_effect ++)
                        {

                            // if the drive is already know
                            if (it_effect->first == it_drive->first && !driveKnown)
                            {
                                it_effect->second.push_back(it_drive->second);
                                driveKnown = true;
                            }
                        }
                        // if first time we see this drive
                        if (!driveKnown)
                        {
                            pair<string, vector <double> > newDrive;
                            newDrive.first = it_drive->first;
                            newDrive.second.push_back(it_drive->second);
                            vDrive.push_back(newDrive);
                        }
                    }
                }
                loop++;
            }
        }
    }       


    //unsigned int sizeMin = 99999;
    if (vDrive.size() == 0)
    {
        return bOutput;
    }
    Bottle bDrive;
    for (unsigned int i = 0 ; i < vDrive.size() ; i++)
    {
        bDrive.clear();
        bDrive.addString(vDrive[i].first.c_str());
        double sum = 0.;
        for (unsigned int j = 0 ; j < vDrive[i].second.size() ; j++)
        {
            sum += vDrive[i].second[j];
        }
        bDrive.addDouble(sum/(1.*vDrive[i].second.size()));
        bOutput.addList() = bDrive;
    }

    return bOutput;
}

/*   -------------------------   GET ID (main.instance)  ------------------------------   */


vector<pair<int, int> > abmReasoning::getIdFromActivity(string actionName, Bottle bRoleAndRoleValue)
{
    //bottle bOutput to reply to the outside module
    Bottle bOutput;

    //bottle bQuery to ask autobiographicalMemory for ABM data 
    Bottle bQuery;

    bQuery.addString("request");

    //build the SQL query - first part is always the same
    ostringstream os;
    if (actionName != abmReasoningFunction::TAG_DB_NONE) {
        os << "SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << actionName << "'" ;
    } else {
        os << "SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance" ;
    }

    //go through the vector to build the ohter part of the SQL query
    for (int i = 0 ; i < bRoleAndRoleValue.size() ; i++)
    {
        os << " AND contentarg.instance IN (SELECT contentarg.instance FROM contentarg WHERE contentarg.role = '" << bRoleAndRoleValue.get(i).asList()->get(0).toString() << "' AND contentarg.argument = '" << bRoleAndRoleValue.get(i).asList()->get(1).toString() << "') " ;
    }

    os << " ORDER BY main.instance ASC" ;

    bOutput = requestFromStream(os.str().c_str());

    std::cout << "Output Bottle in getIdFromActivity : " << bOutput.toString() << endl ;

    return getIdPairsFromBottle(bOutput) ;

}

//Assuming the bottle have a list of sorted id (done in getIdFromActivity with the ORDER BY)
vector<pair<int, int> > abmReasoning::getIdPairsFromBottle(Bottle idBottle)
{
    vector <pair <int, int> > vOutput;

    //  std::cout << "Bottle received in getIdPairsFromBottle : " << idBottle.toString() << endl ;

    //-1 because we have iEnd which is iBegin+1
    for(int iBegin = 0 ; iBegin < idBottle.size()-1 ; iBegin ++ ) {
        int iEnd = iBegin + 1;
        pair<int, int> pActivity;
        Bottle bActBegin = findActivityById(atoi(idBottle.get(iBegin).asList()->get(0).toString().c_str()), "activitytype, begin") ;

        //look for an activity which is begin
        if (bActBegin.get(0).asList()->get(1).toString() == "t" || bActBegin.get(0).asList()->get(1).toString() == "TRUE" ) {

            int nbComplexWithin = 0 ;
            bool endComplexFound = 0 ;

            //          std::cout << "-----> Next activity (" << idBottle.get(iEnd).asList()->get(0).toString() << ")" << endl ;

            //if complex : maybe it is not the next one
            if (bActBegin.get(0).asList()->get(0).toString() == "complex") {

                //              std::cout << "=======> Begin activity is a complex" << endl;

                while (endComplexFound == 0){

                    Bottle bActEnd = findActivityById(atoi(idBottle.get(iEnd).asList()->get(0).toString().c_str())  , "activitytype, begin") ;
                    std::cout << "Is " << atoi(idBottle.get(iEnd).asList()->get(0).toString().c_str()) << " the end of this complex?" ;

                    //go to find the next complex activity
                    if(bActEnd.get(0).asList()->get(0).toString() == "complex"){
                        std::cout << "=======> End activity (id" << idBottle.get(iEnd).asList()->get(0).toString() << ")  is a complex" << endl;

                        //this complex action is a begin : it is a complex within
                        if(bActEnd.get(0).asList()->get(1).toString() == "t" || bActBegin.get(0).asList()->get(1).toString() == "TRUE"){
                            nbComplexWithin += 1 ;
                            std::cout << "=======> End activity is a complex but which Begin : nbComplexWithin == " << nbComplexWithin << endl;
                        } else {
                            nbComplexWithin -= 1 ; //include the -1 for the END of the complex we want so nbComplexWithin will be -1
                            std::cout << "==========> Found an END COMPLEX  (id " << idBottle.get(iEnd).asList()->get(0).toString() << ")" << endl;
                        }

                        if (nbComplexWithin < 0) {
                            endComplexFound = 1 ;
                            std::cout << "==========> Found the proper END COMPLEX  (id " << idBottle.get(iEnd).asList()->get(0).toString() << ")" << endl;
                        }

                    }

                    //incremente iEnd if endComplex is not found
                    if (endComplexFound == 0){
                        iEnd += 1 ;
                    }

                }

            }

            //if activity is action, the next one is the end, or iEnd has been modified if complex
            //          std::cout << "(idBottle)  Pair is ("<< idBottle.get(iBegin).asList()->get(0).toString() << ", " << idBottle.get(iEnd).asList()->get(0).toString() << ")" << endl;
            pActivity.first = atoi(idBottle.get(iBegin).asList()->get(0).toString().c_str());
            pActivity.second = atoi(idBottle.get(iEnd).asList()->get(0).toString().c_str());
            //          std::cout << "(pActivity) Pair is ("<< pActivity.first << ", " << pActivity.second << ")" << endl;

            vOutput.push_back(pActivity);
        }
    }

    std::cout << "____________________________________ FINAL _____________________________________________" << endl ;
    for (vector< pair< int, int> >::iterator it = vOutput.begin() ; it != vOutput.end(); it++)
    {
        pair<int, int> pActivity ( it->first , it->second ) ; 

        std::cout << "Pair is ("<< pActivity.first << ", " << pActivity.second << ")" << endl;
    }
    std::cout << "________________________________________________________________________________________" << endl ;

    return vOutput ;
}


vector<pair<int, int> > abmReasoning::getIDfromTime(Bottle bInput)
{
    vector <pair <int, int> > vOutput;  // output
    string  sBetween,                   // date between which the search will be done
        sRequest;
    if (bInput.size() != 3)
    {
        std::cout << "Error in abmReasoning::getIDfromTime | Wrong number of input  (!=3)" << endl;
        return vOutput;
    }

    if (bInput.get(0).toString() == "between")
    {
        sBetween  = bInput.get(1).toString() + "' AND '";
        sBetween += bInput.get(2).toString();
    }
    else if (bInput.get(0).toString() == "ago")
    {
        pair<int, string> pAgo ( bInput.get(1).asInt() , bInput.get(2).toString() ) ;
        pair<string, string> pDates = abmReasoningFunction::ago2string(pAgo);
        sBetween  = pDates.first + "' AND '";
        sBetween += pDates.second;
    }
    else
    {
        std::cout << "Error in abmReasoning::getIDfromTime | can't recognize temporal argument" << endl;
        //  return vOutput;
    }

    Bottle  bRequest,       // Use for request to ABM
        bActivity,      // Temporary Bottle with the activity
        bIdBegin;       // List of activity begin

    // Get all the activities that begin during the laps of time, with ID and activitytype
    bIdBegin= requestFromStream("SELECT instance, activitytype FROM main WHERE begin = true");
    vector< pair<int, string> > listBegin;

    for (int i = 0; i < bIdBegin.size() ; i++)
    {
        bActivity = *bIdBegin.get(i).asList();
        pair<int, string> pActivity ( atoi(bActivity.get(0).toString().c_str()) , bActivity.get(1).toString() ) ;
        listBegin.push_back(pActivity);
    }


    for (vector< pair< int, string> >::iterator it = listBegin.begin() ; it != listBegin.end(); it++)
    {
        pair<int, int> pActivity;
        pActivity.first = it->first;
        if (it->second == "action")
        {
            pActivity.second = (it->first)+1;
            vOutput.push_back(pActivity);
        }
        else if (it->second == "complex")
        {
            ostringstream osRequest;
            osRequest << "SELECT activityname FROM main WHERE instance = " << it->first;
            bActivity = requestFromStream(osRequest.str().c_str());
            string sActivityName = bActivity.toString().c_str();

            osRequest.str("");
            osRequest << "SELECT instance FROM main WHERE instance > " << it->first << " AND activityname = '" << sActivityName << "' AND begin = false ORDER BY instance LIMIT 1 ";
            bActivity = requestFromStream(osRequest.str().c_str());
            pActivity.second = atoi(bActivity.toString().c_str());
        }
    }

    return vOutput;
}

/**
* Return the position of the object of focus of an action before and after execution.
* @param name : name of the action to get
*/
Bottle abmReasoning::getActionConsequence(pair<string, string> pNameArg)
{
    string sName = pNameArg.first;
    string sArg  = pNameArg.second;
    //bottle bOutput to reply to the outside module
    Bottle bOutput;
    bOutput.addString(sName.c_str());

    Bottle bArguments;

    //bottle bQuery to ask autobiographicalMemory for ABM data 
    Bottle bOpcIdBegin;
    Bottle bOpcIdEnd;

    Bottle bIdArgBegin;
    Bottle bSubTypeArgBegin ;

    Bottle bPosArgBegin ;
    Bottle bPosArgEnd ;

    Bottle bOutputTemp;
    Bottle bQuery;

    //extract the instances of the OPC

    ostringstream osBegin, osEnd;

    osBegin << "SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << sName.c_str() <<"' AND '" << sArg.c_str() << "' IN (contentarg.argument) AND main.begin = TRUE" ;
    osEnd << "SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << sName <<"' AND '" << sArg << "' IN (contentarg.argument) AND main.begin = FALSE" ;

    bOpcIdBegin = requestFromStream(osBegin.str().c_str());
    bOpcIdEnd = requestFromStream(osEnd.str().c_str());

    int numberAction = bOpcIdEnd.size();
    std::cout << "Getting action consequence of : " << sName << " " << sArg << ". " << numberAction << " occurences." << endl;
    for(int i = 0; i < numberAction ; i++){

        std::cout << i+1 << ".. ";

        int opcIdBegin = atoi(bOpcIdBegin.get(i).asList()->get(0).toString().c_str()) ;
        int opcIdEnd = atoi(bOpcIdEnd.get(i).asList()->get(0).toString().c_str()) ;

        //-- 0. extract all the arguments
        ostringstream osArg;
        osArg << "SELECT argument FROM contentarg WHERE instance = " << opcIdBegin;
        bArguments = requestFromStream(osArg.str().c_str());
        //      std::cout << "bArguments : " << bArguments.toString() << endl;
        bool bCheck = false;
        for (int k = 0; k < bArguments.size(); k++)
        {
            if (bArguments.get(k).toString().c_str() == sArg){
                bCheck = true;}
        }
        if (bCheck)
        {

            bOutput.addList() = bArguments;

            ostringstream osArgObj ;
            osArgObj << "SELECT DISTINCT argument FROM contentarg WHERE instance = " << opcIdBegin << " AND role = 'object1'";
            Bottle bArgObj = requestFromStream(osArgObj.str().c_str());

            string argObject = bArgObj.get(0).toString().c_str();

            //-- 1. extract the id of the argument, assuming it is an entity
            ostringstream osEntity;
            osEntity << "SELECT DISTINCT entity.opcid FROM entity, contentarg WHERE entity.name = contentarg.argument AND entity.instance = " << opcIdBegin << " AND contentarg.argument = '" << argObject << "'";
            bIdArgBegin = requestFromStream(osEntity.str().c_str());
            int idArg = atoi(bIdArgBegin.get(0).asList()->get(0).toString().c_str()) ;

            //-- 2. select the subtype of the argument in order to extract it accordingly
            osEntity.str("");
            osEntity << "SELECT contentopc.subtype from contentopc WHERE contentopc.instance = "<< opcIdBegin << " AND contentopc.opcid = " << idArg ;
            bSubTypeArgBegin = requestFromStream(osEntity.str().c_str());
            string subtypeArg = bSubTypeArgBegin.get(0).asList()->get(0).toString().c_str() ;

            //-- 3. extract the x, y of the object at begin and end of the activity
            osEntity.str("");
            osEntity << "SELECT " << subtypeArg << ".position FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << opcIdBegin << " AND " << subtypeArg << ".opcid = " << idArg ;
            bPosArgBegin = requestFromStream(osEntity.str().c_str());

            osEntity.str("");
            osEntity << "SELECT " << subtypeArg << ".position FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << opcIdEnd << " AND " << subtypeArg << ".opcid = " << idArg ;
            bPosArgEnd = requestFromStream(osEntity.str().c_str());

            string posArgBegin = bPosArgBegin.get(0).asList()->get(0).toString().c_str() ;
            string posArgEnd = bPosArgEnd.get(0).asList()->get(0).toString().c_str() ;


            osEntity.str("");
            osEntity << "SELECT presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << opcIdBegin << " AND " << subtypeArg << ".opcid = " << idArg ;
            bPosArgBegin = requestFromStream(osEntity.str().c_str());
            int ObjectPresentBefore, ObjectPresentAfter;
            string test = "t";
            if (bPosArgBegin.get(0).asList()->get(0).toString().c_str() == test)
                ObjectPresentBefore = 1;
            else
                ObjectPresentBefore = 0;

            osEntity.str("");
            osEntity << "SELECT presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << opcIdEnd << " AND " << subtypeArg << ".opcid = " << idArg ;
            bPosArgBegin = requestFromStream(osEntity.str().c_str());
            if (bPosArgBegin.get(0).asList()->get(0).toString().c_str() == test)
                ObjectPresentAfter = 1;
            else
                ObjectPresentAfter = 0;

            bOutputTemp.clear();
            bOutputTemp.addString(posArgBegin.c_str());
            bOutputTemp.addString(posArgEnd.c_str());

            Bottle bContextual;
            bContextual.addString(sName.c_str());
            bContextual.addString(sArg.c_str());
            bContextual.addInt(ObjectPresentBefore);
            bContextual.addInt(ObjectPresentAfter);
            
            ostringstream osAgent ;
            osArgObj << "SELECT DISTINCT argument FROM contentarg WHERE instance = " << opcIdBegin << " AND role = 'agent1'";
            bArgObj = requestFromStream(osArgObj.str().c_str());

            string sSubject = bArgObj.get(0).toString().c_str();
            bContextual.addString(sSubject.c_str());


            bContextual = addContextualKnowledge(bContextual);

            bOutput.addList() = bOutputTemp;
        }
    }
    std::cout << endl;

    return bOutput;
}

/**
* Return the position of the object of focus of an action before and after execution.
* @param name : name of the action to get and name of the argument (e.g. : put east, push west, play gangnam-style, put none, ...). If arg is none, just the actionName is used
*/
Bottle abmReasoning::getActionConsequenceDrives(pair<string, string> pNameArg)
{
    string sName = pNameArg.first;
    string sArg  = pNameArg.second;
    //bottle bOutput to reply to the outside module
    Bottle bOutput;
    bOutput.addString(sName.c_str());

    Bottle bArguments;

    //bottle bQuery to ask autobiographicalMemory for ABM data 
    Bottle bOpcIdBegin;
    Bottle bOpcIdEnd;

    Bottle bIdArgBegin;
    Bottle bSubTypeArgBegin ;

    Bottle bDrivesArgBegin ;
    Bottle bDrivesArgEnd ;

    Bottle bOutputTemp;
    Bottle bQuery;

    //extract the instances of the OPC

    ostringstream osBegin, osEnd;

    //arg of the action is important
    if (sArg != abmReasoningFunction::TAG_DB_NONE) {
        osBegin << "SELECT main.instance, drives.name, drives.value FROM main, drives WHERE main.instance = drives.instance AND main.instance IN (SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << sName.c_str()  << "' AND '" << sArg.c_str() << "' IN (contentarg.argument) AND main.begin = TRUE) ORDER BY main.instance" ;
        osEnd << "SELECT main.instance, drives.name, drives.value FROM main, drives WHERE main.instance = drives.instance AND main.instance IN (SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << sName.c_str()  << "' AND '" << sArg.c_str() << "' IN (contentarg.argument) AND main.begin = FALSE) ORDER BY main.instance" ;
    } else {
        osBegin << "SELECT main.instance, drives.name, value FROM drives, main WHERE main.instance = drives.instance AND main.activityname =  '" << sName.c_str() <<"' AND main.begin = TRUE ORDER BY main.instance" ;
        osEnd << "SELECT main.instance, drives.name, value FROM drives, main WHERE main.instance = drives.instance AND main.activityname =  '" << sName.c_str() <<"' AND main.begin = FALSE ORDER BY main.instance" ;
    }


    bOpcIdBegin = requestFromStream(osBegin.str().c_str());
    bOpcIdEnd = requestFromStream(osEnd.str().c_str());
    
    int numberActionBegin = bOpcIdBegin.size();
    int numberActionEnd = bOpcIdEnd.size();

    //check we have only pair begin/end
    if (numberActionBegin != numberActionEnd) {
        std::cout << "ERROR : there are " << numberActionBegin << " action(s) begin = true and " << numberActionEnd << " action(s) begin = false : CLEAN THE DATABASE" << endl;
        bOutput.addString("ERROR");
        return bOutput ;
    } 

    //here, numberAction = numberActionBegin = numberActionEnd
    std::cout << "Getting action consequence (drives) of : " << sName << " " << sArg << ". " << numberActionBegin << " occurences." << endl;
    for(int i = 0; i < numberActionBegin ; i++){

        std::cout << i+1 << ".. ";

        //bOpcId -> instance, driveName, value
        int opcIdBegin = atoi(bOpcIdBegin.get(i).asList()->get(0).toString().c_str());
        int currentInstance = opcIdBegin ;
        int opcIdEnd = atoi(bOpcIdEnd.get(i).asList()->get(0).toString().c_str());

        std::cout << "instance " << currentInstance << endl ;
        //print and do something for all the drives of the same instance
        // /!\ drives are not in the same order and defaultDrive is replaced after by curiosity
        while(atoi(bOpcIdBegin.get(i).asList()->get(0).toString().c_str()) == currentInstance){

            opcIdBegin = atoi(bOpcIdBegin.get(i).asList()->get(0).toString().c_str());
            opcIdEnd = atoi(bOpcIdEnd.get(i).asList()->get(0).toString().c_str());

            string driveNameBegin = bOpcIdBegin.get(i).asList()->get(1).toString().c_str() ;
            string driveNameEnd = bOpcIdEnd.get(i).asList()->get(1).toString().c_str() ;

            double driveValueBegin = atof(bOpcIdBegin.get(i).asList()->get(2).toString().c_str()) ;
            double driveValueEnd = atof(bOpcIdEnd.get(i).asList()->get(2).toString().c_str()) ;

            std::cout << "(" << driveNameBegin << ")" << " : " << driveValueBegin << " -> " << driveValueEnd << endl;

            //next line. /!\ if the line was the last, we can't ask the next one, so currentInstance -1 to end the while loop
            if (i < numberActionBegin-1) {
                i++ ;
            } else {
                currentInstance = -1 ;
            }
        }

        std::cout << endl ;
        //instance is the next, we need to go to the last of the same because of the ++ in the for loop (except if last line)
        if (currentInstance != -1) {
            i--;
        }

    }
    std::cout << endl;

    return bOutput;
}

Bottle abmReasoning::getActionFromPostCondition(pair<string,int> postCondition)
{
    Bottle bListAction ;

    //for all action, check if they could achieve the postCondition
    for ( vector<plan>::iterator it_Plan = listPlan.begin() ; it_Plan != listPlan.end() ; it_Plan++)
    {   
        //if(it_Plan->)
        {

        }
    }

    //for all action to achieve postCondition, check if they are doable (preCondition is true)

    return bListAction ;
}


Bottle abmReasoning::testGetIdFromActivity()
{
    //simple action
    string actionName = abmReasoningFunction::TAG_DB_UNKNOWN ;
    Bottle bRoleAndRoleValue ;

    /*  Bottle bSub1 ;
    bSub1.addString("object1");
    bSub1.addString("ball");
    bRoleAndRoleValue.addList() = bSub1;

    Bottle bSub2 ;
    bSub2.addString("argument");
    bSub2.addString("blue");
    bRoleAndRoleValue.addList() = bSub2;*/

    //complex action
    //string actionName = "beforeputput" ;
    /*string actionName = abmReasoningFunction::TAG_NONE ;
    Bottle bRoleAndRoleValue ;

    Bottle bSub1 ;
    bSub1.addString("object1");
    bSub1.addString("circle");
    bRoleAndRoleValue.addList() = bSub1;*/

    /*Bottle bSub2 ;
    bSub2.addString("object2");
    bSub2.addString("cross");
    bRoleAndRoleValue.addList() = bSub2;*/

    vector <pair <int, int> > vOutput = getIdFromActivity(actionName, bRoleAndRoleValue) ;

    Bottle bOutput ;
    bOutput.addString("Done");

    return bOutput ;
}
Bottle abmReasoning::pddlPlannerSolParser(){
    Bottle bOutput ;

    //first file is fine
    /*char filename[512] = "";
    strcpy(filename, plannerPath.c_str()) ;
    strcat(filename, pddlOut.c_str());
    strcat(filename, "_1.SOL") ;
    ifstream fileSol;
    fileSol.open(filename);*/


    //to check with multiple actions solution
    char filename[512] = "" ;
    for(int i = 1; i <= pddlNb ; i++){
    
        pddlSolFileName(i,filename);
        ifstream fileSol(filename);
        
        //if the file is found (an not the first) we remove the previous file
        if(fileSol && i > 1){
            char fileSolToRemove[512] ;
            pddlSolFileName(i-1, fileSolToRemove);
            if( remove(fileSolToRemove) != 0){
                std::cout << "ERROR : " << fileSolToRemove << " NOT DELETED" << endl ;
            } else {
                std::cout << "File : " << fileSolToRemove << " successfully deleted" << endl ;
            }
        }

        //the solution file does not exist : the previous solution was the one
        if(!fileSol){
            bestSol = i-1;
            i = pddlNb;
            std::cout << "----> Best solution is in the solution file number " << bestSol << endl ;
        }
    }

    ifstream fileSol;
    //open the bestSol file
    pddlSolFileName(bestSol, filename);
    fileSol.open(filename);

    string sLine = "";

    //open the solution file
    if(fileSol.is_open()){

        unsigned int nbAction = 0;

        while (!fileSol.eof()){
            getline(fileSol, sLine);
            std::cout << sLine << endl;
            int actBegin, actEnd ;
            string currentAction ;

            actBegin = sLine.find("(");
            actEnd = sLine.find(")");

            //if found the action line
            if (actBegin != -1 && actEnd != -1){

                string actionName = abmReasoningFunction::TAG_DB_NONE ;
                string locName = abmReasoningFunction::TAG_DB_NONE;
                string objName = abmReasoningFunction::TAG_DB_NONE;
                string agentName = abmReasoningFunction::TAG_DB_NONE ;
                string argName = abmReasoningFunction::TAG_DB_NONE ;

                nbAction += 1 ;
                currentAction = sLine.substr(actBegin+1, actEnd-actBegin-1) ;
                std::cout << "=> Action nb : " << nbAction << " is : " << currentAction << endl ;

                string bufferString ;
                stringstream ss(currentAction) ;
                vector<string> vSplitAction ;

                while (ss >> bufferString) {
                    vSplitAction.push_back(bufferString);
                }

                unsigned int nbArg = 0 ;

                for(vector<string>::iterator it = vSplitAction.begin() ; it != vSplitAction.end() ; it++){
                    nbArg +=1 ;
                    std::cout << "split : " << it->c_str() << endl ;

                    ///////////////////////////////////////////////////////////////////////
                    //         CHANGE IT TO PARSE ACCORDING TO THE GENERALIZATION        //
                    ///////////////////////////////////////////////////////////////////////

                    //first split if actionName = action-loc (PUT-NORTH, PUSH-SOUTH-EAST, ...)
                    // hanoi-obj to from
                    if (nbArg == 1) {
                        int sepBegin = it->find_first_of("-");
                        if (sepBegin != -1){
                            actionName = it->substr(0, sepBegin) ;

                            //upper to lower case
							transform(actionName.begin(), actionName.end(), actionName.begin(), abmReasoning::fixed_tolower);
                            std::cout << "=> actionName : " << actionName << endl ;

                            if(actionName == "hanoi"){
                                //upper to lower case
                                
                                objName = it->substr(sepBegin+1, it->length())  ;
								transform(objName.begin(), objName.end(), objName.begin(), abmReasoning::fixed_tolower);
                                std::cout << "=> objName : " << objName << endl ;
                            } else {
                                //upper to lower case
                                locName = it->substr(sepBegin+1, it->length()) ;
								transform(locName.begin(), locName.end(), locName.begin(), abmReasoning::fixed_tolower);
                                std::cout << "=> locName : " << locName << endl ;
                            }

                            agentName = "icub" ;
                        } else {
                            //ADD, ASK, REMOVE, ...
                            actionName = it->c_str() ;
							transform(actionName.begin(), actionName.end(), actionName.begin(), abmReasoning::fixed_tolower);
                            std::cout << "=> actionName : " << actionName << endl ;

                            //human has to do these kind of actions
                            agentName = "partner";
                        }
                    }

                    ///////////////////////////////////////////////////////////////////////
                    //         CHANGE IT TO PARSE ACCORDING TO THE GENERALIZATION        //
                    ///////////////////////////////////////////////////////////////////////

                    //Put-north obj   => obj
                    //Hanoi-big loc  => loc
                    else if (nbArg == 2){
                        argName = it->c_str();

                        if(actionName == "hanoi"){
                            //upper to lower case
							transform(argName.begin(), argName.end(), argName.begin(), abmReasoning::fixed_tolower);
                            locName = argName ;
                            std::cout << "=> locName : " << locName << endl ;

                        } else {
                            //upper to lower case
							transform(argName.begin(), argName.end(), argName.begin(), abmReasoning::fixed_tolower);
                            objName = argName;
                            std::cout << "=> objName : " << objName << endl ;
                        }
                    }
                }//end go through action split

                //store the action in a bottle
                Bottle bCurrentAction, bArg, bRole ;
                bCurrentAction.clear() ;
                bArg.clear();
                bRole.clear();

                bCurrentAction.addString("action");
                bCurrentAction.addString(actionName.c_str());
                
                if(agentName.c_str()!= ""){
                    bArg.addString(agentName.c_str());
                    bRole.addString("agent1");
                }
                if(objName.c_str()!= ""){
                    bArg.addString(objName.c_str());
                    bRole.addString("object1");
                }
                if(locName.c_str()!= ""){
                    bArg.addString(locName.c_str());
                    bRole.addString("spatial1");
                }

                bCurrentAction.addList() = bArg ;
                bCurrentAction.addList() = bRole ;

                std::cout << "actionName == (" << actionName.c_str() << ")" << endl ;

                //if the action is a put/push -> executeActivity to have the (move absolut/relative (x y) (action put/push (arg...) (role...))) subottle 
                if ( actionName == "put" || actionName == "push" ) {
                    Bottle bForExecute ;
                    bForExecute.addString("executeActivity");
                    bForExecute.addString("action");
                    bForExecute.addString(actionName.c_str());
                    bForExecute.addList() = bArg;
                    bForExecute.addList() = bRole;


                    bCurrentAction = *executeActivity(bForExecute).get(0).asList() ;
                }


                bOutput.addList() = bCurrentAction ;
        
            }//end parse the line

        }//end parse the file

    } else {
        std::cout << "ERROR : Solution find (" << filename << ") not found" << endl ;
        fileSol.close();
        return bOutput ;
    }

    fileSol.close();

    //not erase the solution : will be done next time we do a reasoning
    //pddlSolDelete(bestSol, bestSol);

    std::cout << "Bottle of actions : " << bOutput.toString().c_str()  << endl ;

    return bOutput ;
}

void abmReasoning::pddlSolFileName(int i, char* filename){
	stringstream ss;
	ss << plannerPath << pddlOut << "_" << i << ".SOL";
	strcpy(filename, ss.str().c_str());
}

//remove all the file given in the range
void abmReasoning::pddlSolDelete(unsigned int begin, unsigned int end){
    
    //check parameters
    if ( begin < 1 || end < begin) {
        std::cout << "ERROR : begin has to be > 1, and end >= begin" << endl ;
        return ;
    }

    char filename[512] = "" ;
    //loop to generate filename from XXX_begin.SOL to XXX_end.SOL
    for(unsigned int i = begin; i <= end ; i++){
    
        pddlSolFileName(i,filename);
        ifstream fileSol(filename);

        bool isFileExist ;

        //check if the file is there
        if(fileSol){
            isFileExist = true ;

        } else {
            isFileExist = false ;
        }

        //close file in order to be able to delete it after
        fileSol.close();

        //if file is there, we delete it
        if (isFileExist) {
            char fileSolToRemove[512] ;
            pddlSolFileName(i, fileSolToRemove);
            if( remove(fileSolToRemove) != 0){
                std::cout << "ERROR : " << fileSolToRemove << " NOT DELETED" << endl ;
            } else {
                std::cout << "File : " << fileSolToRemove << " successfully deleted" << endl ;
            }
        }
    }

    return ;
}

/**
* Compute the PDDL planner launching command according to option specified in the conf file
*/
void abmReasoning::pddlPlannerLauncher(){

    //erase the previous solution if there (so that it is not the first time reasoning is called)
    
    if(bestSol>=1){
        char solFilename[512] ;
        pddlSolFileName(bestSol, solFilename);
        std::cout << "-----> Previous solution file was " << solFilename ;
        //remove the file if present
        if ( remove(solFilename) != 0){ 
            std::cout << "===> Previous solution file NOT deleted (" << solFilename << ") : " << endl ;
            std::cout << "- Wrong path?" << endl ;
        } else {
            std::cout << "Previous Solution file successfully deleted!" << endl ;
        }
    }

    //build the command to launch the planner
    char plannerLauncher[1024] = "";
    char cdPlanner[1024] = "";
    char plannerCmds[1024] = "";

    //cdPlanner build
    strcpy(cdPlanner, "cd ");
    strcat(cdPlanner, plannerPath.c_str()) ;

    //plannerLauncher build

    //1. executable
    strcpy(plannerLauncher, plannerExec.c_str());
    strcat(plannerLauncher, " ");

    //2. domain option
    strcat(plannerLauncher,plannerOptDom.c_str());
    strcat(plannerLauncher, " ");
    strcat(plannerLauncher, pddlDomain.c_str()) ;
    strcat(plannerLauncher, " ");

    //3. problem option
    strcat(plannerLauncher,plannerOptProb.c_str());
    strcat(plannerLauncher, " ");
    strcat(plannerLauncher, pddlProblem.c_str()) ;
    strcat(plannerLauncher, " ");

    //4. output option (solution file)
    strcat(plannerLauncher, plannerOptOut.c_str()) ;
    strcat(plannerLauncher, " ");
    strcat(plannerLauncher, pddlOut.c_str()) ;
    strcat(plannerLauncher, " ");

    //5. nb max file produced
    strcat(plannerLauncher,plannerOptNb.c_str());
    strcat(plannerLauncher, " ");
    ostringstream sPddlNb ;
    sPddlNb << pddlNb ; 
    strcat(plannerLauncher, sPddlNb.str().c_str()) ;
    strcat(plannerLauncher, " ");

    //6. nb max cpu time taken (seconds)
    strcat(plannerLauncher,plannerOptCpu.c_str());
    strcat(plannerLauncher, " ");
    ostringstream sPddlCpu;
    sPddlCpu << pddlCpu ;
    strcat(plannerLauncher, sPddlCpu.str().c_str()) ;
    
    std::cout << "executing the command : " << endl << " - " << cdPlanner << endl << " - " << plannerLauncher << endl;

    //grouping both : cd plannerPath + plannerLauncher
    strcpy(plannerCmds, cdPlanner);
    strcat(plannerCmds, " && ");
    strcat(plannerCmds, plannerLauncher);

    //launch both cmds
    std::cout << "System(" << plannerCmds << ")" << endl ;
    system(plannerCmds);

    return;
}

/**
* Print the Domain PDDL file using contextual knowledge (list of action with preconditions/effects)
*/
Bottle abmReasoning::printPDDLContextualKnowledgeDomain()
{
    Bottle bOutput;

    char buffer[512] = "";
    strcpy(buffer, plannerPath.c_str()) ;
    strcat(buffer, pddlDomain.c_str()) ;
    ofstream myfile;
    std::cout << "Domain PDDL written in : " << buffer << endl ;
    myfile.open(buffer, ios_base::ate);


    //print the define, requirements, predicates which will be used
    myfile << ";; STRIPS domain automatically generated by ABMReasoning, part of EFAA" << endl;
    
    myfile << "(define (domain efaa)" << endl;

    myfile << "\t(:requirements :strips :typing :equality)" << endl ;

    myfile << "\t(:predicates" << endl;
    myfile << "\t\t(object ?obj)" << endl ;
    myfile << "\t\t(location ?loc)" << endl ;
    myfile << "\t\t(isPresent ?obj)" << endl ;
    //myfile << "\t\t(isReachable ?obj)" << endl ;
    myfile << "\t\t(isAtLoc ?obj ?loc)" << endl ;
    myfile << "\t)" << endl ; //end parenthesis of predicates

    for (vector<contextualKnowledge>::iterator it = listContextualKnowledge.begin() ; it != listContextualKnowledge.end() ; it++)
    {

            /**************************  PDDL : action  *****************************/
            //action with no argument (spatial) : remove, add, ... : keep the name
            /**************************  PDDL : action  *****************************/

            string obj1, loc1, loc2 ;

            if(it->sArgument == abmReasoningFunction::TAG_DB_NONE){
                myfile << "\t(:action " << it->sName << endl;
            } else {
                //otherwise the name is name-sArg (e.g. : put-north, put-south, put-near, ...) (in particular to managed put-near)
                
                //hanoi_loc
                if(it->sName == "hanoi"){
                    myfile << "\t(:action " << it->sName << "-" << it->sArgument << endl;

                    obj1 = it->sArgument;
                    loc1 = "?from" ;
                    loc2 = "?to" ;
                } else {
                    //move_obj, put_obj
                    myfile << "\t(:action " << it->sName << "-" << it->sArgument << endl;

                    obj1 = "?obj1" ;
                }
            }

            /***********************  PDDL : parameters  **************************/
            //assuming action always are applied to an object1, and if another argument it is a spatial -> in the name
            //       WARNING : this will need to changed for put-near (obj1, obj2)
            /**************************  PDDL : action  *****************************/

            //hanoi_loc
            if(it->sName == "hanoi"){
                myfile << "\t\t:parameters (?to ?from)" << endl ;
            } else {
                myfile << "\t\t:parameters (?obj1)" << endl ;
            }

            /************************************************************************/          
            /***********************  PDDL : precondition  **************************/
            /************************************************************************/

            myfile << "\t\t:precondition (and " ;

            //----------------------> for arguments
            if(it->sName == "hanoi"){
                myfile << "(location ?to) (location ?from) " ;
            } else {
                myfile << "(object ?obj1) " ;
            }

            //hanoi_loc : presence <obj, <from,to>> to check
            if(it->sName == "hanoi"){

                //----------------------> for isAtLoc
                //For the others objects : mPercentObjectFromTo
                for (map<string ,  pair<double, double > >::iterator itIntersect = it->mPercentObjectFromTo.begin() ; itIntersect != it->mPercentObjectFromTo.end() ; itIntersect++)
                {           
                    //if percentage isAtLoc > treshold_sup : isAtLoc
                    if (itIntersect->second.first > abmReasoningFunction::threshold_intersect_sup) {
                        myfile << "(isAtLoc " << itIntersect->first << " ?from) " ;
                    }

                    //if percentage isAtLoc < treshold_inf : not(isAtLoc)
                    if (itIntersect->second.first < abmReasoningFunction::threshold_intersect_inf) {
                        myfile << "(not (isAtLoc " << itIntersect->first << " ?from)) " ;
                    }

                    //if percentage isAtLoc > treshold_sup : isAtLoc
                    if (itIntersect->second.second > abmReasoningFunction::threshold_intersect_sup) {
                        myfile << "(isAtLoc " << itIntersect->first << " ?to) " ;
                    }

                    //if percentage isAtLoc < treshold_inf : not(isAtLoc)
                    if (itIntersect->second.second < abmReasoningFunction::threshold_intersect_inf) {
                        myfile << "(not (isAtLoc " << itIntersect->first << " ?to)) " ;
                    }

                }   

                //For the moved object : mPercentIntersection
                for (map<string ,  pair<double, double > >::iterator itIntersect = it->mPercentIntersectLocation.begin() ; itIntersect != it->mPercentIntersectLocation.end() ; itIntersect++)
                {           
                    //if loc = from
                    if (itIntersect->first == "from" || itIntersect->first == "to") {

                        //second.first = begin = precondition

                        //if percentage isAtLoc > treshold_sup : isAtLoc
                        if (itIntersect->second.first > abmReasoningFunction::threshold_intersect_sup) {
                            myfile << "(isAtLoc " << it->sArgument << " ?" << itIntersect->first << ") " ;
                        }

                        //if percentage isAtLoc < treshold_inf : not(isAtLoc)
                        if (itIntersect->second.first < abmReasoningFunction::threshold_intersect_inf) {
                            myfile << "(not (isAtLoc " << it->sArgument << " ?" << itIntersect->first << ")) " ;
                        }
                    }
                }   


            } else {

                //----------------------> for isPresent
                if(it->PercentPresence.first > abmReasoningFunction::threshold_presence){
                    myfile << "(isPresent " << obj1 << ") " ;
                } else if (it->PercentPresence.first < abmReasoningFunction::threshold_absence){
                    myfile << "(not (isPresent " << obj1 << ") ) " ;
                }

                //----------------------> for isAtLoc
                for (map<string ,  pair<double, double > >::iterator itIntersect = it->mPercentIntersectLocation.begin() ; itIntersect != it->mPercentIntersectLocation.end() ; itIntersect++)
                {           
                    //second: <before,after> => second.first : percent of presence in a loc before

                    //if percentage isAtLoc > treshold_sup : isAtLoc
                    if (itIntersect->second.first > abmReasoningFunction::threshold_intersect_sup) {
                        myfile << "(isAtLoc " << obj1 << " " << itIntersect->first <<" ) " ;
                    }

                    //if percentage isAtLoc < treshold_inf : not (isAtLoc)
                    if (itIntersect->second.first < abmReasoningFunction::threshold_intersect_inf) {
                        myfile << "(not (isAtLoc " << obj1 << " " << itIntersect->first <<" )) " ;
                    } 
                }
            }

            myfile << ")" << endl ; //end parenthesis of precondition

            /************************************************************************/
            /**************************  PDDL : effect  *****************************/
            /************************************************************************/

            //----------------------> for isPresent
            myfile << "\t\t:effect (and " ;

            
            //hanoi_loc : presence <obj, <from,to>> to check
            //EFFECT : looking for the "to"
            if(it->sName == "hanoi"){
                //----------------------> for isAtLoc
                for (map<string ,  pair<double, double > >::iterator itIntersect = it->mPercentObjectFromTo.begin() ; itIntersect != it->mPercentObjectFromTo.end() ; itIntersect++)
                {   



                }

                //For the moved object : mPercentIntersection
                for (map<string ,  pair<double, double > >::iterator itIntersect = it->mPercentIntersectLocation.begin() ; itIntersect != it->mPercentIntersectLocation.end() ; itIntersect++)
                {           
                    //if loc = from
                    if (itIntersect->first == "from" || itIntersect->first == "to") {

                        //second.second = end = effect

                        //if percentage isAtLoc > treshold_sup : isAtLoc
                        if (itIntersect->second.second > abmReasoningFunction::threshold_intersect_sup) {
                            myfile << "(isAtLoc " << it->sArgument << " ?" << itIntersect->first << ") " ;
                        }

                        //if percentage isAtLoc < treshold_inf : not(isAtLoc)
                        if (itIntersect->second.second < abmReasoningFunction::threshold_intersect_inf) {
                            myfile << "(not (isAtLoc " << it->sArgument << " ?" << itIntersect->first << ")) " ;
                        }
                    }
                }

            } else {

                if(it->PercentPresence.second > abmReasoningFunction::threshold_presence){
                    myfile << "(isPresent " << obj1 << ") " ;
                } else if (it->PercentPresence.second < abmReasoningFunction::threshold_absence){
                    myfile << "(not (isPresent " << obj1 << ") ) ";
                }

                //----------------------> for isAtLoc
                for (map<string ,  pair<double, double > >::iterator itIntersect = it->mPercentIntersectLocation.begin() ; itIntersect != it->mPercentIntersectLocation.end() ; itIntersect++)
                {

                    //second: <before,after> => second.second : percent of presence in a loc after

                    //if percentage isAtLoc > treshold_sup : isAtLoc
                    if (itIntersect->second.second > abmReasoningFunction::threshold_intersect_sup) {
                        myfile << "(isAtLoc " << obj1 << " "<< itIntersect->first <<") " ;
                    } 

                    //if percentage isAtLoc < treshold_inf : not (isAtLoc)
                    if (itIntersect->second.second < abmReasoningFunction::threshold_intersect_inf) {
                        myfile << "(not (isAtLoc " << obj1 << " "<< itIntersect->first <<")) " ;
                    } 
                }
            }

            myfile << ")" << endl ; //end parenthesis of effect
        
            myfile << "\t)" << endl ; //end parenthesis of current Action
        }

    myfile << ")" << endl ; //end parenthesis of domain

    myfile.close();

    bOutput.addString("[ack]");

    return bOutput;
}

/**
* Print the Problem PDDL file using bottle request from user and ABM on the current action
* @param bGoal, bottle of goals (i.e. bottles of OPC relation)
*/
Bottle abmReasoning::printPDDLContextualKnowledgeProblem(Bottle bGoal)
{
    Bottle bOutput, bRequest, bRequestRelation ;

    char buffer[512] = "" ;
    strcpy(buffer, plannerPath.c_str()) ;
    strcat(buffer, pddlProblem.c_str()) ;
    ofstream myfile;
    std::cout << "Problem PDDL written in : " << buffer << endl ;
    myfile.open(buffer, ios_base::ate);

    /************************ 1. define problem name, (:domain) *****************************/ 
    myfile << ";; STRIPS problem automatically generated by ABMReasoning, part of EFAA" << endl;
    
    myfile << "(define (problem efaa-prob)" << endl;

    myfile << "\t(:domain efaa)" << endl ;

    
    /********************************** 2. (:objects) ***************************************/ 
    //2.1. get the objects name
    vector <string> vObjName ;
    ostringstream osRequest, osRequestRelation;

    myfile << "\t(:objects" << endl ;
    osRequest.str("");
    osRequest << "SELECT distinct argument FROM contentarg WHERE subtype = '" << EFAA_OPC_ENTITY_RTOBJECT << "' AND instance > 656" ;
    bRequest = requestFromStream(osRequest.str().c_str());
    myfile << "\t\t" ;
    for (int arg = 0 ; arg < bRequest.size() ; arg++)
    {
        string objectName;
        objectName = bRequest.get(arg).toString();
        myfile <<  " " << objectName;
        vObjName.push_back(objectName);
    }
    //end objects name
    myfile << endl ;

    //2.2 get the locations name
    vector <string> vLocName ;

    osRequest.str("");
    osRequest << "SELECT distinct argument FROM contentarg WHERE (role = 'spatial1' OR role = 'spatial2') AND instance > 656" ;
    bRequest = requestFromStream(osRequest.str().c_str());
    myfile << "\t\t" ;
    for (int arg = 0 ; arg < bRequest.size() ; arg++)
    {
        string locName;
        locName = bRequest.get(arg).toString();
        myfile <<  " " << locName;
        vLocName.push_back(locName) ;
    }
    //end locations name
    myfile << endl << "\t)\t;; end :objects" << endl ; //end objects


    /********************************** 3. (:init) ***************************************/ 
    myfile << "\t(:init" << endl ;

    //3.1 ;;types
    myfile << "\t\t;;types" << endl ;
    myfile << "\t\t" ;

    //3.1.1 init objects types
    for(vector<string>::iterator it = vObjName.begin() ; it != vObjName.end() ; it++)
    {
            myfile << "(object " << *(it) << ") ";
    }
    myfile << endl ; //end of line for objects types

    //3.1.2 init loc types
    myfile << "\t\t" ;
    for(vector<string>::iterator it = vLocName.begin() ; it != vLocName.end() ; it++)
    {
            myfile << "(location " << *(it) << ") ";
    }
    myfile << endl << endl ; //end of line for (:init

    //3.2 ;;init-conditions
    myfile << "\t\t;;init-conditions" << endl ;

    //last instance
    Bottle  bOpcLastInstance = requestFromStream("SELECT instance FROM main WHERE activitytype = 'reasoning' AND begin = TRUE ORDER BY instance DESC LIMIT 1" );
    //std::cout << "bottle = " << bOpcLastInstance.toString().c_str() << " and bOPCLastReasoningInstance = " << atoi(bOpcLastInstance.get(0).asList()->get(0).toString().c_str()) << endl ;
    
    //3.2.1 extract for all rtobjects which ones are present (rtobject
    osRequest.str("");
    osRequest << "SELECT rtobject.name FROM rtobject WHERE rtobject.instance = " << atoi(bOpcLastInstance.get(0).asList()->get(0).toString().c_str()) <<  " AND rtobject.presence = TRUE" ;
    bRequest = requestFromStream(osRequest.str().c_str());
    myfile << "\t\t" ;

    //add relation (obj isAtLoc loc) inside the opc
    Bottle b = updateOpcObjectLocation(abmReasoningFunction::s_realOPC);

    std::cout << "bottle reply update = " << b.toString().c_str();

    for (int arg = 0 ; arg < bRequest.size() ; arg++)
    {
        string objPresentName;
        objPresentName = bRequest.get(arg).toString();
        if (objPresentName != "NULL"){
            myfile <<  "(isPresent " << objPresentName << ") ";

            //check if the present object intersect a/several loc
            osRequestRelation.str("");

            osRequestRelation << "SELECT relation.object  FROM relation WHERE relation.instance = " << atoi(bOpcLastInstance.get(0).asList()->get(0).toString().c_str()) <<  " AND relation.verb = 'isAtLoc' AND relation.subject = '" << objPresentName << "' ;" << endl ;
            bRequestRelation = requestFromStream(osRequestRelation.str().c_str());
            std::cout << "================================================  Relation for " << atoi(bOpcLastInstance.get(0).asList()->get(0).toString().c_str()) << " : " << endl  << bRequestRelation.toString().c_str() << endl;

            for (int argRelation = 0 ; argRelation < bRequestRelation.size() ; argRelation++)
            {
                if (bRequestRelation.get(argRelation).toString() != "NULL") {
                    myfile <<  "(isAtLoc " << objPresentName << " " << bRequestRelation.get(argRelation).toString() << ") ";
                }
            }

        }
    }
    myfile << endl << "\t)\t;; end :init" << endl ; //end (:init

    //3.2.2 extract for all rtobjects where they are (intersect with loc)
    //add relation (obj isAtLoc loc) inside the opc
    //cf inside isPresent check


    /********************************** 4. (:goal) ***************************************/ 
    myfile << "\t(:goal" << endl ;
    myfile << "\t\t(and " ;

    //use the input bottle to fill the goal part
    // condition : manner verb subject object -> some could be at abmReasoningFunction::TAG_NONE
    for (int condition = 0 ; condition < bGoal.size() ; condition++)
    {
        unsigned int isPositiveCondition = 1 ;
        string currentCond ;
        for(int condition_part = 0 ; condition_part < bGoal.get(condition).asList()->size() ; condition_part ++){
            
            string currentCondWord = bGoal.get(condition).asList()->get(condition_part).toString() ;
            //first condition_part = manner = positive or negative condition (none/not)
            if(condition_part == 0) {
                if(currentCondWord == "not") {
                    isPositiveCondition = 0 ;

                    currentCond += "not (";
                }

                //other condition part (except the first one : manner)
            } else if (currentCondWord != abmReasoningFunction::TAG_DB_NONE){
                currentCond += " ";
                currentCond += currentCondWord ;
            }
        }

        if (isPositiveCondition == 0) {
            currentCond += ")";
        } 

        myfile << "(" << currentCond << ") " ;
    }

    myfile << endl << "\t\t)\t;; end and" ;//end and
    myfile << endl << "\t)\t;; end goal" ;//end goal
    myfile << endl << ")\t;; end define" ; //end define

    myfile.close();

    bOutput.addString("[ack]");

    return bOutput;

}

/*
* Change a action in the OPC with the rigth name
* @param bottle renameAction : (<instance_begin> <activity_name> <activity_type>) (argument <argument_name> <argument_type> <argument_subtype>) with n bottle argument
*/
Bottle abmReasoning::renameAction(Bottle bInput)
{   
    Bottle  bOutput,
        bRequest,
        bMain,
        bArgu;

    bMain = (*bInput.get(1).asList());
    bArgu = (*bInput.get(2).asList());

    if (bMain.size() !=3)
    {
        bOutput.addString("error in renameAction, information missing for main table");
        return bOutput;
    }

    ostringstream osRequest;
    string  sActivityName = bMain.get(1).asString().c_str(),
        sActivityType = bMain.get(2).asString().c_str(),
        sArgument = bArgu.get(1).asString().c_str(),
        sType = bArgu.get(2).asString().c_str(),
        sSubtype = bArgu.get(3).asString().c_str(),
        sRole = bArgu.get(4).asString().c_str();

    int iInstance = bMain.get(0).asInt();

    osRequest << "UPDATE main SET activityname = '" << sActivityName << "', activitytype = '" << sActivityType << "' WHERE instance = " << iInstance;
    bRequest = requestFromStream(osRequest.str().c_str());

    osRequest.str("");
    osRequest << "UPDATE contentarg SET argument = '" << sArgument << "', type = '" << sType <<  "', subtype = '" << sSubtype << "', role = '" << sRole <<  "' WHERE instance = " << iInstance << " AND argument = 'unknown' ";
    bRequest = requestFromStream(osRequest.str().c_str());

    osRequest.str("");
    osRequest << "UPDATE contentarg SET argument = '" << sArgument << "', type = '" << sType <<  "', subtype = '" << sSubtype << "', role = '" << sRole <<  "' WHERE instance = " << iInstance << " AND argument = 'none' ";
    bRequest = requestFromStream(osRequest.str().c_str());

    return bOutput;
}


/*   -------------------------   FINDING FUNCTIONS  ------------------------------   */


/**
* Find all the different complex, and put the consequence in the listTimeKnowledge
* TODO
*/
Bottle abmReasoning::findAllComplex()
{
    return findAllComplex(0);
}

Bottle abmReasoning::findAllComplex(int from)
{
    std::cout << endl << "Creating temporal knowledge from complex." << endl;

    //check : simple object query :
    Bottle bTemporal, bOutput;
    ostringstream osRequest;
    osRequest <<"SELECT DISTINCT contentarg.argument FROM main, contentarg WHERE main.activitytype = 'complex' AND contentarg.role = 'temporal' AND main.instance = contentarg.instance AND main.instance > " << from;
    bTemporal = requestFromStream(osRequest.str().c_str());
    //std::cout << "bTemporal : " << bTemporal.toString() << endl ;

    string sNull = "NULL";
    if (bTemporal.toString().c_str() == sNull)
    {
        std::cout << "0 temporal found." << endl;
        bOutput.addString("no temporal to load.");
        return bOutput;
    }
    string sTemporal;
    ostringstream osOutput;
    osOutput << "Temporal(s) found : " ;

    for (int i = 0; i < bTemporal.size(); i++)
    {
        sTemporal = bTemporal.get(i).toString();
        osOutput << sTemporal << " ; ";
        ostringstream osMessenger;
        osMessenger << "SELECT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activitytype = 'complex' AND main.begin = true AND contentarg.argument = '" << sTemporal << "'" ;
        Bottle  bMessenger = requestFromStream(osMessenger.str().c_str());

        for (int j = 0; j < bMessenger.size() ; j++)
        {
            int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
            Bottle bComplex = Interlocutor.askComplexFromId(Id);
            Bottle bResult = addTimeKnowledge(bComplex);
        }
    }

    bOutput.addString(osOutput.str().c_str());
    std::cout << osOutput.str().c_str() << endl;
    return bOutput;
}

/**
* Add all the action in the ABM chronologicaly
*
*/
Bottle abmReasoning::findAllActions()
{
    return findAllActions(0);
}

Bottle abmReasoning::findAllActions(int from)
{
    std::cout << endl << "Getting actions." << endl;
    int iError = 0;
    //check : simple object query :
    Bottle bTemporal, bOutput;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE activitytype = 'action' AND begin = true AND INSTANCE > " << from;
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());
    int numberAction = bMessenger.size();

    vector<int> vError;
    std::cout << "found " << numberAction << " action(s)" << endl;
    pair<double,double> BEGIN,
        MOVE,
        END,
        BEGIN2,
        END2;

    for (int j = 0; j < numberAction ; j++)
    {
        std::cout << j+1 << ".." ;
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        Bottle bAction = Interlocutor.askActionFromId(Id);



        //std::cout << "bAction : " << endl;

        //for (int kk = 0 ; kk < bAction.size() ; kk++)
        //{
        //  std::cout << "element " << kk << " : \t" << bAction.get(kk).toString() << endl;
        //}



        if (bAction.size() <= 6)
        {
            std::cout << endl << "Error in abmReasoning::addLastAction : wrong size of 'askLastAction' \n";
            vError.push_back(Id);
            iError++;
        }
        else
        {
            string  sName = (*bAction.get(0).asList()).get(0).toString().c_str(),
                sArgument = (*bAction.get(1).asList()).get(0).toString().c_str(),
                sXY = bAction.get(2).toString().c_str(),
                sEND = bAction.get(3).toString().c_str(),
                sXY2,
                sEND2,
                sDependance = abmReasoningFunction::TAG_DB_NONE;
            bool bSpatial2 = false;
            if (bAction.size() > 9) // spatial 2 is an object (expl put near toy)
            {
                bSpatial2 = true;
                sXY2 = bAction.get(7).toString().c_str();
                sEND2 = bAction.get(8).toString().c_str();
                sDependance = bAction.get(9).toString().c_str();
            }


            spatialKnowledge skMove;
            BEGIN = abmReasoningFunction::coordFromString(sXY);
            if (bDreaming && mentalOPC->isConnected())
            {
                string  sObject = "dream...";
                RTObject* LOCATION = mentalOPC->addRTObject(sObject);
                LOCATION->m_ego_position[0] = BEGIN.first ;
                LOCATION->m_ego_position[1] = BEGIN.second ;
                LOCATION->m_ego_position[2] = 0.01;
                LOCATION->m_dimensions[0] = 0.04;
                LOCATION->m_dimensions[1] = 0.04;
                LOCATION->m_dimensions[2] = 0.08 ;
                LOCATION->m_present = 1;
                LOCATION->m_color[0] = 100; 
                LOCATION->m_color[1] = 255; 
                LOCATION->m_color[2] = 0; 
                mentalOPC->commit();
            }

            int ObjectPresentBefore = bAction.get(4).asInt(),
                ObjectPresentAfter = bAction.get(5).asInt();
            string sSubject = bAction.get(6).toString();

            Bottle bContextual;
            bContextual.addString(sName.c_str());
            bContextual.addString(sArgument.c_str());
            bContextual.addInt(ObjectPresentBefore);
            bContextual.addInt(ObjectPresentAfter);
            bContextual.addString(sDependance.c_str());
            bContextual.addString(sSubject.c_str());
            bContextual = addContextualKnowledge(bContextual);


            END = abmReasoningFunction::coordFromString(sEND);
            MOVE.first = END.first - BEGIN.first;
            MOVE.second = END.second - BEGIN.second;

            skMove.sName = sName;
            skMove.sArgument = sArgument;
            skMove.sDependance = sDependance;

            if (!bSpatial2)
            {
                skMove.vX.push_back(END.first);
                skMove.vY.push_back(END.second);
                skMove.vDX.push_back(MOVE.first);
                skMove.vDY.push_back(MOVE.second);
            }
            else
            {
                BEGIN2 = abmReasoningFunction::coordFromString(sXY2);
                END2 = abmReasoningFunction::coordFromString(sEND2);

                skMove.vX.push_back(BEGIN2.first-BEGIN.first);
                skMove.vY.push_back(BEGIN2.second-BEGIN.second);

                skMove.vDX.push_back(END2.first-END.first);
                skMove.vDY.push_back(END2.second-END.second);
            }

            if (ObjectPresentAfter== ObjectPresentBefore)
            {
                addSpatialKnowledge(skMove, true);
            }

            if (bDreaming && mentalOPC->isConnected())
            {
                string  sObject = "dream...";
                RTObject* LOCATION = mentalOPC->addRTObject(sObject);
                LOCATION->m_ego_position[0] = END.first ;
                LOCATION->m_ego_position[1] = END.second ;
                LOCATION->m_ego_position[2] = 0.01;
                LOCATION->m_present = 1;
                LOCATION->m_color[0] = 100; 
                LOCATION->m_color[1] = 255; 
                LOCATION->m_color[2] = 0; 
                mentalOPC->commit();
            }
        }
    }
    if (bDreaming && mentalOPC->isConnected())
    {
        string  sObject = "dream...";
        RTObject* LOCATION = mentalOPC->addRTObject(sObject);
        LOCATION->m_present = 0;
        mentalOPC->commit();
    }
    std::cout << endl;
    if (iError != 0)
    {
        std::cout << iError << " errors while getting the actions:" << endl;
        for (unsigned int j = 0 ; j < vError.size() ; j++)
        {
            std::cout << vError[j] << "\t " ;
        }
        std::cout << endl;
    }
    return bOutput;
}

/**
* Add all the sentences in the ABM chronologicaly
*
*/
Bottle abmReasoning::findAllSentence()
{
    return findAllSentence(0);
}

Bottle abmReasoning::findAllSentence(int from)
{
    std::cout << endl << "Getting sentence." << endl;
    //int iError = 0;
    //check : simple object query :
    Bottle bTemporal, bOutput;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE activitytype = 'sentence' AND begin = true AND INSTANCE > " << from << " ORDER by instance";
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());
    int numberSentence = bMessenger.size();

    vector<int> vError;
    std::cout << "found " << numberSentence << " sentence(s)" << endl;
    pair<double,double> BEGIN,
        MOVE,
        END,
        BEGIN2,
        END2;

    for (int j = 0; j < numberSentence ; j++)
    {
        std::cout << endl << j+1 << ".." ;
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        Bottle bSentence = Interlocutor.askSentenceFromId(Id);

        if (bSentence.size() != 4)
            std::cout << "Error in abmReasoning::FindAllSentence - instance " << Id <<"." << endl; 
        else
        {
            // add the grammar knowledge:
            listGrammarKnowledge.addInteraction(bSentence);
        }
    }

    return bOutput;
}

/**
* Find all the different behavior, and put the consequence in the listBehavior
* TODO
*/
Bottle abmReasoning::findAllBehaviors()
{
    return findAllBehaviors(0);
}

Bottle abmReasoning::findAllBehaviors(int from)
{
    std::cout << endl << "Creating drives knowledge from behaviors" << endl;

    //check : simple object query :
    Bottle  bOutput;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE activitytype = 'behavior' AND begin = true AND INSTANCE > " << from;
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());

    string sNull = "NULL";
    if (bMessenger.toString().c_str() == sNull)
    {
        std::cout << "0 behavior found." << endl;
        bOutput.addString("no behavior to load.");
        return bOutput;
    }
    for (int j = 0; j < bMessenger.size() ; j++)
    {
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        behavior beBehavior = Interlocutor.askBehaviorFromId(Id);
        bOutput.addList() = addBehavior(beBehavior);
    }

    std::cout << listBehaviors.size() << " behavior(s) found." << endl;

    return bOutput;
}

/*
* check if the last interaction is an action of a complex and add it.
* @param bInput, bottle of the last interaction
*
*/
Bottle abmReasoning::askLastActivity(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.get(1).isString())
    {
        if (bInput.get(1).toString() == abmReasoningFunction::TAG_DB_ACTION.c_str())
        {
            return Interlocutor.askLastAction();
        }

        if (bInput.get(1).toString() == abmReasoningFunction::TAG_DB_COMPLEX.c_str())
        {
            return Interlocutor.askLastComplex();
        }

        if (bInput.get(1).toString() == abmReasoningFunction::TAG_DB_SHARED_PLAN.c_str())
        {
            addLastPlan();
            return bInput;
        }
    }

    bOutput.addString("Error, ask unknown");
    return bOutput;
}

/**
* Find all the different sharedPlan, and put the consequence in the listSharedPlan
* TODO
*/
Bottle abmReasoning::findAllSharedPlan()
{
    return findAllSharedPlan(0);
}

Bottle abmReasoning::findAllSharedPlan(int from)
{
    std::cout << endl << "Getting known shared plans." << endl;

    //check : simple object query :
    Bottle bTemporal, bOutput;
    ostringstream osRequest;
    osRequest <<"SELECT instance FROM main WHERE activitytype = 'sharedplan' AND begin = true AND INSTANCE > " << from ;
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());

    listPlan.clear();
    listSharedPlan.clear();
    string sNull = "NULL";
    if (bMessenger.toString().c_str() == sNull)
    {
        std::cout << "0 shared plan found." << endl;
        bOutput.addString("no shared plan to load.");
        return bOutput;
    }
    for (int j = 0; j < bMessenger.size() ; j++)
    {
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        plan pSharedPlan = Interlocutor.askSharedPlanFromId(Id);
        plan test;
        test = addPlan(pSharedPlan);

        bool PlanExisting = false,
            SPExisting = false;

        for (vector<sharedPlan>::iterator it_SP = listSharedPlan.begin() ; it_SP != listSharedPlan.end() ;  it_SP ++)
        {
            SPExisting = false;
            if ( (it_SP->sManner == pSharedPlan.sManner) && (it_SP->sName == pSharedPlan.sName) )
            {
                SPExisting = true;
                PlanExisting = false;
                for ( vector< pair <plan, int> >::iterator it_Plan = it_SP->listPlanPossible.begin() ; it_Plan != it_SP->listPlanPossible.end() ; it_Plan++)
                {
                    if ( (it_Plan->first.vActivityArguments == test.vActivityArguments) && (it_Plan->first.vActivityname == test.vActivityname) && (it_Plan->first.vActivitytype == test.vActivitytype) && (it_Plan->first.vArguments == test.vArguments) )
                    {
                        PlanExisting = true;
                        it_Plan->second++;
                        bOutput.addString("plan already known. socre updated");
                    }
                }
                if (SPExisting && !PlanExisting)
                {
                    bOutput.addString("sharedPlan existing but new plan.");
                    pair<plan , int> pPlan (test, 1);
                    it_SP->listPlanPossible.push_back(pPlan);
                }
            }
        }

        if (!SPExisting)
        {
            bOutput.addString("new sharedPlan");
            pair<plan , int> pPlan (test, 1);

            sharedPlan newSP;
            newSP.listPlanPossible.push_back(pPlan);
            newSP.sManner = pSharedPlan.sManner;
            newSP.sName = pSharedPlan.sName;
            newSP.vArguments = pSharedPlan.vArguments;

            listSharedPlan.push_back(newSP);
        }
        addSharedPlan(test);
    }

    std::cout << listPlan.size() << " plan(s) found." <<endl;
    //displaySharedPlan();

    return bOutput;
}

/**
* add the current action to the list of current action and built the corresponding plan. Check if the plan is matching if one of the known SP and return the SP which match

* e.g. : from (7,8), (5,8), (3,8) to (9,10), (7,10), (5,10), (3,10)
* @param idBeginCurrentAction, int of the current Instance, begin = true
* @param idEndCurrentAction,   int of the current Instance, begin = false
*/
Bottle abmReasoning::availableSharedPlan(int idBeginCurrentAction, int idEndCurrentAction)
{
    Bottle bOutput ;

    vBuiltSharedPlan.clear();


    //add action at the end of the currentList
    for (vector <pair <int,int> >::iterator it = vCurrentActions.begin(); it < vCurrentActions.end(); it++ )
    {
        it->second = idEndCurrentAction ;
    }

    //add action as an element of the list
    pair <int, int> solelyCurrentAction (idBeginCurrentAction, idEndCurrentAction);
    vCurrentActions.push_back(solelyCurrentAction);
    vAvailablePlans.push_back(listPlan) ;


    std::cout << "List of currentActions : " << endl ;
    for (vector <pair <int,int> >::iterator it = vCurrentActions.begin(); it < vCurrentActions.end(); it++ )
    {
        //print the list
        std::cout << "- { " << it->first << ", " << it->second << "}" << endl;

        //build associated SP
        vBuiltSharedPlan.push_back(actionsToPlan(it->first, it->second));
    }

    // iterator to access vector of current actions, SP built with them and the available (and known) plans
    vector <pair <int,int> >::iterator it_actions = vCurrentActions.begin() ;
    vector <plan>::iterator it_builtSP = vBuiltSharedPlan.begin() ;
    vector <vector<plan> >::iterator it_plan = vAvailablePlans.begin() ;

    while(it_builtSP < vBuiltSharedPlan.end())
    {
        //1. just print to know where we are

        //name SP
        std::cout << "\n============================================================================= " << endl ;
        std::cout << "name SP : " << it_builtSP->sName << endl;
        std::cout << "=============================================================================\n " << endl ;

        //actionName
        std::cout << "name actions : " ;
        for(vector< string >::iterator itActivityName = it_builtSP->vActivityname.begin(); itActivityName < it_builtSP->vActivityname.end(); itActivityName++ )
        {
            std::cout << *(itActivityName) << " " ;
        }
        std::cout << endl;
        //arguments
        std::cout << "Arguments of the next SP : " ;
        for(vector< list < pair < string , string > > >::iterator itActivityArguments = it_builtSP->vActivityArguments.begin(); itActivityArguments < it_builtSP->vActivityArguments.end(); itActivityArguments++ )
        {
            std::cout << endl;
            for(list < pair < string , string > >::iterator itListArg = itActivityArguments->begin() ; itListArg != itActivityArguments->end(); itListArg++)
            {
                std::cout << " {" << itListArg->first << ", "<< itListArg->second << "}" ;
            }

        }
        std::cout << endl;

        //2. For the current built plan, check if the vector of available plan could match it. Erase the ones which don't. return empty == no plan are available

        *(it_plan) = checkPlan(*(it_builtSP), *(it_plan));

        //if there is not any available plan which could be match with the built SP => erase. (Iterator adjust : no incrementation)
        if(it_plan->empty()){
            it_plan = vAvailablePlans.erase(it_plan) ;
            it_builtSP = vBuiltSharedPlan.erase(it_builtSP) ;
            it_actions = vCurrentActions.erase(it_actions) ;
        } else {
            //build the bOutput

            //subBottle for the current built plan
            Bottle bCurrentBuiltPlan ;  


            //subSubBottle for the current available plan
            Bottle bCurrentAvailablePlan ;

            for(vector <plan>::iterator it_availableSP = it_plan->begin(); it_availableSP < it_plan->end() ; it_availableSP++) 
            {

                bCurrentAvailablePlan.clear() ;

                //********************************* name and manner
                bCurrentAvailablePlan.addString(it_availableSP->sName.c_str());
                bCurrentAvailablePlan.addString(it_availableSP->sManner.c_str());

                //********************************* sub bottle for arg value and arg role
                Bottle bArgValue ;
                Bottle bArgRole ;

                for(vector <pair <string,string> >::iterator it_arguments = it_availableSP->vArguments.begin(); it_arguments < it_availableSP->vArguments.end(); it_arguments++)
                {
                    bArgValue.addString(it_arguments->first.c_str());
                    bArgRole.addString(it_arguments->second.c_str());
                }

                bCurrentAvailablePlan.addList() = bArgValue ;
                bCurrentAvailablePlan.addList() = bArgRole ;

                //********************************* predict next action
                Bottle bNextAction ;


                bNextAction.addString("executeActivity");
                bNextAction.addString("action");

                //if builtSP.size < availableSP, else no nextAction because the SP is full

                //init 
                vector< list < pair < string , string > > >::iterator it_actArg = it_availableSP->vActivityArguments.begin();
                vector<string>::iterator it_actName = it_availableSP->vActivityname.begin();
                unsigned int i = 0 ;

                //incremente until the next action

                // /!\ if the SP is finished!!
                while(i < it_builtSP->vActivityname.size())
                {
                    i++ ;
                    it_actArg++ ;
                    it_actName++ ;
                }

                //std::cout << "Next Action : " << *it_actName << "  : " ;
                string actionName = *it_actName ;
                bNextAction.addString(actionName.c_str());

                Bottle subNextActionArg ;
                Bottle subNextActionRole ;

                for (list < pair < string, string > >::iterator it_Arg = it_actArg->begin() ; it_Arg != it_actArg->end() ; it_Arg++)
                {
                    //std::cout << "argument ( " << it_Arg->first.c_str() << ", " << it_Arg->second.c_str() << ") " << endl ;
                    subNextActionArg.addString(it_Arg->first.c_str()) ;

                    //change the incrementation of the role to always be 1, just like any solely action. Assuming number < 10
                    string commonRole = it_Arg->second.substr (0,it_Arg->second.size()-1);
                    //subNextActionRole.addString(it_Arg->second.c_str());

                    stringstream str;               
                    string stringNumber;

                    int defaultNumber = 1 ;
                    str << defaultNumber;
                    str >> stringNumber;

                    commonRole = commonRole + stringNumber ;

                    subNextActionRole.addString(commonRole.c_str());
                }

                bNextAction.addList() = subNextActionArg ;
                bNextAction.addList() = subNextActionRole ;

                bCurrentAvailablePlan.addList() = bNextAction ;

            }

            bCurrentBuiltPlan.addList() = bCurrentAvailablePlan ;

            bOutput.addList() = bCurrentBuiltPlan ;



            //at least one plan is elligible : incrementation
            it_plan ++ ;
            it_builtSP++;
            it_actions++;
        }
    }

    return bOutput ;

}

/**
* Find all possible SharedPlan according to the current vCurrentAction list, added with the last action
*/
Bottle abmReasoning::findPossibleSharedPlan(int beginLastAction, int endLastAction)
{
    Bottle bOutput ;
    Bottle bQuery  ;
    //1. extract the last instance in the ABM (end of an action) and initiate the begin of this action

    //findPossibleSharedPlan() 
    //this is just to test

    /*ostringstream os;
    os << "SELECT max(instance) FROM main" ;
    bQuery = requestFromStream(os.str());
    int endLastAction = atoi(bQuery.get(0).asList()->get(0).toString().c_str()) ;
    int beginLastAction = endLastAction-1;*/



    //2. Check if there is no gap between this action and the last one : new vCurrentAction otherwise
    if( (saveEndLastAction == -1) || ( (beginLastAction - saveEndLastAction) > 1) || (beginLastAction < saveEndLastAction) )
    {
        std::cout << "\n===========================> New vCurrentActions <===========================\n" << endl ;
        vCurrentActions.clear() ;
        vAvailablePlans.clear() ;
        saveEndLastAction = endLastAction ;
    }

    //3. Call availableSharedPlan with the new action
    bOutput = availableSharedPlan(beginLastAction, endLastAction);


    //4. return the bottle of available shared plans
    return bOutput;
}

/**
* Add all the action in the ABM chronologicaly
*
*/
Bottle abmReasoning::findAllInteractions()
{
    return findAllInteractions(0);
}

Bottle abmReasoning::findAllInteractions(int from)
{
    std::cout << "Getting interactions." << endl;

    Bottle bOutput;
    if (!realOPC->isConnected())
    {
        bOutput.addString("Error in autobiographicalMemory::populateOPC | OpcClient not connected.");
        std::cout << bOutput.toString() << endl;
        return bOutput;
    }

    realOPC->checkout();
    realOPC->update();

    Bottle bMessenger, bReply, bDistinctEntity;
    ostringstream osRequest;
    osRequest <<"SELECT DISTINCT argument FROM contentarg WHERE subtype IN ( 'object', 'rtobject', 'agent') AND INSTANCE > " << from ;
    bDistinctEntity = requestFromStream(osRequest.str().c_str());
    string sNull = "NULL";
    if (bDistinctEntity.toString().c_str() == sNull)
    {
        std::cout << "0 shared plan found." << endl;
        bOutput.addString("no shared plan to load.");
        return bOutput;
    }
    for (int iDE = 0 ; iDE < bDistinctEntity.size() ; iDE++)
    {
        string sSubject = bDistinctEntity.get(iDE).toString();
        ostringstream osEntity;
        osEntity << "SELECT argument, subtype, role FROM contentarg WHERE instance IN(SELECT instance FROM contentarg WHERE argument = '" << sSubject << "') AND argument not in ('" << sSubject << "', 'none', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9')";
        bReply = requestFromStream(osEntity.str().c_str());

        knownInteraction tempInteraction;
        tempInteraction.sSubject = sSubject;

        if (bReply.toString() != "NULL")
        {
            for (int i = 0 ; i < bReply.size() ; i++)
            {
                Bottle tempBottle = *bReply.get(i).asList();

                tuple<string, int, string, string> tInteraction;
                get<0>(tInteraction) = tempBottle.get(0).toString().c_str();
                get<1>(tInteraction) = 1;
                get<2>(tInteraction) = tempBottle.get(1).toString().c_str();
                get<3>(tInteraction) = tempBottle.get(2).toString().c_str();

                tempInteraction.addInteraction(tInteraction);
            }
            listKnownInteraction.push_back(tempInteraction);
        }
    }



    ostringstream osOutput;
    osOutput << listKnownInteraction.size() << " interaction(s) added." << endl;
    std::cout << osOutput.str() << endl;
    bOutput.addString(osOutput.str().c_str());

    return bOutput;
}



/*   -------------------------   ADDING FUNCTIONS  ------------------------------   */


/**
* check if the last interaction is an action of a complex and add it.
*
*/
Bottle abmReasoning::addLastActivity(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.size() == 2)
    {
        if (bInput.get(1).isString())
        {
            if (bInput.get(1).asString().c_str() == abmReasoningFunction::TAG_DB_ACTION)
            {
                return addLastAction();
            }

            if (bInput.get(1).asString().c_str() == abmReasoningFunction::TAG_DB_COMPLEX)
            {
                return addLastComplex();
            }

            if (bInput.get(1).asString().c_str() == abmReasoningFunction::TAG_DB_BEHAVIOR)
            {
                return addLastBehavior();
            }

            if (bInput.get(1).asString().c_str() == abmReasoningFunction::TAG_DB_SHARED_PLAN)
            {
                return addLastSharedPlan();
            }

        }
    }
    bOutput.addString("Error, add unknown");

    return bOutput;
}

/**
* Add the last action of the autobiographicalMemory in listSpatialKnowledge
* @param none
* @return Bottle with state of the add
*/
Bottle abmReasoning::addLastAction()
{
    Bottle  bAction = Interlocutor.askLastAction(),
        bOutput;
    if (bAction.size() <= 5)
    {
        bOutput.addString("Error in abmReasoning::addLastAction : wrong size of 'askLastAction'");
        return bOutput;
    }
    //  std::cout << "bLastAction " << bLastAction.toString() << endl;
    string  sName = (*bAction.get(0).asList()).get(0).toString().c_str(),
        sArgument = (*bAction.get(1).asList()).get(0).toString().c_str(),
        sXY = bAction.get(2).toString().c_str(),
        sEND = bAction.get(3).toString().c_str(),
        sXY2,
        sEND2;

    bool bSpatial2 = false;
    if (bAction.size() == 9) // spatial 2 is an object (expl put near toy)
    {
        bSpatial2 = true;
        sXY2 = bAction.get(7).toString().c_str();
        sEND2 = bAction.get(8).toString().c_str();
    }
    string sHanoi = "hanoi";
    bool    isHanoi = false;
    Bottle bRTOpresent;
    if (sName ==  sHanoi && bAction.size() == 8)
    {
        bRTOpresent = *bAction.get(7).asList();
        isHanoi = true;
    }

    pair<double,double> BEGIN,
        MOVE,
        END,
        BEGIN2,
        END2;

    spatialKnowledge skMove;
    BEGIN = abmReasoningFunction::coordFromString(sXY);
    END = abmReasoningFunction::coordFromString(sEND);
    MOVE.first = END.first - BEGIN.first;
    MOVE.second = END.second - BEGIN.second;

    skMove.sName = sName;
    skMove.sArgument = sArgument;

    if (!bSpatial2)
    {
        skMove.vX.push_back(END.first);
        skMove.vY.push_back(END.second);
        skMove.vDX.push_back(MOVE.first);
        skMove.vDY.push_back(MOVE.second);
    }
    else
    {
        BEGIN2 = abmReasoningFunction::coordFromString(sXY2);
        END2 = abmReasoningFunction::coordFromString(sEND2);

        skMove.vX.push_back(BEGIN2.first-BEGIN.first);
        skMove.vY.push_back(BEGIN2.second-BEGIN.second);

        skMove.vDX.push_back(END2.first-END.first);
        skMove.vDY.push_back(END2.second-END.second);
    }

    addSpatialKnowledge(skMove, true);

    int ObjectPresentBefore = bAction.get(4).asInt(),
        ObjectPresentAfter = bAction.get(5).asInt();
    string sSubject     = bAction.get(6).toString();

    Bottle bContextual;
    bContextual.addString(sName.c_str());
    bContextual.addString(sArgument.c_str());
    bContextual.addInt(ObjectPresentBefore);
    bContextual.addInt(ObjectPresentAfter);
    bContextual = addContextualKnowledge(bContextual);

    return bOutput;
}

/*
* Add the last complex of the autobiographicalMemory in the list of time knowledge.
*
*/
Bottle abmReasoning::addLastComplex()
{
    Bottle  bLastComplex = Interlocutor.askLastComplex(),
        bOutput;
    //  std::cout << "bLastComplex " << bLastComplex.toString() << endl;
    if (bLastComplex.size() <= 2)
    {
        bOutput.addString("Error in abmReasoning::addLastAction : wrong size of 'askLastAction'");
        return bOutput;
    }

    return  addTimeKnowledge(bLastComplex);

}

/*
* Add the last plan of the autobiographicalMemory in the list of plan.
*
*/
plan abmReasoning::addLastPlan()
{
    plan    pLastPlan = Interlocutor.askLastSharedPlan();
    return addPlan(pLastPlan);
}


Bottle abmReasoning::addLastSharedPlan()
{
    std::cout << endl << "Adding last Shared plan" << endl;

    //check : simple object query :
    Bottle bTemporal, bOutput;

    Bottle  bMessenger = requestFromStream("SELECT instance FROM main WHERE activitytype = 'sharedplan' AND begin = true ORDER BY instance DESC LIMIT 1");

    int Id = atoi(bMessenger.get(0).asList()->get(0).toString().c_str());
    plan pSharedPlan = Interlocutor.askSharedPlanFromId(Id);
    plan test;
    test = addPlan(pSharedPlan);

    bool PlanExisting = false,
        SPExisting = false;

    for (vector<sharedPlan>::iterator it_SP = listSharedPlan.begin() ; it_SP != listSharedPlan.end() ;  it_SP ++)
    {
        SPExisting = false;
        if ( (it_SP->sManner == pSharedPlan.sManner) && (it_SP->sName == pSharedPlan.sName) )
        {
            SPExisting = true;
            PlanExisting = false;
            for ( vector< pair <plan, int> >::iterator it_Plan = it_SP->listPlanPossible.begin() ; it_Plan != it_SP->listPlanPossible.end() ; it_Plan++)
            {
                if ( (it_Plan->first.vActivityArguments == test.vActivityArguments) && (it_Plan->first.vActivityname == test.vActivityname) && (it_Plan->first.vActivitytype == test.vActivitytype) && (it_Plan->first.vArguments == test.vArguments) )
                {
                    PlanExisting = true;
                    it_Plan->second++;
                    bOutput.addString("plan already known. socre updated");
                }
            }
            if (SPExisting && !PlanExisting)
            {
                bOutput.addString("sharedPlan existing but new plan.");
                pair<plan , int> pPlan (test, 1);
                it_SP->listPlanPossible.push_back(pPlan);
            }
        }
    }

    if (!SPExisting)
    {
        bOutput.addString("new sharedPlan");
        pair<plan , int> pPlan (test, 1) ;

        sharedPlan newSP;
        newSP.listPlanPossible.push_back(pPlan);
        newSP.sManner = pSharedPlan.sManner;
        newSP.sName = pSharedPlan.sName;
        newSP.vArguments = pSharedPlan.vArguments;

        listSharedPlan.push_back(newSP);
    }
    addSharedPlan(test);

    std::cout << " plan added." <<endl;
    //displaySharedPlan();

    return bOutput;
}

/*
* Add the last behavior of the autobiographicalMemory in the list of plan.
*
*/
Bottle abmReasoning::addLastBehavior()
{
    behavior beLastBehavior= Interlocutor.askLastBehavior();
    return addBehavior(beLastBehavior);
}


/**
* Return the last commplex stored in the ABM
* 
*/
plan abmReasoning::actionsToPlan(int idBegin, int idEnd)
{
    Bottle  bOutput;
    plan newPlan ;

    //ostringstream osOpcEnd,   osArg, osName;
    ostringstream oss;
    oss << idBegin;
    oss << "-" ;
    oss << idEnd ;
    string sName = oss.str();
    string sManner =  abmReasoningFunction::TAG_DEFAULT;

    //put the begin/end before and after the list of action
    idBegin -= 1;
    idEnd += 1;

    int NbActivity = ( (idEnd - idBegin)/2) ;

    int agentNumber = 0;
    int objectNumber = 0;

    // extracting activity of the plan
    for (int acti = 0 ; acti < NbActivity ; acti++)
    {
        //std::cout  << "NEW ACTIVITYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY" << endl;

        // get type and name of activity
        ostringstream osActivity;
        osActivity << "SELECT activitytype, activityname FROM main WHERE instance = " << idBegin + 1 + 2*acti ;
        Bottle bActivity = *(requestFromStream(osActivity.str().c_str()).get(0).asList());

        // fill newPlan 
        newPlan.vActivitytype.push_back(bActivity.get(0).toString().c_str());
        newPlan.vActivityname.push_back(bActivity.get(1).toString().c_str());

        // get argument of activity
        osActivity.str("");
        osActivity << "SELECT argument, role FROM contentarg WHERE instance = " << idBegin + 1 +2*acti;

        bActivity = requestFromStream(osActivity.str().c_str());
        list<pair<string, string> > lArgument;

        for (int arg = 0 ; arg < bActivity.size() ; arg++)
        {
            Bottle bRole = *bActivity.get(arg).asList();
            string sArgument = bRole.get(0).toString().c_str(),
                sRole = bRole.get(1).toString().c_str();

            pair <string, string> pRole (sArgument, sRole) ;

            if(sRole != "spatial1")
            {
                bool isFound = false ;
                bool roleFound = false ;

                //check'
                for (vector< pair <string, string > >::iterator it_p = newPlan.vArguments.begin() ; it_p != newPlan.vArguments.end() ; it_p++)
                {
                    //std::cout << "currentArg = " << it_p->first << "-" << it_p->second << endl ;
                    //std::cout << "sArg = " << sArgument << " and sRole = " << sRole << endl ;
                    //if the name is found is vArgument -> put the proper role
                    if (it_p->first == sArgument)
                    {
                        sRole = it_p->second ;
                        isFound = true ;
                    }

                    if (it_p->second == sRole)
                    {
                        roleFound = true ;
                    }
                }

                //not in the argument : put it
                if (isFound == false)
                {
                    //role not already defined
                    if(roleFound == false)
                    {
                        pair <string, string>   currentArg (sArgument, sRole);

                        if (sRole == "agent1") 
                        {
                            agentNumber += 1;
                        } else {
                            objectNumber += 1;
                        }

                        //std::cout << "Add in vArguments : " << currentArg.first << "-" << currentArg.second << endl ;
                        newPlan.vArguments.push_back(currentArg);

                        //add in vActivityType the corresponding role
                        pRole.second = currentArg.second;

                    } else {

                        pair <string, string>   currentArg ;
                        currentArg.first = sArgument;

                        //role : more difficult : need to check if already exist and create it accordingly if not

                        //common part : agent, object, ...
                        string commonRole = sRole.substr (0,sRole.size()-1);

                        //number of iteration : 1, 2, 3, ...
                        //char stringNumber [1];
                        stringstream str;               
                        string stringNumber;

                        if(commonRole == "agent") 
                        {
                            agentNumber +=1 ;
                            //itoa(agentNumber, stringNumber, 10);
                            str << agentNumber;
                            str >> stringNumber;

                            //std::cout << "agent : stringNumber = " << stringNumber << endl ;

                        } else {
                            objectNumber +=1 ;
                            //itoa(objectNumber, stringNumber, 10);
                            str << objectNumber;
                            str >> stringNumber;

                            //std::cout << "object : stringNumber = " << stringNumber << endl ;
                        }

                        commonRole = commonRole + stringNumber ;
                        //std::cout << "commonRole = " << commonRole << endl;
                        currentArg.second = commonRole;

                        //std::cout << "Add in vArguments : " << currentArg.first << "-" << currentArg.second << endl ;
                        newPlan.vArguments.push_back(currentArg);

                        //add in vActivityType the corresponding role
                        pRole.second = currentArg.second;

                    }
                }
                //add vArguments
                //if role = role in vArguments => if name != => role = role+1

            } 
            //not necessarily sRole => role of the thing when vArgument.first and sArgument match in vArguments

            lArgument.push_back(pRole);
        }
        newPlan.vActivityArguments.push_back(lArgument);
    }

    newPlan.sManner = sManner;
    newPlan.sName = sName;

    return newPlan;
}


/*   -------------------------   DISCRIMINATE FUNCTIONS  ------------------------------   */


/**
* Discriminate an action of an object
* @param bInput (discriminateAction Xt-1 Yt-1 Xt Yt)
*/
Bottle abmReasoning::discriminateLastAction()
{
    //bottle bQuery to ask autobiographicalMemory for ABM data the instance of OPC
    Bottle bOpcIdBegin = requestFromStream("SELECT instance FROM main WHERE activitytype = 'action' AND begin = TRUE ORDER BY instance DESC LIMIT 1"),
        bQuery;

    int opcIdBegin = atoi(bOpcIdBegin.get(0).asList()->get(0).toString().c_str());

    bQuery.addString("discriminateAction");
    bQuery.addInt(opcIdBegin);

    return discriminateAction(bQuery);

}

/**
* Discriminate an action of an object
* @param bInput (discriminateAction <instance>)
*/
Bottle abmReasoning::discriminateAction(Bottle bInput)
{
    Bottle  bOutput,
        bMove;

    if (bInput.size() <2)
    {
        std::cout << "Error in abmReasoning::discriminateAction | wrong number of inputs" << endl ;
        bOutput.addString("Error in abmReasoning::discriminateAction | wrong number of inputs");
        return bOutput;
    }
    int Id = bInput.get(1).asInt();

    bMove = Interlocutor.askActionFromId(Id);
    spatialKnowledge skMove;
    pair<double,double> BEFORE,
        XY,
        MOVE;

    BEFORE  = abmReasoningFunction::coordFromString(bMove.get(2).toString().c_str());
    XY      = abmReasoningFunction::coordFromString(bMove.get(3).toString().c_str());
    MOVE.first = XY.first - BEFORE.first;
    MOVE.second = XY.second - BEFORE.second;

    string  sAction,
        sArgument;

    double  dMin = 1000000,
        dDist;

    vector<double> vcScore;
    vector< pair <string, string> > vcAction;
    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
    {
        if (it->sName!= abmReasoningFunction::TAG_DB_UNKNOWN)
        {

            dDist = fabs((it->distFromMove(XY, MOVE)).second);
            vcScore.push_back(dDist);
            pair <string, string> pAction;
            pAction.first = it->sName;
            pAction.second = it->sArgument;
            vcAction.push_back(pAction);
        }
    }

    // Detect min
    int indiceMin=0;
    for (unsigned int i = 0; i < vcScore.size() ; i++)
    {
        if (vcScore[i]<dMin)
        {
            indiceMin = i;
            dMin = vcScore[i];
        }
    }

    vector<double> vcScoreTemp = vcScore;
    sort(vcScoreTemp.begin(), vcScoreTemp.end());

    double  dScore = vcScoreTemp[1]/vcScoreTemp[0];

    // Insert main discrimination
    Bottle bActionMain;
    sAction = vcAction[indiceMin].first;
    sArgument = vcAction[indiceMin].second;
    bActionMain.addString(sAction.c_str());
    bActionMain.addString(sArgument.c_str());
    //  bActionMain.addDouble(vcScore[1]/vcScore[0]);


    bool bConflict = false; // if there is conflict in the discrimination
    for (unsigned int i = 0 ; i < vcScore.size() ; i++)
    {
        if ((int)i != indiceMin)
        {
            if ( (vcScore[i]/dMin) < THRESHOLD_CONFIDENCE )
            {
                bConflict = true;
                Bottle bActionConflict;
                bActionConflict.addString(vcAction[i].first.c_str());
                bActionConflict.addString(vcAction[i].second.c_str());
                bActionConflict.addDouble(vcScore[i]);

                bOutput.addList() = bActionConflict;
            }
        }
    }

    if (bConflict)
    {

        std::cout << "conflict in the discrimination" << endl;
        bActionMain.addDouble(dMin);
    }
    else
    {
        bActionMain.addDouble(dScore);

        //renameAction (<instance_begin> <activity_name> <activity_type>) (argument <argument_name> <argument_type> <argument_subtype> <argument_role>)

        Bottle bRename,
            bAct,
            bArg;
        bAct.addInt(Id);
        bAct.addString(sAction.c_str());
        bAct.addString(abmReasoningFunction::TAG_DB_ACTION.c_str());
        bArg.addString(abmReasoningFunction::TAG_DB_ARGUMENT.c_str());
        bArg.addString(sArgument.c_str());
        bArg.addString("external");
        bArg.addString(abmReasoningFunction::TAG_DEFAULT.c_str());
        bArg.addString("spatial1");
        bRename.addString("renameAction");
        bRename.addList() = bAct;
        bRename.addList() = bArg;

        renameAction(bRename);

        bRename.clear();
        bAct.clear();
        bArg.clear();
        bAct.addInt(Id+1);
        bAct.addString(sAction.c_str());
        bAct.addString(abmReasoningFunction::TAG_DB_ACTION.c_str());
        bArg.addString(abmReasoningFunction::TAG_DB_ARGUMENT.c_str());
        bArg.addString(sArgument.c_str());
        bArg.addString("external");
        bArg.addString(abmReasoningFunction::TAG_DEFAULT.c_str());
        bArg.addString("spatial1");
        bRename.addString("renameAction");
        bRename.addList() = bAct;
        bRename.addList() = bArg;

        renameAction(bRename);

        std::cout << "Action modified" << endl;
    }

    bOutput.addList() = bActionMain;

    std::cout << "Confidence : " << dScore << endl;
    return bOutput;
}

/*
* Search all the unknown actions in the DB and try to discriminate them
*
*/
Bottle abmReasoning::discriminateUnknownActions()
{
    Bottle bOutput;
    //simple action
    string actionName = abmReasoningFunction::TAG_DB_UNKNOWN ;
    Bottle bRoleAndRoleValue ;

    vector <pair <int, int> > vOutput = getIdFromActivity(actionName, bRoleAndRoleValue) ;
    int iModified =0;
    for (vector <pair <int, int> >::iterator it = vOutput.begin(); it != vOutput.end() ; it++)
    {
        Bottle bMessenger;
        bMessenger.addString("discriminateAction");
        bMessenger.addInt(it->first);
        bMessenger = discriminateAction(bMessenger);
        double dScore = bMessenger.get(2).asDouble();
        if (dScore>10)
        {
            iModified++;
        }
    }
    ostringstream osReturn;
    osReturn << iModified << " action(s) modified on " << vOutput.size() << " unknown action(s)";
    bOutput.addString(osReturn.str().c_str());

    return bOutput;
}



/*   -------------------------   EXECUTE FUNCTIONS  ------------------------------   */


/**
* Search for the given action, and return the properties
* @param bInput : Bottle ("executeAction"       actionName      argument        object      agent)
*/
Bottle abmReasoning::executeAction(Bottle bInput)
{
    Bottle  bOutput, // main output
        bCoord;  // Bottle of coordonates


    // Check format of input
    if (bInput.size() <= 4)
    {
        bOutput.addString("Error in the size of the input in executeAction");
        return bOutput;
    }

    if (!bInput.get(1).isString() || !bInput.get(2).isString() || !bInput.get(3).isString())
    {
        bOutput.addString("Error in the format of the input in executeAction");
        return bOutput;
    }
    string  sAction = bInput.get(1).toString().c_str(),
        sArgument = bInput.get(2).toString().c_str(),
        sObject = bInput.get(3).toString().c_str(),
        sAgent = bInput.get(4).toString().c_str();


    // Search action in the list of knowledge
    bool bFound = false; 
    spatialKnowledge skWantedAction;
    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin() ; it != listSpatialKnowledge.end() ; it++)
    {
        if (it->sName == sAction)
        {
            if (it->sArgument == sArgument)
            {
                skWantedAction = *it;
                bFound = true;
            }
        }
    }

    if (!bFound)
    {
        sAction = "Cannot find action : " + sAction;
        bOutput.addString(sAction.c_str());
        return bOutput;
    }

    bFound = false;
    contextualKnowledge ckAction;
    for (vector<contextualKnowledge>::iterator it = listContextualKnowledge.begin() ; it != listContextualKnowledge.end() ; it++)
    {
        if (it->sName == sAction)
        {
            if (it->sArgument == sArgument)
            {
                ckAction = *it;
                bFound = true;
            }
        }
    }

    if (bFound)
    {
        ckAction.checkConditions();
    }

    // Return
    if (skWantedAction.isAbsolut)
    {
        bOutput.addString("move");
        bOutput.addString("absolut");
        double muX = 0, muY = 0;
        for (unsigned int i=0; i<skWantedAction.vX.size(); i++)
        {
            muX += skWantedAction.vX[i];
            muY += skWantedAction.vY[i];
        }
        muX /= (skWantedAction.vX.size() *1.);
        muY /= (skWantedAction.vX.size() *1.);
        bCoord.addDouble(muX);
        bCoord.addDouble(muY);
        bOutput.addList() = bCoord;
    }
    else if (skWantedAction.isRelative)
    {
        bOutput.addString("move");
        bOutput.addString("relative");
        double muDX = 0, muDY = 0;
        for (unsigned int i=0; i<skWantedAction.vX.size(); i++)
        {
            muDX += skWantedAction.vDX[i];
            muDY += skWantedAction.vDY[i];
        }
        muDX /= (skWantedAction.vX.size() *1.);
        muDY /= (skWantedAction.vX.size() *1.);

        bCoord.addDouble(muDX);
        bCoord.addDouble(muDY);
        bOutput.addList() = bCoord;
    }
    Bottle bArgu, bRole, bAction;

    bAction.addString(abmReasoningFunction::TAG_DB_ACTION.c_str());
    bAction.addString(sAction.c_str());

    bRole.addString("spatial1");
    bRole.addString("object1");
    bRole.addString("agent1");

    bArgu.addString(sArgument.c_str());
    bArgu.addString(sObject.c_str());
    bArgu.addString(sAgent.c_str());

    bAction.addList() = bArgu;
    bAction.addList() = bRole;

    bOutput.addList() = bAction;


    return bOutput;
}

/**
*  Return the command to make, according to the input : complex or action
* @param bInput : Bottle : "executeActivity"  <actionkind> <actionname> (<arg 1>  <arg 2>  ...  <arg n>)  (<role 1>  <role 2 >  ... <role n>)  <isBegin>
*/
Bottle abmReasoning::executeActivity(Bottle bInput)
{
    Bottle bOutput;
    string sErrorFormat = "Error in abmReasoning::executeActivity | wrong format of input";
    Bottle bErrorFormat;
    bErrorFormat.addString(sErrorFormat.c_str());
    if (bInput.size() < 5)
    {
        std::cout << "Error in abmReasoning::executeActivity | wrong number of input" << endl;
        bOutput.addString("Error in abmReasoning::executeActivity | wrong number of input");
        return bOutput;
    }

    // Get activity kind
    string sActionKind;
    if (bInput.get(1).isString())
    {
        sActionKind = bInput.get(1).toString();
    }
    else
    {
        std::cout << sErrorFormat << endl;
        return bErrorFormat;
    }

    // if execute action :
    if (sActionKind == abmReasoningFunction::TAG_DB_ACTION.c_str())
    {
        string sAction, sArgument, sObject, sAgent;
        Bottle bArgu, bRole;

        // get action
        if (bInput.get(2).isString())
        {
            sAction = bInput.get(2).toString();
        }
        else
        {
            std::cout << sErrorFormat << endl;
            return bErrorFormat;
        }

        // get the arguments
        if (bInput.get(3).isList())
        {
            bArgu = *bInput.get(3).asList();
        }
        else
        {
            std::cout << sErrorFormat << endl;
            std::cout << "Arguments missing" << endl;
            return bErrorFormat;
        }

        // get the Roles
        if (bInput.get(4).isList())
        {
            bRole = *bInput.get(4).asList();
        }
        else
        {
            std::cout << sErrorFormat << endl;
            std::cout << "Roles missing" << endl;
            return bErrorFormat;
        }

        // Check size
        int iSizeArg=0;
        if (bArgu.size() == bRole.size())
        {
            iSizeArg = bArgu.size();
        }
        else
        {
            std::cout << sErrorFormat << endl;
            std::cout << "Argument and role of size different" << endl;
            return bErrorFormat;
        }

        // get action object and argument
        bool fObject = false, fArgument = false, fAgent = false;
        for (int i = 0 ; i < iSizeArg ; i++)
        {
            if (bRole.get(i).toString() == "object1")
            {
                sObject = bArgu.get(i).toString();
                fObject = true;
            }
            if (bRole.get(i).toString() == "spatial1")
            {
                sArgument = bArgu.get(i).toString();
                fArgument = true;
            }
            if (bRole.get(i).toString() == "agent1")
            {
                sAgent = bArgu.get(i).toString();
                fAgent = true;
            }
        }
        if (fObject && fArgument)
        {
            Bottle bExecute;
            bExecute.addString("executeAction");
            bExecute.addString(sAction.c_str());
            bExecute.addString(sArgument.c_str());
            bExecute.addString(sObject.c_str());
            bExecute.addString(sAgent.c_str());
            Bottle bOutput;
            bOutput.addList() = (executeAction(bExecute));
            return  bOutput;
        }
        else
        {
            std::cout << sErrorFormat << endl;
            std::cout << "Can't find argument : object1 or spatial1" << endl;
            return bErrorFormat;
        }
    }


    // if execute complex : 
    if (sActionKind == "complex")
    {
        return executeComplex(bInput);
    }

    // if execute reason : 
    if (sActionKind == "reasoning")
    {
        Bottle bReasoning;
        bReasoning.addString("executeReasoning");
        Bottle bTemp;
        Bottle bCond;

        bTemp.addString(abmReasoningFunction::TAG_DB_ACTION.c_str());
        bTemp.addString(bInput.get(2).toString().c_str());
        bTemp.addString("reasoning");
        bReasoning.addList() = bTemp;

        //build the condition bottle : ( (sub1 vb1 obj1 manner1) (sub2 vb2 obj2 manner2) ...)
        // from 2 bottles (arg1, arg2, arg3, ..., argn) (role1, role2, role3, ..., rolen)
        //                        bInput.get(3)                 bInput.get(4)
        int nbCond = bInput.get(3).asList()->size()/4 ;


        for(int i = 1; i <= nbCond ; i++) {
            Bottle bCurrentCond ;
            ostringstream ossVb, ossSub, ossObj, ossManner ;
            ossManner << abmReasoningFunction::TAG_DB_MANNER << i ;
            ossVb << "verb" << i ;
            ossSub << "subject" << i ;
            ossObj << "object" << i ;


            //go to string because cannot do vector of osstringstream
            string sManner = ossManner.str().c_str() ;
            string sVb = ossVb.str().c_str() ;
            string sSub = ossSub.str().c_str() ;
            string sObj = ossObj.str().c_str() ;


            //vector for searching role : /!\ order is important (vb first, then subject and [object])
            vector <string> vString ;
            vString.push_back(sManner);
            vString.push_back(sVb);
            vString.push_back(sSub);
            vString.push_back(sObj);


            //go through the 4 elements role to build a valid condition
            for(vector<string>::iterator itString = vString.begin() ; itString != vString.end() ; itString++) {
                int j = 0;
                bool roleFound = false ;

                //check where is the position of the current role in the role bottle
                while(!roleFound && j < bInput.get(4).asList()->size()){

                    string currentRole = bInput.get(4).asList()->get(j).toString().c_str();

                    //std::cout << "element nb " << j << " in role bottle is : {" << currentRole.c_str() << "} and itString is {" << itString->c_str() << "}" <<endl ; 
                    
                    if (*itString == currentRole){
                        roleFound = true ;
                        //put the arg corresponding to this role in the current condition bottle
                        bCurrentCond.addString(bInput.get(3).asList()->get(j).toString().c_str()) ;
                    }

                    j += 1 ;
                }

                //there is a role missing
                if (roleFound == false) {
                    std::cout << "ERROR when building condition bottle for executeReasoning : one role in condition number " << nbCond << " is missing" << endl ; 
                }
            } //end of the currentBottle build

            //add the currentCondition to the condition bottle (if the condition is valid, 4 elements)
            if (bCurrentCond.size() == 4) {
                bCond.addList() = bCurrentCond;
            }
        }

        //bReasoning.addList() = *bInput.get(3).asList();
        bReasoning.addList() = bCond ;

        std::cout << "Bottle sent to executeReasoning : " << bReasoning.toString().c_str() << endl ;

        return executeReasoning(bReasoning);
    }


    // if execute sharedPlan
    if (sActionKind == abmReasoningFunction::TAG_DB_SHARED_PLAN)
    {
        Bottle bSP;
        bSP.addString("executeSharedPlan");
        Bottle bTemp;
        bTemp.addString(abmReasoningFunction::TAG_DB_ACTION.c_str());
        bTemp.addString(bInput.get(2).toString().c_str());
        bTemp.addString(abmReasoningFunction::TAG_DB_SHARED_PLAN.c_str());
        bSP.addList() = bTemp;
        bSP.addList() = *bInput.get(3).asList();
        bSP.addList() = *bInput.get(4).asList();

        return executeSharedPlan(bSP);
    }


    return bErrorFormat;
}

/*
* Return the list of action to execute for a complex
* @param bInput : Bottle : "executeComplex"  complex <actionname> (<arg 1>  <arg 2>  ...  <arg n>)  (<role 1>  <role 2 >  ... <role n>)  <isBegin>
*/
Bottle abmReasoning::executeComplex(Bottle bInput)
{
    string sErrorFormat = "Error in abmReasoning::executeComplex | wrong format of input";
    Bottle bErrorFormat;
    bErrorFormat.addString(sErrorFormat.c_str());


    // Extract the arguments : 
    string sTemporal,
        sObject1,   sObject2,
        sAgent1,    sAgent2,
        sAction1,   sAction2,
        sArgument1, sArgument2;

    Bottle bArgu, bRole;

    // get the bottle of arguments
    if (bInput.get(3).isList())
    {
        bArgu = *bInput.get(3).asList();
    }
    else
    {
        std::cout << sErrorFormat << endl;
        std::cout << "Arguments missing" << endl;
        return bErrorFormat;
    }

    // get the bottle of Roles
    if (bInput.get(4).isList())
    {
        bRole = *bInput.get(4).asList();
    }
    else
    {
        std::cout << sErrorFormat << endl;
        std::cout << "Roles missing" << endl;
        return bErrorFormat;
    }

    // Check size
    int iSizeArg=0;
    if (bArgu.size() == bRole.size())
    {
        iSizeArg = bArgu.size();
    }
    else
    {
        std::cout << sErrorFormat << endl;
        std::cout << "Argument and role of size different" << endl;
        return bErrorFormat;
    }

    // get action object and argument
    bool fObject1 = false, fObject2 = false, fAction1 = false, fAction2 = false, fArgument1 = false, fArgument2 = false, fAgent1 = false, fAgent2 = false, fTemporal = false;
    for (int i = 0 ; i < iSizeArg ; i++)
    {
        if (bRole.get(i).toString() == "object1")
        {
            sObject1 = bArgu.get(i).toString();
            fObject1 = true;
        }
        if (bRole.get(i).toString() == "object2")
        {
            sObject2 = bArgu.get(i).toString();
            fObject2 = true;
        }
        if (bRole.get(i).toString() == "action1")
        {
            sAction1 = bArgu.get(i).toString();
            fAction1 = true;
        }
        if (bRole.get(i).toString() == "action2")
        {
            sAction2 = bArgu.get(i).toString();
            fAction2 = true;
        }
        if (bRole.get(i).toString() == "spatial1")
        {
            sArgument1 = bArgu.get(i).toString();
            fArgument1 = true;
        }
        if (bRole.get(i).toString() == "spatial2")
        {
            sArgument2 = bArgu.get(i).toString();
            fArgument2 = true;
        }
        if (bRole.get(i).toString() == "agent1")
        {
            sAgent1 = bArgu.get(i).toString();
            fAgent1 = true;
        }
        if (bRole.get(i).toString() == "agent2")
        {
            sAgent2 = bArgu.get(i).toString();
            fAgent2 = true;
        }
        if (bRole.get(i).toString() == "temporal")
        {
            sTemporal = bArgu.get(i).toString();
            fTemporal = true;
        }
    }
    if (!fObject1 || !fObject2 || !fAction1 || !fAction2 || !fArgument1 || !fArgument2 || !fTemporal)
    {
        std::cout << sErrorFormat << endl;
        std::cout << "some arguments are missing" << endl;
        return bErrorFormat;
    }

    Bottle bAction1, bAction2;

    bAction1.addString("executeAction");
    bAction1.addString(sAction1.c_str());
    bAction1.addString(sArgument1.c_str());
    bAction1.addString(sObject1.c_str());
    bAction1.addString(sAgent1.c_str());
    bAction1 = executeAction(bAction1);

    bAction2.addString("executeAction");
    bAction2.addString(sAction2.c_str());
    bAction2.addString(sArgument2.c_str());
    bAction2.addString(sObject2.c_str());
    bAction2.addString(sAgent2.c_str());
    bAction2 = executeAction(bAction2);

    double dProba = -1.;

    for (vector<timeKnowledge>::iterator it = listTimeKnowledge.begin() ; it != listTimeKnowledge.end() ; it++)
    {
        if (it->sTemporal == sTemporal)
        {
            dProba = it->T1inferiorT2percent();
        }
    }
    std::cout << "Temporal : " << sTemporal << "\ndProba : " << dProba << endl;
    if (dProba == -1)
    {
        std::cout << sErrorFormat << endl;
        std::cout << "temporal not found" << endl;
        return bErrorFormat;
    }

    Bottle bComplex;

    if (dProba > 0.5)
    {
        bComplex.addList() = bAction1;
        bComplex.addList() = bAction2;
    }
    else
    {
        bComplex.addList() = bAction2;
        bComplex.addList() = bAction1;
    }


    return bComplex;
}

/*
* Return the list of action to execute for a complex
* @param bInput : Bottle : "executeSharedPlan"  ('action' name_action 'sharedplan') (<arg 1>  <arg 2>  ...  <arg n>)  (<role 1>  <role 2 >  ... <role n>)
*/
Bottle abmReasoning::executeSharedPlan(Bottle bInput)
{
    string sErrorFormat = "Error in abmReasoning::executeSharedPlan | wrong format of input";
    Bottle bErrorFormat, bOutput, bTemp;
    bErrorFormat.addString(sErrorFormat.c_str());

    // check input
    if (bInput.size() != 4)
    {
        string sError = "Error in abmReasoning::executeSharedPlan | Wrong number of input (!= 4)" ;
        std::cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList())
    {
        string sError =  "Error in abmReasoning::executeSharedPlan | Wrong format of input" ;
        std::cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    string sName;
    bTemp = *(bInput.get(1).asList());
    if (bTemp.get(0).toString() == abmReasoningFunction::TAG_DB_ACTION.c_str())
    {
        sName = bTemp.get(1).asString();
    }


    int iNbArg = 0;
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError =  "Error in autobiographicalMemory::snapshotSP | number of argument different of number of role" ;
        std::cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    iNbArg = bInput.get(2).asList()->size();


    // Fill contentArg
    vector< pair<string, string> > vArgument;

    string sManner;
    bool fManner = false;

    // catch the arguments and the role associate

    Bottle bArguments   = *bInput.get(2).asList();
    Bottle bRoles       = *bInput.get(3).asList();

    for (int i = 0 ; i < iNbArg ; i++)
    {
        pair <string, string > pArgu;
        pArgu.first = (bArguments.get(i).toString().c_str());
        pArgu.second = (bRoles.get(i).toString().c_str());

        vArgument.push_back(pArgu);

        if (bRoles.get(i).toString() == abmReasoningFunction::TAG_DB_MANNER.c_str() )
        {
            sManner = pArgu.first;
            fManner = true;
        }
    }

    if (!fManner)
    {
        pair <string, string > pArgu;
        pArgu.first = abmReasoningFunction::TAG_DB_NONE;
        pArgu.second = abmReasoningFunction::TAG_DB_MANNER;
        std::cout << "manner not found. Auto set to : none" << endl;
        sManner = abmReasoningFunction::TAG_DB_NONE;
        vArgument.push_back(pArgu);
    }



    // search the shared plan associate
    bool fSP = false, fAgent, fArgu, fObject;
    Bottle bActionTemp;
    string sActionTemp, sArgTemp, sObjectTemp, sAgentTemp;
    for (vector<plan>::iterator it_SP = listPlan.begin() ; it_SP != listPlan.end() ; it_SP ++)
    {
        if (it_SP->sManner == sManner && it_SP->sName == sName && !fSP)
        {
            //std::cout << "sharedPlan found" << endl;
            fSP = true;

            for (unsigned int iAction = 0 ; iAction < it_SP->vActivitytype.size() ; iAction++)
            {
                fAgent = false;
                fArgu = false;
                fObject = false;
                bActionTemp.clear();
                bActionTemp.addString("executeAction");
                bActionTemp.addString(it_SP->vActivityname[iAction].c_str());
                for ( list < pair < string , string > >::iterator it_ACTI =  it_SP->vActivityArguments[iAction].begin() ; it_ACTI != it_SP->vActivityArguments[iAction].end() ; it_ACTI++)
                {

                    for (vector < pair < string, string > >::iterator it_input_arg = vArgument.begin() ; it_input_arg != vArgument.end() ; it_input_arg ++)
                    {
                        if (it_ACTI->second == it_input_arg->second)
                        {
                            string stempX = it_ACTI->second;
                            string stemp = stempX ;
                            if (stempX.length()>1){
                                stemp = stempX.substr (0,stempX.length()-1) ;
                            }

                            if (stemp == "object")
                            {
                                sObjectTemp = it_input_arg->first;
                                fObject = true;
                            }

                            if (stemp == "agent")
                            {
                                sAgentTemp = it_input_arg->first;
                                fAgent = true;
                            }
                        }
                        if (it_ACTI->second == "spatial1")
                        {
                            sArgTemp = it_ACTI->first;
                            fArgu = true;
                        }
                    }
                }
                if (!fObject)
                {
                    for ( list < pair < string , string > >::iterator it_ACTI =  it_SP->vActivityArguments[iAction].begin() ; it_ACTI != it_SP->vActivityArguments[iAction].end() ; it_ACTI++)
                    {
                        string stempX = it_ACTI->second;
                        string stemp = stempX ;
                        if (stempX.length()>1){
                            stemp = stempX.substr (0,stempX.length()-1) ;
                        }

                        if (stemp == "object")
                        {
                            sObjectTemp = it_ACTI->first;
                            fObject = true;
                        }
                    }
                }
                if (!fAgent)
                {
                    for ( list < pair < string , string > >::iterator it_ACTI =  it_SP->vActivityArguments[iAction].begin() ; it_ACTI != it_SP->vActivityArguments[iAction].end() ; it_ACTI++)
                    {
                        string stempX = it_ACTI->second;
                        string stemp = stempX ;
                        if (stempX.length()>1){
                            stemp = stempX.substr (0,stempX.length()-1) ;
                        }

                        if (stemp == "agent")
                        {
                            sAgentTemp = it_ACTI->first;
                            fAgent = true;
                        }
                    }
                }
                if (!fAgent || !fObject || ! fArgu)
                {
                    bOutput.clear();
                    string sError = "Error in abmReasoning::executeSharedPlan | wrong argument of sharedplan" ;
                    std::cout << sError << endl;
                    bOutput.addString(sError.c_str());
                    return bOutput;
                }
                bActionTemp.addString(sArgTemp.c_str());
                bActionTemp.addString(sObjectTemp.c_str());
                bActionTemp.addString(sAgentTemp.c_str());
                bOutput.addList() = executeAction(bActionTemp);
            }
        }
    }

    if (!fSP)
    {
        std::cout << "in abmReasoning::executeSharedPlan : can't find shared plan : " << sName << "_" << sManner << endl;
    }
    return bOutput;
}

/*
* Return the list of action to execute for a reasoning request
* @param bInput : Bottle : "executeReasoning"  ('action' want 'reasoning') ( (sub1 verb1 obj1) (sub2 verb2 obj2) ...) where (sub verb obj) defined a condition/goal
*/
Bottle abmReasoning::executeReasoning(Bottle bInput)
{
    string sErrorFormat = "Error in abmReasoning::executeReasoning | wrong format of input";
    Bottle bErrorFormat, bOutput, bTemp;
    bErrorFormat.addString(sErrorFormat.c_str());

    // check input
    if (bInput.size() != 3)
    {
        string sError = "Error in abmReasoning::executeReasoning | Wrong number of input (!= 3)" ;
        std::cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() )
    {
        string sError =  "Error in abmReasoning::executeReasoning | Wrong format of input" ;
        std::cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    string sName;
    bTemp = *(bInput.get(1).asList());
    if (bTemp.get(0).toString() == abmReasoningFunction::TAG_DB_ACTION.c_str())
    {
        sName = bTemp.get(1).asString();
    }


    int iNbCond = 0;
    if (bInput.get(2).asList()->size() <= 0)
    {
        string sError =  "Error in abmReasoning::executeReasoning | no goal are given" ;
        std::cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    iNbCond = bInput.get(2).asList()->size();


    // Fill contentArg
    vector< pair<string, string> > vArgument;

    // catch the arguments and the role associate
    Bottle bConditions  = *bInput.get(2).asList();
    
    //---------------- > create the domain PDDL file
    printPDDLContextualKnowledgeDomain();   

    //---------------- > create the problem PDDL file
    printPDDLContextualKnowledgeProblem(bConditions);

    //---------------- > launch the planner solver
    pddlPlannerLauncher();

    //---------------- > launch the planner solver
    bOutput = pddlPlannerSolParser();

    //bool planPossible = false ;

    //format the plan in bOutput
    //if (!planPossible)
    //{
    //  std::cout << "in abmReasoning::executeReasoning : can't find a proper plan :  the goal is not achievable" << endl;
    //}
    return bOutput;
}



/*   -------------------------   KNOWLEDGE RELATED FUNCTIONS  ------------------------------   */


/*
* Print in a text file the spatial konwledges
*   X   Y   TYPE    NAME
*/
void abmReasoning::printSpatialKnowledge()
{
    string filepath = (path+"/spatial_knowledge.txt").c_str();
    ofstream file(filepath.c_str(), ios::out | ios::trunc);  // ouverture en écriture avec effacement du fichier ouvert
    file << "X\tY\ttype\tname" << endl;

    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
    {
        string sNameFile = it->sName;
        sNameFile += "_";
        sNameFile += it->sArgument;
        for (unsigned int i=0; i<it->vX.size(); i++)
        {
            if (it->isAbsolut)
            {
                file << it->vX[i] << "\t"<< it->vY[i] << "\tpoint\t" << sNameFile << endl;
            }
            else if (it->isRelative)
            {
                file << it->vDX[i] << "\t"<< it->vDY[i] << "\tvector\t" << sNameFile << endl;
            }
        }
    }
    std::cout << "file " << filepath << " written" << endl;
}

/*
* Check for each action is their is a context linked to the known location
*/
void abmReasoning::checkContextLocation()
{

    Bottle bOutput;
    mapLocation.clear();

    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end() ; it++)
    {
        // Is the spatialKnowledge absolut (put) or relative (push)
        if (it->isAbsolut && it->vX.size() >= abmReasoningFunction::threshold_determine_Location && it->sArgument != "near" && it->sName != "hanoi")
        {
            pair <vector <double> , vector<double> > vData;
            vData.first = it->vX;
            vData.second = it->vY;
            mapLocation[it->sArgument] = vData;
        }
    }

    // for kind of action :
    for (vector<spatialKnowledge>::iterator itSK = listSpatialKnowledge.begin(); itSK != listSpatialKnowledge.end(); itSK++)
    {
        int iNbOccurence = itSK->vX.size();

        // search for the CK associated
        for ( vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin(); itCK != listContextualKnowledge.end() ; itCK++)
        {
            // CK associated found
            if ( itCK->sArgument == itSK->sArgument && itCK->sName == itSK->sName )
            {

                // For each move
                for (int Oc = 0 ; Oc< iNbOccurence ; Oc++)
                {
                    // get the coordinates
                    pair<double, double>    BEFORE (itSK->vX[Oc]-itSK->vDX[Oc],itSK->vY[Oc]-itSK->vDY[Oc] ),
                        AFTER (itSK->vX[Oc], itSK->vY[Oc]) ;
                    
                    for (map<string, pair<vector<double> ,vector<double> > >::iterator itMAP = mapLocation.begin() ; itMAP != mapLocation.end() ; itMAP++)
                    {

                        bool before = false,
                            after = false;

                        // BEFORE :
                        if (abmReasoningFunction::getMahalaDist(itMAP->second.first, itMAP->second.second, BEFORE) < abmReasoningFunction::threshold_is_at_location)
                            before = true;

                        // AFTER
                        if (abmReasoningFunction::getMahalaDist(itMAP->second.first, itMAP->second.second, AFTER) < abmReasoningFunction::threshold_is_at_location)
                            after = true;

                        pair < bool, bool > pLoc (before, after);

                        itCK->mIntersectLocation[itMAP->first].push_back(pLoc);
                    }
                }
            }
        }
    }
}


/**
* Add a spatialKnowledge in listSpatialKnowledge. Create a new one if the action in new or add the knowledge to an existing spatialKnowledge.
*
*/
Bottle abmReasoning::addSpatialKnowledge(spatialKnowledge skInput, bool b_Update)
{
    Bottle bOutput;
    pair<double, double> XY (skInput.vX[0],skInput.vY[0]) ,
        MOVE (skInput.vDX[0] , skInput.vDY[0]) ;

    string sName  = skInput.sName,
        sArgument = skInput.sArgument,
        sDependance = skInput.sDependance;

    bool bFound = false;
    if (skInput.sName == abmReasoningFunction::TAG_DB_UNKNOWN.c_str())
    {
        bOutput.addString("in abmReasoning::addSpatialKnowledge : name of action unknown");
        return bOutput;
    }
    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
    {
        if ((it->sName == sName) && (it->sArgument == sArgument) && (it->sDependance == sDependance))
        {
            it->vX.push_back(XY.first);
            it->vY.push_back(XY.second);
            it->vDX.push_back(MOVE.first);
            it->vDY.push_back(MOVE.second);
            it->determineInfluence();
            bFound = true;
            bOutput.addString("action added");
        }
    }

    if (!bFound)
    {
        skInput.iSize = skInput.vDX.size();
        skInput.determineInfluence();
        listSpatialKnowledge.push_back(skInput);
        bOutput.addString("action created");
    }

    if (b_Update)   {
        string updateLoc = updateLocation(sArgument).toString().c_str();
        bOutput.addString(updateLoc.c_str()); }

    return bOutput;
}

/**
* Add a timeKnowledge in listTimeKnowledge. Create a new one if the action in new or add the knowledge to an existing timeKnowledge.
*
*/
Bottle abmReasoning::addTimeKnowledge(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.size() != 3)
    {
        std::cout << "in abmReasoning::addTimeKnowledge : wrong number of inputs" << endl;
        bOutput.addString("in abmReasoning::addTimeKnowledge : wrong number of inputs");
        return bOutput;
    }
    if (!(bInput.get(0).isString() && bInput.get(1).isString() && bInput.get(2).isString()))
    {
        std::cout << "in abmReasoning::addTimeKnowledge : wrong number of inputs" << endl;
        bOutput.addString("in abmReasoning::addTimeKnowledge : wrong number of inputs");
        return bOutput;
    }
    bool bFound = false;
    for (vector<timeKnowledge>::iterator it = listTimeKnowledge.begin(); it != listTimeKnowledge.end(); it++)
    {
        string sTemp = it->sTemporal,
            sInput = bInput.get(0).toString().c_str();
        bool test = (sTemp == sInput);
        if (test)
        {
            it->addKnowledge(bInput);
            bOutput.addString("complex added");
            bFound = true;
        }
    }

    if (!bFound)
    {
        timeKnowledge tmNew;
        tmNew.fromBottle(bInput);
        listTimeKnowledge.push_back(tmNew);
        bOutput.addString("complex created");
    }

    return bOutput;
}

/**
* Add a plan in listPlan.
*
*/
plan abmReasoning::addPlan(plan pInput)
{
    Bottle bOutput;
    bool found = false,
        escapePlan;
    //  listPlan.push_back(pInput);

    int itePlan = 0;
    // for each existing plan
    for (vector<plan>::iterator current_plan = listPlan.begin() ; current_plan != listPlan.end() ; current_plan++)
    {
        bool fPlan = false;
        // check size and number of argument
        if ( (current_plan->vActivitytype.size() == pInput.vActivitytype.size()) && (current_plan->vArguments.size() == pInput.vArguments.size()) )
        {
            escapePlan = false;
            // check if arguments are the same : 
            for (unsigned int role = 0 ; role < pInput.vArguments.size() ; role++ )
            {
                // check if each argument if found
                bool fRole = false;
                for (unsigned int j = 0 ; j < current_plan->vArguments.size() ; j++)
                {
                    if (pInput.vArguments[role].second == current_plan->vArguments[j].second)
                    {
                        fRole = true;
                        if (pInput.vArguments[role].second == abmReasoningFunction::TAG_DB_MANNER)
                        {
                            if (pInput.vArguments[role].first != current_plan->vArguments[j].first)
                                fRole = false;
                        }
                    }
                }
                if (!fRole)
                    escapePlan = true;
            }

            // all the arguments are presents, check the activities
            if (!escapePlan)
            {
                fPlan = true;
                int iRankActivity = 0;
                bool fActivity = true;
                // if all the activities are the same :
                if ((pInput.vActivityname == current_plan->vActivityname) && (pInput.vActivitytype == current_plan->vActivitytype))
                {
                    // check if the arguments are the same for each activity

                    // for each activity of the input plan
                    for (vector<list<pair<string,string> > >::iterator input_act = pInput.vActivityArguments.begin() ; input_act != pInput.vActivityArguments.end() ; input_act++)
                    {

                        // for each argument of the current activity of the input plan
                        for (list<pair<string, string> >::iterator input_arg = input_act->begin() ; input_arg != input_act->end() ; input_arg++)
                        {
                            bool fRole = false;
                            // each argument of one activity of the plan investigated
                            for (list<pair<string, string> >::iterator investigated_arg = current_plan->vActivityArguments[iRankActivity].begin() ; investigated_arg != current_plan->vActivityArguments[iRankActivity].end() ; investigated_arg++)
                            {
                                if (input_arg->second == investigated_arg->second )
                                {
                                    fRole = true;
                                    if (input_arg->second == "spatial1")
                                    {
                                        if (input_arg->first != investigated_arg->first )
                                            fRole = false;
                                    }
                                }
                            }

                            // if the argument isn't found : 
                            if (!fRole)
                                fActivity = false;
                        }
                        iRankActivity++;
                    }
                    if (!fActivity)
                        fPlan = false;
                }
            }
        }
        if (fPlan)
        {
            //          std::cout << "plan already existing" << endl;
            bOutput.addString("plan already existing");
            found = true;
            plan pReturn;
            pReturn = (listPlan[itePlan]);
            return pReturn;
        }
        itePlan++;
    }

    if (!found)
    {
        //      std::cout << "new plan. Added in listPlan" << endl;
        bOutput.addString("new plan. Added in listPlan");
        listPlan.push_back(pInput);
        plan pReturn;
        pReturn = (listPlan[listPlan.size()-1]);
        return pReturn;
    }
    plan Empty;
    return Empty;
}

/**
* Check if the plan could be a part of an existing plan
*
*/
vector<plan> abmReasoning::checkPlan(plan pInput, vector<plan> listPlanAvailable)
{
    Bottle bOutput;
    //  listPlan.push_back(pInput);


    // for each existing plan
    //for (vector<plan>::iterator current_plan = listPlanAvailable.begin() ; current_plan != listPlanAvailable.end() ; current_plan++)

    vector<plan>::iterator current_plan = listPlanAvailable.begin() ; 
    while(current_plan != listPlanAvailable.end())
    {
        std::cout << "*************************************************************\ncurrent_plan->name : " << current_plan->sName << "\n*************************************************************"<< endl;
        // check size and number of argument : has to be >= 
        if ( current_plan->vActivitytype.size() >= pInput.vActivitytype.size() )
        {
            //to check the activityName
            vector<string>::iterator currentActivityName = current_plan->vActivityname.begin() ;
            vector<string>::iterator pInputActivityName = pInput.vActivityname.begin();
            bool sameActivityName = true ;

            //to check the vActivityArguments
            vector< list < pair < string , string > > >::iterator currentActivityArgument = current_plan->vActivityArguments.begin() ;
            vector< list < pair < string , string > > >::iterator pInputActivityArgument = pInput.vActivityArguments.begin();
            bool sameActivityArguments = true ;

            //check for generalization (isRoleGeneral) and sameConstArg if not, roleFound if yes
            bool isRoleGeneral = false ;
            bool sameConstArg = false ;

            //1. check the activityName are the same
            while(pInputActivityName != pInput.vActivityname.end())
            {

                if(*pInputActivityName != *currentActivityName)
                {
                    std::cout << "STOP : pInputActivityName = " << *pInputActivityName << " is different from currenActivityName = " << *currentActivityName << endl ;
                    std::cout << "[NACK] : " << current_plan->sName << " is the weak link because of missmatched in activityName. Bybye! \n" << endl ;
                    sameActivityName = false ;
                    break ;
                }

                //incr iterator
                currentActivityName ++ ;
                pInputActivityName ++ ;
            }

            if(sameActivityName == false){

                //remove the plan from the list
                current_plan = listPlanAvailable.erase(current_plan) ;
            } 
            else
            {
                //2. if same activities : check the roles are the same (e.g. : agent1 <action> object 1 spatial1)
                //WARNING : SIZE, should be the same because same activity

                while(pInputActivityArgument != pInput.vActivityArguments.end())
                {
                    std::cout << "-------- new Activity --------" << endl ;
                    //check the argument role is somewhere in vArgument of the current plan investigated :
                    //     - if yes : name could be switch
                    //     - if not : name/role is always name/role1 and name should be the same


                    // for each argument of the current activity of the input plan
                    for (list<pair<string, string> >::iterator pInputArg = pInputActivityArgument->begin() ; pInputArg != pInputActivityArgument->end() ; pInputArg++)
                    {
                        //bool for generalization of the argument or not  [init]
                        isRoleGeneral = false ;

                        //bool for spatial (no generalization ever) or object (no generalization sometimes)  [init]
                        sameConstArg = false;                       

                        for (vector< pair < string , string > >::iterator vArg_currentPlan = current_plan->vArguments.begin() ; vArg_currentPlan != current_plan->vArguments.end() ; vArg_currentPlan++)
                        {
                            if(vArg_currentPlan->second == pInputArg->second)
                            {
                                isRoleGeneral = true ;
                                std::cout << "Generalization allowed for " << vArg_currentPlan->second << endl;

                                //change the name according to the generalization
                                //WARNING : COULD BE RESPONSIBLE FOR A BUG
                                //std::cout << "\n******************************************\nWARNING : COULD BE RESPONSIBLE FOR A BUG\n******************************************\n" ; 

                                if(vArg_currentPlan->first != pInputArg->first ) 
                                {
                                    std::cout << "Change the name of the " << vArg_currentPlan->second << " from " << vArg_currentPlan->first << " to " ;
                                    vArg_currentPlan->first = pInputArg->first ;
                                    std::cout << vArg_currentPlan->first << endl ;
                                }

                                break ;
                            }
                        }

                        bool roleFound = false ;
                        //each argument of one activity of the current plan investigated
                        for (list<pair<string, string> >::iterator currentArg = currentActivityArgument->begin() ; currentArg != currentActivityArgument->end() ; currentArg++)
                        {
                            //if no Generalization : the name should be inside the arguments (role is not relevant)
                            if (isRoleGeneral == false)
                            {
                                if(pInputArg->first == currentArg->first)
                                {
                                    std::cout << "constant argument for " << pInputArg->second << " is found (" << currentArg->first << ")" << endl ;

                                    sameConstArg = true ;

                                    break ;
                                }
                            }
                            //same role
                            else if (pInputArg->second == currentArg->second )
                            {
                                std::cout << "role " << pInputArg->second << " is found" << endl ;
                                roleFound = true ;  

                                break ;
                            }
                        }

                        //generalization : error if role not found
                        if ( (isRoleGeneral == true) && (roleFound == false))
                        {
                            std::cout << "\nSTOP : role " << pInputArg->second << " is not found" << endl ;
                            std::cout << "[NACK] : " << current_plan->sName << " is the weak link because of missmatched in a role. Bybye! \n" << endl ;
                            sameActivityArguments = false ;

                            //remove the plan from the list
                            current_plan = listPlanAvailable.erase(current_plan) ;

                            break ;
                        }

                        //no generalization : error if not same argument name
                        if( (isRoleGeneral == false) && (sameConstArg == false))
                        {
                            std::cout << "\nSTOP : Constant Argument  " << pInputArg->first << " is not found" << endl ;
                            std::cout << "[NACK] : " << current_plan->sName << " is the weak link because of missmatched in a constant argument. Bybye! \n" << endl ;
                            sameActivityArguments = false ;

                            //remove the plan from the list
                            current_plan = listPlanAvailable.erase(current_plan) ;

                            break ;
                        }
                    }   

                    std::cout << "------------------------------" << endl ;

                    if( (sameActivityArguments == false) )
                    {
                        break ;
                    }

                    std::cout << "go to the next activityArgument" << endl ;
                    //incr iterator
                    currentActivityArgument ++ ;
                    pInputActivityArgument ++ ;
                }

                if ( (sameActivityName == true) && (sameActivityArguments == true) )
                {
                    std::cout << "\n[ACK] : Shared Plan " << current_plan->sName << " is elligible \n " << endl ;
                    current_plan ++ ;
                } 
            }



            //current SP is smaller than the inputPlan
        } else {
            std::cout << "\nSTOP : Shared Plan " << current_plan->sName << " is smaller than the plan input " << endl ;
            std::cout << "[NACK] : " << current_plan->sName << " is the weak link because of missmatched in a role. Bybye! \n" << endl ;

            //remove the plan from the list
            current_plan = listPlanAvailable.erase(current_plan) ;
        }
    }

    return listPlanAvailable ;
}

/**
* Add a plan in listPlan.
*
*/
Bottle abmReasoning::addSharedPlan(plan pInput)
{
    Bottle bOutput;


    return bOutput;
}

/**
* Add a behavior in listPlan.
*
*/
Bottle abmReasoning::addBehavior(behavior beInput)
{
    Bottle bOutput;
    bool found = false;

    for (unsigned int b = 0 ; b < listBehaviors.size() ; b++)
    {
        if (listBehaviors[b].sName == beInput.sName && !found && listBehaviors[b].sArgument == beInput.sArgument)
        {
            found = true;
            listBehaviors[b].vEffect.push_back(beInput.vEffect[0]);
            bOutput.addString("behavior found. Behavior updated");
            return bOutput;
        }
    }

    listBehaviors.push_back(beInput);
    bOutput.addString("behavior new. Behavior created");

    return bOutput;
}

/**
* Take all the acttivities in the database and build the spatial and time knowledges
*/
Bottle abmReasoning::resetKnowledge()
{
    return resetKnowledge(0);
}


Bottle abmReasoning::resetKnowledge(int from)
{

    bDreaming = true;
    DeleteKnownLocations();
    Bottle  bOutput,
        bRequest,
        bMessenger;

    std::cout << endl << "starting to reset knowledge" << endl;

    bMessenger.addString("resetKnowledge");
    bMessenger = request(bMessenger);

    listSpatialKnowledge.clear();
    listTimeKnowledge.clear();
    listBehaviors.clear();
    listPlan.clear();
    listKnownInteraction.clear();

    findAllActions(from);
    findAllSharedPlan(from);
    findAllComplex(from);
    findAllBehaviors(from);
    findAllInteractions(from);

    int serialSpatial       = Interlocutor.sendSpatialKnowledge(listSpatialKnowledge),
        serialTime          = Interlocutor.sendTemporalKnowledge(listTimeKnowledge),
        serialBehavior      = Interlocutor.sendBehaviors(listBehaviors),
        serialPlan          = Interlocutor.sendPlan(listPlan),
        serialContext       = Interlocutor.sendContextual(listContextualKnowledge),
        serialInteraction   = Interlocutor.sendInteractionKnowledge(listKnownInteraction);


//  checkContextLocation();

    ostringstream osOutput;
    osOutput << "resetKnowledge : " << serialSpatial << " spatialKnowledge(s) added; " << serialTime << " timeKnowledge(s) added; " << serialBehavior << " behavior(s) added; " << serialPlan << " plan(s) added; " << serialContext << " contextualKnowledge(s) added; " << serialInteraction << " interaction(s) added" ;
    bOutput.addString(osOutput.str().c_str());
    bDreaming = false;

    string sLocation = updateKnownLocations().toString().c_str();
    bOutput.addString(sLocation.c_str());
    printSpatialKnowledge();

    for ( vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin() ; itCK != listContextualKnowledge.end() ; itCK++)
    {
        itCK->updateAgentRelated();
    }


    std::cout << endl << osOutput.str().c_str() << endl << endl;

    return bOutput;
}

/*
* Take the knowledge in the spatialknowledge and timeknowledge DB
*/
Bottle abmReasoning::getKnowledge()
{
    std::cout << endl <<  "loading knowledge :" ;
    Bottle bOutput,
        bTimeData,
        bTimeKnowledge,
        bComplex,
        bSpatialData,
        bSpatialKnowledge,
        bContextualKnowledge,
        bContextualData,
        bBehavior,
        bBehaviorData,
        bAction,
        bRequest,
        bInteraction,
        bMainInteraction;

    listSpatialKnowledge.clear();
    listTimeKnowledge.clear();
    listKnownInteraction.clear();
    DeleteKnownLocations();

    ostringstream //osSpatialKnowledge,
        osSpatialData,
        osContextKnowled,
        osTimeKnowledge,
        osTimeData;

    bSpatialKnowledge = requestFromStream("SELECT DISTINCT instance, name, argument, dependance FROM spatialknowledge");
    string sNull = "NULL",
        sResult = bSpatialKnowledge.toString().c_str();

    if (sNull == sResult)
    {
        std::cout << " no spatial data to load" ;
        bOutput.addString("no spatial data");
    }
    else
    {
        for (int i = 0 ; i < bSpatialKnowledge.size() ; i++)
        {
            if (bSpatialKnowledge.get(0).isList())
            {       
                bAction = (*bSpatialKnowledge.get(i).asList());
                int instance = atoi(bAction.get(0).toString().c_str());
                string sName = bAction.get(1).toString().c_str(),
                 sArg = bAction.get(2).toString().c_str(),
                 sDependance = bAction.get(3).toString().c_str();
                spatialKnowledge skAction;
                skAction.sArgument = sArg;
                skAction.sName = sName;
                skAction.sDependance = sDependance;

                osSpatialData.str("");
                osSpatialData << "SELECT vx, vy, vdx, vdy FROM spatialdata WHERE instance = " << instance;
                bSpatialData = requestFromStream(osSpatialData.str().c_str());

                for (int j = 0 ; j < bSpatialData.size() ; j++)
                {
                    skAction.vX.push_back(atof((*bSpatialData.get(j).asList()).get(0).toString().c_str()));
                    skAction.vY.push_back(atof((*bSpatialData.get(j).asList()).get(1).toString().c_str()));
                    skAction.vDX.push_back(atof((*bSpatialData.get(j).asList()).get(2).toString().c_str()));
                    skAction.vDY.push_back(atof((*bSpatialData.get(j).asList()).get(3).toString().c_str()));
                }
                addSpatialKnowledge(skAction, false);
            }
        }
    }

    updateKnownLocations();

    std::cout << " ... ";


    bTimeKnowledge = requestFromStream("SELECT DISTINCT temporal FROM timeknowledge");
    sResult = bTimeKnowledge.toString();

    if (sNull == sResult)
    {
        std::cout << " no temporal data to load" ;
        bOutput.addString("no temporal data");
    }
    else
    {
        for (int i = 0 ; i < bTimeKnowledge.size(); i++)
        {       
            if (bTimeKnowledge.get(0).isList())
            {
                bComplex = (*bTimeKnowledge.get(i).asList());
                string sTemporal = bComplex.get(0).toString().c_str();

                timeKnowledge tkComplex;
                tkComplex.sTemporal = sTemporal;

                osTimeData.str("");
                osTimeData << "SELECT timearg1, timearg2 FROM timedata WHERE temporal = '" << sTemporal << "'";
                bTimeData = requestFromStream(osTimeData.str().c_str());

                for (int j = 0 ; j < bTimeData.size() ; j++)
                {
                    tkComplex.timeArg1.push_back(abmReasoningFunction::string2Time(((*bTimeData.get(j).asList()).get(0).toString()).c_str() ));
                    tkComplex.timeArg2.push_back(abmReasoningFunction::string2Time(((*bTimeData.get(j).asList()).get(1).toString()).c_str() ));
                }
                listTimeKnowledge.push_back(tkComplex);
            }
        }
    }


    std::cout << " ... ";

    Bottle  bMainBehavior = requestFromStream("SELECT DISTINCT instance, name, argument FROM behavior");
    sResult = bMainBehavior.toString();
    ostringstream osBehavior;
    if (sNull == sResult)
    {
        std::cout << "no behavior data to load";
        bOutput.addString("no behavior data");
    }
    else
    {
        for (int i = 0 ; i < bMainBehavior.size() ; i++)
        {
            if (bMainBehavior.get(i).isList())
            {       
                bAction = (*bMainBehavior.get(i).asList());
                int instance = atoi(bAction.get(0).toString().c_str());
                string sName = bAction.get(1).toString().c_str();
                string sArg = bAction.get(2).toString().c_str();
                behavior newBehavior;
                newBehavior.sArgument = sArg;
                newBehavior.sName = sName;
                osBehavior.str("");
                osBehavior << "SELECT DISTINCT occurence FROM behaviordata WHERE instance = " << instance ;
                Bottle bOccurence = requestFromStream(osBehavior.str().c_str());
                vector<int>  vOccurence;

                for (int oc = 0 ; oc < bOccurence.size() ; oc++)
                {
                    vOccurence.push_back(atoi(bOccurence.get(oc).asList()->get(0).toString().c_str()));
                }

                for (vector<int>::iterator  it_occu = vOccurence.begin() ; it_occu != vOccurence.end() ; it_occu++)
                {

                    osBehavior.str("");
                    osBehavior << "SELECT drive, effect FROM behaviordata WHERE instance = " << instance << " AND occurence = " << *it_occu;
                    bBehaviorData = requestFromStream(osBehavior.str().c_str());

                    vector< pair <string, double> >         vEffect;
                    for (int j = 0 ; j < bBehaviorData.size() ; j++)
                    {
                        // name of the drive, value
                        pair<string, double>  pTemp ((bBehaviorData.get(j).asList())->get(0).toString(), (atof((*bBehaviorData.get(j).asList()).get(1).toString().c_str()))) ;

                        vEffect.push_back(pTemp);
                    }
                    newBehavior.vEffect.push_back(vEffect);
                }
                listBehaviors.push_back(newBehavior);
            }
        }
    }

    bContextualKnowledge = requestFromStream("SELECT DISTINCT instance, name, argument, dependance FROM contextknowledge");
    sResult = bContextualKnowledge.toString();

    if (sNull == sResult)
    {
        std::cout << " no contextual data to load" ;
        bOutput.addString("no contextual data");
    }
    else
    {
        for (int i = 0 ; i < bContextualKnowledge.size() ; i++)
        {
            contextualKnowledge ckAction;
            if (bSpatialKnowledge.get(0).isList())
            {       
                bAction = (*bContextualKnowledge.get(i).asList());
                int instance = atoi(bAction.get(0).toString().c_str());
                string sName = bAction.get(1).toString().c_str();
                string sArg = bAction.get(2).toString().c_str();
                string sDependance = bAction.get(3).toString().c_str();

                ckAction.sArgument = sArg;
                ckAction.sName = sName;
                ckAction.sDependance = sDependance;

                //Presence
                osContextKnowled.str("");
                osContextKnowled << "SELECT presencebegin, presenceend FROM contextdata WHERE instance = " << instance;
                bContextualData = requestFromStream(osContextKnowled.str().c_str());

                for (int j = 0 ; j < bContextualData.size() ; j++)
                {
                    string before = (*bContextualData.get(j).asList()).get(0).toString().c_str(),
                        after = (*bContextualData.get(j).asList()).get(1).toString().c_str();
                    pair <bool , bool > pTemp (before == "t",after == "t") ;
                    ckAction.vObjectPresent.push_back(pTemp);
                }

                //Loc
            }
            ckAction.updatePresence();
            listContextualKnowledge.push_back(ckAction);
        }
    }


    std::cout << " ... ";

    Bottle  bMainPlan = requestFromStream("SELECT DISTINCT instance, name, manner FROM sharedplan");
    sResult = bMainPlan.toString();
    osBehavior.str("");;
    if (sNull == sResult)
    {
        std::cout << "no sharedplan data to load";
        bOutput.addString("no sharedplan data");
    }
    else
    {
        for (int i = 0 ; i < bMainPlan.size() ; i++)
        {
            plan NewPlan;

            // for each plan
            if (bMainPlan.get(i).isList())
            {       
                int instance = atoi(bMainPlan.get(i).asList()->get(0).toString().c_str());
                NewPlan.sName = bMainPlan.get(i).asList()->get(1).toString();
                NewPlan.sManner = bMainPlan.get(i).asList()->get(2).toString();

                ostringstream osPlanArgument;
                osPlanArgument << "SELECT argument, role FROM sharedplanarg WHERE instance = " << instance ;
                bRequest = requestFromStream(osPlanArgument.str().c_str());

                // get the argument of the plan
                for (int arg = 0 ; arg < bRequest.size() ; arg++)
                {
                    pair<string, string> pArgument 
                        (bRequest.get(arg).asList()->get(0).toString(),
                        bRequest.get(arg).asList()->get(1).toString());
                    
                    NewPlan.vArguments.push_back(pArgument);
                }

                ostringstream osPlanAct,
                    osActArg;
                osPlanAct << "SELECT activitytype, activityname, id FROM sharedplandata WHERE instance = " << instance << "  ORDER BY id ";
                bRequest = requestFromStream(osPlanAct.str().c_str());

                // get the activities of the plan
                for (int act = 0 ; act < bRequest.size() ; act++)
                {
                    NewPlan.vActivitytype.push_back(bRequest.get(act).asList()->get(0).toString().c_str());
                    NewPlan.vActivityname.push_back(bRequest.get(act).asList()->get(1).toString().c_str());
                    osActArg.str("");
                    osActArg << "SELECT argument, role FROM spdataarg WHERE instance = " << instance << " AND id = " << atoi(bRequest.get(act).asList()->get(2).toString().c_str()) ;
                    Bottle bArgAct = requestFromStream(osActArg.str().c_str());
                    list <pair <string, string> > lArgAct;

                    // get the argument of the current activity
                    for (int arg = 0 ; arg < bArgAct.size() ; arg++)
                    {
                        pair<string, string> pArgRole;
                        pArgRole.first = bArgAct.get(arg).asList()->get(0).toString();
                        pArgRole.second = bArgAct.get(arg).asList()->get(1).toString();
                        lArgAct.push_back(pArgRole);
                    }
                    NewPlan.vActivityArguments.push_back(lArgAct);
                } // out of the current activity

            }// out of the current plan

            NewPlan = addPlan(NewPlan);
        }
    }


    std::cout << " ...";

    bMainInteraction = requestFromStream("SELECT DISTINCT subject FROM interactionknowledge");
    sResult = bMainInteraction.toString();
    osBehavior.str("");
    if (sNull == sResult)
    {
        std::cout << "no interaction data to load";
        bOutput.addString("no interaction data");
    }
    else
    {
        for (int i = 0 ; i < bMainInteraction.size() ; i++)
        {
            // for each subject
            if (bMainInteraction.get(i).isList())
            {       
                string sSubject = bMainInteraction.get(i).asList()->get(0).toString();

                ostringstream osInteraction;
                osInteraction << "SELECT * FROM interactionknowledge WHERE subject = '" << sSubject << "'";
                bInteraction = requestFromStream(osInteraction.str().c_str());
                knownInteraction TempInt;
                TempInt.sSubject = sSubject;

                // get the argument of the plan
                for (int arg = 0 ; arg < bInteraction.size() ; arg++)
                {
                    tuple<string, int, string, string> tInteraction;
                    get<0>(tInteraction) = bInteraction.get(arg).asList()->get(1).toString();
                    get<1>(tInteraction) = atoi(bInteraction.get(arg).asList()->get(2).toString().c_str());
                    get<2>(tInteraction) = bInteraction.get(arg).asList()->get(3).toString();
                    get<3>(tInteraction) = bInteraction.get(arg).asList()->get(4).toString();
        
                    TempInt.addInteraction(tInteraction);
                }
                listKnownInteraction.push_back(TempInt);
            }
        }
    }



    checkContextLocation();
    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin() ; itCK != listContextualKnowledge.end() ; itCK++)
    {
        itCK->updatePresence();
    }


    std::cout << " done ! " << endl;
    std::cout << listSpatialKnowledge.size() << " spatialKnowledge(s) - " << listTimeKnowledge.size() << " temporalKnowledge(s) - " << listBehaviors.size() << " behavior(s) - " << listPlan.size() << " sharedplan(s) - " << listContextualKnowledge.size() << " contextualKnowledge(s) - " << listKnownInteraction.size() << " knownInteraction(s)." << endl << endl;;

    bOutput.addString("knowledge added");

    return bOutput;
} 

/*
* Search for the spatialKnowledge absolut and store them in the OPC
*
*/
Bottle abmReasoning::updateKnownLocations()
{
    Bottle bOutput;
    if ((realOPC->isConnected() || mentalOPC->isConnected()) && bPopulateOPC)
    {
        int iAdded = 0;
        for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end() ; it++)
        {

            // Is the spatialKnowledge absolut (put) or relative (push)
            if (it->isAbsolut && it->vX.size() >= abmReasoningFunction::threshold_determine_Location && it->sArgument != "near" && it->sName != "hanoi")
            {
                vector<double> CovMat = abmReasoningFunction::getCovMatrix(it->vX, it->vY);
                double a = CovMat[0],
                    b = CovMat[1],
                    c = CovMat[2],
                    d = CovMat[3];

                double VP1, VP2;
                pair<double , double > Vect1, Vect2;

                if (a < d)
                {
                    VP1 = .5 * (a+d+sqrt((a-d)*(a-d) + 4*b*c));
                    VP2 = .5 * (a+d-sqrt((a-d)*(a-d) + 4*b*c));
                }
                else
                {
                    VP1 = .5 * (a+d-sqrt((a-d)*(a-d) + 4*b*c));
                    VP2 = .5 * (a+d+sqrt((a-d)*(a-d) + 4*b*c));
                }

                Vect1.first = c;
                Vect1.second = VP1 - a;

                Vect2.first = c ; 
                Vect2.second = VP2 - a;


                vector<double>  vData = it->determineAbsolut();
                string sLoc = "loc_";
                sLoc+=it->sArgument;
                string large = "large_";
                large+=it->sArgument;

                int colorR = abmReasoningFunction::color_loc_R,
                    colorG = abmReasoningFunction::color_loc_G,
                    colorB = abmReasoningFunction::color_loc_B;

                if (it->sName == "move")
                {
                    colorR = 238;
                    colorG = 221;
                    colorB = 130;
                }

                if (realOPC->isConnected())
                {
                    Object* LOCATION = realOPC->addObject(sLoc);
                    LOCATION->m_ego_position[0] = vData[0] ;
                    LOCATION->m_ego_position[1] = vData[1] ;
                    LOCATION->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION->m_dimensions[0] = sqrt(VP2)*abmReasoningFunction::factor_location;
                    LOCATION->m_dimensions[1] = sqrt(VP1)*abmReasoningFunction::factor_location;
                    LOCATION->m_dimensions[2] = abmReasoningFunction::size_location*2 ;
                    LOCATION->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180./PI;   
                    LOCATION->m_present = 1;
                    LOCATION->m_color[0] = colorR / 2; 
                    LOCATION->m_color[1] = colorG / 2; 
                    LOCATION->m_color[2] = colorB / 2; 

                    Object* LOCATION_LARGE = realOPC->addObject(large);
                    LOCATION_LARGE->m_ego_position[0] = vData[0] ;
                    LOCATION_LARGE->m_ego_position[1] = vData[1] ;
                    LOCATION_LARGE->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION_LARGE->m_dimensions[0] = sqrt(VP2)*2*abmReasoningFunction::factor_location;
                    LOCATION_LARGE->m_dimensions[1] = sqrt(VP1)*2*abmReasoningFunction::factor_location;
                    LOCATION_LARGE->m_dimensions[2] = abmReasoningFunction::size_location ;
                    LOCATION_LARGE->m_present = 1;
                    LOCATION_LARGE->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180./PI; 
                    LOCATION_LARGE->m_color[0] = colorR ; 
                    LOCATION_LARGE->m_color[1] = colorG ; 
                    LOCATION_LARGE->m_color[2] = colorB ; 

                    iAdded++;
                    LOCATION->m_present = 1;
                    LOCATION_LARGE->m_present = 1;
                    realOPC->commit();
                }


                if (mentalOPC->isConnected())
                {
                    Object* LOCATION = mentalOPC->addObject(sLoc);
                    LOCATION->m_ego_position[0] = vData[0] ;
                    LOCATION->m_ego_position[1] = vData[1] ;
                    LOCATION->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION->m_dimensions[0] = sqrt(VP2)*abmReasoningFunction::factor_location;
                    LOCATION->m_dimensions[1] = sqrt(VP1)*abmReasoningFunction::factor_location;
                    LOCATION->m_dimensions[2] = abmReasoningFunction::size_location*2 ;
                    LOCATION->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180./PI;   
                    LOCATION->m_present = 1;
                    LOCATION->m_color[0] = colorR / 2; 
                    LOCATION->m_color[1] = colorG / 2; 
                    LOCATION->m_color[2] = colorB / 2; 

                    Object* LOCATION_LARGE = mentalOPC->addObject(large);
                    LOCATION_LARGE->m_ego_position[0] = vData[0] ;
                    LOCATION_LARGE->m_ego_position[1] = vData[1] ;
                    LOCATION_LARGE->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION_LARGE->m_dimensions[0] = sqrt(VP2)*2*abmReasoningFunction::factor_location;
                    LOCATION_LARGE->m_dimensions[1] = sqrt(VP1)*2*abmReasoningFunction::factor_location;
                    LOCATION_LARGE->m_dimensions[2] = abmReasoningFunction::size_location ;
                    LOCATION_LARGE->m_present = 1;
                    LOCATION_LARGE->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180./PI; 
                    LOCATION_LARGE->m_color[0] = colorR ; 
                    LOCATION_LARGE->m_color[1] = colorG ; 
                    LOCATION_LARGE->m_color[2] = colorB ; 

                    iAdded++;
                    LOCATION->m_present = 1;
                    LOCATION_LARGE->m_present = 1;
                }
            }
        }

        ostringstream osReturn;
        osReturn << iAdded << " locations set in the OPC." ;
        mentalOPC->commit();
        realOPC->commit();

        bOutput.addString(osReturn.str().c_str());
        return bOutput;
    }

    std::cout << "update locations : OPC not connected" << endl;
    bOutput.addString("update locations : OPC not connected");

    return bOutput;
}


Bottle abmReasoning::updateLocation(string sLocation)
{
    Bottle bOutput;
    if ((realOPC->isConnected() || mentalOPC->isConnected()) && bPopulateOPC)
    {
        int iAdded = 0;
        for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end() ; it++)
        {
            // Is the spatialKnowledge absolut (put) or relative (push)
            if (it->isAbsolut && it->sArgument == sLocation  && it->vX.size() >= abmReasoningFunction::threshold_determine_Location && it->sArgument != "near" && it->sName != "hanoi")
            {
                vector<double> CovMat = abmReasoningFunction::getCovMatrix(it->vX, it->vY);
                double a = CovMat[0],
                    b = CovMat[1],
                    c = CovMat[2],
                    d = CovMat[3];


                double VP1, VP2;

                if (a < d)
                {
                    VP1 = .5 * (a+d+sqrt((a-d)*(a-d) + 4*b*c));
                    VP2 = .5 * (a+d-sqrt((a-d)*(a-d) + 4*b*c));
                }
                else
                {
                    VP1 = .5 * (a+d-sqrt((a-d)*(a-d) + 4*b*c));
                    VP2 = .5 * (a+d+sqrt((a-d)*(a-d) + 4*b*c));
                }

                pair<double , double > Vect1 (c, VP1-a),
                    Vect2 (c , VP2-a);

                vector<double>  vData = it->determineAbsolut();
                string sLoc = "loc_";
                sLoc+=it->sArgument;
                string large = "large_";
                large+=it->sArgument;

                int colorR = abmReasoningFunction::color_loc_R,
                    colorG = abmReasoningFunction::color_loc_G,
                    colorB = abmReasoningFunction::color_loc_B;

                if (it->sName == "move")
                {
                    colorR = 238;
                    colorG = 221;
                    colorB = 130;
                }

                if (realOPC->isConnected() )
                {
                    Object* LOCATION = realOPC->addObject(sLoc);
                    LOCATION->m_ego_position[0] = vData[0] ;
                    LOCATION->m_ego_position[1] = vData[1] ;
                    LOCATION->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION->m_dimensions[0] = sqrt(VP2)*abmReasoningFunction::factor_location;
                    LOCATION->m_dimensions[1] = sqrt(VP1)*abmReasoningFunction::factor_location;
                    LOCATION->m_dimensions[2] = 0.04 ;
                    LOCATION->m_present = 1;
                    LOCATION->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180./PI;   
                    LOCATION->m_color[0] = colorR /2 ; 
                    LOCATION->m_color[1] = colorG /2 ; 
                    LOCATION->m_color[2] = colorB /2 ; 

                    Object* LOCATION_LARGE = realOPC->addObject(large);
                    LOCATION_LARGE->m_ego_position[0] = vData[0] ;
                    LOCATION_LARGE->m_ego_position[1] = vData[1] ;
                    LOCATION_LARGE->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION_LARGE->m_dimensions[0] = sqrt(VP2)*2*abmReasoningFunction::factor_location;
                    LOCATION_LARGE->m_dimensions[1] = sqrt(VP1)*2*abmReasoningFunction::factor_location;
                    LOCATION_LARGE->m_dimensions[2] = 0.02 ;
                    LOCATION_LARGE->m_present = 1;
                    LOCATION_LARGE->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180./PI; 
                    LOCATION_LARGE->m_color[0] = colorR ; 
                    LOCATION_LARGE->m_color[1] = colorG ; 
                    LOCATION_LARGE->m_color[2] = colorB ; 

                    ostringstream osReturn;
                    osReturn << iAdded << " locations set in the OPC." ;
                    realOPC->commit();
                    bOutput.addString(osReturn.str().c_str());
                }

                if (mentalOPC->isConnected() )
                {
                    Object* LOCATION = mentalOPC->addObject(sLoc);
                    LOCATION->m_ego_position[0] = vData[0] ;
                    LOCATION->m_ego_position[1] = vData[1] ;
                    LOCATION->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION->m_dimensions[0] = sqrt(VP2)*abmReasoningFunction::factor_location;
                    LOCATION->m_dimensions[1] = sqrt(VP1)*abmReasoningFunction::factor_location;
                    LOCATION->m_dimensions[2] = 0.04 ;
                    LOCATION->m_present = 1;
                    LOCATION->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180./PI;   
                    LOCATION->m_color[0] = abmReasoningFunction::color_loc_R /2 ; 
                    LOCATION->m_color[1] = abmReasoningFunction::color_loc_G /2 ; 
                    LOCATION->m_color[2] = abmReasoningFunction::color_loc_B /2 ; 

                    Object* LOCATION_LARGE = mentalOPC->addObject(large);
                    LOCATION_LARGE->m_ego_position[0] = vData[0] ;
                    LOCATION_LARGE->m_ego_position[1] = vData[1] ;
                    LOCATION_LARGE->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION_LARGE->m_dimensions[0] = sqrt(VP2)*2*abmReasoningFunction::factor_location;
                    LOCATION_LARGE->m_dimensions[1] = sqrt(VP1)*2*abmReasoningFunction::factor_location;
                    LOCATION_LARGE->m_dimensions[2] = 0.02 ;
                    LOCATION_LARGE->m_present = 1;
                    LOCATION_LARGE->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180./PI; 
                    LOCATION_LARGE->m_color[0] = colorR ; 
                    LOCATION_LARGE->m_color[1] = colorG ; 
                    LOCATION_LARGE->m_color[2] = colorB ; 

                    ostringstream osReturn;
                    osReturn << iAdded << " locations set in the OPC." ;
                    mentalOPC->commit();
                    bOutput.addString(osReturn.str().c_str());
                }

                return bOutput;

            }
        }

        //std::cout << "location unknown" << endl;
        bOutput.addString("location unknown");
        return bOutput;
    }
    //std::cout << "update locations : OPC not connected" << endl;
    bOutput.addString("update locations : OPC not connected");

    return bOutput;
}

/*
* Delete from the OPC the location (presence = 0)
*
*/
Bottle abmReasoning::DeleteKnownLocations()
{
    Bottle bOutput;
    if (realOPC->isConnected() || mentalOPC->isConnected())
    {
        for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end() ; it++)
        {
            // Is the spatialKnowledge absolut (put) or relative (push)
            if (it->isAbsolut)
            {
                vector<double>  vData = it->determineAbsolut();
                string sLoc = "loc_";
                sLoc += it->sArgument;
                string large = "large_",
                    test = "test_";
                large+=it->sArgument;


                if (realOPC->isConnected() )
                {
                Object* LOCATION = realOPC->addObject(sLoc);
                LOCATION->m_present = 0;

                Object* LOCATION_LARGE = realOPC->addObject(large);
                LOCATION_LARGE->m_present = 0;
                }

                if (mentalOPC->isConnected() )
                {
                Object* LOCATION = mentalOPC->addObject(sLoc);
                LOCATION->m_present = 0;

                Object* LOCATION_LARGE = mentalOPC->addObject(large);
                LOCATION_LARGE->m_present = 0;
                }

            }
        }

        realOPC->commit();
        mentalOPC->commit();

        return bOutput;
    }

    //  std::cout << "update locations : OPC not connected" << endl;
    bOutput.addString("update locations : OPC not connected");

    return bOutput;
}

/*
* Add some information to a contextualKnowledge existing, or create a new one
* bInput : name , argument , before (int) , after (int)
*/
Bottle abmReasoning::addContextualKnowledge(Bottle bInput)
{
    Bottle bOutput;

    string sName = bInput.get(0).toString().c_str(),
        sArgument = bInput.get(1).toString().c_str(),
        sXY = bInput.get(4).toString().c_str(),
        sEND = bInput.get(5).toString().c_str();
    int Before = bInput.get(2).asInt(),
        After = bInput.get(3).asInt();

    string sDependance = bInput.get(4).toString().c_str();
    pair<bool, bool> pPresence((Before == 1),(After == 1)) ;

    for (vector<contextualKnowledge>::iterator it_CK = listContextualKnowledge.begin() ; it_CK != listContextualKnowledge.end() ; it_CK++)
    {
        if (it_CK->sName == sName.c_str() && it_CK->sArgument == sArgument.c_str() && it_CK->sDependance == sDependance.c_str())
        {
            it_CK->vObjectPresent.push_back(pPresence);
            it_CK->updatePresence();
            it_CK->mAgentRelated[bInput.get(5).toString().c_str()]+=1;
            //          std::cout << "ContextualKnowledge already existing" << endl;
            bOutput.addString("ContextualKnowledge already existing");
            return bOutput;
        }
    }

    contextualKnowledge newCK;
    newCK.sArgument = sArgument;
    newCK.sName = sName;
    newCK.sDependance = sDependance;
    newCK.vObjectPresent.push_back(pPresence);
    newCK.updatePresence();
    newCK.mAgentRelated[bInput.get(5).toString().c_str()] = 1;

    listContextualKnowledge.push_back(newCK);
    //  std::cout << "ContextualKnowledge created" << endl;
    bOutput.addString("ContextualKnowledge created");

    return bOutput;
}

/*
*   Save the knowledge in the semantical memory
*/
Bottle abmReasoning::saveKnowledge()
{
    Bottle bOutput =  Interlocutor.saveKnowledge(listSpatialKnowledge, listTimeKnowledge, listBehaviors, listPlan, listContextualKnowledge, listKnownInteraction);

    printSpatialKnowledge();
    
    return bOutput;
}


/*
*   askGrammar
*   get a bottle with
*   (speaker "speaker")
*   (addressee "addressee")
*   (subject "subject")
*   (agent "agent)
*
*  with one none, and return it filled
*/
Bottle abmReasoning::askGrammar(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.size() !=2)
    {
        std::cout << "Error in abmReasoning::askGrammar - wrong size of input." << endl;
        bOutput.addString("Error in abmReasoning::askGrammar - wrong size of input.");
        return bOutput;
    }

    if (!bInput.get(1).isList())
    {
        std::cout << "Error in abmReasoning::askGrammar - wrong format of input." << endl;
        bOutput.addString("Error in abmReasoning::askGrammar - wrong formats of input.");
        return bOutput;
    }

    Bottle bContent = *bInput.get(1).asList();

    if (bContent.size() != 4)
    {       
        std::cout << "Error in abmReasoning::askGrammar - wrong size of arguments." << endl;
        bOutput.addString("Error in abmReasoning::askGrammar - wrong size of arguments.");
        return bOutput;
    }


    string X,
        Y,
        Z,
        P;

    bool    fSpeaker = false,
        fAddressee = false,
        fSubject = false,
        fAgent = false;

    X   = bContent.check(abmReasoningFunction::TAG_SPEAKER.c_str(),Value(abmReasoningFunction::TAG_NONE)).asString().c_str();
    Y   = bContent.check(abmReasoningFunction::TAG_ADRESSEE.c_str(),Value(abmReasoningFunction::TAG_NONE)).asString().c_str();
    P   = bContent.check(abmReasoningFunction::TAG_SUBJECT.c_str(),Value(abmReasoningFunction::TAG_NONE)).toString().c_str();
    Z       = bContent.check(abmReasoningFunction::TAG_AGENT.c_str(),Value(abmReasoningFunction::TAG_NONE)).toString().c_str();

    if (X != abmReasoningFunction::TAG_NONE)
        fSpeaker = true;
    if (Y != abmReasoningFunction::TAG_NONE)
        fAddressee = true;
    if (P != abmReasoningFunction::TAG_NONE)
        fSubject = true;
    if (Z != abmReasoningFunction::TAG_NONE)
        fAgent = true;

    // if all arguments are here, or miss 2 or more arguments
    if((int)fSpeaker + (int)fAddressee + (int)fSubject +(int)fAgent == 4 || (int)fSpeaker + (int)fAddressee + (int)fSubject +(int)fAgent <3)
    {
        std::cout << "Error in abmReasoning::askGrammar - wrong size of arguments." << endl;
        bOutput.addString("Error in abmReasoning::askGrammar - wrong size of arguments.");
        return bOutput;
    }


    /*------------------------
    IF SUBJECT IS MISSING
    ------------------------*/
    if (!fSubject)
    {
        pair<string, double> pResult = listGrammarKnowledge.findSubject(X, Y, Z);
        X = pResult.first;
    }

    /*------------------------
    END IF SUBJECT IS MISSING
    ------------------------*/


    /*------------------------
    IF AGENT IS MISSING
    ------------------------*/
    if (!fAgent)
    {
        pair<string, double> pResult = listGrammarKnowledge.findAgent(X, Y, P);
        Z = pResult.first;
    }   

    /*------------------------
    END IF AGENT IS MISSING
    ------------------------*/


    /*------------------------
    IF ADDRESSEE IS MISSING
    ------------------------*/
    if (!fAddressee)
    {
        pair<string, double> pResult = listGrammarKnowledge.findAddressee(X, Z, P);
        Y = pResult.first;
    }   

    /*------------------------
    END IF ADDRESSEE IS MISSING
    ------------------------*/
    

    /*------------------------
    IF SPEAKER IS MISSING
    ------------------------*/
    if (!fSpeaker)
    {
        pair<string, double> pResult = listGrammarKnowledge.findSpeaker(Y, Z, P);
        X = pResult.first;
    }   

    /*------------------------
    END IF SPEAKER IS MISSING
    ------------------------*/


    Bottle bSpeaker,
        bAddressee,
        bSubject,
        bAgent;

    bSpeaker.addString(abmReasoningFunction::TAG_SPEAKER.c_str());
    bSpeaker.addString(X.c_str());

    bAddressee.addString(abmReasoningFunction::TAG_ADRESSEE.c_str());
    bAddressee.addString(Y.c_str());

    bSubject.addString(abmReasoningFunction::TAG_SUBJECT.c_str());
    bSubject.addString(P.c_str());

    bAgent.addString(abmReasoningFunction::TAG_AGENT.c_str());
    bAgent.addString(Z.c_str());

    bOutput.addList() = bSpeaker;
    bOutput.addList() = bAddressee;
    bOutput.addList() = bSubject;
    bOutput.addList() = bAgent;

    return bOutput;

}




Bottle abmReasoning::retroReasoning()
{
    return retroReasoning(0);
}



/////             RETRO REASONING

/**
* Retro Reasoning
* set retroactivly the semantical knowledge into the ABM
*/
Bottle abmReasoning::retroReasoning(int from)
{
    Bottle bOutput;
    if (!mentalOPC->isConnected())
    {
        std::cout << "Error in abmReasoning::retroReasoning | mentalOPC not connect, retroReasoning impossible." << endl;
        bOutput.addString("Error in abmReasoning::retroReasoning | mentalOPC not connect, retroReasoning impossible.");
        return bOutput;
    }

    std::cout << "Retro Reasoning engaged." << endl;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE instance > " << from << " ORDER BY instance";
    Bottle  bMessenger = requestFromStream(osRequest.str());
    int numberAction = bMessenger.size();

    std::cout << "found " << numberAction << " action(s)" << endl;

    for (int j = 0; j < numberAction ; j++)
    {

        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
//      std::cout << j+1 << "\t" << Id << "\t" ;

        Interlocutor.imagineOPC(Id);
        updateOpcObjectLocation(abmReasoningFunction::s_mentalOPC);
        Interlocutor.sendRelation(Id);
    }

    std::cout << "retroReasoning Done on " << numberAction << " situations." << endl;
    bOutput.addString("retroReasoning Done");

    return bOutput;
}

/**
* Third level of retro Reasoning
* Building of the map of properties
*/

Bottle abmReasoning::level3Reasoning()
{
    return level3Reasoning(0);
}

Bottle abmReasoning::level3Reasoning(int from)
{
    Bottle bOutput;

    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE activitytype = 'action' AND begin = true AND INSTANCE > " << from << " ORDER by INSTANCE";
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());
    int numberAction = bMessenger.size();

    vector<int> vError;
    std::cout << "found " << numberAction << " action(s)" << endl;

    for (int j = 0; j < numberAction ; j++)
    {   // begin for each action
        std::cout << j+1 << ".." ;
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());

        Bottle bAction = Interlocutor.askActionForLevel3Reasoning(Id);

        Bottle bName = *bAction.get(0).asList();
        Bottle bRelationsBefore = *bAction.get(1).asList();
        Bottle bRelationsAfter  = *bAction.get(2).asList();

        string sName = bName.get(0).toString().c_str(),
            sArgument = bName.get(1).toString().c_str();

        std::cout << "bName : \t" << bName.toString() << endl;
        std::cout << "bRelationsBefore : \t" << bRelationsBefore.toString() << endl;
        std::cout << "bRelationsAfter  : \t" << bRelationsAfter.toString() << endl;

        // Before
        vector<string>  vLocFocusBefore, vLocFocusAfter ;       // Vector with the location of the focus object
        vector< pair <string, string> > vObjectLocBefore,
            vObjectLocAfter;    // vector with the location of the other objects
        vector<pair <string, bool> > vObjectBoolBefore,
            vObjectBoolAfter;       // vector is the object was on the same location


        string sNull = "NULL";

        // begin if bRelationBefore is Null
        if (bRelationsBefore.toString().c_str() != sNull)
        {
            string sObject,
                sLocation;
            for (int i = 0 ; i < bRelationsBefore.size() ; i++)
            {   // begin decomposition bRelationBefore
                Bottle bTemp = *bRelationsBefore.get(i).asList();
                sObject = bTemp.get(0).toString().c_str();
                sLocation = bTemp.get(1).toString().c_str();
                if (sObject == sArgument)
                {
                    vLocFocusBefore.push_back(sLocation);
                }
                else
                {
                    pair<string, string> pTemp (sObject, sLocation);
                    vObjectLocBefore.push_back(pTemp);
                }
            }   // end decomposition bRelationBefore

            // for each object create a pair object-bool
            for (vector<pair<string, string> >::iterator pItLocation = vObjectLocBefore.begin() ; pItLocation != vObjectLocBefore.end() ; pItLocation++)
            {   // begin for each object create a pair object-bool
                bool bFound = false;
                for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolBefore.begin(); pItBool != vObjectBoolBefore.end() ; pItBool++)
                {
                    if (pItLocation->first == pItBool->first)
                    {
                        bFound = true;
                        pItBool->second = false;
                    }
                }
                if (!bFound)
                {   // begin IF the pair doesn't exist yet
                    pair <string, bool> pTemp(pItLocation->first, false);
                    vObjectBoolBefore.push_back(pTemp);
                }   // end IF
            }   // end for each object create a pair object-bool

            // begin for each object-location check
            for (vector<pair<string, string> >::iterator pItLocation = vObjectLocBefore.begin() ; pItLocation != vObjectLocBefore.end() ; pItLocation++)
            {   
                // for any location where the object of focus is
                for (vector<string>::iterator itLocFocus = vLocFocusBefore.begin() ; itLocFocus != vLocFocusBefore.end() ; itLocFocus++ )
                {
                    // is it the location is the same that the object of focus
                    if (pItLocation->second == *itLocFocus)
                    {
                        // search the appropriate pair
                        for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolBefore.begin(); pItBool != vObjectBoolBefore.end() ; pItBool++)
                        {
                            if (pItLocation->first == pItBool->first)
                            {
                                pItBool->second = true;
                            }
                        }
                    }
                }   // end FOR any location where the object of focus is
            }   // end FOR each object-location check
        }   // end IF bRelationBefore is Null


        // After

        if (bRelationsAfter.toString().c_str() != sNull)
        {   // begin IF bRelationAfter is Null
            string sObject,
                sLocation;
            
            for (int i = 0 ; i < bRelationsAfter.size() ; i++)
            {   // begin decomposition of bRelationAfter
                Bottle bTemp = *bRelationsAfter.get(i).asList();
                sObject = bTemp.get(0).toString().c_str();
                sLocation = bTemp.get(1).toString().c_str();
                if (sObject == sArgument)
                {
                    vLocFocusAfter.push_back(sLocation);
                }
                else
                {
                    pair<string, string> pTemp (sObject, sLocation);
                    vObjectLocAfter.push_back(pTemp);
                }
            }   // end FOR : decomposition of bRelationAfter

            // for each object create a pair object-bool
            for (vector<pair<string, string> >::iterator pItLocation = vObjectLocAfter.begin() ; pItLocation != vObjectLocAfter.end() ; pItLocation++)
            {
                bool bFound = false;
                for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolAfter.begin(); pItBool != vObjectBoolAfter.end() ; pItBool++)
                {
                    if (pItLocation->first == pItBool->first)
                    {
                        bFound = true;
                        pItBool->second = false;
                    }
                }
                if (!bFound)
                {   // begin IF the pair doesn't exist yet
                    pair <string, bool> pTemp(pItLocation->first, false);
                    vObjectBoolAfter.push_back(pTemp);
                }   // end IF

            }   // end FOR each object create a pair object-bool


            // FOR each object-location check
            for (vector<pair<string, string> >::iterator pItLocation = vObjectLocAfter.begin() ; pItLocation != vObjectLocAfter.end() ; pItLocation++)
            {
                for (vector<string>::iterator itLocFocus = vLocFocusAfter.begin() ; itLocFocus != vLocFocusAfter.end() ; itLocFocus++ )
                {   // begin FOR any location where the object of focus is

                    if (pItLocation->second == *itLocFocus)
                    {   // begin IF the location is the same that the object of focus

                        for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolAfter.begin(); pItBool != vObjectBoolAfter.end() ; pItBool++)
                        {   // FOR search the appropriate pair
                            if (pItLocation->first == pItBool->first)
                            {
                                pItBool->second = true;
                            }
                        }   // end FOR search the appropriate pair
                    }   // end IF the location is the same that the object of focus
                }   // end FOR any location where the object of focus is
            }   // end FOR each object-location check
        }   // end IF bRelationAfter is Null


        // create vector before / after
        vector<tuple<string, bool, bool> > vTObjects;

        // BEFORE
        for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolBefore.begin(); pItBool != vObjectBoolBefore.end() ; pItBool++)
        {
            tuple<string, bool, bool> tuTemp(pItBool->first, pItBool->second, false);
            vTObjects.push_back(tuTemp);
        }

        // AFTER
        for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolAfter.begin(); pItBool != vObjectBoolAfter.end() ; pItBool++)
        {   //  FOR each object present after
            bool bFound = false;

            for (vector<tuple<string, bool, bool> >::iterator itTu = vTObjects.begin() ; itTu != vTObjects.end() ; itTu ++)
            {   // FOR each object present before
                
                if (get<0>(*itTu) == pItBool->first)
                {   // IF match
                    bFound = true;
                    get<2>(*itTu) = pItBool->second;
                }   // end IF match
            }   // end FOR each object present before

            if (!bFound)
            {
                tuple<string, bool, bool> tuTemp(pItBool->first, false, pItBool->second);
                vTObjects.push_back(tuTemp);
            }
        }


        // updating the contextual knowledge
        for (vector<contextualKnowledge>::iterator  itCK =  listContextualKnowledge.begin() ; itCK !=listContextualKnowledge.end() ; itCK++)
        {   //begin FOR itCK : listContextualKnowledge

            if (itCK->sName == sName && itCK->sArgument == sArgument)
            {   // IF CK found

                for (vector<tuple<string, bool, bool> >::iterator itTu = vTObjects.begin() ; itTu != vTObjects.end() ; itTu ++)
                {   // FOR itTu: vOTobject each object present 
                    bool bFound = false;

                    for (map<string , vector <pair <bool, bool> > >::iterator itMap = itCK->mObjectFromTo.begin() ; itMap != itCK->mObjectFromTo.end() ; itMap++)
                    {   // FOR each object already in the CK
                        
                        if (itMap->first == get<0>(*itTu))
                        {   // IF the object of the CK is the same of the object present
                            bFound = true;
                            pair<bool, bool>    pTemp(get<1>(*itTu), get<2>(*itTu));
                            itMap->second.push_back(pTemp);
                        }   // endIF the object of the CK is the same of the object present
                    }   // end FOR each object already in the CK

                    if (!bFound)
                    {   // IF object wasn't present
                        pair<bool, bool>    pTemp(get<1>(*itTu), get<2>(*itTu));
                        itCK->mObjectFromTo[get<0>(*itTu)].push_back(pTemp);
                    }   // end IF object wasn't present
                }   // end FOR itTu: vOTobject each object present 
            }   // end IF CK found
        }   //end FOR itCK : listContextualKnowledge


        // check if Hanoi
        if (bAction.size() == 4)
        {   // begin IF has spatial1 and spatial 2
            Bottle bSpatial = *bAction.get(3).asList();
            
            std::cout << "bSpatial : " << bSpatial.toString() << endl;

            string sFrom, sTo; // locations from and to of the move
            // Get the argument FROM and TO
            for (int i = 0 ; i < 2 ; i++)
            {
                Bottle bTemp = *bSpatial.get(i).asList();
                if (bTemp.get(1).toString() == "spatial1")
                {   //  IF bottle is TO (spatial1)
                    sTo = bTemp.get(0).toString().c_str();
                }
                else
                {   // If bottle is FROM (spatial2)
                    sFrom = bTemp.get(0).toString().c_str();
                }
            }


            pair<bool, bool>    pbFROM(false, false),   // if the focus was at the location FROM before and after
                pbTO(false, false);                 // if the focus was at the location TO before and after


            // CHECK IF FOCUS AT LOCATION FROM AND TO
            
            //before
            for (vector<string>::iterator itLocFoc = vLocFocusBefore.begin() ;itLocFoc != vLocFocusBefore.end() ; itLocFoc++)
            {   // begin FOR each location where the focus was before 
                if (*itLocFoc == sFrom)
                {
                    pbFROM.first = true;
                }

                if (*itLocFoc == sTo)
                {
                    pbTO.first = true;
                }
            }   // end FOR each location where the focus was before


            //after
            for (vector<string>::iterator itLocFoc = vLocFocusAfter.begin() ;itLocFoc != vLocFocusAfter.end() ; itLocFoc++)
            {   // begin FOR each location where the focus was after
                if (*itLocFoc == sFrom)
                {
                    pbFROM.second = true;
                }

                if (*itLocFoc == sTo)
                {
                    pbTO.second = true;
                }
            }   // end FOR each location where the focus was after

            // updating the contextual knowledge
            for (vector<contextualKnowledge>::iterator  itCK =  listContextualKnowledge.begin() ; itCK !=listContextualKnowledge.end() ; itCK++)
            {   //begin FOR itCK : listContextualKnowledge

                if (itCK->sName == sName && itCK->sArgument == sArgument)
                {   // IF CK found

                    // update FROM
                    itCK->mIntersectLocation["from"].push_back(pbFROM);
                    // update TO
                    itCK->mIntersectLocation["to"].push_back(pbTO);

                }   // end IF CK found
            }   //end FOR itCK : listContextualKnowledge
        }   // end IF has spatial1 and spatial 2

    } // end for each action

    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin() ; itCK != listContextualKnowledge.end() ; itCK++)
    {
        itCK->updatePresence();
    }

    return bOutput;
}





/////             DISPLAY FUNCTION

/**
* Display the result of a sql Query on the console
* @param bInput : Bottle to display
*/
void abmReasoning::displayResult(Bottle bInput)
{
    //print the result of the query, decompose elements/column
    std::cout << "query answer : " << bInput.toString() << endl ;
    for(int i = 0; i < bInput.size(); i++){
        std::cout << "***************************************" << endl ;
        //std::cout << "opc instance  " << bOpcId.get(i).asList()->get(0).asString() << " : " << bOutput.get(i).toString() << endl;
        // std::cout << "opc instance  " << bOpcId.get(i).asList()->get(0).asString() << " : " << endl;
        if (bInput.get(i).toString() != "NULL") {
            for(int j = 0; j < bInput.get(i).asList()->size(); j++){
                std::cout << "---" << opcNameTable[j] << " : " << bInput.get(i).asList()->get(j).toString() << endl;
            }
        }
    }
}

/*
* Internal function
* display the content of listSharedPlan
*/
void abmReasoning::displaySharedPlan()
{

    for (vector<sharedPlan>::iterator it_SP = listSharedPlan.begin() ; it_SP != listSharedPlan.end() ; it_SP++)
    {
        std::cout << "Shared plan : name =  " << it_SP->sName << " and manner = " << it_SP->sManner << endl;

        for (vector < pair <plan , int > >::iterator it_Plan = it_SP->listPlanPossible.begin() ; it_Plan != it_SP->listPlanPossible.end() ; it_Plan ++)
        {
            std::cout << "\tPlan : name = " << (it_Plan->first).sName << " and manner = " << it_Plan->first.sManner << ". score = " << it_Plan->second<< endl;
        }
    }
}

/*
*   Include in the OPC at a given time, the relation between the Object and the relations
*
*/
Bottle abmReasoning::updateOpcObjectLocation(string sOPCname)
{
    Bottle bOutput;
    bool bReal = sOPCname == "OPC";

    if (bReal)
        realOPC->checkout();
    else
        mentalOPC->checkout();

    // Set the Bottles for queryOPC to the 

    Bottle isRtobject, condition, present;
    isRtobject.addString(EFAA_OPC_ENTITY_TAG);
    isRtobject.addString("==");
    isRtobject.addString(EFAA_OPC_ENTITY_RTOBJECT);

    Bottle isPresent;
    isPresent.addString(EFAA_OPC_OBJECT_PRESENT_TAG);
    isPresent.addString("==");
    isPresent.addInt(1);

    condition.addList() = isRtobject;
    condition.addString("&&");
    condition.addList() = isPresent;

    list<Entity*> PresentEntities;
    if (!bReal)
        PresentEntities = (mentalOPC->Entities(condition));
    else
        PresentEntities = (realOPC->Entities(condition));
    
    // update known location
    mapLocation.clear();
    mapTemporalLocation.clear();

    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end() ; it++)
    {
        // Is the spatialKnowledge absolut (put) or relative (push)
        if (it->isAbsolut && it->vX.size() > 2 && it->sDependance == abmReasoningFunction::TAG_DB_NONE && it->sName != "hanoi")
        {
            pair <vector <double> , vector<double> > vData (it->vX, it->vY) ;
            mapLocation[it->sArgument] = vData;
        }

        if (it->isRelative && it->vX.size() > 2 && it->sDependance != abmReasoningFunction::TAG_DB_NONE)
        {
            for (list<Entity*>::iterator it_E = PresentEntities.begin() ; it_E != PresentEntities.end() ; it_E++)
            {
                if ((*it_E)->entity_type() == it->sDependance)
                {

                    pair<double, double>    pObjDep;        // coordinate of the dependent object

                    if (it->sDependance == EFAA_OPC_ENTITY_RTOBJECT)
                    {
                        RTObject *RTOTemp;
                        if (!bReal)
                            RTOTemp = (mentalOPC->addRTObject((*it_E)->name()));
                        else
                            RTOTemp = (realOPC->addRTObject((*it_E)->name()));

                        pObjDep = pair<double, double> (RTOTemp->m_ego_position[0],RTOTemp->m_ego_position[1]);

                    }

                    if (it->sDependance == EFAA_OPC_ENTITY_OBJECT)
                    {
                        Object *OTemp;
                        if (!bReal)
                            OTemp   = (mentalOPC->addObject((*it_E)->name()));
                        else
                            OTemp   = (realOPC->addObject((*it_E)->name()));

                        pObjDep = pair<double, double> (OTemp->m_ego_position[0],OTemp->m_ego_position[1]);

                    }

                    if (it->sDependance == EFAA_OPC_ENTITY_AGENT)
                    {
                        Agent *ATemp;
                        if (!bReal)
                            ATemp   = (mentalOPC->addAgent((*it_E)->name()));
                        else
                            ATemp   = (realOPC->addAgent((*it_E)->name()));

                        pObjDep = pair<double, double> (ATemp->m_ego_position[0],ATemp->m_ego_position[1]);

                    }

                    vector<double> vXTemp = it->vDX,
                        vYTemp = it->vDY;

                    for (unsigned int point = 0 ; point < vXTemp.size() ; point++)
                    {
                        vXTemp[point] = pObjDep.first - vXTemp[point];
                        vYTemp[point] = pObjDep.second - vYTemp[point];
                    }

                    tuple<string, vector<double> , vector<double> > mapTemp ((*it_E)->name(), vXTemp, vYTemp);
                    string sTemporalLocationName = it->sArgument;
                    sTemporalLocationName += "_" + (*it_E)->name();
                    mapTemporalLocation[sTemporalLocationName] = mapTemp;

                    
                vector<double> CovMat = abmReasoningFunction::getCovMatrix(vXTemp, vYTemp );
                
                }
            }
        }
    }

    // for each object present
    for (list<Entity*>::iterator it_E = PresentEntities.begin() ; it_E != PresentEntities.end() ; it_E++)
    {

        // get all object
        RTObject *RTTemp;
        if (bReal)
            RTTemp = realOPC->addRTObject((*it_E)->name());
        else
            RTTemp = mentalOPC->addRTObject((*it_E)->name());

        //detect if they are is a location

        for (map<string, pair<vector<double> ,vector<double> > >::iterator itMAP = mapLocation.begin() ; itMAP != mapLocation.end() ; itMAP++)
        {
            pair <double, double > pLoc (RTTemp->m_ego_position[0], RTTemp->m_ego_position[1]) ;

            if (abmReasoningFunction::getMahalaDist(itMAP->second.first, itMAP->second.second, pLoc) < abmReasoningFunction::threshold_is_at_location)
            {
                Adjective* presence;
                if (bReal)
                {   
                    presence = realOPC->addAdjective(itMAP->first.c_str());
                    presence->m_quality = (abmReasoningFunction::TAG_LOCATION);
                    Action* is = realOPC->addAction(abmReasoningFunction::TAG_IS_AT_LOC);
                    realOPC->commit();
                    realOPC->addRelation(*it_E, is, presence, 2*abmReasoningFunction::lifetime_relation);
                }
                else
                {       

                    presence = mentalOPC->addAdjective(itMAP->first.c_str());
                    presence->m_quality = (abmReasoningFunction::TAG_LOCATION);
                    Action* is = mentalOPC->addAction(abmReasoningFunction::TAG_IS_AT_LOC);
                    mentalOPC->commit();
                    mentalOPC->addRelation(*it_E, is, presence, 2*abmReasoningFunction::lifetime_relation);
                }

            }
        }

        // temporal location (dependance)
        for (map<string, tuple<string , vector<double> , vector<double> > >::iterator itMAP = mapTemporalLocation.begin() ; itMAP != mapTemporalLocation.end() ; itMAP++)
        {
            if (RTTemp->name() != get<0>(itMAP->second))
            {
                pair <double, double > pLoc (RTTemp->m_ego_position[0], RTTemp->m_ego_position[1]) ;

                if (abmReasoningFunction::getMahalaDist(get<1>(itMAP->second), get<2>(itMAP->second), pLoc) < abmReasoningFunction::threshold_is_at_temporal_location)
                {
                    Adjective* presence;
                    if (bReal)
                    {
                        presence = realOPC->addAdjective(itMAP->first.c_str());
                        presence->m_quality = (abmReasoningFunction::TAG_LOCATION);
                        Action* is = realOPC->addAction(abmReasoningFunction::TAG_IS_AT_LOC);
                        realOPC->commit();
                        realOPC->addRelation(*it_E, is, presence, 2*abmReasoningFunction::lifetime_relation);
                    }
                    else
                    {
                        presence = mentalOPC->addAdjective(itMAP->first.c_str());
                        presence->m_quality = (abmReasoningFunction::TAG_LOCATION);
                        Action* is = mentalOPC->addAction(abmReasoningFunction::TAG_IS_AT_LOC);
                        mentalOPC->commit();
                        mentalOPC->addRelation(*it_E, is, presence, 2*abmReasoningFunction::lifetime_relation);
                    }
                }
            }
        }
    }

    if (bReal)
    {
        realOPC->commit();
    }
    else
    {
        mentalOPC->commit();
    }

    bOutput.addString("Relation added in the OPC");
    bOutput.addString("Beliefs updated");

    return bOutput;
}


/*
* Return a list of information about an entity.
* input : name of the entity
*/
Bottle abmReasoning::getInfoAbout(string sName)
{
    Bottle bMessenger, bOutput;

    if (realOPC->getEntity(sName) != NULL)
    {
        // The object is not in the OPC, we have to search in the memory to get the type.
        ostringstream osEntity;
        osEntity << "SELECT instance, opcid FROM entity WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1" ;
        bMessenger = requestFromStream(osEntity.str());
        
        int Instance = atoi(bMessenger.get(0).asList()->get(0).toString().c_str()),
             Opcid = atoi(bMessenger.get(0).asList()->get(1).toString().c_str());

        osEntity.str("");
        osEntity << "SELECT subtype FROM contentopc WHERE instance = " << Instance << " AND opcid = " << Opcid ;
        bMessenger = requestFromStream(osEntity.str());

        string sSubType = bMessenger.get(0).asList()->get(0).toString();

        osEntity.str("");
        osEntity << "SELECT count(*) FROM contentarg WHERE argument = '" << sName << "'" ;
        bMessenger = requestFromStream(osEntity.str());

        int iNbInteraction = atoi(bMessenger.get(0).asList()->get(0).toString().c_str());
        std::cout << "I have interacted with this " << sSubType << " " << iNbInteraction/2 << " times ! " << endl;


    }



    return bOutput;

}
//

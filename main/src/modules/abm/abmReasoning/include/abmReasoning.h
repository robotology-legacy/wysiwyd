#include <interlocutor.h>



class abmReasoning : public yarp::os::RFModule
{
private:

    // Internal member, module-related :
    abmReasoningFunction *iFunction;            //  class of generic functions
    std::string  savefile,
        path;
    interlocutor        Interlocutor;           // interlocutor with the ABM
    wysiwyd::wrdac::OPCClient           *realOPC;                       // OPC
    wysiwyd::wrdac::OPCClient           *mentalOPC;

    wysiwyd::wrdac::ICubClient *iCub;

    yarp::os::Bottle              connectOPC(yarp::os::Bottle bInput);
    std::string              moduleName;

    yarp::os::Port handlerPort;     //a port to handle messages 
    yarp::os::Port senderPort;      //a port to send command to autobiographicalMemory (retrieve data from SQL db)

    // internal boolean
    bool                bDreaming;      // is the dreaming display is active for the KCF
    bool                bReady;         // is the system initialize
    bool                bPopulateOPC;   // send the knowledge to the OPC

    yarp::os::Bottle getActionFromPostCondition(std::pair<std::string, int>);

    // List of knowledge : 
    std::vector<spatialKnowledge>        listSpatialKnowledge;
    std::vector<timeKnowledge>           listTimeKnowledge;
    std::vector<sharedPlan>              listSharedPlan;
    std::vector<plan>                    listPlan;
    std::vector<behavior>                listBehaviors;
    std::vector<contextualKnowledge>     listContextualKnowledge;
    std::vector<knownInteraction>        listKnownInteraction;
    std::map<std::string, std::pair<std::vector<double>, std::vector<double> > >    mapLocation;                    //  known durable locations
    std::map<std::string, std::tuple<std::string, std::vector<double>, std::vector<double> > >   mapTemporalLocation;    // relative location
    grammarKnowledge                listGrammarKnowledge;           // list of known subject of sentence
    wordKnowledge                    WordKnowledge;

    //PDDL planner variable
    std::string      plannerPath;
    std::string      plannerExec;
    std::string      plannerOptDom;
    std::string      plannerOptProb;
    std::string      plannerOptOut;
    std::string      plannerOptNb;  //option : number of plan/solution before stop
    std::string      plannerOptCpu; //option : nb of second max taken by the planner

    std::string      pddlDomain;    //name of the domain pddl file
    std::string      pddlProblem; //name of the problem pddl file
    std::string      pddlOut;      //name of the solution plan file
    int         pddlNb;       //nb max of produced solution
    int         pddlCpu;      //nb of second max taken by the planner to find solution

    void        initialisePlanner(yarp::os::ResourceFinder &rf);      // initialise all variables of the planner
    int         pddlPlannerLauncher();
    yarp::os::Bottle      pddlPlannerSolParser();

    std::vector<std::vector<int> >    testListSharedPlan;
    std::vector<std::vector<int> >    testListSharedPlanPossible;
    std::vector<int>             testListCurrentAction;

    std::vector <plan>               vBuiltSharedPlan;
    std::vector <std::vector<plan> >      vAvailablePlans;
    std::vector < std::pair <int, int> >   vCurrentActions;

    int saveEndLastAction;


public:

    abmReasoning(yarp::os::ResourceFinder &rf);
    ~abmReasoning();

    std::deque<std::string> opcNameTable;

    //
    // module related functions
    double getPeriod();
    bool updateModule();    //  This is our main function. Will be called periodically every getPeriod() seconds.
    bool configure(yarp::os::ResourceFinder &rf);
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool interruptModule();
    bool close();   //  Close function, to perform cleanup.
    void changeDreaming();

    //method called via handlerPort
    yarp::os::Bottle request(yarp::os::Bottle request);
    yarp::os::Bottle requestFromStream(std::string sInput);
    yarp::os::Bottle save(yarp::os::Bottle bInput);
    yarp::os::Bottle sqlQueryTest();


    // query from outside
    yarp::os::Bottle findActivity(std::string actionName, std::string beginOrEnd, std::string select = "*");
    yarp::os::Bottle findActivityById(int id, std::string select = "*");
    // yarp::os::Bottle findOPC(std::string actionName, std::string beginOrEnd);
    yarp::os::Bottle queryBehavior(yarp::os::Bottle bInput);


    // Get ID (main.instance)

    std::vector<std::pair<int, int> > getIdFromActivity(std::string actionName, yarp::os::Bottle bRoleAndRoleValue);
    std::vector<std::pair<int, int> > getIdPairsFromBottle(yarp::os::Bottle idBottle);
    std::vector<std::pair<int, int> > getIDfromTime(yarp::os::Bottle bInput);

    yarp::os::Bottle getActionConsequence(std::pair<std::string, std::string> pNameArg);
    yarp::os::Bottle getActionConsequenceDrives(std::pair<std::string, std::string> pNameArg);
    yarp::os::Bottle testGetIdFromActivity();

    yarp::os::Bottle renameAction(yarp::os::Bottle bInput);


    // finding function
    yarp::os::Bottle findAllActions();
    yarp::os::Bottle findAllBehaviors();
    yarp::os::Bottle findAllComplex();
    yarp::os::Bottle findAllSentence();
    yarp::os::Bottle findAllSharedPlan();
    yarp::os::Bottle findAllInteractions();
    yarp::os::Bottle findAllActions(int from);
    yarp::os::Bottle findAllBehaviors(int from);
    yarp::os::Bottle findAllComplex(int from);
    yarp::os::Bottle findAllSentence(int from);
    yarp::os::Bottle findAllSharedPlan(int from);
    yarp::os::Bottle findAllInteractions(int from);

    //find possible plan into list action
    yarp::os::Bottle findPossibleSharedPlan(int, int);
    yarp::os::Bottle availableSharedPlan(int, int);
    plan actionsToPlan(int, int);
    std::vector<plan> checkPlan(plan, std::vector<plan>);


    // adding functions
    yarp::os::Bottle addLastActivity(yarp::os::Bottle bInput);
    yarp::os::Bottle addLastAction();
    yarp::os::Bottle addLastComplex();
    yarp::os::Bottle addLastBehavior();
    yarp::os::Bottle addLastSharedPlan();
    plan addLastPlan();

    // asking functions
    yarp::os::Bottle askLastActivity(yarp::os::Bottle bInput);

    // discriminate functions
    yarp::os::Bottle discriminateLastAction();
    yarp::os::Bottle discriminateAction(yarp::os::Bottle bInput);
    yarp::os::Bottle discriminateUnknownActions();


    // execution functions
    yarp::os::Bottle executeAction(yarp::os::Bottle bInput);
    yarp::os::Bottle executeComplex(yarp::os::Bottle bInput);
    yarp::os::Bottle executeActivity(yarp::os::Bottle bInput);
    yarp::os::Bottle executeSharedPlan(yarp::os::Bottle bInput);
    yarp::os::Bottle executeReasoning(yarp::os::Bottle bInput);


    // knowledge related functions
    void printSpatialKnowledge();
    void checkContextLocation();
    yarp::os::Bottle addSpatialKnowledge(spatialKnowledge skInput, bool b_update);
    yarp::os::Bottle addTimeKnowledge(yarp::os::Bottle bInput);
    plan addPlan(plan pInput);
    yarp::os::Bottle addSharedPlan(plan pInput);
    yarp::os::Bottle resetKnowledge();
    yarp::os::Bottle resetKnowledge(int from);
    yarp::os::Bottle reasoningFrom(int instance);
    yarp::os::Bottle getKnowledge();
    yarp::os::Bottle updateKnownLocations();
    yarp::os::Bottle DeleteKnownLocations();
    yarp::os::Bottle updateLocation(std::string sLocation);
    yarp::os::Bottle addContextualKnowledge(yarp::os::Bottle bInput);
    yarp::os::Bottle addBehavior(behavior beInput);
    yarp::os::Bottle saveKnowledge();
    yarp::os::Bottle askGrammar(yarp::os::Bottle bInput);
    yarp::os::Bottle askWordKnowledge(yarp::os::Bottle bInput);


    // memory related
    yarp::os::Bottle getInfoAbout(std::string sName);


    // RETRO REASONING
    yarp::os::Bottle  retroReasoning();
    yarp::os::Bottle  level3Reasoning();
    yarp::os::Bottle  retroReasoning(int from);
    yarp::os::Bottle  level3Reasoning(int from);

    //print functions
    yarp::os::Bottle printPDDLContextualKnowledgeDomain();
    yarp::os::Bottle printPDDLContextualKnowledgeProblem(yarp::os::Bottle bGoal);

    //pddl File functions
    void pddlSolFileName(int i, char* filename);
    void pddlSolDelete(unsigned int begin, unsigned int end);
    static char fixed_tolower(char c) { return tolower((unsigned char)c); }        //Avoiding ptr_fun bug

    int bestSol;

    // display functions
    void displayResult(yarp::os::Bottle bInput);
    void displaySharedPlan();

    // OPC related
    yarp::os::Bottle updateOpcObjectLocation(std::string sOPCname);

};

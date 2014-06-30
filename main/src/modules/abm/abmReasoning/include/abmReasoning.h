#include <interlocutor.h>
using namespace std;



class abmReasoning: public RFModule
{
private :

    // Internal member, module-related :
    abmReasoningFunction *iFunction;            //  class of generic functions
    string  savefile,
            path;
    interlocutor        Interlocutor;           // interlocutor with the ABM
    OPCClient           *realOPC;                       // OPC
    OPCClient           *mentalOPC;
    
    ICubClient *iCub;

    Bottle              connectOPC(Bottle bInput);
    string              moduleName;

    Port handlerPort;     //a port to handle messages 
    Port senderPort;      //a port to send command to autobiographicalMemory (retrieve data from SQL db)

    // internal boolean
    bool                bDreaming;      // is the dreaming display is active for the KCF
    bool                bReady;         // is the system initialize
    bool                bPopulateOPC;   // send the knowledge to the OPC

    Bottle getActionFromPostCondition(pair<string,int>) ;

    // List of knowledge : 
    vector<spatialKnowledge>        listSpatialKnowledge;
    vector<timeKnowledge>           listTimeKnowledge;
    vector<sharedPlan>              listSharedPlan;
    vector<plan>                    listPlan;
    vector<behavior>                listBehaviors;
    vector<contextualKnowledge>     listContextualKnowledge;
    vector<knownInteraction>        listKnownInteraction;
    map<string, pair<vector<double> , vector<double> > >    mapLocation;                    //  known durable locations
    map<string, tuple<string, vector<double> , vector<double> > >   mapTemporalLocation;    // relative location
    grammarKnowledge                listGrammarKnowledge;           // list of known subject of sentence
    

    //PDDL planner variable
    string      plannerPath ;
    string      plannerExec ;
    string      plannerOptDom ;
    string      plannerOptProb ;
    string      plannerOptOut ;
    string      plannerOptNb ;  //option : number of plan/solution before stop
    string      plannerOptCpu ; //option : nb of second max taken by the planner

    string      pddlDomain;    //name of the domain pddl file
    string      pddlProblem  ; //name of the problem pddl file
    string      pddlOut ;      //name of the solution plan file
    int         pddlNb ;       //nb max of produced solution
    int         pddlCpu ;      //nb of second max taken by the planner to find solution

    void        initialisePlanner(ResourceFinder &rf);      // initialise all variables of the planner
    int         pddlPlannerLauncher() ;
    Bottle      pddlPlannerSolParser() ;

    vector<vector<int> >    testListSharedPlan ;
    vector<vector<int> >    testListSharedPlanPossible ;
    vector<int>             testListCurrentAction ;

    vector <plan>               vBuiltSharedPlan;
    vector <vector<plan> >      vAvailablePlans ;
    vector < pair <int,int> >   vCurrentActions ;

    int saveEndLastAction;
    

public : 
    
    abmReasoning(ResourceFinder &rf);
    ~abmReasoning();

    deque<string> opcNameTable;

//
    // module related functions
    double getPeriod();
    bool updateModule();    //  This is our main function. Will be called periodically every getPeriod() seconds.
    bool configure(yarp::os::ResourceFinder &rf);
    bool respond(const Bottle& command, Bottle& reply);
    bool interruptModule();
    bool close();   //  Close function, to perform cleanup.
    void changeDreaming();

    //method called via handlerPort
    Bottle request(Bottle request);
    Bottle requestFromStream(string sInput);
    Bottle save(Bottle bInput);
    Bottle sqlQueryTest();

  
    // query from outside
    Bottle findActivity(string actionName, string beginOrEnd, string select = "*");
    Bottle findActivityById(int id, string select = "*");
    // Bottle findOPC(string actionName, string beginOrEnd);
    Bottle queryBehavior(Bottle bInput);


    // Get ID (main.instance)

    vector<pair<int, int> > getIdFromActivity(string actionName, Bottle bRoleAndRoleValue);
    vector<pair<int, int> > getIdPairsFromBottle(Bottle idBottle);
    vector<pair<int, int> > getIDfromTime(Bottle bInput);

    Bottle getActionConsequence(pair<string, string> pNameArg);
    Bottle getActionConsequenceDrives(pair<string, string> pNameArg);
    Bottle testGetIdFromActivity();

    Bottle renameAction(Bottle bInput);
    

    // finding function
    Bottle findAllActions();
    Bottle findAllBehaviors();
    Bottle findAllComplex();
    Bottle findAllSentence();
    Bottle findAllSharedPlan();
    Bottle findAllInteractions();
    Bottle findAllActions(int from);
    Bottle findAllBehaviors(int from);
    Bottle findAllComplex(int from);
    Bottle findAllSentence(int from);
    Bottle findAllSharedPlan(int from);
    Bottle findAllInteractions(int from);

    //find possible plan into list action
    Bottle findPossibleSharedPlan(int, int);
    Bottle availableSharedPlan(int, int);
    plan actionsToPlan(int, int) ;
    vector<plan> checkPlan(plan, vector<plan>);

    
    // adding functions
    Bottle addLastActivity(Bottle bInput);
    Bottle addLastAction();
    Bottle addLastComplex();
    Bottle addLastBehavior();
    Bottle addLastSharedPlan();
    plan addLastPlan();

    // asking functions
    Bottle askLastActivity(Bottle bInput);

    // discriminate functions
    Bottle discriminateLastAction();
    Bottle discriminateAction(Bottle bInput);
    Bottle discriminateUnknownActions();


    // execution functions
    Bottle executeAction(Bottle bInput);
    Bottle executeComplex(Bottle bInput);
    Bottle executeActivity(Bottle bInput);
    Bottle executeSharedPlan(Bottle bInput);
    Bottle executeReasoning(Bottle bInput);


    // knowledge related functions
    void printSpatialKnowledge();
    void checkContextLocation();
    Bottle addSpatialKnowledge(spatialKnowledge skInput, bool b_update);
    Bottle addTimeKnowledge(Bottle bInput);
    plan addPlan(plan pInput);
    Bottle addSharedPlan(plan pInput);
    Bottle resetKnowledge();
    Bottle resetKnowledge(int from);
    Bottle reasoningFrom(int instance);
    Bottle getKnowledge();
    Bottle updateKnownLocations();
    Bottle DeleteKnownLocations();
    Bottle updateLocation(string sLocation);
    Bottle addContextualKnowledge(Bottle bInput);
    Bottle addBehavior(behavior beInput);
    Bottle saveKnowledge();
    Bottle askGrammar(Bottle bInput);


    // memory related
    Bottle getInfoAbout(string sName);


    // RETRO REASONING
    Bottle  retroReasoning();
    Bottle  level3Reasoning();
    Bottle  retroReasoning(int from);
    Bottle  level3Reasoning(int from);

    //print functions
    Bottle printPDDLContextualKnowledgeDomain();
    Bottle printPDDLContextualKnowledgeProblem(Bottle bGoal);

    //pddl File functions
    void pddlSolFileName(int i, char* filename);
    void pddlSolDelete(unsigned int begin, unsigned int end);
    static char fixed_tolower(char c) { return tolower((unsigned char)c); }        //Avoiding ptr_fun bug

    int bestSol ;

    // display functions
    void displayResult(Bottle bInput);
    void displaySharedPlan();

    // OPC related
    Bottle updateOpcObjectLocation(string sOPCname);

};

#include <spatialKnowledge.h>
#include <timeKnowledge.h>
#include <sharedPlan.h>
#include <contextualKnowledge.h>
#include <adjKnowledge.h>
#include <behavior.h>
#include <knownInteraction.h>
#include <grammarKnowledge.h>
#include <wordKnowledge.h>
#include <deque>
#include <iostream>


/*
*   Interlocutor for the autobiographicMemory.
*   Used to communicate with the autobiographicalMemory and OPC
*/
class interlocutor
{
public:

    OPCClient *realOPC;
    OPCClient *mentalOPC;
    Bottle                      connectOPC();
    ICubClient *iCub;

    Port senderPort;                            //a port to send command to autobiographicalMemory (retrieve data from SQL db)
    Port port_to_OPCManager;                    // a port to send command to the OPCManager
    void    initialize();

    //method called via handlerPort
    Bottle request(Bottle request);
    Bottle requestFromStream(string sInput);

    // asking functions
    Bottle askLastAction();
    Bottle askActionFromId(int Id);
    Bottle askLastComplex();
    Bottle askComplexFromId(int Id);
    Bottle imagineOPC(int Id);
    Bottle askActionForLevel3Reasoning(int Id);
    plan askLastSharedPlan();
    plan askSharedPlanFromId(int Id);
    behavior askLastBehavior();
    behavior askBehaviorFromId(int Id);
    Bottle  updateBeliefs(bool bOPC);           // send a command to OPCManager to update the beliefs. true is for the real opc, false for the mental
    Bottle askSentenceFromId(int Id);


    // Functions to send knowledge to the semantic memory
    int                 sendSpatialKnowledge(vector<spatialKnowledge> listSK);
    int                 sendTemporalKnowledge(vector<timeKnowledge> listTk);
    int                 sendBehaviors(vector<behavior> listBehavior);
    int                 sendPlan(vector<plan> listPlan);
    int                 sendContextual(vector<contextualKnowledge> listCK);
    int                 sendInteractionKnowledge(vector<knownInteraction> listIn);

    // Retro reasoning
    Bottle              sendRelation(int Instance); // send the current relation of the mentalOPC to the ABM at the given instance


    // RETRO REASONING
    void    setMentalOPC(int instance);
    int     getNumberRelation(int instance);

    // Knowledge related function
    Bottle              saveKnowledge(vector<spatialKnowledge> listSK , vector<timeKnowledge> listTK, vector<behavior> listBehavior , vector<plan> listPlan , vector<contextualKnowledge> listCK , vector<knownInteraction> listInc);
    void                close();


};


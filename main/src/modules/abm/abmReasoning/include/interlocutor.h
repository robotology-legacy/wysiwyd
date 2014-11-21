#ifndef _INTERLOCUTOR_H_
#define _INTERLOCUTOR_H_

#include <spatialKnowledge.h>
#include <timeKnowledge.h>
#include <sharedPlan.h>
#include <contextualKnowledge.h>
#include <advKnowledge.h>
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

    wysiwyd::wrdac::OPCClient *realOPC;
    wysiwyd::wrdac::OPCClient *mentalOPC;
    yarp::os::Bottle                      connectOPC();
    wysiwyd::wrdac::ICubClient *iCub;

    yarp::os::Port senderPort;                            //a port to send command to autobiographicalMemory (retrieve data from SQL db)
    yarp::os::Port port_to_OPCManager;                    // a port to send command to the OPCManager
    void    initialize();

    //method called via handlerPort
    yarp::os::Bottle request(yarp::os::Bottle request);
    yarp::os::Bottle requestFromStream(std::string sInput);

    // asking functions
    yarp::os::Bottle askLastAction();
    yarp::os::Bottle askActionFromId(int Id);
    yarp::os::Bottle askLastComplex();
    yarp::os::Bottle askComplexFromId(int Id);
    yarp::os::Bottle imagineOPC(int Id);
    yarp::os::Bottle askActionForLevel3Reasoning(int Id);
    plan askLastSharedPlan();
    plan askSharedPlanFromId(int Id);
    behavior askLastBehavior();
    behavior askBehaviorFromId(int Id);
    yarp::os::Bottle  updateBeliefs(bool bOPC);           // send a command to OPCManager to update the beliefs. true is for the real opc, false for the mental
    yarp::os::Bottle askSentenceFromId(int Id);


    // Functions to send knowledge to the semantic memory
    int                 sendSpatialKnowledge(std::vector<spatialKnowledge> listSK);
    int                 sendTemporalKnowledge(std::vector<timeKnowledge> listTk);
    int                 sendBehaviors(std::vector<behavior> listBehavior);
    int                 sendPlan(std::vector<plan> listPlan);
    int                 sendContextual(std::vector<contextualKnowledge> listCK);
    int                 sendInteractionKnowledge(std::vector<knownInteraction> listIn);

    // Retro reasoning
    yarp::os::Bottle              sendRelation(int Instance); // send the current relation of the mentalOPC to the ABM at the given instance


    // RETRO REASONING
    void    setMentalOPC(int instance);
    int     getNumberRelation(int instance);

    // Knowledge related function
    yarp::os::Bottle              saveKnowledge(std::vector<spatialKnowledge> listSK , std::vector<timeKnowledge> listTK, std::vector<behavior> listBehavior , std::vector<plan> listPlan , std::vector<contextualKnowledge> listCK , std::vector<knownInteraction> listInc);
    void                close();


};

#endif
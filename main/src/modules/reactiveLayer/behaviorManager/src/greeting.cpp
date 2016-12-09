#include "greeting.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

void Greeting::configure() {
    // Is there a need for another module for greetings? This is simpler than proactive tagging, can be handled within here
    // => named external port None.
    from_sensation_port_name = "/opcSensation/greeting:o";
}

void Greeting::run(const Bottle &args) {
    yInfo() << "Greeting::run";
    // Bottle *objects = sensation.get(1).asList();
    // Might want to include gestures like looking at partner and waving a hand
    iCub->say("Hello, I'm iCub. What's your name?");
    int id = 0; //Should be a random
    yDebug() << "send rpc to salutation";
    Bottle *sensation = sensation_port_in.read();
    // Greet new partner
    Bottle cmd;
    Bottle rply;
    cmd.clear();
    cout << 1 << endl;
    cmd.addString("exploreUnknownPartner");
    cout << sensation->toString()<<endl;//asList()->toString() << endl;
    cout << 2 << endl;
    cmd.addString(sensation->get(id).asList()->get(2).asString());
    yInfo() << "Greeting unknown partner in the scene...";
    rpc_out_port.write(cmd, rply);
}

/* From reactiveLayerEFAA
bool ReactiveLayer::handleSalutation(bool& someoneIsPresent)
{
    someoneIsPresent = false;
    //Handle the salutation of newcomers
    list<Entity*> allAgents = iCub->opc->Entities(EFAA_OPC_ENTITY_TAG, "==", EFAA_OPC_ENTITY_AGENT);
    list<Relation> salutedAgents = iCub->opc->getRelations("saluted");
    list<Relation> identity = iCub->opc->getRelationsMatching("partner", "named");
    string identityName = "unknown";
    if (identity.size() > 0)
        identityName = identity.front().object();

    for (list<Entity*>::iterator currentAgentIt = allAgents.begin(); currentAgentIt != allAgents.end(); currentAgentIt++)
    {
        Agent* currentAgent = dynamic_cast<Agent*>(*currentAgentIt);


        if (currentAgent->name() != "icub" && currentAgent->m_present)
        {
            someoneIsPresent = true;
            string currentPartner = currentAgent->name();
            //cout<<"Testing salutation for "<<currentPartner<<" with name "<<identityName<<endl;

            bool saluted = false;
            for (list<Relation>::iterator it = salutedAgents.begin(); it != salutedAgents.end(); it++)
            {
                //cout<< it->toString()<<endl;
                if (it->subject() == identityName)
                {
                    //cout<<"Same agent detected... Reseting salutation lifetime"<<endl;
                    //This guy has already been saluted, we reset the lifetime
                    iCub->opc->setLifeTime(it->ID(), salutationLifetime);
                    saluted = true;
                }
            }

            if (!saluted)
            {
                iCub->look(currentPartner);
                iCub->say(salutationEffects["humanEnter"].getRandomSentence(), false);
                iCub->playChoregraphy("wave");
                if (identityName != "unknown")
                    iCub->say(identityName + "! nice to see you again!", false);
                iCub->opc->addRelation(Relation(identityName, "is", "saluted"), salutationLifetime);
                return true;
            }
        }
    }
    return false;
}
*/

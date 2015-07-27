/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau, Tobias Fischer, Maxime Petit
 * email:   greg.pointeau@gmail.com, t.fischer@imperial.ac.uk, m.petit@imperial.ac.uk
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "proactiveTagging.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

bool proactiveTagging::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("proactiveTagging")).asString().c_str();
    setName(moduleName.c_str());

    GrammarYesNo = rf.findFileByName(rf.check("GrammarYesNo", Value("nodeYesNo.xml")).toString());
    GrammarAskNameObject = rf.findFileByName(rf.check("GrammarAskNameObject", Value("GrammarAskNameObject.xml")).toString());
    GrammarAskNameAgent = rf.findFileByName(rf.check("GrammarAskNameAgent", Value("GrammarAskNameAgent.xml")).toString());
    GrammarAskNameBodypart = rf.findFileByName(rf.check("GrammarAskNameBodypart", Value("GrammarAskNameSelf.xml")).toString());

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "proactiveTagging", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    if (!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        Time::delay(1.0);
    }

    configureOPC(rf);

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    //--------------------------------------------- output port

    //out to BodySchema
    portToBodySchema.open(("/" + moduleName + "/toBodySchema:o").c_str()) ;
    string bodySchemaRpc = rf.check("bodySchemaRpc",Value("/bodySchema/rpc")).asString().c_str();

    if (!Network::connect(portToBodySchema.getName().c_str(),bodySchemaRpc.c_str())) {
        yWarning() << " BODY SCHEMA NOT CONNECTED : selfTagging will not work" ;
    }

    if (!iCub->getRecogClient())
    {
        yWarning() << "WARNING SPEECH RECOGNIZER NOT CONNECTED" ;
    }
    if (!iCub->getABMClient())
    {
       yWarning() << "WARNING ABM NOT CONNECTED" ;
    }

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    iCub->getSpeechClient()->TTS("proactive tagging is ready", false);

    return true;
}


bool proactiveTagging::close() {
    iCub->close();
    delete iCub;

    rpcPort.interrupt();
    rpcPort.close();

    return true;
}


bool proactiveTagging::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n"
        "exploreUnknowneEntity entity_type entity_name \n" +
        "exploreEntityByName entity_name \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString() == "exploreUnknownEntity") {
        yInfo() << " exploreUnknownEntity";
        reply = exploreUnknownEntity(command);
    }
    else if (command.get(0).asString() == "exploreEntityByName") {
        yInfo() << " exploreEntityByName";
        reply = exploreEntityByName(command);
    }
    else {
        cout << helpMessage;
        reply.addString("ok");
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool proactiveTagging::updateModule() {
    return true;
}

void proactiveTagging::checkRelations()
{
    iCub->opc->update();
    iCub->opc->checkout();

    string none = "none";

    list<Relation> lRelations = iCub->opc->getRelations();

    //FOR EACH RELATION
    for (list<Relation>::iterator itRel = lRelations.begin(); itRel != lRelations.end(); itRel++)
    {
        cout << itRel->toString() << endl;

        if (itRel->complement_manner() == none)
        {
            cout << "should ask the manner complement" << endl;
            string sManner = askManner(itRel->subject(), itRel->verb(), itRel->object());

            iCub->opc->removeRelation(*itRel);

            cout << "relation is now: " << itRel->toString() << endl;
            itRel->m_complement_manner = sManner;
            iCub->opc->addEntity<Adjective>(sManner);
            iCub->opc->addRelation(*itRel);
            iCub->opc->commit();
        }
        cout << itRel->complement_manner() << endl;
        cout << itRel->complement_place() << endl;
        cout << itRel->complement_time() << endl;
    }

}



string proactiveTagging::askManner(string agent, string verb, string object)
{

    ostringstream osSentenceToSay, osError;
    osSentenceToSay << "Do you know how " << agent << " " << verb << " the " << object << " ?";

    iCub->say(osSentenceToSay.str());


    Bottle bOutput;

    //bool fGetaReply = false;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bMessenger; //to be send TO speech recog

    //bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarAskManner), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        cout << bRecognized.get(1).toString() << endl;
        return "none";
    }

    bAnswer = *bRecognized.get(1).asList();
    cout << "bAnswer is : " << bAnswer.toString() << endl;

    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)
    bSemantic = *bAnswer.get(1).asList();


    if (bAnswer.get(0).asString() == "stop")
    {
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
        return "stop";
    }


    string sManner = bSemantic.check("manner", Value("none")).asString();
    cout << "answer is : " << sManner << endl;

    return sManner;
    //TODO dialogue interaction to ask for complement of manner
}

/*
* Recognize the name of an unknown entity
* Recognize through speech the name of an unknwon entity.
* @return (error errorDescription) or (sName)
*/
Bottle proactiveTagging::recogName(string entityType)
{
    Bottle bOutput;

    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
    bAnswer, //response from speech recog without transfer information, including raw sentence
    bSemantic; // semantic information of the content of the recognition

    //Load the Speech Recognition with grammar according to entityType
    if(entityType == "agent"){
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameAgent), 20);
    } else if (entityType == "object" || entityType == "rtobject"){
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameObject), 20);
    } else if (entityType == "bodypart"){
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameBodypart), 20);
    } else {
        yError() << " error in proactiveTagging::recogName | for " << entityType << " | Entity Type not managed" ;
        bOutput.addString("error");
        bOutput.addString("Entity Type not managed");
        return bOutput;
    }

    if (bRecognized.get(0).asInt() == 0)
    {
        yError() << " error in proactiveTagging::askName | for " << entityType << " | Error in speechRecog" ;
        bOutput.addString("error");
        bOutput.addString("error in speechRecog");
        return bOutput;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

    if (bAnswer.get(0).asString() == "stop")
    {
        yError() << " in proactiveTagging::askName | for " << entityType << " | stop called";
        bOutput.addString("error");
        bOutput.addString("stop called");
        return bOutput;
    }

    bSemantic = *bAnswer.get(1).asList();
    string sName;
    if(entityType == "agent") {
        sName = bSemantic.check("agent", Value("unknown")).asString();
    } else if(entityType == "object" || entityType == "rtobject") {
        sName = bSemantic.check("object", Value("unknown")).asString();
    } else if(entityType == "bodypart") {
        sName = bSemantic.check("fingerName", Value("unknown")).asString();
    } else {
        yError("recogName ERROR entitytype not known!");
    }

    bOutput.addString(sName);

    return bOutput;
}

/*
* Explore an unknown entity by asking the name
* input: exploreUnknownEntity entityType entityName (eg: exploreUnknownEntity agent unknown_25)
* ask through speech the name of an unknwon entity
*/
Bottle proactiveTagging::exploreUnknownEntity(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 3)
    {
        yInfo() << " proactiveTagging::exploreEntity | Problem in input size.";
        bOutput.addString("Problem in input size");
        return bOutput;
    }

    string currentEntityType = bInput.get(1).toString();
    string sNameTarget = bInput.get(2).toString();

    yInfo() << " EntityType : " << currentEntityType;
    double timeDelay = 1.;


    //Ask question for the human, or ask to pay attention (if action to focus attention after)
    string sQuestion ;
    if (currentEntityType == "agent") {
        sQuestion = " Hello, I don't know you. Who are you ?";
    } else if (currentEntityType == "object" || currentEntityType == "rtobject") {
        sQuestion = " Hum, what is this object ?" ;
    } else if (currentEntityType == "bodypart") {
        sQuestion = " Watch please, I will move a part of my body" ;
        yInfo() << " sQuestion: " << sQuestion;
    } else {
        yError() << " error in proactiveTagging::exploreUnknownEntity | for " << currentEntityType << " | Entity Type not managed" ;
        bOutput.addString("error");
        bOutput.addString("Entity Type not managed");
        return bOutput;
    }

    //TODO : choose between say and TTS. say put stuff in ABM, TTS?
    yInfo() << sQuestion;
    //iCub->getSpeechClient()->TTS(sQuestion, false);
    iCub->say(sQuestion);

    //Act to determine the entity to be named, according to entityType (e.g. bodypart is sending a command to move the joint, ...)
    if(currentEntityType == "bodypart") {
        Bodypart* BPtemp = dynamic_cast<Bodypart*>(iCub->opc->getEntity(sNameTarget));
        yInfo() << "Cast okay";
        int joint = BPtemp->m_joint_number;
        string sBodyPartType = BPtemp->m_part;
        //send rpc command to bodySchema to move the corresponding part
        yInfo() << "Start bodySchema";
        Bottle bReplyFromBodySchema = moveJoint(joint, sBodyPartType);

        if(bReplyFromBodySchema.get(0).asString() == "nack"){
            yError() << " error in proactiveTagging::exploreUnknownEntity | for " << currentEntityType << " | Joint has not moved" ;
            bOutput.addString("error");
            bOutput.addString("Joint has not moved");
            return bOutput;
        }

        sQuestion = " How do you call this part of my body?" ;
        yInfo() << sQuestion;
        //iCub->getSpeechClient()->TTS(sQuestion, false);
        iCub->say(sQuestion);
    }

    Bottle bName = recogName(currentEntityType) ;
    string sName;

    //if error, bName = (error errorDescription) -> return it
    if(bName.get(0).asString() == "error"){
        return bName ;
    } else {
        sName = bName.get(0).asString();
    }

    string sReply ;
    Entity* e = iCub->opc->getEntity(sNameTarget);
    e->changeName(sName);
    iCub->opc->commit(e);

    if (currentEntityType == "agent") {
        sReply = " Well, Nice to meet you " + sName;
    } else if (currentEntityType == "object") {
        sReply = " I get it, this is a " + sName;
    } else if (currentEntityType == "rtobject") {
        sReply = " So this is a " + sName;
    } else if (currentEntityType == "bodypart") {
        sReply = " Nice, I know that I have a " + sName + " finger.";
    }//go out before if not one of those entityType

    yInfo() << sReply;
    iCub->say(sReply);
    Time::delay(timeDelay);

    iCub->opc->update();

    bOutput.addString("success");
    bOutput.addString(currentEntityType);

    return bOutput;
}

/*
* Search for the entity corresponding to a certain name in all the unknown entities
* return a bottle of 2 elements.
* First element is: error - warning - success
* Second element is: information about the action
*/
Bottle proactiveTagging::exploreEntityByName(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 2)
    {
        yInfo() << " proactiveTagging::exploreEntityByName | Problem in input size.";
        bOutput.addString("Problem in input size");
        return bOutput;
    }

    string sNameTarget = bInput.get(1).toString();
    yInfo() << " Entity to find: " << sNameTarget;

    // check if the entity is already present in the OPC
    if (iCub->opc->isConnected())
    {
        iCub->opc->checkout();
        list<Entity*> lEntities = iCub->opc->EntitiesCacheCopy();

        for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
        {
            if ((*itEnt)->name() == sNameTarget)
            {
                yInfo() << " Entity " << sNameTarget << " is already known.";
                bOutput.addString("warning");
                bOutput.addString("entity already exists");
                return bOutput;
            }
        }
    }
    else
    {
        yWarning() << " in proactiveTagging::exploreEntityByName | OPC not Connected";
        bOutput.addString("error");
        bOutput.addString("OPC not connected");

        return bOutput;
    }


    // if there is several objects unknown (or at least one)
    string sSentence = "I don't known which of these objects is a " + sNameTarget;
    iCub->say(sSentence);
    yInfo() << " " << sSentence;

    sSentence = "Can you show me the " + sNameTarget;
    iCub->say(sSentence);
    yInfo() << " " << sSentence;

    bool bFound = false;

    Time::delay(2.);

    // start detecting unknown objects
    while (!bFound)
    {
        iCub->opc->checkout();
        list<Entity*> lEntities = iCub->opc->EntitiesCacheCopy();

        double highestSaliency = 0.0;
        double secondSaliency = 0.0;
        string sNameBestEntity = "none";
        string sTypeBestEntity = "none";

        for (list<Entity*>::iterator itEnt = lEntities.begin(); itEnt != lEntities.end(); itEnt++)
        {
            string sName = (*itEnt)->name();
            string sNameCut = sName;
            string delimiter = "_";
            size_t pos = 0;
            string token;
            while ((pos = sName.find(delimiter)) != string::npos) {
                token = sName.substr(0, pos);
                sName.erase(0, pos + delimiter.length());
                sNameCut = token;
            }
            // check is label is known

            if (sNameCut == "unknown")
            {
                if ((*itEnt)->entity_type() == "object" || (*itEnt)->entity_type() == "agent" || (*itEnt)->entity_type() == "rtobject")
                {
                    Object* temp = dynamic_cast<Object*>(*itEnt);
                    if (temp->m_saliency > highestSaliency)
                    {
                        if (secondSaliency != 0.0)
                        {
                            secondSaliency = highestSaliency;
                        }
                        highestSaliency = temp->m_saliency;
                        sNameBestEntity = temp->name();
                        sTypeBestEntity = temp->entity_type();
                    }
                    else
                    {
                        if (temp->m_saliency > secondSaliency)
                        {
                            secondSaliency = temp->m_saliency;
                        }
                    }
                }
            }
        }

        bFound = false;
        if (highestSaliency > thresholdSalienceDetection)
        {
            //the object with highest salience is salient enough
            if (secondSaliency != 0.0)
            {
                // there are other salient objects
                if ((highestSaliency / secondSaliency) > thresholdDistinguishObjectsRatio)
                {
                    //but it is enough difference
                    bFound = true;
                }
            }
            else
            {
                //other object are not salient
                bFound = true;
            }
        }
        if (sNameBestEntity == "none")
        {
            bFound = false;
        }

        if (bFound)
        {
            // change name
            Entity* TARGET = iCub->opc->getEntity(sNameBestEntity);
            TARGET->changeName(sNameTarget);
            yInfo() << " name changed: " << sNameBestEntity << " is now " << sNameTarget;
            bOutput.addString("name changed");
        }
    }

    iCub->opc->commit();

    return bOutput;
}

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

    GrammarYesNo           = rf.findFileByName(rf.check("GrammarYesNo", Value("nodeYesNo.xml")).toString());
    GrammarAskNameObject   = rf.findFileByName(rf.check("GrammarAskNameObject", Value("GrammarAskNameObject.xml")).toString());
    GrammarAskNameAgent    = rf.findFileByName(rf.check("GrammarAskNameAgent", Value("GrammarAskNameAgent.xml")).toString());
    GrammarAskNameBodypart = rf.findFileByName(rf.check("GrammarAskNameBodypart", Value("GrammarAskNameSelf.xml")).toString());
    GrammarDescribeAction  = rf.findFileByName(rf.check("GrammarDescribeAction", Value("GrammarDescribeAction.xml")).toString());

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
    portToBodySchema.open(("/" + moduleName + "/toBodySchema:o").c_str());
    string bodySchemaRpc = rf.check("bodySchemaRpc",Value("/bodySchema/rpc")).asString().c_str();

    if (!Network::connect(portToBodySchema.getName().c_str(),bodySchemaRpc.c_str())) {
        yWarning() << " BODY SCHEMA NOT CONNECTED: selfTagging will not work";
    }

    //out to BodySchema, bufferedPort for no wait and allow describe actions
    portNoWaitToBodySchema.open(("/" + moduleName + "/BufferedPort/toBodySchema:o").c_str());
    if (!Network::connect(portNoWaitToBodySchema.getName().c_str(),bodySchemaRpc.c_str())) {
        yWarning() << " BODY SCHEMA BUFFERED PORT NOT CONNECTED: actionTagging will not work";
    }

    //out to LRH
    portToLRH.open(("/" + moduleName + "/toLRH:o").c_str());
    string LRHRpc = rf.check("LRHRpc",Value("/lrh/rpc")).asString().c_str();
    if (!Network::connect(portToLRH.getName().c_str(),LRHRpc.c_str())) {
        yWarning() << " LRH NOT CONNECTED: will not produce sentences";
    }

    // out to pasar
    portToPasar.open(("/" + moduleName + "/pasar:o").c_str());
    if (!Network::connect(portToPasar.getName().c_str(),"/pasar/rpc")) {
        yWarning() << " PASAR NOT CONNECTED: will not engage pointing";
    }


    //in from TouchDetector
    portFromTouchDetector.open(("/" + moduleName + "/fromTouch:i").c_str());
    string portTouchDetectorOut = rf.check("touchDetectorOut",Value("/touchDetector/touch:o")).asString().c_str();

    if (!Network::connect(portTouchDetectorOut.c_str(),portFromTouchDetector.getName().c_str(), "tcp+recv.portmonitor+type.lua+context.touchDetector+file.conversion_cluster_list")) {
        yWarning() << " TOUCH DETECTOR NOT CONNECTED: selfTagging will not work";
    }

    if (!iCub->getRecogClient())
    {
        yWarning() << "WARNING SPEECH RECOGNIZER NOT CONNECTED";
    }
    if (!iCub->getABMClient())
    {
        yWarning() << "WARNING ABM NOT CONNECTED";
    }

    //std::string ttsOptions = rf.check("ttsOptions", yarp::os::Value("iCub")).toString();
    //if (iCub->getSpeechClient())
    //iCub->getSpeechClient()->SetOptions(ttsOptions);

    iCub->say("proactive tagging is ready", false);
    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    return true;
}


bool proactiveTagging::interruptModule() {
    portToLRH.interrupt();
    portFromTouchDetector.interrupt();
    portToBodySchema.interrupt();
    rpcPort.interrupt();

    return true;
}

bool proactiveTagging::close() {
    iCub->close();
    delete iCub;

    portToLRH.interrupt();
    portToLRH.close();

    portFromTouchDetector.interrupt();
    portFromTouchDetector.close();

    portToBodySchema.interrupt();
    portToBodySchema.close();

    rpcPort.interrupt();
    rpcPort.close();

    return true;
}


bool proactiveTagging::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n"
        "exploreUnknownEntity entity_type entity_name \n" +
        "exploreEntityByName entity_name \n" + 
        "exploreKinematicByName entity_name bodypart [true/false] \n" +
        "exploreKinematicByJoint joint bodypart [true/false] \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    }
    else if (command.get(0).asString() == "exploreUnknownEntity") {
        yInfo() << " exploreUnknownEntity";
        string type = command.get(1).toString();
        string name = command.get(2).toString();
        if(type=="bodypart" && name.find("unknown") == std::string::npos) {
            iCub->opc->checkout();
            Bodypart* bp = dynamic_cast<Bodypart*>(iCub->opc->getEntity(name));
            // TODO: Make calls of exploreTactileEntityWithName and assignKinematicStructureByName consistent
            if(bp->m_tactile_number == -1) {
                reply = exploreTactileEntityWithName(command);
            } else if(bp->m_kinStruct_instance == -1) {
                bool forcingKS   = true;
                reply = assignKinematicStructureByName(name, type, forcingKS);
            } else {
                yWarning("Not sure what to do, name + kinematic structure + tactile information already known");
            }
        } else {
            reply = exploreUnknownEntity(command);
        }
    }
    else if (command.get(0).asString() == "searchingEntity") {
        yInfo() << " searchingEntity";
        reply = searchingEntity(command);
    }
    else if (command.get(0).asString() == "exploreKinematicByName") {
        //exploreKinematicByName name bodypart [true/false]
        //name : index, thumb, biceps, etc.
        //bodypart : left_hand, right_arm, etc.
        //true/false : OPTIONAL : forcingKinematicStructure : do you want to launch a KS generation (may take minutes)

        yInfo() << " exploreKinematicByName";

        if (command.size() < 3) {
            yError() << " error in proactiveTagging::respond | for " << command.get(0).asString() << " | Not enough argument : exploreKinematicByName name bodypart [true/false]";
            reply.addString("error");
            reply.addString("Not enough argument : exploreKinematicByName name bodypart [true/false]");

            rpcPort.reply(reply);
            return true;
        }

        string sName     = command.get(1).asString();
        string sBodyPart = command.get(2).asString();
        bool forcingKS = false;
        if (command.size() == 4) {
            forcingKS = (command.get(3).asString() == "true");
        }
        reply = assignKinematicStructureByName(sName, sBodyPart, forcingKS);
    }
    else if (command.get(0).asString() == "exploreKinematicByJoint") {
        //exploreKinematicByName name bodypart [true/false]
        //joint : 9, 10, ...
        //bodypart : left_hand, right_arm, etc.
        //true/false : OPTIONAL : forcingKinematicStructure : do you want to launch a KS generation (may take minutes)

        yInfo() << " exploreKinematicByJoint";

        if (command.size() < 3) {
            yError() << " error in proactiveTagging::respond | for " << command.get(0).asString() << " | Not enough argument : exploreKinematicByJoint joint bodypart [true/false]";
            reply.addString("error");
            reply.addString("Not enough argument : exploreKinematicByJoint joint bodypart [true/false]");

            rpcPort.reply(reply);
            return true;
        }

        if (!command.get(1).isInt()) {
            yError() << " error in proactiveTagging::respond | for " << command.get(0).asString() << " | Second argument (joint) should be an Int!";
            reply.addString("error");
            reply.addString("Second argument (joint) should be an Int!");

            rpcPort.reply(reply);
            return true;
        }

        int BPjoint      = command.get(1).asInt();
        string sBodyPart = command.get(2).asString();
        bool forcingKS   = false;
        if (command.size() == 4) {
            forcingKS = (command.get(3).asString() == "true");
        }
        reply = assignKinematicStructureByJoint(BPjoint, sBodyPart, forcingKS);
    }
    else if (command.get(0).asString() == "describeAction"){  //describeAction : TODO -> protection and stuff
        string sName   = command.get(1).asString() ;
        string sTarget = command.get(2).asString() ;

        reply = describeAction(sName, sTarget);
    }
    else {
        cout << helpMessage;
        reply.addString(helpMessage);
    }

    rpcPort.reply(reply);

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
        yError() << " error in proactiveTagging::recogName | for " << entityType << " | Entity Type not managed";
        bOutput.addString("error");
        bOutput.addString("Entity Type not managed");
        return bOutput;
    }

    if (bRecognized.get(0).asInt() == 0)
    {
        yError() << " error in proactiveTagging::askName | for " << entityType << " | Error in speechRecog";
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

    //Check if name is known or not. if yes, and body part : ask tactile

    //Ask question for the human, or ask to pay attention (if action to focus attention after)
    string sQuestion;
    if (currentEntityType == "agent") {
        sQuestion = " Hello, I don't know you. Who are you ?";
    } else if (currentEntityType == "object" || currentEntityType == "rtobject") {
        sQuestion = " Hum, what is this object ?";
    } else if (currentEntityType == "bodypart") {
        sQuestion = " Watch please, I will move a part of my body";
        yInfo() << " sQuestion: " << sQuestion;
    } else {
        yError() << " error in proactiveTagging::exploreUnknownEntity | for " << currentEntityType << " | Entity Type not managed";
        bOutput.addString("error");
        bOutput.addString("Entity Type not managed");
        return bOutput;
    }

    //TODO : choose between say and TTS. say put stuff in ABM, TTS?
    yInfo() << sQuestion;
    iCub->say(sQuestion);
    iCub->opc->checkout();

    //Act to determine the entity to be named, according to entityType (e.g. bodypart is sending a command to move the joint, ...)
    if(currentEntityType == "bodypart") {
        Bodypart* BPtemp = dynamic_cast<Bodypart*>(iCub->opc->getEntity(sNameTarget));
        yInfo() << "Cast okay : name BP = " << BPtemp->name();
        int joint = BPtemp->m_joint_number;
        string sBodyPartType = BPtemp->m_part;
        //send rpc command to bodySchema to move the corresponding part
        yInfo() << "Start bodySchema";
        Bottle bReplyFromBodySchema = moveJoint(joint, sBodyPartType);

        if(bReplyFromBodySchema.get(0).asString() == "nack"){
            yError() << " error in proactiveTagging::exploreUnknownEntity | for " << currentEntityType << " | Joint has not moved or ABM cannot stores images";
            bOutput.addString("error");
            bOutput.addString("Joint has not moved or ABM cannot stores images");
            return bOutput;
        }

        sQuestion = " How do you call this part of my body?";
        yInfo() << sQuestion;
        //iCub->getSpeechClient()->TTS(sQuestion, false);
        iCub->say(sQuestion);
    }
    else if(currentEntityType == "object" || currentEntityType == "rtobject") {
        Object* obj1 = iCub->opc->addOrRetrieveEntity<Object>("sNameTarget");
        string sHand = "right";
        if (obj1->m_ego_position[1]<0) sHand = "left";
        Bottle bHand(sHand);
        iCub->point(sNameTarget, bHand);
    } else if(currentEntityType == "agent") {
        iCub->getARE()->waving(true);
        yarp::os::Time::delay(3.0);
        iCub->getARE()->waving(false);
    }

    Bottle bName = recogName(currentEntityType);
    string sName;

    //if error, bName = (error errorDescription) -> return it
    if(bName.get(0).asString() == "error"){
        return bName;
    } else {
        sName = bName.get(0).asString();
    }

    iCub->home();

    string sReply;
    Entity* e = iCub->opc->getEntity(sNameTarget);
    e->changeName(sName);
    iCub->opc->commit(e);

    if (currentEntityType == "agent") {
        sReply = " Nice to meet you " + sName;
    } else if (currentEntityType == "object") {
        SubSystem_IOL2OPC* iol2opcClient = iCub->getIOL2OPCClient();
        if(iol2opcClient!=NULL) {
            iol2opcClient->changeName(sNameTarget, sName);
        }
        else {
            yError() << "Could not connect to IOL2OPC subsystem";
            sReply = " I get it, this is a " + sName;
        }
        //sReply = " I get it, this is a " + sName;
        Bottle bToLRH, bFromLRH;
        bToLRH.addString("production");
        bToLRH.addString(sName);
        if(portToLRH.getOutputCount()>0)
            portToLRH.write(bToLRH, bFromLRH);
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
Bottle proactiveTagging::searchingEntity(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 2)
    {
        yInfo() << " proactiveTagging::searchingEntity | Problem in input size.";
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
        yWarning() << " in proactiveTagging::searchingEntity | OPC not Connected";
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


    Bottle bToPasar;
    bToPasar.addString("pointing");
    bToPasar.addString("on");
    portToPasar.write(bToPasar);

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
            if ((pos = sName.find(delimiter)) != string::npos) {
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
            if(TARGET->entity_type()=="object") {
                SubSystem_IOL2OPC* iol2opcClient = iCub->getIOL2OPCClient();
                if(iol2opcClient!=NULL) {
                    iol2opcClient->changeName(sNameBestEntity, sNameTarget);
                }
                else {
                    yError() << "Could not connect to IOL2OPC subsystem";
                }
            }
        }
    }

    bToPasar.clear();
    bToPasar.addString("pointing");
    bToPasar.addString("off");
    portToPasar.write(bToPasar);


    iCub->opc->commit();

    return bOutput;
}

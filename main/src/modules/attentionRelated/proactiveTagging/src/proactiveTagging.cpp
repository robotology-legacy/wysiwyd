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

    GrammarAskNameObject = rf.findFileByName(rf.check("GrammarAskNameObject", Value("GrammarAskNameObject.xml")).toString());
    GrammarAskNameAgent = rf.findFileByName(rf.check("GrammarAskNameAgent", Value("GrammarAskNameAgent.xml")).toString());
    GrammarAskNameBodypart = rf.findFileByName(rf.check("GrammarAskNameBodypart", Value("GrammarAskNameSelf.xml")).toString());
    GrammarDescribeAction = rf.findFileByName(rf.check("GrammarDescribeAction", Value("GrammarDescribeAction.xml")).toString());

    babblingArm = rf.check("babblingArm", Value("left")).toString();

    thresholdDistinguishObjectsRatio = rf.check("thresholdDistinguishObjectsRatio", Value(3.0)).asDouble();
    thresholdSalienceDetection = rf.check("thresholdSalienceDetection", Value(2.0)).asDouble();

    yDebug() << "------------> babblingArm: " << babblingArm;

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    defaultPartnerName = "partner";

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "proactiveTagging", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    while (!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        Time::delay(1.0);
    }

    configureOPC(rf);

    //--------------------------------------------- output port
    std::string ttsOptions = rf.check("ttsOptions", yarp::os::Value("iCub")).toString();
    if (ttsOptions != "iCub") {
        if (iCub->getSpeechClient())
            iCub->getSpeechClient()->SetOptions(ttsOptions);
    }

    //out to BodySchema, bufferedPort for no wait and allow describe actions
    portNoWaitToBodySchema.open(("/" + moduleName + "/BufferedPort/toBodySchema:o").c_str());
    if (!Network::connect(portNoWaitToBodySchema.getName().c_str(), bodySchemaRpc.c_str())) {
        yWarning() << "BODY SCHEMA BUFFERED PORT NOT CONNECTED: actionTagging will not work";
        iCub->say("BODY SCHEMA BUFFERED PORT NOT CONNECTED");
    }

    //out to SAM
    portToSAM.open(("/" + moduleName + "/toSAM:o").c_str());
    SAMRpc = rf.check("SAMRpc", Value("/sam/face/rpc:i")).asString().c_str();

    if (!Network::connect(portToSAM.getName().c_str(), SAMRpc.c_str())) {
        yWarning() << "SAM NOT CONNECTED: face recognition will not work";
        iCub->say("SAM NOT CONNECTED");
    }

    //out to LRH
    portToLRH.open(("/" + moduleName + "/toLRH:o").c_str());
    LRHRpc = rf.check("LRHRpc", Value("/lrh/rpc")).asString().c_str();
    if (!Network::connect(portToLRH.getName().c_str(), LRHRpc.c_str())) {
        yWarning() << "LRH NOT CONNECTED: will not produce sentences";
        iCub->say("LRH NOT CONNECTED");
    }

    // out to pasar
    portToPasar.open(("/" + moduleName + "/pasar:o").c_str());
    if (!Network::connect(portToPasar.getName().c_str(), "/pasar/rpc")) {
        yWarning() << "PASAR NOT CONNECTED: will not engage pointing";
        iCub->say("PASAR NOT CONNECTED");
    }

    //in from TouchDetector
    portFromTouchDetector.open(("/" + moduleName + "/fromTouch:i").c_str());
    touchDetectorRpc = rf.check("touchDetectorOut", Value("/touchDetector/touchClean:o")).asString().c_str();

    if (!Network::connect(touchDetectorRpc.c_str(), portFromTouchDetector.getName().c_str())) {
        yWarning() << "TOUCH DETECTOR NOT CONNECTED: selfTagging will not work";
        iCub->say("TOUCH DETECTOR NOT CONNECTED");
    }

    if (!iCub->getRecogClient())
    {
        iCub->say("speech recognizer not connected");
        yWarning() << "SPEECH RECOGNIZER NOT CONNECTED";
    }
    if (!iCub->getABMClient())
    {
        iCub->say("ABM not connected");
        yWarning() << "ABM NOT CONNECTED";
    }

    iCub->home();

    iCub->say("proactive tagging is ready", false);
    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    return true;
}


bool proactiveTagging::interruptModule() {
    portToLRH.interrupt();
    portFromTouchDetector.interrupt();
    portNoWaitToBodySchema.interrupt();
    portToSAM.interrupt();
    portToPasar.interrupt();
    rpcPort.interrupt();

    return true;
}

bool proactiveTagging::close() {
    if(iCub) {
        iCub->close();
        delete iCub;
    }

    portToLRH.interrupt();
    portToLRH.close();

    portFromTouchDetector.interrupt();
    portFromTouchDetector.close();

    portNoWaitToBodySchema.interrupt();
    portNoWaitToBodySchema.close();

    portToSAM.interrupt();
    portToSAM.close();

    portToPasar.interrupt();
    portToPasar.close();

    rpcPort.interrupt();
    rpcPort.close();

    return true;
}


bool proactiveTagging::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n" +
        "change_name oldname newname \n" +
        "exploreUnknownEntity entity_type entity_name \n" +
        "searchingEntity entity_type entity_name \n" +
        "exploreKinematicByName entity_name bodypart [true/false] \n" +
        "exploreKinematicByJoint joint bodypart [true/false] \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    }
    else if(command.get(0).asString() == "change_name") {
        string oldname = command.get(1).toString();
        string newname = command.get(2).toString();
        yInfo() << "change_name from" << oldname << "to" << newname;
        Entity *e = iCub->opc->getEntity(oldname);
        if(!e) {
            iCub->say("No entity with name " + oldname);
            yError() << "No entity with name " << oldname;
            reply.addString("nack");
        } else {
            iCub->changeName(e, newname);
            reply.addString("ack");
        }
    }
    else if (command.get(0).asString() == "exploreUnknownEntity") {
        // disable EARS early on
        iCub->getRecogClient()->listen(false);

        yInfo() << " exploreUnknownEntity";
        string type = command.get(1).toString();
        string name = command.get(2).toString();
        yDebug() << "exploreUnknownEntity with name = " << name ;
        //yDebug() << "It is " <<  !(name.find("unknown") == std::string::npos) << "that it contains 'unknown'" ;
        //type is bodypart and it has been named! name.find... is true when name DOES NOT contain unknown
        if ((type == "bodypart") && (name.find("unknown") == std::string::npos)) {
            iCub->opc->checkout();
            Bodypart* bp = dynamic_cast<Bodypart*>(iCub->opc->getEntity(name));
            if(!bp) {
                yError() << "Could not cast" << name << "to bodypart";
                iCub->say("Could not cast " + name + " to bodypart");
            }
            // TODO: Make calls of exploreTactileEntityWithName and assignKinematicStructureByName consistent
            if (bp->m_tactile_number == -1) {
                yInfo() << "Going to tag skin part";
                reply = exploreTactileEntityWithName(command);
            }
            else if (bp->m_kinStruct_instance == -1) {
                yInfo() << "Going to tag kinematic structure";
                bool forcingKS = true;
                reply = assignKinematicStructureByName(name, type, forcingKS);
            }
            else {
                yWarning("Not sure what to do, name + kinematic structure + tactile information already known");
            }
        } else if (type == "bodypart") {
            yInfo() << "Going to tag bodypart (include babbling)";
            reply = exploreUnknownEntity(command);
        }
        else if (type == "object") {
            yInfo() << "Going to tag object (include a pointing)";
            reply = exploreUnknownEntity(command);
        } else if (type == "agent") {
            yInfo() << "Going to tag an agent (include a gazing)";
            reply = exploreUnknownEntity(command);
        } else {
            yWarning() << "Type = " << type << " is NON valid for exploreUnknownEntity/ doing nothing";
            reply.addString("error");
            reply.addString("Type is NON valid for exploreUnknownEntity");
        }

        // enable EARS again
        iCub->getRecogClient()->listen(true);
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

        string sName = command.get(1).asString();
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

        int BPjoint = command.get(1).asInt();
        string sBodyPart = command.get(2).asString();
        bool forcingKS = false;
        if (command.size() == 4) {
            forcingKS = (command.get(3).asString() == "true");
        }
        reply = assignKinematicStructureByJoint(BPjoint, sBodyPart, forcingKS);
    }
    else if (command.get(0).asString() == "describeBabbling"){  //describeAction : TODO -> protection and stuff
        string sJointName = command.get(1).asString();
        int jointNumber = command.get(2).asInt();

        reply = describeBabbling(sJointName, jointNumber);
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

    yDebug() << "Going to load grammar.";
    //Load the Speech Recognition with grammar according to entityType
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)
    if (entityType == "agent"){
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameAgent), 20);
        bAnswer = *bRecognized.get(1).asList();
        if(bAnswer.get(1).asList()->get(0).toString() != "SENTENCEAGENT") {
            iCub->say("I asked you something else");
            yError() << "Wrong sentence type returned (not SENTENCEAGENT)";
            bOutput.addString("error");
            bOutput.addString("Wrong sentence type returned (not SENTENCEAGENT)");
            return bOutput;
        }
    }
    else if (entityType == "object" || entityType == "rtobject"){
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameObject), 20);
        bAnswer = *bRecognized.get(1).asList();
        if(bAnswer.get(1).asList()->get(0).toString() != "SENTENCEOBJECT") {
            iCub->say("I asked you something else");
            yError() << "Wrong sentence type returned (not SENTENCEOBJECT)";
            bOutput.addString("error");
            bOutput.addString("Wrong sentence type returned (not SENTENCEOBJECT)");
            return bOutput;
        }
    }
    else if (entityType == "bodypart"){
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarAskNameBodypart), 20);
        bAnswer = *bRecognized.get(1).asList();
        if(bAnswer.get(1).asList()->get(0).toString() != "SENTENCEBODYPART") {
            iCub->say("I asked you something else");
            yError() << "Wrong sentence type returned (not SENTENCEBODYPART)";
            bOutput.addString("error");
            bOutput.addString("Wrong sentence type returned (not SENTENCEBODYPART)");
            return bOutput;
        }
    }
    else {
        yError() << " error in proactiveTagging::recogName | for " << entityType << " | Entity Type not managed";
        bOutput.addString("error");
        bOutput.addString("Entity Type not managed");
        return bOutput;
    }
    yDebug() << "Response from recogClient: " << bRecognized.toString();

    if (bRecognized.get(0).asInt() == 0)
    {
        yError() << " error in proactiveTagging::askName | for " << entityType << " | Error in speechRecog";
        bOutput.addString("error");
        bOutput.addString("error in speechRecog");
        return bOutput;
    }

    string sSentence = bAnswer.get(0).asString();
    iCub->say("I've understood " + sSentence);

    if (bAnswer.get(0).asString() == "stop")
    {
        yError() << " in proactiveTagging::askName | for " << entityType << " | stop called";
        bOutput.addString("error");
        bOutput.addString("stop called");
        return bOutput;
    }

    bSemantic = *bAnswer.get(1).asList();
    string sName;
    if (entityType == "agent") {
        sName = bSemantic.check("agent", Value("unknown")).asString();
    }
    else if (entityType == "object" || entityType == "rtobject") {
        sName = bSemantic.check("object", Value("unknown")).asString();
    }
    else if (entityType == "bodypart") {
        sName = bSemantic.check("fingerName", Value("unknown")).asString();
    }
    else {
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
Bottle proactiveTagging::exploreUnknownEntity(const Bottle& bInput)
{
    Bottle bOutput;
    if (bInput.size() != 3)
    {
        yInfo() << " proactiveTagging::exploreEntity | Problem in input size.";
        bOutput.addString("Problem in input size");
        return bOutput;
    }

    iCub->opc->checkout();

    string currentEntityType = bInput.get(1).toString();
    string sNameTarget = bInput.get(2).toString();

    yInfo() << " EntityType : " << currentEntityType;
    double timeDelay = 1.0;

    //Check if name is known or not. if yes, and body part : ask tactile

    //Ask question for the human, or ask to pay attention (if action to focus attention after)
    string sQuestion;
    if (currentEntityType == "agent") {
        Agent* TARGET = dynamic_cast<Agent*>(iCub->opc->getEntity(sNameTarget));
        iCub->look(TARGET->name());

        if (!Network::connect(portToSAM.getName().c_str(), SAMRpc.c_str())) {
            yWarning() << " SAM NOT CONNECTED: face recognition will not work";
        }
        if(portToSAM.getOutputCount()>0) {
            Bottle bToSam, bReplySam;
            bToSam.addString("ask_name");

            yDebug() << "Request to SAM: " << bToSam.toString();
            portToSAM.write(bToSam, bReplySam);
            yDebug() << "Reply from SAM: " << bReplySam.toString();
            string sNameSAM = bReplySam.get(0).asString();
            if(sNameSAM != "nack" && sNameSAM != "partner" && sNameSAM != "") {
                yDebug() << "Changing name from " << TARGET->name() << " to " << sNameSAM;
                iCub->changeName(TARGET,sNameSAM);
                iCub->opc->commit(TARGET);

                iCub->say("Nice to see you " + sNameSAM);
                yarp::os::Time::delay(timeDelay);
                iCub->home();

                bOutput.addString("success");
                bOutput.addString(currentEntityType);
                return bOutput;
            }
        }

        sQuestion = " Hello, I don't know you. Who are you?";
    }
    else if (currentEntityType == "object" || currentEntityType == "rtobject") {
        Object* TARGET = dynamic_cast<Object*>(iCub->opc->getEntity(sNameTarget));
        iCub->look(TARGET->name());
        sQuestion = " Hum, what is this object?";
    }
    else if (currentEntityType == "bodypart") {
        iCub->lookAtPartner();
        sQuestion = " Watch please, I will move a part of my body";
    }
    else {
        yError() << " error in proactiveTagging::exploreUnknownEntity | for " << currentEntityType << " | Entity Type not managed";
        bOutput.addString("nack");
        bOutput.addString("Entity Type not managed");
        return bOutput;
    }

    //TODO : choose between say and TTS. say put stuff in ABM, TTS?
    yInfo() << sQuestion;
    iCub->say(sQuestion, false);

    //Act to determine the entity to be named, according to entityType (e.g. bodypart is sending a command to move the joint, ...)
    if (currentEntityType == "bodypart") {
        Bodypart* BPtemp = dynamic_cast<Bodypart*>(iCub->opc->getEntity(sNameTarget));
        if(!BPtemp) {
            bOutput.addString("nack");
            bOutput.addString("Could not cast to Bodypart");
            iCub->say(bOutput.toString());
            return bOutput;
        }
        yInfo() << "Cast okay : name BP = " << BPtemp->name();
        int joint = BPtemp->m_joint_number;
        //send rpc command to bodySchema to move the corresponding part
        yInfo() << "Start bodySchema";
        iCub->babbling(joint, babblingArm);

        //add name of the partner in the question if defined
        string partnerName = iCub->getPartnerName();
        iCub->lookAtPartner();
        if(partnerName == defaultPartnerName){
            sQuestion = " How do you call this part of my body?";
        } else {
            sQuestion = partnerName + ", How do you call this part of my body?";
        }

        yInfo() << sQuestion;
        //iCub->getSpeechClient()->TTS(sQuestion, false);
        iCub->say(sQuestion, false);
    }
    else if (currentEntityType == "object" || currentEntityType == "rtobject") {
        Object* obj1 = dynamic_cast<Object*>(iCub->opc->getEntity(sNameTarget));
        if(!obj1) {
            yError() << "Could not cast " << sNameTarget << " to object";
            bOutput.addString("nack");
            bOutput.addString("Could not cast to object");
            iCub->say(bOutput.toString());
            return bOutput;
        }

        yDebug() << "Going to point " << sNameTarget;
        iCub->point(sNameTarget);
        yDebug() << "pointing done";
    }

    Bottle bName = recogName(currentEntityType);
    string sName;

    //if error, bName = (error errorDescription) -> return it
    if (bName.get(0).asString() == "error") {
        return bName;
    }
    else {
        sName = bName.get(0).asString();
    }

    string sReply;
    Entity* e = iCub->opc->getEntity(sNameTarget);
    iCub->changeName(e,sName);

    iCub->lookAtPartner();
    if (currentEntityType == "agent") {
        sReply = " Nice to meet you " + sName;
    }
    else if (currentEntityType == "object") {
        sReply = " I get it, this is a " + sName;
        Bottle bToLRH, bFromLRH;
        bToLRH.addString("production");
        bToLRH.addString(sName);
        if (portToLRH.getOutputCount() > 0)
            portToLRH.write(bToLRH, bFromLRH);
    }
    else if (currentEntityType == "rtobject") {
        sReply = " So this is a " + sName;
    }
    else if (currentEntityType == "bodypart") {
        sReply = " Nice, I know that I have a " + sName;
    }//go out before if not one of those entityType


    if (iCub->getABMClient()->Connect())
    {
        //                yInfo() << "\t\t START POINTING OF: " << it.second.o.name();
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>("rename", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sNameTarget, "object"));
        lArgument.push_back(std::pair<std::string, std::string>(sName, "recipient"));
        iCub->getABMClient()->sendActivity("action",
            "rename",
            "proactiveTagging",
            lArgument,
            true);
    }

    yInfo() << sReply;
    iCub->say(sReply);
    Time::delay(timeDelay);

    yDebug() << "Going home";
    iCub->home();

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
Bottle proactiveTagging::searchingEntity(const Bottle &bInput)
{
    Bottle bOutput;

    if (bInput.size() != 3)
    {
        yInfo() << " proactiveTagging::searchingEntity | Problem in input size.";
        bOutput.addString("error");
        bOutput.addString("Problem in input size");
        return bOutput;
    }

    string sTypeTarget = bInput.get(1).toString();
    string sNameTarget = bInput.get(2).toString();
    yInfo() << " Entity to find: " << sNameTarget;

    bool unknownEntityPresent = false;

    // check if the entity is already present in the OPC
    if (iCub->opc->isConnected())
    {
        iCub->opc->checkout();
        list<Entity*> lEntities = iCub->opc->EntitiesCacheCopy();

        for (auto& entity : lEntities)
        {
            if (entity->name() == sNameTarget && entity->entity_type() == sTypeTarget)
            {
                yInfo() << " Entity " << sNameTarget << " is already known.";
                bOutput.addString("warning");
                bOutput.addString("entity already exists");
                return bOutput;
            }
            Object *o = dynamic_cast<Object*>(entity);
            if(o && o->name().find("unknown") != string::npos && o->entity_type() == sTypeTarget && o->m_present == 1.0) {
                unknownEntityPresent = true;
            }
        }

        if(!unknownEntityPresent) {
            iCub->say("I know all the " + sTypeTarget + " present. The " + sNameTarget + " is not here");
            yInfo() << "No unknown entity is present.";
            bOutput.addString("nack");
            bOutput.addString("No unknown entity is present.");
            return bOutput;
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
    string sSentence;
    if(sTypeTarget == "object") {
        sSentence = "I don't known which of these objects is a " + sNameTarget + ". Can you show me the " + sNameTarget;
    } else if (sTypeTarget == "bodypart") {
        sSentence = "I don't known my " + sNameTarget + ". Can you please touch my " + sNameTarget;
    }

    //add name of the partner at the end of the question if defined
    string partnerName = iCub->getPartnerName();
    if(partnerName != defaultPartnerName){
        sSentence += ", " + partnerName;
    }

    iCub->lookAtPartner();
    iCub->say(sSentence);
    yInfo() << " " << sSentence;

    if(sTypeTarget == "object") {
        iCub->home();

        Bottle bToPasar;
        Bottle bFromPasar;
        bToPasar.addString("pointing");
        bToPasar.addString("on");
        if (!Network::connect(portToPasar.getName().c_str(), "/pasar/rpc")) {
            yError() << "Could not connect to pasar";
            iCub->say("Could not connect to pasar");

            bOutput.addString("error");
            bOutput.addString("Could not connect to pasar");
            return bOutput;
        } else {
            portToPasar.write(bToPasar, bFromPasar);
            if(bFromPasar.get(0).asString()!="ack") {
                yError() << "Pasar did not change pointing to on";
                iCub->say("Pasar did not change pointing to on");

                bOutput.addString("error");
                bOutput.addString("Pasar did not change pointing to on");
                return bOutput;
            }
        }
    }

    bool bFound = false;

    Time::delay(2.0);

    // start detecting unknown objects
    while (!bFound)
    {
        iCub->opc->checkout();
        list<Entity*> lEntities = iCub->opc->EntitiesCacheCopy();

        int unknownObjects = 0;
        double highestSaliency = -1.0;
        double secondSaliency = -1.0;
        string sNameBestEntity = "none";
        string sTypeBestEntity = "none";

        for (auto& entity : lEntities)
        {
            string sName = entity->name();
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
                if ((sTypeTarget == "object" && (entity->entity_type() == "object" || entity->entity_type() == "rtobject")) ||
                    (sTypeTarget == "bodypart" && (entity->entity_type() == "bodypart")))
                {
                    unknownObjects++;
                    Object* temp = dynamic_cast<Object*>(entity);
                    if(!temp) {
                        yError() << "Could not cast " << entity->name() << " to an object";
                        iCub->say("Could not cast " + entity->name() + " to an object");
                    }
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
        if (unknownObjects == 1)
        {
            bFound = true;
        }
        if (sNameBestEntity == "none")
        {
            bFound = false;
        }

        if (bFound)
        {
            // change name
            Object* TARGET = dynamic_cast<Object*>(iCub->opc->getEntity(sNameBestEntity));
            if(sTypeTarget == "object") {
                iCub->look(TARGET->name());
            }

            iCub->changeName(TARGET,sNameTarget);

            yInfo() << " name changed: " << sNameBestEntity << " is now " << sNameTarget;
            bOutput.addString("name changed");
            if(unknownObjects == 1) {
                iCub->say("There was only one object which I did not know.");
            }
            iCub->say("Now I know the" + sNameTarget);
        }
    }

    if(sTypeTarget == "object") {
        Bottle bToPasar, bFromPasar;
        bToPasar.addString("pointing");
        bToPasar.addString("off");
        if (!Network::connect(portToPasar.getName().c_str(), "/pasar/rpc")) {
            yError() << "Could not connect to pasar";
            iCub->say("Could not connect to pasar");
        } else {
            portToPasar.write(bToPasar, bFromPasar);
            if(bFromPasar.get(0).asString()!="ack") {
                yError() << "Pasar did not change pointing to off";
                iCub->say("Pasar did not change pointing to off");
            }
        }
    }

    iCub->home();

    if (iCub->getABMClient()->Connect())
    {
        //                yInfo() << "\t\t START POINTING OF: " << it.second.o.name();
        std::list<std::pair<std::string, std::string> > lArgument;
        lArgument.push_back(std::pair<std::string, std::string>("iCub", "agent"));
        lArgument.push_back(std::pair<std::string, std::string>("name", "predicate"));
        lArgument.push_back(std::pair<std::string, std::string>(sNameTarget, "object"));
        iCub->getABMClient()->sendActivity("action",
            "rename",
            "proactiveTagging",
            lArgument,
            true);
    }

    return bOutput;
}

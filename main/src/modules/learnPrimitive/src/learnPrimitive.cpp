/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors:  Maxime Petit
 * email:  m.petit@imperial.ac.uk
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

#include "learnPrimitive.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

bool learnPrimitive::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("learnPrimitive")).asString().c_str();
    setName(moduleName.c_str());

    GrammarYesNo           = rf.findFileByName(rf.check("GrammarYesNo", Value("nodeYesNo.xml")).toString());
    GrammarNameAction      = rf.findFileByName(rf.check("GrammarNameAction", Value("GrammarNAmeAction.xml")).toString());

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "learnPrimitive", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    string robot = rf.check("robot", Value("icubSim")).asString().c_str();
    string arm   = rf.check("arm", Value("left_arm")).asString().c_str();

    portToArm.open(("/" + moduleName + "/toArm:rpc").c_str());
    string portRobotArmName = "/" + robot + "/" + arm + "/rpc:i";

    yInfo() << "================> port controlling the arm : " << portRobotArmName;
    if (!Network::connect(portToArm.getName().c_str(),portRobotArmName))
    {
        yWarning() << "WARNING PORT TO CONTROL ARM (" << portRobotArmName << ") IS NOT CONNECTED";
    }


    if (!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        Time::delay(1.0);
    }

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    if (!iCub->getRecogClient())
    {
        yWarning() << "WARNING SPEECH RECOGNIZER NOT CONNECTED";
    }
    if (!iCub->getABMClient())
    {
       yWarning() << "WARNING ABM NOT CONNECTED";
    }

    updateProtoAction(rf);

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    iCub->say("learn Primitive is ready", false);

    return true;
}


bool learnPrimitive::interruptModule() {
    rpcPort.interrupt();

    return true;
}

bool learnPrimitive::close() {
    iCub->close();
    delete iCub;

    rpcPort.interrupt();
    rpcPort.close();

    return true;
}


bool learnPrimitive::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n" +
        "basicCommand actionName fingerName [true/false] \n";

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    }
    else if (command.get(0).asString() == "basicCommand"){  //describeAction : TODO -> protection and stuff

        if(command.size() < 2){
            yError() << " error in learnPrimitive::basicCommand | Too few arguments!";
            reply.addString("error");
            reply.addString("Too few arguments");

            rpcPort.reply(reply);
            return false;

        }
        string sActionName   = command.get(1).asString() ;
        string sBodypartName   = command.get(2).asString() ;
        int maxAngle = 10;
        if( command.size() >= 3 && command.get(3).isInt()) {
            maxAngle   = command.get(3).asInt() ;
        }
        reply = basicCommand(sActionName, sBodypartName, maxAngle);
    }
    else if (command.get(0).asString() == "learn"){  //describeAction : TODO -> protection and stuff
        reply = learn();
    }
    else {
        cout << helpMessage;
        reply.addString("ok");
    }

    rpcPort.reply(reply);

    return true;
}

/* Called periodically every getPeriod() seconds */
bool learnPrimitive::updateModule() {
    return true;
}


//action type : proto-action, primitive, action and stop
Bottle learnPrimitive::nodeNameAction(string actionTypeNeeded){
    Bottle bOutput ;
    Bottle bRecognized, //received FROM speech recog with transfer information (1/0 (bAnswer))
    bAnswer, //response from speech recog without transfer information, including raw sentence
    bSemantic; // semantic information of the content of the recognition

    //Load the Speech Recognition with grammar according to entityType
    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarNameAction), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        yError() << " error in proactiveTagging::nodeNameAction | Error in speechRecog (nodeNamePrimitive)";
        bOutput.addString("error");
        bOutput.addString("error in speechRecog (nodeNameAction)");
        return bOutput;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

    string actionType = bAnswer.get(0).asString() ;


    //If not "any" : should match (e.g. when "I will teach you how to ...")
    if (actionType != "any"){
        if(actionType != actionTypeNeeded){
            yError() << " error in proactiveTagging::nodeNameAction | Error in speechRecog (nodeNameAction) : actionType mismatch" ;
            bOutput.addString("error");
            bOutput.addString("Error in speechRecog (nodeNameAction) : actionType mismatch");
            return bOutput;
        }
    }

    bSemantic = *bAnswer.get(1).asList();
    string sName, sArg;

    if(actionType == "proto-action") {
        sName = bSemantic.check("proto-action_name", Value("unknown")).asString();
        sArg = bSemantic.check("proto-action_arg", Value("unknown")).asString();
    } else if(actionType == "primitive") {
        sName = bSemantic.check("primitive_name", Value("unknown")).asString();
        sArg = bSemantic.check("primitive_arg", Value("unknown")).asString();
    } else if(actionType == "action") {
        sName = bSemantic.check("action_name", Value("unknown")).asString();
        sArg = bSemantic.check("action_arg", Value("unknown")).asString();
    } else if (actionType == "stop") {
        bOutput.addString("stop") ;
        return bOutput ;
    } else {
        yError() << " error in proactiveTagging::nodeNameAction | ACtion type " << actionType << "is not known" ;
        bOutput.addString("error");
        bOutput.addString("Entity type is not known");
        return bOutput;
    }

    bOutput.addString(actionType);
    bOutput.addString(sName);
    bOutput.addString(sArg);

    return bOutput ;
}

//For now only control in position. Careful, angle is in percentage of maxAngle
Bottle learnPrimitive::learn(){
    Bottle bOutput;

    //1. recog for name primitive : I will teach you how to <close> your <hand>
    bOutput = nodeNameAction("any");

    string sSay;
    if(bOutput.get(0).asString() == "stop"){
        sSay = " Can you repeat please?";
        yInfo() << sSay;
        iCub->say(sSay);
        bOutput = learn();
    } else if (bOutput.get(0).asString() == "proto-action") {
        sSay = " Oh I cannot learn a proto-action right now";
        yInfo() << sSay;
        iCub->say(sSay);
        bOutput.addString("proto-action");
        return bOutput;
    }

    //2. Check that action is not already there (need to update list like in updateProtoAction)

    //3. Loop in recog to build with protoAction

    //4. write it

    return bOutput;
}

//For now only control in position. Careful, angle is in percentage of maxAngle
Bottle learnPrimitive::learnPrim(){
    Bottle bOutput;

    //1. recog for name primitive : I will teach you how to <close> your <hand>
    bOutput = nodeNameAction("primitive");

    //2. Check that action is not already there (need to update list like in updateProtoAction)

    //3. Loop in recog to build with protoAction

    //4. write it

    return bOutput;
}

//For now only control in position. Careful, angle is in percentage of maxAngle
Bottle learnPrimitive::learnAction(){
    Bottle bOutput;

    //1. recog for name primitive : I will teach you how to <close> your <hand>
    bOutput = nodeNameAction("action");

    //2. Check that action is not already there (need to update list like in updateProtoAction)

    //3. Loop in recog to build with protoAction

    //4. write it

    return bOutput;
}

//For now only control in position. Careful, angle is in percentage of maxAngle
Bottle learnPrimitive::basicCommand(string sActionName, string sBodyPartName, int maxAngle){
    Bottle bOutput;

    int targetAngle = 0;
    //1. check if proto is known
    if ( mProtoActionEnd.find(sActionName) == mProtoActionEnd.end() ) {
        yError() << " error in learnPrimitive::basicCommand | for " << sActionName << " | sActionName is unknown";
        bOutput.addString("error");
        bOutput.addString("sActionName is unknown");
        return bOutput;
    } else {
        targetAngle = mProtoActionEnd.at(sActionName);
    }

    yInfo() << " Basic angle position for action " << sActionName << " is " << targetAngle ;

    //2. if yes, check if bodypart is known. Warning if not found for generalization
    int effectAngleBodyPart = 0 ;
    if ( mBodyPartEnd.find(sBodyPartName) == mBodyPartEnd.end() ) {
        yWarning() << " warning in learnPrimitive::basicCommand | for " << sBodyPartName << " | not protoaction effect define, GENERALIZATION MODE then";
    } else {
        effectAngleBodyPart = mBodyPartEnd.at(sBodyPartName.c_str());
        yInfo() << " effect of the bodypart : " <<  effectAngleBodyPart ;
    }

    //3. add protoaction with bodypart effect. Careful, Angle is in percentage of max angle
    yInfo() << " Target Angle Final (percentage) : " << (targetAngle + effectAngleBodyPart) ;
    int finalTargetAngle = (targetAngle + effectAngleBodyPart)*maxAngle/100;
    yInfo() << " Target Angle Final : " << finalTargetAngle ;
    //TODOOOOOOOOOOOOOOOOOOOOOOOOOOO : For now use maxangle provided but should extract it or provided in the OPC bodypart!

    iCub->opc->checkout();
    //should check at some point that the bodypart is there and loaded no?
    Bodypart* bp = dynamic_cast<Bodypart*>(iCub->opc->getEntity(sBodyPartName));
    int joint = bp->m_joint_number ;

    //4. execute action.
    Bottle bToArm;
    bToArm.addString("set");
    bToArm.addString("pos");
    bToArm.addInt(joint);
    bToArm.addInt(finalTargetAngle);

    yInfo() << " cmd sent : " << bToArm.toString();

    portToArm.write(bToArm, bOutput);

    return bOutput;
}

bool learnPrimitive::updateProtoAction(ResourceFinder &rf){
    Bottle bOutput;

    //1. Protoaction
    Bottle bProtoAction = rf.findGroup("Proto_Action");

    if (!bProtoAction.isNull())
    {
        Bottle * bProtoActionName = bProtoAction.find("protoActionName").asList();
        Bottle * bProtoActionEnd = bProtoAction.find("protoActionEnd").asList();
        Bottle * bProtoActionSpeed = bProtoAction.find("protoActionSpeed").asList();

       if(bProtoActionName->isNull() || bProtoActionEnd->isNull() || bProtoActionSpeed->isNull()){
           yError() << " [updateProtoAction] : one of the protoAction conf is null : protoActionName, protoActionEnd, protoActionSpeed" ;
            return false ;
        }


        int protoActionSize = -1 ;
        if(bProtoActionName->size() == bProtoActionEnd->size() && bProtoActionEnd->size() == bProtoActionSpeed->size()){
            protoActionSize =  bProtoActionName->size() ;
        } else {
            yError() << " [updateProtoAction] : one of the protoAction conf has different size!" ;
            return false ;
        }

        if(protoActionSize == 0) {
            yWarning() << " [updateProtoAction] : there is no protoaction defined at startup!" ;
        } else {

            for(int i = 0; i < protoActionSize ; i++) {

                //insert protoaction even if already there
                yInfo() << "ProtoAction added : (" << bProtoActionName->get(i).asString() << ", " << bProtoActionEnd->get(i).asInt() << ", " << bProtoActionSpeed->get(i).asDouble() << ")" ;
                mProtoActionEnd[bProtoActionName->get(i).asString()] = bProtoActionEnd->get(i).asInt();
                mProtoActionSpeed[bProtoActionName->get(i).asString()] = bProtoActionSpeed->get(i).asDouble();
            }
        }
    } else {
        yError() << " error in learnPrimitive::updateProtoAction | Proto_Action is NOT defined in the learnPrimitive.ini";
        return false;
    }

    //2. Effect of bodypart
    Bottle bBodyPart = rf.findGroup("BodyPart");

    if (!bBodyPart.isNull())
    {
        Bottle * bBodyPartName = bBodyPart.find("bodyPartName").asList();
        Bottle * bBodyPartEnd = bBodyPart.find("bodyPartProtoEnd").asList();
        Bottle * bBodyPartSpeed = bBodyPart.find("bodyPartProtoSpeed").asList();
        
        //Crash if no match : isnull is not good for that

        if(bBodyPartName->isNull() || bBodyPartEnd->isNull() || bBodyPartSpeed->isNull()){
            yError() << "[updateProtoAction] : one of the bodyPartProto conf is null : bodyPartName, bodyPartProtoEnd, bodyPartProtoSpeed" ;
            return false ;
        }


        int bodyPartSize = -1 ;
        if(bBodyPartName->size() == bBodyPartEnd->size() && bBodyPartEnd->size() == bBodyPartSpeed->size()){
            bodyPartSize =  bBodyPartName->size() ;
        } else {
            yError() << "[updateProtoAction] : one of the bodyPartProto conf has different size!" ;
            return false ;
        }

        if(bodyPartSize == 0) {
            yWarning() << "[updateProtoAction] : there is no bodyPartProto defined at startup!" ;
        } else {
            for(int i = 0; i < bodyPartSize ; i++) {

                //insert protoaction even if already there
                yInfo() << "bodyPartProto added : (" << bBodyPartName->get(i).asString() << ", " << bBodyPartEnd->get(i).asInt() << ", " << bBodyPartSpeed->get(i).asDouble() << ")" ;
                mBodyPartEnd[bBodyPartName->get(i).asString()] = bBodyPartEnd->get(i).asInt();
                mBodyPartSpeed[bBodyPartName->get(i).asString()] = bBodyPartSpeed->get(i).asDouble();
            }
        }
    } else {
        yError() << " error in learnPrimitive::updateProtoAction | BodyPart is NOT defined in the learnPrimitive.ini";
        return false;
    }

    return true;
}

/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string learnPrimitive::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    cout << "path is: " << sPath << endl;

    if (!isGrammar)
    {
        cout << "Error in proactiveTagging::grammarToString. Couldn't open file : " << sPath << "." << endl;
        return "Error in proactiveTagging::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}



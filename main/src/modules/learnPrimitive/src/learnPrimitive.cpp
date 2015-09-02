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
    GrammarDescribeAction  = rf.findFileByName(rf.check("GrammarDescribeAction", Value("GrammarDescribeAction.xml")).toString());

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "learnPrimitive", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

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
        string sActionName   = command.get(1).asString() ;
        string sBodypartName   = command.get(2).asString() ;

        reply = basicCommand(sActionName, sBodypartName);
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


Bottle learnPrimitive::basicCommand(string sActionName, string sBodypartName){
    Bottle bOutput;

    //1. check if proto is known

    //2. if yes, check if bodypart is known

    //3.a if yes : add protoaction with bodypart effect

    //3.b if no, ask if should generalize and check only the protoaction value without correction

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
            yError() << "ERROOOOOOOOOOOOOOOOOOOOOOOOOR ELEMENT NULL TODO" ;
            return false ;
        }


        int protoActionSize = -1 ;
        if(bProtoActionName->size() == bProtoActionEnd->size() && bProtoActionEnd->size() == bProtoActionSpeed->size()){
            protoActionSize =  bProtoActionName->size() ;
        } else {
            yError() << "ERRORRRRRRRRRRRRRRRRRRRRRRR TODO" ;
            return false ;
        }

        if(protoActionSize == 0) {
            yWarning() << "No protoaction !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" ; 
        } else {

            for(int i = 0; i < protoActionSize ; i++) {

                //TODOOOOOOO : check the key is not existing!!!
                mProtoActionEnd[bProtoActionName->get(i).asString()] = bProtoActionEnd->get(i).asInt();
                mProtoActionSpeed[bProtoActionName->get(i).asString()] = bProtoActionSpeed->get(i).asDouble();
            }
        }
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
            yError() << "ERROOOOOOOOOOOOOOOOOOOOOOOOOR ELEMENT NULL TODO" ;
            return false ;
        }


        int bodyPartSize = -1 ;
        if(bBodyPartName->size() == bBodyPartEnd->size() && bBodyPartEnd->size() == bBodyPartSpeed->size()){
            bodyPartSize =  bBodyPartName->size() ;
        } else {
            yError() << "ERRORRRRRRRRRRRRRRRRRRRRRRR TODO" ;
            return false ;
        }

        if(bodyPartSize == 0) {
            yWarning() << "No bodypart !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" ; 
        } else {
            for(int i = 0; i < bodyPartSize ; i++) {

                //TODOOOOOOO : check the key is not existing!!! or use insert?
                mBodyPartEnd[bBodyPartName->get(i).asString()] = bBodyPartEnd->get(i).asInt();
                mBodyPartSpeed[bBodyPartName->get(i).asString()] = bBodyPartSpeed->get(i).asDouble();
            }
        }
    }

    return true;
}



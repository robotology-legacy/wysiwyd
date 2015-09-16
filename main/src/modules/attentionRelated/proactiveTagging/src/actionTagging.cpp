/* 
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau, Tobias Fischer, Maxime Petit
 * email:   greg.pointeau@gmail.com, t.fischer@imperial.ac.uk, m.petit@imperial.ac.uk
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later versions published by the Free Software Foundation.
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

/*
* Test send ARE command (1 param) + describe sentence => point mouse
* input: todo
* todo
*/
Bottle proactiveTagging::describeBabbling(string sJointName, int joint) {
    Bottle bOutput, bSingleJoint;

    yInfo() << " sJointName : " << sJointName;

    //Check if name is known or not. if yes, and body part : ask tactile

    //Ask question for the human, or ask to pay attention (if action to focus attention after)
    string sQuestion = "I will move my " + sJointName + " finger. Can you describe the proto-actions I will do?" ;

    //TODO : choose between say and TTS. say put stuff in ABM, TTS?
    yInfo() << sQuestion;
    iCub->say(sQuestion);
    iCub->opc->checkout();

    Bottle bHand("left");
    yInfo() << "Sending command : Babbling " << sJointName ;

    //iCub->point(sNameTarget, bHand, false); //false for shouldWait = False : will continue to the looping describe
    //bottle to move single joint
    bSingleJoint.addString("singleJointBabbling");

    //9 11 13 15
    bSingleJoint.addInt(joint);

    portNoWaitToBodySchema.prepare() = bSingleJoint ;
    portNoWaitToBodySchema.writeStrict() ;


    //------------------------------------------------- Loop Speech Recog -------------------------------------------

    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
    bAnswer; //response from speech recog without transfer information, including raw sentence

    yInfo() << "before do loop" ;
    //Load the Speech Recognition with grammar according to entityType
    do {
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarDescribeAction), 20);
        bAnswer = *bRecognized.get(1).asList();

        yInfo() << "----> Sentence type = " << bAnswer.get(1).asList()->get(0).toString();
    } while (bAnswer.get(1).asList()->get(0).toString() != "stop");

    //iCub->home();

    /*bSemantic = *bAnswer.get(1).asList();
    string sName;
    if(entityType == "agent") {
        sName = bSemantic.check("agent", Value("unknown")).asString();
    } else if(entityType == "object" || entityType == "rtobject") {
        sName = bSemantic.check("object", Value("unknown")).asString();
    } else if(entityType == "bodypart") {
        sName = bSemantic.check("fingerName", Value("unknown")).asString();
    } else {
        yError("recogName ERROR entitytype not known!");
    }*/


    yInfo() << "Out of the do loop" ;
    bOutput.addString("success");


    return bOutput;
}


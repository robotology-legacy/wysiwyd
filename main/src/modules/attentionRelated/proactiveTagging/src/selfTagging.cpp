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

/*
* Send a rpc command to BodySchema to move a single joint
* input: joint number to be moved + body part
* ask through speech the name of an unknwon entity
*/
Bottle proactiveTagging::moveJoint(int joint, string sBodyPart) {

    Bottle bBodyPart, bSingleJoint, bOutput;

    //TODO : bodySchema should be able to change bodyPart on the fly. Or provide the bodyPart activated to allow or not moving

    //1. prepare first Bottle to change bodyPart
    //bBodyPart.addString("XXX");
    //bBodyPart.addString(sBodyPart);
    //2. Send bodyPart
    //portToBodySchema.write(bBodyPart, bOutput);

    /*TODO : check reply
    if(bOutput.get(0).asString() == "nack"){
        return bOutput ;
    }*/

    bOutput.clear();

    //3. prepare second Bottle to move the single joint
    bSingleJoint.addString("singleJointBabbling");
    bSingleJoint.addInt(joint);

    //4. send single joint moving bottle
    portToBodySchema.write(bSingleJoint, bOutput);
    yDebug() << "Reply from bodySchema:" << bOutput.toString();

    //bOutput == "nack" if something goes wrong, "ack" otherwise

    return bOutput;

}


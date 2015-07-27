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
* input: joint number to be moved + body part type (e.g. left_arm, right_arm, ...)
* ask through speech the name of an unknwon bodypart entity
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

/*
* Send a rpc command to ABM to obtain the kinematicStructure of part of the body (assuming it has already been named)
* Send a rpc command to kinematicStructure if no kinematicStructure were provided
* input: name of the bodypart to be moved (e.g. index, thumb, ...) + body part type (e.g. left_arm, right_arm, ...)
*/
Bottle proactiveTagging::assignKinematicStructureByName(std::string sName, std::string sBodyPartType, bool forcingKS) {

    Bottle bOutput ;

    //1. search through opc for the m_joint_number corresponding to the bodypart name
    Entity* e = iCub->opc->getEntity(sName);

    //Error if the name does NOT correspond to a bodypart
    Bodypart* BPentity ;
    if(!e->isType("bodypart")) {
        yError() << " error in proactiveTagging::assignKinematicStructureByName | for " << sName << " | " << sName << " is NOT a bodypart : no kinematicStructure are allowed!" ;
        bOutput.addString("error");
        bOutput.addString("NOT a bodypart : no kinematicStructure are allowed!");
        return bOutput;
    } else {
        BPentity = dynamic_cast<Bodypart*>(iCub->opc->getEntity(sName));
    }
    int BPjoint = BPentity->m_joint_number ;

    //2. go through ABM to find a singleJointBabbling with the corresponding joint, retrieve the instance number
    Bottle bResultByJoint = assignKinematicStructureByJoint(BPjoint, sBodyPartType, forcingKS);

    if(bResultByJoint.get(0).asString() == "error") {
        return bResultByJoint ;
    } else {
        yInfo() << " [assignKinematicStructureByName] | for name " << sName << " | instance found : " << bResultByJoint.get(1).asInt() ;
        bOutput.addString("ack");
        bOutput.addInt(bResultByJoint.get(1).asInt());
    }

    return bOutput ;
}


/*
* Send a rpc command to ABM to obtain the kinematicStructure of part of the body (using the joint info)
* Send a rpc command to kinematicStructure if no kinematicStructure were provided
* input: joint of the bodypart to be moved (e.g. m_joint_number) + body part type (e.g. left_arm, right_arm, ...)
*/
Bottle proactiveTagging::assignKinematicStructureByJoint(int BPjoint, std::string sBodyPartType, bool forcingKS){


    //1. extract instance from singleJointAction for joint BPjoint, from ABM
    Bottle bResult, bOutput;
    ostringstream osRequest;
    osRequest << "SELECT max(instance) FROM main, contentarg WHERE activitytype = 'singleJointBabbling' AND contentarg.type = 'limb' AND contarg.value = '" << BPjoint << "' ;";
    bResult = iCub->getABMClient()->requestFromString(osRequest.str().c_str());

    int ksInstance ;
    if (bResult.toString() != "NULL") {
        ksInstance = atoi(bResult.get(0).asList()->get(0).toString().c_str());
    } else {
        yError() << " error in proactiveTagging::assignKinematicStructureByName | for joint " << BPjoint << " | No instance corresponding to singleJointBabbling for this part" ;
        bOutput.addString("error");
        bOutput.addString("No instance corresponding to singleJointBabbling for this part");
        return bOutput;
    }

    Bottle bResultCheckKS = checkForKinematicStructure(ksInstance, forcingKS);
    if(bResultCheckKS.get(0).asString() == "error") {
        return bResultCheckKS ;
    } else {
        yInfo() << " [assignKinematicStructureByJoint] | for joint " << BPjoint << " | instance found : " << ksInstance ;

    }

    //WRITE IN OPC : TODOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

    bOutput.addString("ack");
    bOutput.addInt(ksInstance);
    return bOutput ;
}




/*
* Check if the instance has KS, other order it if forcing in ON
* input: instance to check, bool forcingKS to allow or not KS generation
*/
Bottle proactiveTagging::checkForKinematicStructure(int instance, bool forcingKS) {

    //1. Check that the instance number has some augmented kinematicStructure images
    Bottle bOutput, bResult;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main, visualdata WHERE main.instance = visualdata.instance AND augmented = 'kinematic_structure' ;";
    bResult = iCub->getABMClient()->requestFromString(osRequest.str().c_str());


    //2.a if yes, assign it to the bodypart in the opc
    //ELSE
    // 2.b i) launch kinematicStructure if forcingKS = true, ii) go out with error/warning otherwise
    if (bResult.toString() == "NULL") {    
        if (!forcingKS) { //Instance with no KS, no forcingKS : send an error
            yError() << " error in proactiveTagging::checkForKinematicStructure | for instance " << instance << " | No instance corresponding to singleJointBabbling for this part (forcingKS = false, no attempt to launch it)" ;
            bOutput.addString("error");
            bOutput.addString("No instance corresponding to singleJointBabbling for this part (forcingKS = false, no attempt to launch it)");
            return bOutput;
        } else { //Instance with no KS, forcingKS, launch and try again
            yWarning() << " error in proactiveTagging::checkForKinematicStructure | for insance " << instance << " | No instance corresponding to singleJointBabbling for this part (forcingKS = true, attentoing to launch it, this may take some time)" ;
            Bottle bResultKS = orderKinematicStructure(instance);
            if (bResultKS.get(0).asString() == "error") {
                return bResultKS;               
            } else {
                checkForKinematicStructure(instance, forcingKS); //recursive : try again if KS generation was fine
            }
        }
    }

    bOutput.addString("ack") ;
    return bOutput ;


}

/*
* Launch a kinematicStructure generation for the instance
* input: instance for the KS generation
*/
Bottle proactiveTagging::orderKinematicStructure(int instance) {
    Bottle bOutput;


    return bOutput ;
}


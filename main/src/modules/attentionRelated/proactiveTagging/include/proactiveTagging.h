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

#include "wrdac/clients/icubClient.h"
#include "wrdac/subsystems/subSystem_recog.h"
#include "wrdac/subsystems/subSystem_speech.h"

class proactiveTagging : public yarp::os::RFModule {
private:

    wysiwyd::wrdac::ICubClient  *iCub;

    double      period;

    std::string bodySchemaRpc;
    std::string SAMRpc;
    std::string LRHRpc;
    std::string touchDetectorRpc;

    yarp::os::Port   rpcPort;
    yarp::os::Port   portToSAM;
    yarp::os::Port   portToPasar;
    yarp::os::BufferedPort<yarp::os::Bottle>   portFromTouchDetector;
    yarp::os::BufferedPort<yarp::os::Bottle>   portNoWaitToBodySchema;

    std::string      GrammarAskNameAgent;
    std::string      GrammarAskNameObject;
    std::string      GrammarAskNameBodypart;
    std::string      GrammarDescribeAction;
	std::string      GrammarYesNo;

    std::string      babblingArm; //side of the babbling arm : left or right
    std::string      defaultPartnerName; //default name of the partner: considered NOT named

    double  thresholdDistinguishObjectsRatio; //ratio of saliency needed to detect if 1 object is more salient that the other
    double  thresholdSalienceDetection; //value of saliency needed to detect if 1 object is more salient that the other

    yarp::os::Bottle   recogName(std::string entityType);
	
	bool recogYesNo();
	
    //Configure
    void configureOPC(yarp::os::ResourceFinder &rf);
    void subPopulateObjects(yarp::os::Bottle* objectList, bool addOrRetrieve);
    void subPopulateBodyparts(yarp::os::Bottle* bodyPartList, yarp::os::Bottle* bodyPartJointList, bool addOrRetrieve);

    //objectTagging
    yarp::os::Bottle  exploreUnknownEntity(const yarp::os::Bottle &bInput);
    yarp::os::Bottle  searchingEntity(const yarp::os::Bottle &bInput);


    //selfTagging.cpp
    yarp::os::Bottle assignKinematicStructureByName(std::string sName, std::string sBodyPartType, bool forcingKS = false);
    yarp::os::Bottle assignKinematicStructureByJoint(int joint, std::string sBodyPartType, bool forcingKS = false);
    yarp::os::Bottle checkForKinematicStructure(int instance, bool forcingKS = false);
    yarp::os::Bottle orderKinematicStructure(int instance);

    yarp::os::Bottle exploreTactileEntityWithName(yarp::os::Bottle bInput);

    //actionTagging
    yarp::os::Bottle describeBabbling(std::string sJointName, int jointNumber); // to change extract the joint from the name

    bool setPasarPointing(bool on);
    std::string getBestEntity(std::string sTypeTarget);
    yarp::os::Bottle getNameFromSAM(std::string sNameTarget, std::string currentEntityType);

public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    //RPC & scenarios
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
};

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

class proactiveTagging : public yarp::os::RFModule {
private:

    wysiwyd::wrdac::ICubClient  *iCub;

    double      period;

    std::string      grammarToString(std::string sPath);
    yarp::os::Port   rpcPort;
    yarp::os::Port   portToBodySchema;

    std::string      GrammarAskNameAgent;
    std::string      GrammarAskNameObject;
    std::string      GrammarYesNo;

    double  thresholdDistinguishObjectsRatio; //ratio of saliency needed to detect if 1 object is more salient that the other
    double  thresholdSalienceDetection; //value of saliency needed to detect if 1 object is more salient that the other

    void        checkRelations();

    std::string        askManner(std::string agent, std::string verb, std::string object);
    yarp::os::Bottle   recogName(std::string entityType);

    //Configure
    void configureOPC(yarp::os::ResourceFinder &rf);

    //selfTagging.cpp
    yarp::os::Bottle moveJoint(int joint, std::string sBodypart) ;

public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule()
    {
        rpcPort.interrupt();
        return true;
    }

    bool close();

    double getPeriod()
    {
        return period;
    }


    yarp::os::Bottle  exploreUnknownEntity(yarp::os::Bottle bInput);
    yarp::os::Bottle  exploreEntityByName(yarp::os::Bottle bInput);

    bool updateModule();

    //RPC & scenarios
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
};

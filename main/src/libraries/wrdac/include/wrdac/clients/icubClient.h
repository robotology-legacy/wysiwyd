/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com
 * website: http://efaa.upf.edu/ 
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

#ifndef __EFAA_ICUBCLIENT_H__
#define __EFAA_ICUBCLIENT_H__
#include <fstream>
#include <yarp/os/Network.h>
#include <wrdac/clients/opcClient.h>
#include "wrdac/subsystems/all.h"
#include "animation.h"

namespace wysiwyd{namespace wrdac{


/**
* \ingroup wrdac_clients
*
* Provide a compact way to access the iCub functionalities within the EFAA framework.
*
* Grants access to high level motor commands (grasp, touch, look, goto, etc) of the robot as well as its internal state
* (drives, emotions, beliefs) and its interaction means (speech).
*/
class ICubClient
{
private:
    std::map<std::string, SubSystem*>  subSystems;
    bool                               isFullyConnected;
    std::list<Action*>                 actionsKnown;
    std::map<std::string, BodyPosture> posturesKnown;
    std::map<std::string, std::list< std::pair<std::string, double> > > choregraphiesKnown;

    //Reachability area
    double xRangeMin,yRangeMin,zRangeMin;
    double xRangeMax,yRangeMax,zRangeMax;

public:
    std::map<std::string, BodyPosture> getPosturesKnown()
    {
        return posturesKnown;
    }

    SubSystem*  getSubSystem(const std::string &name){return subSystems[name];}

    SubSystem_Expression* getExpressionClient()
    {         
        if (subSystems.find(SUBSYSTEM_EXPRESSION) == subSystems.end())
            return NULL;
       else
           return ((SubSystem_Expression*) subSystems[SUBSYSTEM_EXPRESSION]);
    } 

    SubSystem_Reactable* getReactableClient() 
    {         
        if (subSystems.find(SUBSYSTEM_REACTABLE) == subSystems.end())
            return NULL;
       else
           return (SubSystem_Reactable*) subSystems[SUBSYSTEM_REACTABLE];
    } 

    SubSystem_iKart* getIkartClient() 
    {         
        if (subSystems.find(SUBSYSTEM_IKART) == subSystems.end())
            return NULL;
       else
           return (SubSystem_iKart*) subSystems[SUBSYSTEM_IKART];
    }

    SubSystem_ABM* getABMClient() 
    {         
        if (subSystems.find(SUBSYSTEM_ABM) == subSystems.end())
            return NULL;
       else
           return (SubSystem_ABM*) subSystems[SUBSYSTEM_ABM];
    }

    SubSystem_SlidingController* getSlidingController()
    {         
        if (subSystems.find(SUBSYSTEM_SLIDING_CONTROLLER) == subSystems.end())
            return NULL;
       else
           return (SubSystem_SlidingController*)subSystems[SUBSYSTEM_SLIDING_CONTROLLER];
    }

    SubSystem_ARE* getARE()
    {
        if (subSystems.find(SUBSYSTEM_ARE) == subSystems.end())
            return NULL;
        else
            return (SubSystem_ARE*)subSystems[SUBSYSTEM_ARE];
    }

    SubSystem_Speech* getSpeechClient() 
    {         
        if (subSystems.find(SUBSYSTEM_SPEECH) == subSystems.end())
            if (subSystems.find(SUBSYSTEM_SPEECH_ESPEAK) == subSystems.end())
                return NULL;
           else
               return (SubSystem_Speech*) subSystems[SUBSYSTEM_SPEECH_ESPEAK];
       else
           return (SubSystem_Speech*) subSystems[SUBSYSTEM_SPEECH];
    }

    OPCClient*                  opc;
    Agent*                      icubAgent;
        
    /**
    * Create an iCub client based on a specific RF
    */ 
    ICubClient(const std::string &moduleName, yarp::os::ResourceFinder &rf);


    /**
    * Create an iCub client
    * @param moduleName The port namespace that will precede the client ports names.
    */ 
    ICubClient(const std::string &moduleName,const std::string &context="icubClient/conf", const std::string &clientConfigFile="client.ini", bool isRFVerbose = false);

    /**
    * Load a library of postures from config file specified in rf
    */ 
    void LoadPostures(yarp::os::ResourceFinder &rf);

    /**
    * Load a library of choregraphies from config file specified in rf
    */ 
    void LoadChoregraphies(yarp::os::ResourceFinder &rf);

    /**
    * Try to connect all functionalities.
    * @return true in case of success false if some connections are missing.
    */ 
    bool connect();
        
    /**
    * Retrieve fresh definition of the iCub agent from the OPC
    */ 
    void updateAgent();

    /**
    * Commit the local definition of iCub agent to the OPC
    */ 
    void commitAgent();

    /**
    * Navigate to a place with a given name.
    * @param place is the name of the entity in the OPC where the robot should go.
    * @return true in case of successfull navigation, false either (Entity non existing, impossible to reach, etc.).
    */ 
    bool goTo(const std::string &place);
     
    /**
    * Move the body to a given posture if it is known
    */ 
    bool moveToPosture(const std::string &name, double time);

    /**
    * Move a part of the body to a given posture if it is known
    */ 
    bool moveBodyPartToPosture(const std::string &name, double time, const std::string &bodyPart);

    /**
    * Replay a known choregraphy
    */ 
    bool playChoregraphy(const std::string &name, double speedFactor = 1.0,  bool isBlocking=true);
   
    /**
    * Replay a known choregraphyWith only a specific body part
    */ 
    bool playBodyPartChoregraphy(const std::string &name, const std::string &bodyPart, double speedFactor = 1.0, bool isBlocking=true );
    
    /**
    * Get the duration of a choregraphy given a specific speedFactor
    */ 
    double getChoregraphyLength(const std::string &name, double speedFactor = 1.0 );

    /**
    * Grasp an object with a position and a list of way points
    * @param target is the position.
    * @param waypoints is a list of waypoints, relative to the target position.
    * @param shouldWait is the function blocking?
    * @return true in case of successfull motor command, false either.
    */
    bool grasp(yarp::sig::Vector &target, const std::string &usedHand = "right", std::list<yarp::sig::Vector>* waypoints = NULL );

    /**
    * Grasp an object with a given name
    * @param oName is the name of the entity in the OPC that the robot should grasp.
    * @param usedHand NOT IMPLEMENTED!! the hand to be used. Right by default.
    * @param wait is the function blocking ?
    * @param controlGaze should the robot look at the target?
    * @return true in case of successfull motor command, false either (Entity non existing, impossible to reach, etc.).
    */ 
    bool sideGrasp(const std::string &oName, const std::string &usedHand = "right", bool wait = true, bool controlGaze = true);

    /**
    * Release the content of a hand on a given location
    * @param oName is the name of the entity in the OPC where the robot should release.
    * @return true in case of success navigation, false either (Entity non existing, impossible to reach, etc.).
    */ 
    bool release(const std::string &oLocation, const std::string &usedHand = "right");
       
    /**
    * Start tracking a given entity
    * @param target is the name of the entity in the OPC where the robot should look.
    */ 
    bool look(const std::string &target);
    
    /**
    * Start tracking randomly objects in the field of view
    */ 
    bool lookAround();

    /**
    * Pause the attention control. Allowing the head to be controlled.
    */ 
    bool lookStop();

    /**
    * Ask the robot to perform speech synthesis of a given sentence
    * @param text to be said.
    */ 
    bool say(const std::string &text, bool shouldWait= true, bool emotionalIfPossible = false, const std::string &overrideVoice = "default");

    /**
    * Ask the robot to perform speech recognition of a given sentence/grammar
    * @param timeout Timeout. If -1 the robot will wait until a sentence is recognized.
    * @return the sentence heard
    */ 
    yarp::os::Bottle hear(const std::string &grammar, double timeout = -1.0);

    /**
    * Ask the robot to execute a generic action, that can be composite
    * @param what The action to be executed.
    * @param applyEstimatedDriveEffect Should the iCub automatically modify its drives based on its estimation? False by default.
    * @return true in case of success, false either (Entity non existing, impossible to reach, etc.).
    */ 
    bool execute(Action &what, bool applyEstimatedDriveEffect = false);
       
    /**
    * Get the strongest emotion
    */        
    void getHighestEmotion(std::string &emotionName, double &intensity);

    /**
    * Get the list of actions known the iCub
    */   
    std::list<Action*>  getKnownActions();

    /**
    * Get the list of object that are in front of the iCub
    * Warning: this will update the local icubAgent
    */ 
    std::list<Object*> getObjectsInSight();
        
    /**
    * Get the list of objects that are graspable by the iCub
    * Warning: this will update the local icubAgent
    */ 
    std::list<Object*> getObjectsInRange();

    /**
    * Check if a given cartesian position is within the reach of the robot
    */ 
    bool isTargetInRange(const yarp::sig::Vector &target);

    /**
    * Closes properly ports opened. 
    */ 
    void close();
};


}}//Namespace
#endif



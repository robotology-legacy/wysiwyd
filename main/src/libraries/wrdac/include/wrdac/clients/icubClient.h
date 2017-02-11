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

#include <iostream>
#include <iomanip>
#include <fstream>

#include <yarp/os/Network.h>

#include "wrdac/clients/opcClient.h"
#include "wrdac/subsystems/all.h"
#include "animation.h"

namespace wysiwyd{
    namespace wrdac{


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
            bool                               closed;
            std::list<Action*>                 actionsKnown;
            std::map<std::string, BodyPosture> posturesKnown;
            std::map<std::string, std::list< std::pair<std::string, double> > > choregraphiesKnown;

            //Reachability area
            double xRangeMin, yRangeMin, zRangeMin;
            double xRangeMax, yRangeMax, zRangeMax;

            std::string robot;  // Name of robot           

        public:

            std::map<std::string, BodyPosture> getPosturesKnown()
            {
                return posturesKnown;
            }

            bool addPosture(const std::string &postureName, BodyPosture postureJnts)
            {
                posturesKnown[postureName] = postureJnts;
                return true;
            }

            void savePostures(const std::string &fileName = "defaultPostures.ini")
            {
                std::cout << "Saving postures...";
                std::ofstream of(fileName.c_str());
                of << "posturesCount \t" << posturesKnown.size() << std::endl;
                int pCnt = 0;
                for (std::map<std::string, BodyPosture>::iterator it = posturesKnown.begin(); it != posturesKnown.end(); it++)
                {
                    of << "[posture_" << pCnt << "]" << std::endl;
                    of << "name \t" << it->first << std::endl;
                    of << "head \t (" << it->second.head.toString(3, 3).c_str() << ")" << std::endl;
                    of << "right_arm \t (" << it->second.right_arm.toString(3, 3).c_str() << ")" << std::endl;
                    of << "left_arm \t (" << it->second.left_arm.toString(3, 3).c_str() << ")" << std::endl;
                    of << "torso \t (" << it->second.torso.toString(3, 3).c_str() << ")" << std::endl;
                    pCnt++;
                }
                of.close();
                std::cout << "Done." << std::endl;
            }

            SubSystem*  getSubSystem(const std::string &name){ return subSystems[name]; }
            SubSystem_Expression* getExpressionClient();
            SubSystem_Reactable* getReactableClient();
            SubSystem_iKart* getIkartClient();
            SubSystem_ABM* getABMClient();
            SubSystem_IOL2OPC* getIOL2OPCClient();
            SubSystem_Recog* getRecogClient();
            SubSystem_SlidingController* getSlidingController();
            SubSystem_ARE* getARE();
            SubSystem_LRH* getLRH();
            SubSystem_Speech* getSpeechClient();
            SubSystem_KARMA* getKARMA();

            OPCClient*                  opc;
            Agent*                      icubAgent;

            /**
            * Create an iCub client
            * @param moduleName The port namespace that will precede the client ports names.
            */
            ICubClient(const std::string &moduleName, const std::string &context = "icubClient",
                const std::string &clientConfigFile = "client.ini", bool isRFVerbose = false,
                bool bLoadChore = false, bool bLoadPostures = false);

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
            * @param opcName the stem-name of the OPC server.
            * @return true in case of success false if some connections are missing.
            */
            bool connect(const std::string &opcName = "OPC");

            /**
            * Try to connect to OPC
            * @param opcName the stem-name of the OPC server.
            * @return true on success.
            */
            bool connectOPC(const std::string &opcName = "OPC");

            /**
            * Try to connect to sub-systems.
            * @return true on success.
            */
            bool connectSubSystems();

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
            bool playChoregraphy(const std::string &name, double speedFactor = 1.0, bool isBlocking = true);

            /**
            * Replay a known choregraphyWith only a specific body part
            */
            bool playBodyPartChoregraphy(const std::string &name, const std::string &bodyPart,
                double speedFactor = 1.0, bool isBlocking = true);

            /**
            * Get the duration of a choregraphy given a specific speedFactor
            */
            double getChoregraphyLength(const std::string &name, double speedFactor = 1.0);

            /**
            * Go in home position.
            * @param part the part to be homed ("gaze", "head", "arms",
            *             "fingers", "all"; "all" by default).
            * @return true in case of successfull motor command, false
            *         otherwise (Entity non existing, impossible to reach,
            *         etc.).
            */
            bool home(const std::string &part = "all");

            /**
            * Grasp an object.
            * @param oName is the name of the entity in the OPC that the robot should grasp.
            * @param options bottle containing a list of options (e.g. force
            *                to use specific hand with "left"|"right"
            *                option; grasp type such as "above", "side").
            * @return true in case of successfull motor command, false
            *         otherwise (Entity non existing, impossible to reach,
            *         not grasped, etc.).
            */
            bool grasp(const std::string &oName, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Release the hand-held object on a given location.
            * @param oName is the name of the entity in the OPC where the robot should release.
            * @param options bottle containing a list of options (e.g. force
            *                to use specific hand with "left"|"right"
            *                option).
            * @return true in case of success release, false otherwise
            *         (Entity non existing, impossible to reach, etc.).
            */
            bool release(const std::string &oLocation, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Release the hand-held object on a given location.
            * @param target contains spatial information about the location
            *               where releasing the object.
            * @param options bottle containing a list of options (e.g. force
            *                to use specific hand with "left"|"right"
            *                option).
            * @return true in case of success release, false otherwise
            *         (Entity non existing, impossible to reach, etc.).
            */
            bool release(const yarp::sig::Vector &target, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Point at a specified location far from the iCub.
            * @param oName is the name of the entity in the OPC where the
            *              robot should point at.
            * @param options bottle containing a list of options (e.g. force
            *                to use specific hand with "left"|"right"
            *                option).
            * @return true in case of success release, false otherwise
            *         (Entity non existing, impossible to reach, etc.).
            */
            bool pointfar(const std::string &oLocation, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Point at a specified location far from the iCub.
            * @param target contains spatial information about the location
            *               where pointing at. The target can be far away from the iCub
            * @param options bottle containing a list of options (e.g. force
            *                to use specific hand with "left"|"right"
            *                option).
            * @return true in case of success release, false otherwise
            *         (Entity non existing, impossible to reach, etc.).
            */
            bool pointfar(const yarp::sig::Vector &target, const yarp::os::Bottle &options = yarp::os::Bottle(), std::string sName = "target");

            /**
            * Point at a specified location.
            * @param oName is the name of the entity in the OPC where the
            *              robot should point at.
            * @param options bottle containing a list of options (e.g. force
            *                to use specific hand with "left"|"right"
            *                option).
            * @return true in case of success release, false otherwise
            *         (Entity non existing, impossible to reach, etc.).
            */
            bool point(const std::string &oLocation, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Enable/disable arms waving.
            * @param sw enable/disable if true/false.
            * @return true in case of successfull request, false otherwise.
            */
            bool waving(const bool sw);

            /**
            * Push at a specified location.
            * @param oName is the name of the entity in the OPC where the
            *              robot should push at.
            * @param options bottle containing a list of options (e.g. force
            *                to use specific hand with "left"|"right"
            *                option).
            * @return true in case of success release, false otherwise
            *         (Entity non existing, impossible to reach, etc.).
            */
            bool push(const std::string &oLocation, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
             * @brief pushKarmaLeft: push an object by name to right side
             * @param objName: name of object, which will be looked for in OPC
             * @param targetPosYLeft: Y coordinate of object to push to
             * @param armType: "left" or "right" arm to conduct action, otherwise arm will be chosen by KARMA
             * @param options
             * @return true in case of success release, false otherwise
             */
            bool pushKarmaLeft(const std::string &objName, const double &targetPosYLeft,
                const std::string &armType = "selectable",
                const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
             * @brief pushKarmaRight: push an object by name to right side
             * @param objName: name of object, which will be looked for in OPC
             * @param targetPosYRight: Y coordinate of object to push to
             * @param armType: "left" or "right" arm to conduct action, otherwise arm will be chosen by KARMA
             * @param options
             * @return true in case of success release, false otherwise
             */
            bool pushKarmaRight(const std::string &objName, const double &targetPosYRight,
                const std::string &armType = "selectable",
                const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
             * @brief pushKarmaFront: push an object by name to front
             * @param objName: name of object, which will be looked for in OPC
             * @param targetPosXFront: Y coordinate of object to push to
             * @param armType: "left" or "right" arm to conduct action, otherwise arm will be chosen by KARMA
             * @param options
             * @return true in case of success release, false otherwise
             */
            bool pushKarmaFront(const std::string &objName, const double &targetPosXFront,
                const std::string &armType = "selectable",
                const yarp::os::Bottle &options = yarp::os::Bottle());
            /**
             * @brief pullKarmaBack: pull an object by name back
             * @param objName: name of object, which will be looked for in OPC
             * @param targetPosXBack: Y coordinate of object to pull back
             * @param armType: "left" or "right" arm to conduct action, otherwise arm will be chosen by KARMA
             * @param options
             * @return true in case of success release, false otherwise
             */
            bool pullKarmaBack(const std::string &objName, const double &targetPosXBack,
                const std::string &armType = "selectable",
                const yarp::os::Bottle &options = yarp::os::Bottle());
            /**
             * @brief pushKarmaLeft: push an object to left side, this wrapper simplify pure push action of KARMA
             * @param objName: name of object's center
             * @param objCenter: coordinate of object's center
             * @param targetPosYLeft: Y coordinate of object to push to
             * @param armType: "left" or "right" arm to conduct action, otherwise arm will be chosen by KARMA
             * @param options
             * @param sName: name of object to push
             * @return true in case of success release, false otherwise
             */
            bool pushKarmaLeft(const std::string &objName, const yarp::sig::Vector &objCenter, const double &targetPosYLeft,
                const std::string &armType = "selectable",
                const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
             * @brief pushKarmaRight: push an object to right side, this wrapper simplify pure push action of KARMA
             * @param objName: name of object's center
             * @param objCenter: coordinate of object's center
             * @param targetPosYRight: Y coordinate of object to push to
             * @param armType: "left" or "right" arm to conduct action, otherwise arm will be chosen by KARMA
             * @param options
             * @param sName: name of object to push
             * @return true in case of success release, false otherwise
             */
            bool pushKarmaRight(const std::string &objName, const yarp::sig::Vector &objCenter, const double &targetPosYRight,
                const std::string &armType = "selectable",
                const yarp::os::Bottle &options = yarp::os::Bottle());
            /**
             * @brief pushKarmaFront: push an object to front, this wrapper simplify pure push action of KARMA
             * @param objName: name of object's center
             * @param objCenter: coordinate of object's center
             * @param targetPosXFront: X coordinate of object to push to
             * @param armType: "left" or "right" arm to conduct action, otherwise arm will be chosen by KARMA
             * @param options
             * @param sName: name of object to push
             * @return true in case of success release, false otherwise
             */
            bool pushKarmaFront(const std::string &objName, const yarp::sig::Vector &objCenter, const double &targetPosXFront,
                const std::string &armType = "selectable",
                const yarp::os::Bottle &options = yarp::os::Bottle());
            /**
             * @brief pullKarmaBack: pull an object back, this wrapper simplify pure draw action of KARMA
             * @param objCenter: coordinate of object's center
             * @param targetPosXBack: X coordinate of object to pull back
             * @param armType: "left" or "right" arm to conduct action, otherwise arm will be chosen by KARMA
             * @param options
             * @param sName: name of object to pull
             * @return true in case of success release, false otherwise
             */
            bool pullKarmaBack(const std::string &objName, const yarp::sig::Vector &objCenter, const double &targetPosXBack,
                const std::string &armType = "selectable",
                const yarp::os::Bottle &options = yarp::os::Bottle());
            /**
            * Take at a specified location.
            * @param oName is the name of the entity in the OPC where the
            *              robot should take at.
            * @param options bottle containing a list of options (e.g. force
            *                to use specific hand with "left"|"right"
            *                option).
            * @return true in case of success grasp, false otherwise
            *         (Entity non existing, impossible to reach, etc.).
            */
            bool take(const std::string &oLocation, const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * @brief pushKarma (KARMA): push to certain position, along a direction
            * @param targetCenter: position to push to.
            * @param theta: angle between the y-axis (in robot FoR) and starting position of push action, defines the direction of push action
            * @param radius: radius of the circle with center at @see targetCenter
            * @param options
            * @param sName
            * @return true in case of success release, false otherwise
            */
            bool pushKarma(const yarp::sig::Vector &targetCenter, const double &theta, const double &radius,
                const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * @brief drawKarma (KARMA): draw action, along the positive direction of the x-axis (in robot FoR)
            * @param targetCenter: center of a circle
            * @param theta: angle between the y-axis (in robot FoR) and starting position of draw action.
            * @param radius: radius of the circle with center at @see targetCenter
            * @param dist: moving distance of draw action
            * @param options
            * @param sName
            * @return true in case of success release, false otherwise
            */
            bool drawKarma(const yarp::sig::Vector &targetCenter, const double &theta,
                const double &radius, const double &dist,
                const yarp::os::Bottle &options = yarp::os::Bottle());

            /**
            * Start tracking a given entity
            * @param target is the name of the entity in the OPC where the robot should look.
            * @param options contains options to be passed on the gaze
            *                controller.
            */
            bool look(const std::string &target, const yarp::os::Bottle &options = yarp::os::Bottle());

            bool look(const yarp::sig::Vector &target, const yarp::os::Bottle &options = yarp::os::Bottle(),
                      const std::string &sName="target");

            /**
            * Start tracking randomly objects in the field of view
            */
            bool lookAround();

            /**
            * Pause the attention control. Allowing the head to be controlled.
            */
            bool lookStop();

            /**
            * Looks at the agent if present in the scene.
            */
            bool lookAtPartner();

            /**
            * Looks at the agent specific bodypart if present in the scene.
            * @param sBodypartName is the name of the bodypart (kinect skeleton node) to be looked at
            */
            bool lookAtBodypart(const std::string &sBodypartName);

            /**
            * Extract the name of the agent interaction with the iCub (present, not iCub nor 'unnamed' partner)
            * @return string, the name of the agent
            */
            std::string getPartnerName(bool verbose = true);

            /**
            * Extract the localisation of the bodypart name of the partner
            * @param sBodypartName is the name of the bodypart (kinect skeleton node) to be looked at
            * @return vLoc, the vector of the bodypart localisation
            */
            yarp::sig::Vector getPartnerBodypartLoc(std::string sBodypartName);

            /**
            * Babbling a single joint
            * @param jointNumber contains the int corresponding to an arm joint
            * @param babblingLimb contains the string corresponding to the side of the arm used ("left" or "right")
            * @return true in case of success release, false otherwise
            */
            bool babbling(int jointNumber, const std::string &babblingArm);

            /**
            * Babbling a single joint using the name of a corresponding bodypart
            * @param bpName contains the string with the name of the bodypart
            * @param babblingLimb contains the string corresponding to the side of the arm used ("left" or "right")
            * @return true in case of success release, false otherwise
            *         (bodypart non existing, no joint number assigned, etc.).
            */
            bool babbling(const std::string &bpName, const std::string &babblingArm);


            /**
            * Babbling a single joint with modified duration
            * @param jointNumber contains the int corresponding to an arm joint
            * @param babblingLimb contains the string corresponding to the side of the arm used ("left" or "right")
            * @param train_dur optional param, set the babbling time to the specified double
            * @return true in case of success release, false otherwise
            */
            bool babbling(int jointNumber, const std::string &babblingArm, double train_dur);

            /**
            * Babbling a single joint using the name of a corresponding bodypart with modified duration
            * @param bpName contains the string with the name of the bodypart
            * @param babblingLimb contains the string corresponding to the side of the arm used ("left" or "right")
            * @param train_dur optional param, set the babbling time to the specified double
            * @return true in case of success release, false otherwise
            *         (bodypart non existing, no joint number assigned, etc.).
            */
            bool babbling(const std::string &bpName, const std::string &babblingArm, double train_dur);


            /**
            * Ask the robot to perform speech synthesis of a given sentence
            * @param text to be said.
            */
            bool say(const std::string &text, bool shouldWait = true, bool emotionalIfPossible = false,
                const std::string &overrideVoice = "default", bool recordABM = true, std::string addressee = "none");

            bool changeName(Entity *e, const std::string &newName);

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
            std::list<Action*> getKnownActions();

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
            bool isTargetInRange(const yarp::sig::Vector &target) const;

            /**
            * Closes properly ports opened.
            */
            void close();

            /**
            * Destructor.
            */
            virtual ~ICubClient() { close(); }
        };


    }
}//Namespace
#endif



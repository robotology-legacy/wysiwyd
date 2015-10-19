/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau
 * email:   gregoire.pointeau@inserm.fr
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

// STD
#include <vector>
#include <string>
#include <sstream>


class humanRobotDump : public yarp::os::RFModule {
private:

    wysiwyd::wrdac::ICubClient  *iCub;

    

    yarp::os::Port  rpcPort;
    yarp::os::Port  portInfoDumper;     // port for setting parameters for dumper of the robot information
    yarp::os::Port  DumperPort;         // port to dump information about the human

    void DumpHumanObject();

    bool humanDump;
    bool robotDump;
    std::string sObjectToDump;
    std::string sAgentName;


    // SWS
    bool m_bIsRunning;                      /**<  Whether the thread is running */
    bool m_bInitialized;

    // body parts activated by default
    bool m_bHeadActivatedDefault;                /**< ... */
    bool m_bLeftArmActivatedDefault;               /**< ... */
    bool m_bRightArmActivatedDefault;               /**< ... */
    
    // body parts activated
    bool m_bHeadActivated;                /**< ... */
    bool m_bLeftArmActivated;               /**< ... */
    bool m_bRightArmActivated;               /**< ... */

    int m_i32Fps;                           /**< fps (define the period for calling updateModule) */
    int m_iterator;

    int m_iterator; /* iterator of action dumped */

    // Config variables retrieved from the ini file
    std::string m_sModuleName;              /**< name of the mondule (config) */
    std::string m_sRobotName;               /**< name of the robot (config) */

    
    // sync yarp data port 
    std::string m_sSynchronizedDataPortName;
    yarp::os::BufferedPort<yarp::os::Bottle> m_oSynchronizedDataPort; /**< head yarp tracker port  */


    // head control
    yarp::os::Property m_oHeadOptions;              /**< robot interfaces for head/gaze movements */
    yarp::dev::PolyDriver        m_oRobotHead;      /**< ... */
    yarp::dev::IPositionControl *m_pIHeadPosition;                           /**< arm position control pointer */
    yarp::dev::IEncoders        *m_pIHeadEncoders;  /**< ... */
    yarp::dev::IVelocityControl *m_pIHeadVelocity;
    int m_i32HeadJointsNb;  /**< ... */

    // left arm control
    yarp::os::Property m_oLeftArmOptions;              /**< robot interfaces for head/gaze movements */
    yarp::dev::IEncoders        *m_pILeftArmEncoders;                           /**< arm encoder pointer */
    yarp::dev::IPositionControl *m_pILeftArmPosition;                           /**< arm position control pointer */
    yarp::dev::PolyDriver        m_oRobotLeftArm;                               /**< robot arm controller */
    yarp::dev::IVelocityControl *m_pILeftArmVelocity;
    int  m_i32LeftArmJointsNb;  /**< ... */

    // Right arm control
    yarp::os::Property m_oRightArmOptions;              /**< robot interfaces for head/gaze movements */
    yarp::dev::IEncoders        *m_pIRightArmEncoders;                           /**< arm encoder pointer */
    yarp::dev::IPositionControl *m_pIRightArmPosition;                           /**< arm position control pointer */
    yarp::dev::PolyDriver        m_oRobotRightArm;                               /**< robot arm controller */
    yarp::dev::IVelocityControl *m_pIRightArmVelocity;
    int  m_i32RightArmJointsNb;  /**< ... */

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool configureSWS(yarp::os::ResourceFinder &oRF);

    bool interruptModule();
    void createSWS();

    bool close();
    bool closeSWS();

    bool updateSWS();

    double getPeriod()
    {
        return 1/m_i32Fps;
    }

    bool updateModule();

    //RPC & scenarios
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
};

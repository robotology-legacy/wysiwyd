#ifndef RPCLISTENERMODULEH
#define RPCLISTENERMODULEH

/* 
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Bertand Higy
 * email:  bertrand.higy@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include "touchDetectorThread.h"

class TouchDetectorModule: public yarp::os::RFModule
{
    public:
        bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
        bool interruptModule();                       // interrupt, e.g., the ports 
        bool close();                                 // close and shut down the module
        bool respond();
        bool updateModule();
       
    protected:
        /* module parameters */
        int period;
        double threshold;
        int taxelThreshold;
        std::string moduleName;
        std::string torsoPortName;
        std::string leftArmPortName;
        std::string rightArmPortName;
        std::string leftForearmPortName;
        std::string rightForearmPortName;
        std::string leftHandPortName;
        std::string rightHandPortName;
        std::string touchPortName;
        std::string touchPortCleanName;
        std::string clustersConfFilepath;

        /* class variables */
        yarp::os::BufferedPort<yarp::os::Bottle> torsoPort;
        yarp::os::BufferedPort<yarp::os::Bottle> leftArmPort;
        yarp::os::BufferedPort<yarp::os::Bottle> rightArmPort;
        yarp::os::BufferedPort<yarp::os::Bottle> leftForearmPort;
        yarp::os::BufferedPort<yarp::os::Bottle> rightForearmPort;
        yarp::os::BufferedPort<yarp::os::Bottle> leftHandPort;
        yarp::os::BufferedPort<yarp::os::Bottle> rightHandPort;
        yarp::os::BufferedPort<yarp::os::Bottle> touchPort;
        yarp::os::BufferedPort<yarp::os::Bottle> touchPortCleaned;
        TouchDetectorThread *thread;
        
        void initializeParameters(yarp::os::ResourceFinder &rf);
        bool openPorts();
};

#endif

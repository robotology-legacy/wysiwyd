#ifndef RPCLISTENERTHREADH
#define RPCLISTENERTHREADH

/* 
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Bertand HIGY
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

#include <string>
#include <vector>
#include <exception>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>

class TouchDetectorThread : public yarp::os::RateThread
{
    public:
        TouchDetectorThread(yarp::os::BufferedPort<yarp::os::Bottle> *torsoPort, yarp::os::BufferedPort<yarp::os::Bottle> *leftArmPort, yarp::os::BufferedPort<yarp::os::Bottle> *rightArmPort, yarp::os::BufferedPort<yarp::os::Bottle> *leftForearmPort, yarp::os::BufferedPort<yarp::os::Bottle> *rightForearmPort, yarp::os::BufferedPort<yarp::os::Bottle> *leftHandPort, yarp::os::BufferedPort<yarp::os::Bottle> *rightHandPort, yarp::os::BufferedPort<yarp::os::Bottle> *touchPort, int period, std::string *clustersConfFilepath, int threshold);   
        void run(); 
        bool threadInit();
       
    protected:
        static const int noCluster;
        static const int nbBodyParts;
        static const char* bodyParts[7];
        static const int nbTaxels[7];
        
        int nbClusters;
        int threshold;
        std::string *clustersConfFilepath;
        std::vector<int> taxels2Clusters[7];
        
        /* ports */
        yarp::os::BufferedPort<yarp::os::Bottle> *torsoPort;
        yarp::os::BufferedPort<yarp::os::Bottle> *leftArmPort;
        yarp::os::BufferedPort<yarp::os::Bottle> *rightArmPort;
        yarp::os::BufferedPort<yarp::os::Bottle> *leftForearmPort;
        yarp::os::BufferedPort<yarp::os::Bottle> *rightForearmPort;
        yarp::os::BufferedPort<yarp::os::Bottle> *leftHandPort;
        yarp::os::BufferedPort<yarp::os::Bottle> *rightHandPort;
        yarp::os::BufferedPort<yarp::os::Bottle> *touchPort;
        
        bool readTaxelsMapping(const char* filename);
        void parseMappingLine(std::string line, int &bodyPart, int &firstTaxel, int &lastTaxel);
        int getBodyPartId(std::string bodyPartName);
        void updateMapping(int bodyPart, int firstTaxel, int lastTaxel, int cluster);
        void countActivations(int bodyPart, yarp::os::Bottle *data, std::vector<int> &activations);
        void processPort(int portNum, yarp::os::BufferedPort<yarp::os::Bottle> *port, std::vector<int> &activations);
};

class ParsingException: public std::exception
{
    public:
        int line;
        
        ParsingException();
        virtual const char* what() const throw();
};

class BadFormatException: public std::exception
{
    public:
        const char* expectedType;
        const char* portName;
        
        BadFormatException();
        virtual const char* what() const throw();
};

#endif

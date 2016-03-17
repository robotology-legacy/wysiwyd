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

#include <fstream>
#include <string>
#include <sstream>

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include "touchDetectorThread.h"

using namespace std;
using namespace yarp::os;

const int TouchDetectorThread::noCluster = -1;
const int TouchDetectorThread::nbBodyParts = 7;
const char* TouchDetectorThread::bodyParts[7] = {"torso", "left_arm", "right_arm", "left_forearm", "right_forearm", "left_hand", "right_hand"};
const int TouchDetectorThread::nbTaxels[7] = {4 * 192, 4 * 192, 4 * 192, 2 * 192, 2 * 192, 192, 192};

TouchDetectorThread::TouchDetectorThread(BufferedPort<Bottle> *torsoPort, BufferedPort<Bottle> *leftArmPort, BufferedPort<Bottle> *rightArmPort, BufferedPort<Bottle> *leftForearmPort, BufferedPort<Bottle> *rightForearmPort, BufferedPort<Bottle> *leftHandPort, BufferedPort<Bottle> *rightHandPort, BufferedPort<Bottle> *touchPort, int period, string clustersConfFilepath, int threshold)
    : RateThread(period), threshold(threshold), clustersConfFilepath(clustersConfFilepath), torsoPort(torsoPort), leftArmPort(leftArmPort), rightArmPort(rightArmPort), leftForearmPort(leftForearmPort), rightForearmPort(rightForearmPort), leftHandPort(leftHandPort), rightHandPort(rightHandPort), touchPort(touchPort)
{
    for (int i = 0; i < nbBodyParts; ++i)
    {
        taxels2Clusters[i].resize(nbTaxels[i]);
        for (int j = 0; j < nbTaxels[i]; ++j)
        {
            taxels2Clusters[i][j] = noCluster;
        }
    }
}

bool TouchDetectorThread::threadInit()
{
    try
    {
        readTaxelsMapping(clustersConfFilepath);
    }
    catch (ParsingException ex)
    {
        yError() << ex.what();
        return false;
    }

    return true;
}

bool TouchDetectorThread::readTaxelsMapping(string filename)
{
    Property p;
    p.fromConfigFile(filename);
    int clusterId = 0;
    for (int iPart = 0; iPart < nbBodyParts; ++iPart)
    {
        Bottle clusters = p.findGroup(bodyParts[iPart]).tail();
        for (int iCluster = 0; iCluster < clusters.size(); ++iCluster)
        {
            ++clusterId;
            Bottle *c = clusters.get(iCluster).asList();
            for (int iRange = 0; iRange < c->size(); ++iRange)
            {
                Bottle *range = c->get(iRange).asList();
                if (range->size() == 1)
                {
                    int taxel = range->get(0).asInt();
                    updateMapping(iPart, taxel, taxel, clusterId);
                }
                if (range->size() >= 2)
                {
                    int first = range->get(0).asInt();
                    int last = range->get(1).asInt();
                    updateMapping(iPart, first, last, clusterId);
                }
            }
        }
    }

    nbClusters = clusterId;

    return true;
}

int TouchDetectorThread::getBodyPartId(string bodyPartName)
{
    int id = -1;
    for (int i = 0; i < nbBodyParts; ++i)
    {
        if (bodyParts[i] == bodyPartName)
        {
            id = i;
            break;
        }
    }
    if (id == -1)
    {
        throw ParsingException();
    }

    return id;
}

void TouchDetectorThread::updateMapping(int bodyPart, int firstTaxel, int lastTaxel, int cluster)
{
    for (int i = firstTaxel - 1; i < lastTaxel; ++i)
    {
        taxels2Clusters[bodyPart][i] = cluster;
    }
}

void TouchDetectorThread::run()
{
    vector<int> activations;
    for(int i = 0; i < nbClusters; i++) {
        activations.push_back(0);
    }

    int port = 0;
    try
    {
        processPort(port, torsoPort, activations);
        processPort(++port, leftArmPort, activations);
        processPort(++port, rightArmPort, activations);
        processPort(++port, leftForearmPort, activations);
        processPort(++port, rightForearmPort, activations);
        processPort(++port, leftHandPort, activations);
        processPort(++port, rightHandPort, activations);
    }
    catch(BadFormatException &ex)
    {
        ex.portName = bodyParts[port];
        throw ex;
    }
    Bottle& output = touchPort->prepare();
    output.clear();
    for (auto& activation : activations)
    {
        output.addInt(activation);
    }
    touchPort->write();
}

void TouchDetectorThread::processPort(int portNum, yarp::os::BufferedPort<yarp::os::Bottle> *port, vector<int> &activations)
{
    Bottle *tactileData = port->read(false);
    if (tactileData != NULL)
    {
        countActivations(portNum, tactileData, activations);
    }
    else
    {
        yError() << "Unable to read data for " << bodyParts[portNum];
    }
}

void TouchDetectorThread::countActivations(int bodyPart, Bottle* data, vector<int> &activations)
{
    int i = 0;
    for (vector<int>::iterator it = taxels2Clusters[bodyPart].begin() ; it != taxels2Clusters[bodyPart].end(); ++it)
    {
        Value v = data->get(i);
        if (v.isNull() || !v.isDouble())
        {
            BadFormatException ex;
            ex.expectedType = "double";
            throw ex;
        }
        if (v.asInt() > threshold && *it != noCluster)
        {
            ++activations[*it];
        }
        ++i;
    }
}

ParsingException::ParsingException()
{
    line = -1;
}

const char* ParsingException::what() const throw()
{
    if (line != -1)
    {
        stringstream ss;
        ss << "Error parsing line " << line;
        return ss.str().c_str();
    }
    else
    {
        return "Parsing error";
    }
}

BadFormatException::BadFormatException()
{
    expectedType = NULL;
    portName = NULL;
}

const char* BadFormatException::what() const throw()
{
    string msg = "Bad format encountered ";
    if (expectedType != NULL)
    {
        msg += "(expecting ";
        msg += expectedType;
        msg += ") ";
    }
    msg += "while reading ";
    if (portName != NULL)
    {
        msg += "port ";
        msg += portName; 
    }
    else
    {
        msg += "a port";
    }
    return msg.c_str();
}


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

#include "touchDetectorThread.h"

using namespace std;
using namespace yarp::os;

const int TouchDetectorThread::noCluster = -1;
const int TouchDetectorThread::nbBodyParts = 7;
const char* TouchDetectorThread::bodyParts[7] = {"torso", "left_arm", "right_arm", "left_forearm", "right_forearm", "left_hand", "right_hand"};
const int TouchDetectorThread::nbTaxels[7] = {4 * 192, 4 * 192, 4 * 192, 2 * 192, 2 * 192, 192, 192};

TouchDetectorThread::TouchDetectorThread(BufferedPort<Bottle> *torsoPort, BufferedPort<Bottle> *leftArmPort, BufferedPort<Bottle> *rightArmPort, BufferedPort<Bottle> *leftForearmPort, BufferedPort<Bottle> *rightForearmPort, BufferedPort<Bottle> *leftHandPort, BufferedPort<Bottle> *rightHandPort, BufferedPort<Bottle> *touchPort, int period, string *clustersConfFilepath, int threshold)
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
		readTaxelsMapping(clustersConfFilepath->c_str());
	}
	catch (ParsingException ex)
	{
		cerr << ex.what() << endl;
		return false;
	}
	
	return true;
}

bool TouchDetectorThread::readTaxelsMapping(const char* filename)
{
	ifstream is(filename);
	if (!is.is_open())
	{
		cerr << "Unable to open file: " << clustersConfFilepath << endl;
		return false;
	}
	
	int clusterId = -1;
	bool newCluster = true;
	int bodyPart, firstTaxel, lastTaxel;
	int lineNumber = 0;
	while(!is.eof())
	{
		string line;
		std::getline(is, line);
		++lineNumber;
		
		if (line.empty())
		{
			newCluster = true;
		}
		else
		{
			if (newCluster)
			{
				++clusterId;
				newCluster = false;
			}
				
			try
			{
				parseMappingLine(line, bodyPart, firstTaxel, lastTaxel);
				updateMapping(bodyPart, firstTaxel, lastTaxel, clusterId);
			}
			catch (ParsingException &ex)
			{
				ex.line = lineNumber;
				throw ex;
			}
		}
	}
	
	nbClusters = clusterId + 1;
	
	is.close();
	return true;
}

void TouchDetectorThread::parseMappingLine(string line, int &bodyPart, int &firstTaxel, int &lastTaxel)
{
	istringstream iss(line);
	
	// Parsing the body part
	string bodyPartName;
	if (!(iss >> bodyPartName))
	{
		throw ParsingException();
	}
	bodyPart = getBodyPartId(bodyPartName);
	
	// Parsing the first taxel id
	if (!(iss >> firstTaxel))
	{
		throw ParsingException();
	}
	
	// Parsing the second taxel id (optional)
	if (!(iss >> lastTaxel))
	{
		lastTaxel = firstTaxel;
	}
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
	vector<int> activations(nbClusters, 0);
	
	int port = 0;
	try
	{
		Bottle *tactileData = torsoPort->read();
		countActivations(port, tactileData, activations);
		tactileData = leftArmPort->read();
		countActivations(++port, tactileData, activations);
		tactileData = rightArmPort->read();
		countActivations(++port, tactileData, activations);
		tactileData = leftForearmPort->read();
		countActivations(++port, tactileData, activations);
		tactileData = rightForearmPort->read();
		countActivations(++port, tactileData, activations);
		tactileData = leftHandPort->read();
		countActivations(++port, tactileData, activations);
		tactileData = rightHandPort->read();
		countActivations(++port, tactileData, activations);
	}
	catch(BadFormatException &ex)
	{
		ex.portName = bodyParts[port];
	}
	
	Bottle& output = touchPort->prepare();
	output.clear();
	for (vector<int>::iterator it = activations.begin() ; it != activations.end(); ++it)
	{
		output.addInt(*it);
	}
	touchPort->write();
}

void TouchDetectorThread::countActivations(int bodyPart, Bottle* data, vector<int> &activations)
{
	int i = 2;
	for (vector<int>::iterator it = taxels2Clusters[bodyPart].begin() ; it != taxels2Clusters[bodyPart].end(); ++it)
	{
		if (bodyPart == 5)
		{
			cerr << *it << ' ';
		}
		Value v = data->get(i);
		if (v.isNull() || !v.isDouble())
		{
			BadFormatException ex;
			ex.expectedType = "double";
			throw ex;
		}
		if (!v.isNull() && v.asInt() > threshold && *it != noCluster)
		{
			++activations[*it];
		}
	}
	cerr << endl;
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
	string msg = "Bad format encoountered ";
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


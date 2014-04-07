/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Stéphane Lallée
* email:   stephane.lallee@gmail.com
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

#ifndef __CVZ_CVZSTACK_H__
#define __CVZ_CVZSTACK_H__
#include "yarp/os/ResourceFinder.h"
#include "ICvz.h"
#include "CvzBuilder.h"
#include <list>
#include <map>

namespace cvz {
	namespace core {

		/**
		* CvzStack
		* A simple helper class that allows to create a graph of CVZ from code
		* and to manage the connections between those.
		*/
		class CvzStack
		{
			std::list< IConvergenceZone* > nodes;
			std::map<std::string, IModality* > nodesIO;
			std::map< IModality*, std::map<IModality*, bool > > connections;

			/**
			* Complete the connection matrix by setting to false the connections that do not exist.
			*/
			void fillConnectionMatrix()
			{
				//1. complete the matrix
				for (std::map<std::string, IModality* >::iterator it = nodesIO.begin(); it != nodesIO.end(); it++)
				{
					if (connections.find(it->second) == connections.end())
					{
						connections[it->second][it->second] = false;
					}
				}

				//2. fill the gaps
				for (std::map<std::string, IModality* >::iterator it = nodesIO.begin(); it != nodesIO.end(); it++)
				{
					for (std::map<std::string, IModality* >::iterator it2 = nodesIO.begin(); it2 != nodesIO.end(); it2++)
					{
						if (connections[it->second].find(it2->second) == connections[it2->second].end())
						{
							connections[it->second][it2->second] = false;
						}
					}
				}
			}

		public:

			/**
			* Add a new CVZ from a config file. The CVZ is instantiated and added to the graph.
			* @param configFile Name of the config file containing the information about the cvz.
			* @return true/false on success/failure.
			*/
			bool addCvz(std::string configFile)
			{
				IConvergenceZone* cvz;
				yarp::os::ResourceFinder rf;
				rf.setVerbose(true);
				rf.setDefaultContext("cvz");
				rf.setDefaultConfigFile(configFile.c_str());
				rf.configure(NULL, NULL);
				
				yarp::os::Property prop; 
				prop.fromConfigFile(rf.findFile("from"));
				std::string cvzType = prop.check("type", yarp::os::Value(cvz::core::TYPE_ICVZ)).asString();

				if (cvz::core::CvzBuilder::allocate(&cvz, cvzType))
				{
					cvz->configure(prop);
				}

				//Expand the connection list and the connection matrix
				for (std::map<std::string, IModality* >::iterator it = cvz->modalitiesBottomUp.begin(); it != cvz->modalitiesBottomUp.end(); it++)
					nodesIO[it->second->GetFullName()] = it->second;
				for (std::map<std::string, IModality* >::iterator it = cvz->modalitiesTopDown.begin(); it != cvz->modalitiesTopDown.end(); it++)
					nodesIO[it->second->GetFullName()] = it->second;
				fillConnectionMatrix();
			}
			

			/**
			* Connect two modalities given their names (i.e: /cvzName/modalityName ).
			* Also establish the yarp connection between them.
			* @param from Source modality name.
			* @param to Target modality name.
			* @param tryConnectYarp Do you want to establish the yarp connection now (true) or just add it to the graph?
			* @return True/False in case of success/failure.
			*/
			bool connectModalities(std::string from, std::string to, bool tryConnectYarp=true)
			{
				connections[nodesIO[from]][nodesIO[to]] = true;
				bool result = true;
				if (tryConnectYarp)
				{
					result &= yarp::os::Network::connect(nodesIO[from]->GetFullNamePrediction(), nodesIO[to]->GetFullNameReal());
					result &= yarp::os::Network::connect(nodesIO[to]->GetFullNamePrediction(), nodesIO[from]->GetFullNameReal());
				}
				return result;
			}

			/**
			* Connect all modalities following the graph connection matrix.
			* Also establish the yarp connection between them.
			* @param from Source modality name.
			* @param to Target modality name.
			* @return True/False in case of success/failure.
			*/
			void connectModalities()
			{
				for (std::map<std::string, IModality* >::iterator it = nodesIO.begin(); it != nodesIO.end(); it++)
				{
					for (std::map<std::string, IModality* >::iterator it2 = nodesIO.begin(); it2 != nodesIO.end(); it2++)
					{
						if (connections[it->second][it2->second])
						{
							connectModalities(it->first, it2->first);
						}
					}
				}
			}

		};
	}
}
#endif

/*
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Gregoire Pointeau, Maxime Petit
* email:   gregoire.pointeau@inserm.fr, m.petit@imperial.ac.uk
* website: http://wysiwyd.upf.edu/
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

#ifndef __EFAA_SUBSYSTEM_LRH_H__
#define __EFAA_SUBSYSTEM_LRH_H__

#define SUBSYSTEM_LRH        "lrh"

#include "wrdac/subsystems/subSystem.h"
#include "wrdac/subsystems/subSystem_ABM.h"

#include <iostream>

namespace wysiwyd{
    namespace wrdac{

        /**
        * \ingroup wrdac_clients
        *
        * Abstract subSystem for speech recognizer
        */
        class SubSystem_LRH : public SubSystem
        {
        protected:
            virtual bool connect();
            SubSystem_ABM* SubABM;
            bool ABMconnected();
            bool SAMconnected();

        public:
            bool bForwardABM;
            std::string interlocutor;
            std::string narrator;

            yarp::os::Port portRPC;
            yarp::os::Port portSAM;

            SubSystem_LRH(const std::string &masterName);

            virtual void Close();

            std::string meaningToSentence(std::string sInput);
            std::string SentenceToMeaning(std::string sInput);
            std::string SentenceFromPAORSimple(std::string P1, std::string A1, std::string O1 = "none", std::string R1 = "none");
            std::string SentenceFromPAORDouble(std::string P1, std::string A1, std::string P2, std::string A2, std::string O1 = "none", std::string R1 = "none", std::string O2 = "none", std::string R2 = "none");

            std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
            std::vector<std::string> split(const std::string &s, char delim);
        };


    }
}
//Namespace



#endif



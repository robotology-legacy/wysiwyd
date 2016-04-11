/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Anne-Laure MEALIER
 * email:   anne-laure.mealier@inserm.fr
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

#ifndef _LRH_H_
#define _LRH_H_

#ifdef WIN32
//#include <windows.h>
#define  BOOST_ALL_NO_LIB
#endif

//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <string>
//#include <list>

#include <boost/python.hpp>

//#include <yarp/sig/all.h>
//#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"


class LRH : public yarp::os::RFModule {
private:

    int iCurrentInstance;

    std::string sKeyWord;
    std::string moduleName;
    std::string handlerPortName;

    std::string scorpusFileSD;
    std::string scorpusFile;
    std::string sfileResult;
    std::string stemporaryCorpus;
    
    std::string sreservoirAP;
    std::string sreservoirSD;
    std::string sreservoirNarratif;

    std::string sclosed_class_wordsSD;
    std::string sclosed_class_words;
    
    std::string smax_nr_ocw;
    std::string smax_nr_actionrelation;
    std::string selt_pred;
    std::string sNbNeurons;
    std::string sMode;
    std::string sdataTestSD;
    std::string sHand;
    float offsetGrasp;

    std::string sobjectFocusChanged;

    yarp::os::Port handlerPort;               // a port to handle messages
    yarp::os::Port PortToSam;               // a port to forward PAOR structure to SAM
    std::string nameSamInputPort;

    wysiwyd::wrdac::ICubClient *iCub;

    bool callReservoir(std::string pythonFile, std::string sCCW);
    int copyPastTrainFile(const char* fileNameIn, const char* fileNameOut);
    int createTest(const char* filename, std::string sMeaningSentence);
    int createTest(const char* filename, std::list<std::string> lMeaningsSentences);
    std::string openResult(const char* fileNameIn);
    bool populateOPC();

    bool AREactions(std::vector<std::string> seq);
    bool spatialRelation(std::string sObjectFocus);
    std::string production(std::string stest);

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();

    std::string meaningToSentence(std::string meaning);
    std::string sentenceToMeaning(std::string sentence);
};


#endif // __LRH_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

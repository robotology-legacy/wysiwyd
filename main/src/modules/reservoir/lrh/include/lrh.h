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
#include <windows.h>
#define  BOOST_ALL_NO_LIB
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <list>

#include <boost/python.hpp>
using namespace boost::python;

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include "wrdac/clients/icubClient.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;

class LRH : public RFModule {
private:

    int iCurrentInstance;

    string sKeyWord;
    string moduleName;
    string handlerPortName;
    string scorpusFileAP;
    string scorpusFileSD;
    string sfileResult;
    string stemporaryCorpus;
    string sreservoirAP;
    string sreservoirSD;
    string sclosed_class_wordsAP;
    string sclosed_class_wordsSD;
    string smax_nr_ocw;
    string smax_nr_actionrelation;
    string selt_pred;

    string sdataTestSD;
    string sHand;
    float offsetGrasp;

    string sobjectFocusChanged;

    Port handlerPort;				// a port to handle messages

    ICubClient *iCub;

    bool callReservoir(string fPython, string closed_class_words);
    int copyPastTrainFile(const char* fileNameIn, const char* fileNameOut);
    int createTest(const char* filename, string sMeaningSentence);
    string openResult(const char* fileNameIn);
    bool populateOPC();

    bool AREactions(vector<string> seq);
    bool spatialRelation(string sObjectFocus);

public:
    bool configure(ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod();
    bool updateModule();

    bool meaningToSentence(string meaning);
    bool sentenceToMeaning(string sentence);
};


#endif // __LRH_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

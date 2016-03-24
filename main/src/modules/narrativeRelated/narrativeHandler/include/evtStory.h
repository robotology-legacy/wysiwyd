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
#include <time.h>


class evtStory{
public:
    int instance;
    std::string activity_name;
    std::string activity_type;

    std::vector<std::pair< std::string, double> >  mFocus;

    std::string predicate;
    std::string agent;
    std::string object;
    std::string recipient;

    bool isIn(std::vector<std::string> vec, std::string str);

    bool isNarration; // set if the event is due to the narration of an external agent


    yarp::os::Bottle bRelations;

    std::vector<std::pair<std::string, std::string> >  vArgument;
    bool begin;

    std::vector<std::string> initialize(int _instance, yarp::os::Bottle bActivity, yarp::os::Bottle bArguments, yarp::os::Bottle bRelations);

    void removeUnderscore();
    void addUnderscore();
    void print();

    void removeUnderscoreString(std::string &input);
    void addUnderscoreString(std::string &input);
};


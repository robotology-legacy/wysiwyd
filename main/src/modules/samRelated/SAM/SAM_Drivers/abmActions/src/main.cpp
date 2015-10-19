// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Uriel Martinez, Luke Boorman
* email:   uriel.martinez@sheffield.ac.uk
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* $WYSIWYD_ROOT/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

/**
* @file main.cpp
* @brief main code.
*/

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include "abmActions.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


int main(int argc, char **argv)
{
    Network yarp;

    if( !yarp.checkNetwork() )
    {
        cout << "yarp server not found..." << endl;
        return 1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("abmActions");
    rf.setDefaultConfigFile("abmActions.ini");
    rf.configure(argc,argv);
    
    abmActions mod;

    return mod.runModule(rf);
}


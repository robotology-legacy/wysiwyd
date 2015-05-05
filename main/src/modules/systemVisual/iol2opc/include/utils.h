/* 
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Ugo Pattacini, Tobias Fischer
 * email:   ugo.pattacini@iit.it, t.fischer@imperial.ac.uk
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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <cv.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class IOL2OPCBridge;  // forward declaration


/**********************************************************/
class RtLocalization : public RateThread
{
protected:
    IOL2OPCBridge *bridge;
    
    bool threadInit();
    void run();

public:
    RtLocalization();
    void setBridge(IOL2OPCBridge *bridge);
};


/**********************************************************/
class OpcUpdater : public RateThread
{
protected:
    IOL2OPCBridge *bridge;
    
    bool threadInit();
    void run();

public:
    OpcUpdater();
    void setBridge(IOL2OPCBridge *bridge);
};


/**********************************************************/
class ClassifierReporter : public PortReport
{
protected:
    IOL2OPCBridge *bridge;

public:
    ClassifierReporter();
    void setBridge(IOL2OPCBridge *bridge);
    void report(const PortInfo &info);
};


#endif


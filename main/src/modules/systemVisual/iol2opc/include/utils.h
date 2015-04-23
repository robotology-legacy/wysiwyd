/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <cv.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class Manager;  // forward declaration

/**********************************************************/
class Speaker : public Port
{
protected:
    bool speaking;

public:
    Speaker() : speaking(true) { }
    void setSpeaker(const bool speaking) { this->speaking=speaking; }
    bool getSpeaker() const              { return speaking;         }
    void speak(const string &phrase, const bool force=false);
};


/**********************************************************/
class PointedLocationPort : public BufferedPort<Bottle>
{
protected:
    CvPoint loc;
    double rxTime;
    double timeout;

    void onRead(Bottle &b);

public:
    PointedLocationPort();
    bool getLoc(CvPoint &loc);
};


/**********************************************************/
class StopCmdPort : public BufferedPort<Bottle>
{
protected:
    Manager *manager;

    void onRead(Bottle &b);

public:
    StopCmdPort();
    void setManager(Manager *manager);
};


/**********************************************************/
class Attention : public RateThread
{
protected:
    Manager *manager;
    
    bool threadInit();
    void run();

public:
    Attention();
    void suspend();
    void setManager(Manager *manager);
};


/**********************************************************/
class RtLocalization : public RateThread
{
protected:
    Manager *manager;
    
    bool threadInit();
    void run();

public:
    RtLocalization();
    void setManager(Manager *manager);
};


/**********************************************************/
class Exploration : public RateThread
{
protected:
    Manager *manager;
    string   object;
    Vector   position;
    
    bool threadInit();
    void run();

public:
    Exploration();    
    void setManager(Manager *manager);
    void setInfo(const string &object, const Vector &position);
};


/**********************************************************/
class MemoryUpdater : public RateThread
{
protected:
    Manager *manager;
    
    bool threadInit();
    void run();

public:
    MemoryUpdater();
    void setManager(Manager *manager);
};


/**********************************************************/
class MemoryReporter : public PortReport
{
    Manager *manager;

public:
    MemoryReporter();
    void setManager(Manager *manager);
    void report(const PortInfo &info);
};


#endif


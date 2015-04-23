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

#include <yarp/os/Time.h>

#include "utils.h"
#include "module.h"


/**********************************************************/
void Speaker::speak(const string &phrase, const bool force)
{
    if ((force || speaking) && (getOutputCount()>0))
    {
        Bottle request;
        request.addString(phrase.c_str());
        write(request);
    }
}


/**********************************************************/
void PointedLocationPort::onRead(Bottle &b)
{
    if (b.size()>1)
    {
        loc.x=(int)b.get(0).asDouble();
        loc.y=(int)b.get(1).asDouble();
        rxTime=Time::now();
    }
}


/**********************************************************/
PointedLocationPort::PointedLocationPort()
{
    useCallback();
    rxTime=-1.0;
    timeout=5.0;
}


/**********************************************************/
bool PointedLocationPort::getLoc(CvPoint &loc)
{
    double t0=Time::now();

    if ((rxTime>0.0) && (t0-rxTime<timeout))
    {
        loc=this->loc;
        return true;
    }
    
    while (Time::now()-t0<timeout)
    {
        if ((rxTime>0.0) && (Time::now()-rxTime<timeout))
        {
            loc=this->loc;
            return true;
        }

        Time::delay(0.1);
    }

    return false;
}


/**********************************************************/
void StopCmdPort::onRead(Bottle &b)
{
    if (manager!=NULL)
    {
        if (b.size()==2)
        {
            string command=b.get(0).asString().c_str();
            double confidence=b.get(1).asDouble();

            if ((command=="icub-stop-now") && (confidence>0.8))
                manager->interruptMotor();
        }
    }
}


/**********************************************************/
StopCmdPort::StopCmdPort()
{
    useCallback();
    manager=NULL;
}


/**********************************************************/
void StopCmdPort::setManager(Manager *manager)
{
    this->manager=manager;
}


/**********************************************************/
Attention::Attention() : RateThread(4000)
{
    manager=NULL;
}


/**********************************************************/
void Attention::setManager(Manager *manager)
{
    this->manager=manager;
}


/**********************************************************/
bool Attention::threadInit()
{
    return (manager!=NULL);
}


/**********************************************************/
void Attention::suspend()
{
    if (!isSuspended())
    {
        manager->mutexAttention.wait();
        RateThread::suspend();
        manager->mutexAttention.post();
    }
}


/**********************************************************/
void Attention::run()
{
    manager->switchAttention();
}


/**********************************************************/
RtLocalization::RtLocalization() : RateThread(30)
{
    manager=NULL;
}


/**********************************************************/
void RtLocalization::setManager(Manager *manager)
{
    this->manager=manager;
}


/**********************************************************/
bool RtLocalization::threadInit()
{
    return ((manager!=NULL) && (getRate()!=0.0));
}


/**********************************************************/
void RtLocalization::run()
{
    manager->doLocalization();
}


/**********************************************************/
Exploration::Exploration() : RateThread(30)
{
    manager=NULL;
}


/**********************************************************/
void Exploration::setManager(Manager *manager)
{
    this->manager=manager;
}


/**********************************************************/
void Exploration::setInfo(const string &object,
                          const Vector &position)
{
    this->object=object;
    this->position=position;
}


/**********************************************************/
bool Exploration::threadInit()
{
    return (manager!=NULL);
}


/**********************************************************/
void Exploration::run()
{
    manager->doExploration(object,position);
}


/**********************************************************/
MemoryUpdater::MemoryUpdater() : RateThread(100)
{
    manager=NULL;
}


/**********************************************************/
void MemoryUpdater::setManager(Manager *manager)
{
    this->manager=manager;
}


/**********************************************************/
bool MemoryUpdater::threadInit()
{
    return ((manager!=NULL) && (getRate()!=0.0));
}


/**********************************************************/
void MemoryUpdater::run()
{
    manager->updateMemory();
}


/**********************************************************/
MemoryReporter::MemoryReporter()
{
    manager=NULL;
}


/**********************************************************/
void MemoryReporter::setManager(Manager *manager)
{
    this->manager=manager;
}


/**********************************************************/
void MemoryReporter::report(const PortInfo &info)
{
    if ((manager!=NULL) && info.created && !info.incoming)
        manager->scheduleLoadMemory=true;
}





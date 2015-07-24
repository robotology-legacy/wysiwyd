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

#ifndef __WYSIWYD_ANIMATION_H__
#define __WYSIWYD_ANIMATION_H__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <string>
#include <vector>
#include <map>
#include <list>
#include <iostream>

namespace wysiwyd{namespace wrdac{
/**
 * @defgroup wrdac_animation Motor Animations
 *  
 * @ingroup wrdac
 *  
 * Contains classes to represent motor sequences as animations
 * that can be recorded and replayed at runtime.
 *
 * @author Stéphane Lallée
 */ 


/**
* \ingroup wrdac_animation
*
* Animation, a sequence of frames together with which joints should be controlled
* 
*/
struct Animation
{
    std::string partUsed;
    std::list<std::pair<yarp::sig::Vector,double> > frames;
    std::vector<bool> jntsControlled;

    Animation(std::string _partUsed, int size)
    {
        partUsed = _partUsed;
        jntsControlled.resize(size, true);
    }

    bool addFrame(const yarp::sig::Vector &encs, double time)
    {
        if (encs.size() != jntsControlled.size())
            return false;
        frames.push_back(std::pair<yarp::sig::Vector,double>(encs,time));
        return true;
    }

    bool  setCtrl(int jnt, bool value)
    {
        if ((jnt<0) && (jnt>=(int)jntsControlled.size()))
            return false;
        jntsControlled[jnt] = value;
        return true;
    }
};

struct BodyPosture
{
    yarp::sig::Vector head;
    yarp::sig::Vector left_arm;
    yarp::sig::Vector right_arm;
    yarp::sig::Vector torso;
};

typedef std::map<std::string, Animation*> AnimationLibrary;

/**
* \ingroup wrdac_animation
*
* Main class, manages a library of animations. Methods names are self explanatory.
* 
*/
class Animator: public yarp::os::RateThread
{    
    //General
    std::string moduleName;
    std::string robotName;

    //Motor interfaces
    virtual bool startInterfaces();
    bool interfacesRunning;
    std::list<std::string> workingLimbs;
    std::map<std::string, yarp::dev::PolyDriver*>       drivers;
    std::map<std::string, yarp::dev::IEncoders*>        encs;
    std::map<std::string, yarp::dev::IPositionControl*> ctrls;

    //Animation storage
    std::map<std::string, AnimationLibrary> limbsAnimations;
    Animation* currentlyRecorded;
    Animation* currentlyReplayed;
    double startReplayingTime;

public:
    Animator(std::string moduleName, std::string robotName = "icub",
             int period = 10);

    virtual bool threadInit();
    virtual void run();

    bool startRecording(std::string part, std::string animationName);
    void stopRecording();
    bool startReplaying(std::string part, std::string animationName);
    void stopReplaying();
    bool playFrame(Animation* anim, std::list<std::pair<yarp::sig::Vector, double> >::iterator frameIterator);
};
}}
#endif

/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com
 * website: http://efaa.upf.edu/ 
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

#ifndef __EFAA_SUBSYSTEM_ARE_H__
#define __EFAA_SUBSYSTEM_ARE_H__


#define SUBSYSTEM_ARE     "ARE"

#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/knowledge/object.h"

namespace wysiwyd{namespace wrdac{

    //Defines extracted from ARE main.cpp
#define CMD_GET                     VOCAB3('g','e','t')
#define CMD_TAKE                    VOCAB4('t','a','k','e')
#define CMD_GRASP                   VOCAB4('g','r','a','s')
#define CMD_PUSH                    VOCAB4('p','u','s','h')
#define CMD_POINT                   VOCAB4('p','o','i','n')
#define CMD_EXPECT                  VOCAB4('e','x','p','e')
#define CMD_GIVE                    VOCAB4('g','i','v','e')
#define CMD_CLOSE                   VOCAB4('c','l','o','s')
#define CMD_DROP                    VOCAB4('d','r','o','p')
#define CMD_OBSERVE                 VOCAB4('o','b','s','e')
#define CMD_HOME                    VOCAB4('h','o','m','e')

/**
* \ingroup wrdac_clients
*
* SubSystem for actionsRenderingEngine (a.k.a. ARE).
*/
class SubSystem_ARE : public SubSystem
{
private:
    yarp::os::Port portARE;

    void appendCartesianTarget(yarp::os::Bottle& b, const yarp::sig::Vector &t)
    {
        yarp::os::Bottle& sub = b.addList();
        sub.addString("cartesian");
        for (size_t i = 0; i < t.length(); i++)
            sub.addDouble(t[i]);
    }

    bool sendCmd(yarp::os::Bottle &cmd, bool shouldWait)
    {
        yarp::os::Bottle bReply;
        if (shouldWait)
        {
            portARE.write(cmd, bReply);
            return bReply.get(0).asString() == "ACK";
        }
        else
            portARE.write(cmd);
        return true;
    }

protected:
    virtual bool connect() 
    { 
        return yarp::os::Network::connect(portARE.getName(), "/actionsRenderingEngine/cmd:io"); 
    }

public:
    SubSystem_ARE(std::string &masterName) : SubSystem(masterName)
    {
        portARE.open(("/"+masterName+"/ARE:rpc").c_str());
        m_type = SUBSYSTEM_ARE;
    }

    virtual void Close() 
    { 
        portARE.interrupt();
        portARE.close();
    }

    /**
    * Robot goes to home position
    * @param part (gaze, head, hands, finguers, all. All by default)
    * @param shouldWait is the function blocking?
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool home(const std::string &part = "all", bool shouldWait = true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(CMD_HOME);
        bCmd.addString(part);
        return sendCmd(bCmd, shouldWait);
    }

    /**
    * the robot tries to reach the specified [target] and grasp it. Optional parameter "side" or "above" can be supplied to choose the orientation the robot should try to mantain while performing the action (default: "above").
    * @param target Target to grasp in cartesian coordinates
    * @param opts Options of ARE commands ("no_head", "no_gaze", "no_sacc", "still", "left", "right" Please refer to http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html)
    * @param shouldWait is the function blocking ? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool take(const yarp::sig::Vector &target, const std::string &opts = "", const bool shouldWait = true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(CMD_TAKE);
        appendCartesianTarget(bCmd, target);
        if (!opts.empty())
        {
            yarp::os::Bottle params(opts.c_str());
            bCmd.append(params);
        }
        return sendCmd(bCmd, shouldWait);
    }

    /**
    * the robot tries to reach the specified [target] from one side and then push it laterally. Optional parameter "away" can be supplied in order to have the robot push the object away from its root reference frame.
    * @param target Target to grasp in cartesian coordinates
    * @param opts Options of ARE commands ("no_head", "no_gaze", "no_sacc", "still", "left", "right" Please refer to http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html)
    * @param shouldWait is the function blocking ? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool push(const yarp::sig::Vector &target, const std::string &opts = "", const bool shouldWait = true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(CMD_PUSH);
        appendCartesianTarget(bCmd, target);
        if(!opts.empty())
        {
            yarp::os::Bottle params(opts.c_str());
            bCmd.append(params);
        }
        return sendCmd(bCmd, shouldWait);
    }

    /**
    * the robot tries to point the specified [target] with its index finger. 
    * @param target Target to grasp in cartesian coordinates
    * @param opts Options of ARE commands ("no_head", "no_gaze", "no_sacc", "still", "left", "right" Please refer to http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html)
    * @param shouldWait is the function blocking ? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool point(const yarp::sig::Vector &target, const std::string &opts = "", const bool shouldWait = true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(CMD_POINT);
        appendCartesianTarget(bCmd, target);
        if(!opts.empty())
        {
            yarp::os::Bottle params(opts.c_str());
            bCmd.append(params);
        }
        return sendCmd(bCmd, shouldWait);
    }

    /**
    * if the robot is holding an object it brings it over the table and drops it on a random position approximatively in front of it. 
    * @param opts Options of ARE commands ("no_head", "no_gaze", "no_sacc", "still", "left", "right" Please refer to http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html)
    * @param shouldWait is the function blocking ? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool drop(const std::string &opts = "", const bool shouldWait = true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(CMD_DROP);
        if(!opts.empty())
        {
            yarp::os::Bottle params(opts.c_str());
            bCmd.append(params);
        }
        return sendCmd(bCmd, shouldWait);
    }

    /**
    * Drop the object on a given target
    * @param target Target where to drop in cartesian coordinates
    * @param opts Options of ARE commands ("no_head", "no_gaze", "no_sacc", "still", "left", "right" Please refer to http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html)
    * @param shouldWait is the function blocking ? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool dropOn(const yarp::sig::Vector &target, const std::string &opts = "", const bool shouldWait = true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(CMD_DROP);
        bCmd.addString("over");
        appendCartesianTarget(bCmd, target);
        if(!opts.empty())
        {
            yarp::os::Bottle params(opts.c_str());
            bCmd.append(params);
        }
        return sendCmd(bCmd, shouldWait);
    }

    /**
    * Bring the hand in the visual field and move it with the purpose of visual exploration.
    * @param opts Options of ARE commands ("no_head", "no_gaze", "no_sacc", "still", "left", "right" Please refer to http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html)
    * @param shouldWait is the function blocking ? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool observe(const std::string &opts = "", const bool shouldWait = true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(CMD_OBSERVE);
        if(!opts.empty())
        {
            yarp::os::Bottle params(opts.c_str());
            bCmd.append(params);
        }
        return sendCmd(bCmd, shouldWait);
    }

    /**
    * The robot puts one arm forward with the palm of the hand facing up and waiting for an object to be put on it.
    * @param opts Options of ARE commands ("no_head", "no_gaze", "no_sacc", "still", "left", "right" Please refer to http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html)
    * @param shouldWait is the function blocking ? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool expect(const std::string &opts = "", const bool shouldWait = true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(CMD_EXPECT);
        if(!opts.empty())
        {
            yarp::os::Bottle params(opts.c_str());
            bCmd.append(params);
        }
        return sendCmd(bCmd, shouldWait);
    }

    /**
    * The robot puts one arm forward with the palm of the hand facing up and opens the fingers so that the object held in the hand is free to be taken.
    * @param opts Options of ARE commands ("no_head", "no_gaze", "no_sacc", "still", "left", "right" Please refer to http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html)
    * @param shouldWait is the function blocking ? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool give(const std::string &opts = "", const bool shouldWait = true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(CMD_GIVE);
        if(!opts.empty())
        {
            yarp::os::Bottle params(opts.c_str());
            bCmd.append(params);
        }
        return sendCmd(bCmd, shouldWait);
    }
};


}}//Namespace
#endif



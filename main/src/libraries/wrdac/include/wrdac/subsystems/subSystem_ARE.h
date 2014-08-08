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

#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "wrdac/subsystems/subSystem.h"
#include "wrdac/knowledge/object.h"

#define SUBSYSTEM_ARE       "ARE"

namespace wysiwyd {
namespace wrdac {

/**
* \ingroup wrdac_clients
*
* SubSystem to deal with the <b>actionsRenderingEngine</b> 
* module (a.k.a. <b>ARE</b>) for motor control. 
*  
* For further details, please refer to the ARE main page:  
* http://wiki.icub.org/iCub_documentation/group__actionsRenderingEngine.html 
*/
class SubSystem_ARE : public SubSystem
{
protected:
    yarp::os::RpcClient cmdPort;

    /********************************************************************************/
    void appendCartesianTarget(yarp::os::Bottle& b, const yarp::sig::Vector &t)
    {
        yarp::os::Bottle &sub=b.addList();
        sub.addString("cartesian");
        for (size_t i=0; i<t.length(); i++)
            sub.addDouble(t[i]);
    }

    /********************************************************************************/
    bool sendCmd(yarp::os::Bottle &cmd, const bool shouldWait=true)
    {        
        if (shouldWait)
        {
            yarp::os::Bottle bReply;
            if (cmdPort.write(cmd,bReply))
                return (bReply.get(0).asVocab()==yarp::os::Vocab::encode("ack"));
        }
        else
            return cmdPort.asPort().write(cmd);

        return false;
    }

    /********************************************************************************/
    bool connect() 
    { 
        return yarp::os::Network::connect(cmdPort.getName(),"/actionsRenderingEngine/cmd:io");
    }

public:
    /**
    * Default constructor.
    * @param masterName stem-name used to open up ports.
    */
    SubSystem_ARE(const std::string &masterName) : SubSystem(masterName)
    {
        cmdPort.open(("/"+masterName+"/"+SUBSYSTEM_ARE+"/cmd:io").c_str());
        m_type=SUBSYSTEM_ARE;
    }

    /**
    * Clean up resources.
    */
    void Close()
    { 
        cmdPort.interrupt();
        cmdPort.close();
    }

    /**
    * Put the specified part ih home position.
    * @param part the part to be homed ("gaze", "head", "arms", 
    *             "fingers", "all"; "all" by default).
    * @param shouldWait is the function blocking?
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool home(const std::string &part="all", const bool shouldWait=true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(yarp::os::Vocab::encode("home"));
        bCmd.addString(part.c_str());
        return sendCmd(bCmd,shouldWait);
    }

    /**
    * Reach the specified [target] and grasp it. Optional parameter
    * "side" or "above" can be supplied to choose the orientation 
    * the robot should keep while performing the action (default: 
    * "above"). 
    * @param target Target to grasp in cartesian coordinates
    * @param options Options of ARE commands ("no_head", "no_gaze", 
    *             "no_sacc", "still", "left", "right").
    * @param shouldWait is the function blocking? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool take(const yarp::sig::Vector &target, const yarp::os::Bottle &options=yarp::os::Bottle(),
              const bool shouldWait=true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(yarp::os::Vocab::encode("take"));
        appendCartesianTarget(bCmd,target);
        bCmd.append(options);
        return sendCmd(bCmd,shouldWait);
    }

    /**
    * Reach the specified [target] from one side and then push it
    * laterally. Optional parameter "away" can be supplied in order 
    * to have the robot push the object away from its root reference 
    * frame. 
    * @param target Target to grasp in cartesian coordinates
    * @param options Options of ARE commands ("no_head", "no_gaze", 
    *             "no_sacc", "still", "left", "right").
    * @param shouldWait is the function blocking? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool push(const yarp::sig::Vector &target, const yarp::os::Bottle &options=yarp::os::Bottle(),
              const bool shouldWait=true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(yarp::os::Vocab::encode("push"));
        appendCartesianTarget(bCmd,target);
        bCmd.append(options);
        return sendCmd(bCmd,shouldWait);
    }

    /**
    * Point at the specified [target] with the index finger.
    * @param target Target to grasp in cartesian coordinates
    * @param options Options of ARE commands ("no_head", "no_gaze", 
    *             "no_sacc", "still", "left", "right").
    * @param shouldWait is the function blocking? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool point(const yarp::sig::Vector &target, const yarp::os::Bottle &options=yarp::os::Bottle(),
               const bool shouldWait=true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(yarp::os::Vocab::encode("point"));
        appendCartesianTarget(bCmd,target);
        bCmd.append(options);
        return sendCmd(bCmd,shouldWait);
    }

    /**
    * If an object is held, bring it over the table and drop it on a
    * random position. 
    * @param options Options of ARE commands ("no_head", "no_gaze", 
    *             "no_sacc", "still", "left", "right").
    * @param shouldWait is the function blocking? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool drop(const yarp::os::Bottle &options=yarp::os::Bottle(), const bool shouldWait=true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(yarp::os::Vocab::encode("drop"));
        bCmd.append(options);
        return sendCmd(bCmd,shouldWait);
    }

    /**
    * Drop the object on a given target.
    * @param target Target where to drop in cartesian coordinates
    * @param options Options of ARE commands ("no_head", "no_gaze", 
    *             "no_sacc", "still", "left", "right").
    * @param shouldWait is the function blocking? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool dropOn(const yarp::sig::Vector &target, const yarp::os::Bottle &options=yarp::os::Bottle(),
                const bool shouldWait=true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(yarp::os::Vocab::encode("drop"));
        bCmd.addString("over");
        appendCartesianTarget(bCmd,target);
        bCmd.append(options);
        return sendCmd(bCmd,shouldWait);
    }

    /**
    * Bring the hand in the visual field and move it with the 
    * purpose of visual exploration.
    * @param options Options of ARE commands ("no_head", "no_gaze", 
    *             "no_sacc", "still", "left", "right").
    * @param shouldWait is the function blocking? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool observe(const yarp::os::Bottle &options=yarp::os::Bottle(), const bool shouldWait=true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(yarp::os::Vocab::encode("observe"));
        bCmd.append(options);
        return sendCmd(bCmd,shouldWait);
    }

    /**
    * Put one hand forward with the palm facing up and wait for an
    * object. 
    * @param options Options of ARE commands ("no_head", "no_gaze", 
    *             "no_sacc", "still", "left", "right").
    * @param shouldWait is the function blocking? 
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool expect(const yarp::os::Bottle &options=yarp::os::Bottle(), const bool shouldWait=true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(yarp::os::Vocab::encode("expect"));
        bCmd.append(options);
        return sendCmd(bCmd,shouldWait);
    }

    /**
    * Put one hand forward with the palm facing up and open the 
    * fingers so that the object held in the hand is free to be 
    * taken. 
    * @param options Options of ARE commands ("no_head", "no_gaze", 
    *             "no_sacc", "still", "left", "right").
    * @param shouldWait is the function blocking?
    * @return true in case of successfull motor command, false 
    *         otherwise.
    */
    bool give(const yarp::os::Bottle &options=yarp::os::Bottle(), const bool shouldWait=true)
    {
        yarp::os::Bottle bCmd;
        bCmd.addVocab(yarp::os::Vocab::encode("give"));
        bCmd.append(options);
        return sendCmd(bCmd,shouldWait);
    }
};

}
}

#endif



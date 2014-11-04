/* 
 * Copyright (C) 2011 EFAA Consortium, European Commission FP7 Project IST-270490
 * Authors: Stephane Lallee
 * email:   stephane.lallee@gmail.com
 * website: http://efaa.upf.edu/ 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * $EFAA_ROOT/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/** 
\defgroup choregraphyServer choregraphyServer
 
@ingroup efaa_modules
 
Module responsible for providing a server-based implementation to the choregraphy capabilities.

\section intro_sec Description 
 
The purpose of this module is to make use of the choregraphy part of EFAA Helpers in order to allow a rpc access to them.
 
\section lib_sec Libraries 
- YARP libraries
- EFAA Helpers

\section parameters_sec Parameters
--name <string> 
- To specify the module's name; all the open ports will be 
  tagged with the prefix /<moduleName>/. If not specified
  \e choregraphy.
 
\section portsa_sec Ports Accessed
Expect the CTP services to be running for head/torso/arms.

\section portsc_sec Ports Created
- /moduleName/rpc port is the way to interact with the module. Commands are:

@b "chore <name> <speed>" play the choregraphy "name"  with a speed ratio of speed (1.0 by default) 

\section tested_os_sec Tested OS
Linux and Windows.

\author Stephane Lallee
*/ 

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include "wrdac/helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;



/************************************************************************/
class ChoregraphyModule: public RFModule
{
    wysiwyd::wrdac::ICubClient *icub;
    Port rpc;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {    
        setName( rf.check("name",Value("choregraphy")).asString().c_str() );
	string context = rf.getContext();
	string configFile = rf.check("from",Value("client.ini")).asString();
        icub = new ICubClient(getName().c_str(), context, configFile, true);
        icub->connect();

        //Create RPC
        string rpcName = "/";
        rpcName += getName() + "/rpc";
        rpc.open(rpcName.c_str());
        attach(rpc); 

        return true;
    }

    /************************************************************************/
    bool respond(const Bottle& cmd, Bottle& reply) 
    {
        std::string keyWord = cmd.get(0).asString().c_str();
        if (keyWord == "chore")
        {
            std::string choreName = cmd.get(1).asString().c_str();
            double speedRation = 1.0;
            if (cmd.size()== 3)
                speedRation = cmd.get(2).asDouble();
            if (icub->playChoregraphy(choreName,speedRation,true))
                reply.addString(getName()+" : ack.");
            else
                reply.addString(getName()+" : nack.");
        }
        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        rpc.interrupt();

        return true;
    }

    /************************************************************************/
    bool close()
    {
        rpc.close();
        icub->close();
        delete icub;
        return true;
    }

    /************************************************************************/
    bool updateModule()
    {
        //cout<<"Reference frame handler is running happily..."<<endl;
        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.5;
    }
};

////////////////////////MAIN//////////////////////////////////////////

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);
    ChoregraphyModule mod;

    return mod.runModule(rf);
}


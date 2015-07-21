/*
* Copyright (C) 2015 WYSIWYD
* Authors: Matej Hoffmann  and Ugo Pattacini
* email:   matej.hoffmann@iit.it, ugo.pattacini@itt.it
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

/**
\defgroup cartControlReachAvoid cartControlReachAvoid

@ingroup allostaticControl

Reaching while avoiding obstacles on the run. 

Date first release: 23/04/2015

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
Reaching while avoiding obstacles on the run. 

\section lib_sec Libraries 
YARP, ICUB libraries 

\section parameters_sec Parameters

--context       \e path
- Where to find the called resource.

--from          \e from
- The name of the .ini file with the configuration parameters.

--name          \e name
- The name of the module (default virtualContactGeneration).

--robot         \e rob
- The name of the robot (either "icub" or "icub"). Default icubSim.

--rate          \e rate
- The period used by the thread. Default 100ms, i.e. 10 Hz.

--verbosity  \e verb
- Verbosity level (default 0). The higher is the verbosity, the more
  information is printed out.



\section portsc_sec Ports Created
- <i> /<name>/reachingTarget:i </i> Position of target to reach in root FoR.
- <i> /<name>/avoidanceVectors:i </i> 1 to n avoidance vectors as supplied by the pps module - position and normal;  in root FoR.
- <i> /<name>/reachingGain:i </i> Gain for the reaching behavior (from allostasis module) 
- <i> /<name>/avoidanceGain:i </i> Gain for the avoidance behavior (from allostasis module) 

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04).

\author: Matej Hoffmann and Ugo Pattacini
*/ 
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include "cartControlReachAvoidThread.h"

using namespace yarp;
using namespace yarp::os;
using namespace std;

/**
* \ingroup cartControlReachAvoid
*
* The module that achieves combining of cartesian control for reaching while avoiding obstacles.
*  
*/

class cartControlReachAvoid: public RFModule 
{
private:
    cartControlReachAvoidThread *cartCtrlReachAvoidThrd;
    Port           rpcPort;
  
    string robot;
    string name;
    int verbosity;
    int threadPeriod;
    
public:
    cartControlReachAvoid()
    {
        cartCtrlReachAvoidThrd = 0;
    }

    bool configure(ResourceFinder &rf)
    {
        
        name = "cartControlReachAvoid";
        robot =  "icubSim";
        threadPeriod = 100; //period of the virtContactGenThread in ms
        verbosity = 0;
      
        //******************************************************
        //********************** CONFIGS ***********************

       //******************* NAME ******************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                yInfo("Module name set to %s", name.c_str());
            }
            else yInfo("Module name set to default, i.e. %s", name.c_str());
            setName(name.c_str());

        //******************* ROBOT ******************
            if (rf.check("robot"))
            {
                robot = rf.find("robot").asString();
                yInfo("Robot is: %s", robot.c_str());
            }
            else yInfo("Could not find robot option in the config file; using %s as default",robot.c_str());

        //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                yInfo("doubleTouchThread verbosity set to %i", verbosity);
            }
            else yInfo("Could not find verbosity option in the config file; using %i as default",verbosity);

        //****************** rate ******************
            if (rf.check("rate"))
            {
                threadPeriod = rf.find("rate").asInt();
                yInfo("doubleTouchThread rateThread working at period of %i ms.",threadPeriod);
            }
            else yInfo("Could not find rate in the config file; using %i as default",threadPeriod);
            
        
        //******************************************************
        //*********************** THREAD **********************
        cartCtrlReachAvoidThrd = new cartControlReachAvoidThread(threadPeriod,name,robot,verbosity,rf);
          
        if (!cartCtrlReachAvoidThrd -> start())
        {
              delete cartCtrlReachAvoidThrd;
              cartCtrlReachAvoidThrd = 0;
              yError("cartControlReachAvoidThread wasn't instantiated!!");
                    return false;
        }
        yInfo("cartControlReachAvoidThread instantiated...");

        /*if (rf.check("autoconnect"))
        {
            if (Network::connect(("/"+name+"/virtualContacts:o").c_str(),"/doubleTouch/contacts:i"))
            {
                yInfo("Autoconnection to doubleTouch port, i.e. /doubleTouch/contacts:i, successful!");
            }
        }*/
        
        rpcPort.open("/"+getName("/rpc:i"));
        attach(rpcPort);
        
        return true;
    }
    
    bool respond(const Bottle &command, Bottle &reply)
    {
        Vector target_pos;
        int ack =Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        target_pos.resize(3,0.0);
        
        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case VOCAB2('g','o'):
                //case VOCAB2('G','O'):
                {
                    
                    //int res=Vocab::encode("new target");
                    if (command.size()==4){ //"go x y z"
                        target_pos(0)=command.get(1).asDouble();
                        target_pos(1)=command.get(2).asDouble();
                        target_pos(2)=command.get(3).asDouble();
                        //printf("reachingModule():respond: Will set target to %f %f %f.\n",target_pos[0],target_pos[1],target_pos[2]);
                        if (cartCtrlReachAvoidThrd-> setTargetFromRPC(target_pos))
                        {
                            reply.addVocab(ack);
                            reply.addVocab(VOCAB2('g','o'));
                            reply.addDouble(target_pos(0));
                            reply.addDouble(target_pos(1));
                            reply.addDouble(target_pos(2));
                        }
                        else{
                            reply.addVocab(nack);
                        }
                    }
                    else{
                        reply.addVocab(nack);
                    }
                    
                   // reply.addVocab(res);
                    return true;
                }
                case VOCAB4('h','o','m','e'):
                //case VOCAB4('H','O','M','E'):
                     //int res2=Vocab::encode("home");
                     if (cartCtrlReachAvoidThrd -> setHomeFromRPC())
                     {
                            reply.addVocab(ack);
                            reply.addVocab(VOCAB4('h','o','m','e'));
                     }
                     else{
                        reply.addVocab(nack);
                     }
                     return true;
                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    
    bool close()
    {
        yInfo("cartControlReachAvoid: Stopping thread..");
        if (cartCtrlReachAvoidThrd)
        {
            cartCtrlReachAvoidThrd -> stop();
            delete cartCtrlReachAvoidThrd;
            cartCtrlReachAvoidThrd =  0;
        }
        
        return true;
    }

    double getPeriod()
    {
        return 1.0;
    }

    bool updateModule()
    {
        return true;
    }
    
};



/**
* Main function.
*/
int main(int argc, char * argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("ppsAllostatic");
    rf.setDefaultConfigFile("cartControlReachAvoid.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {   
        yInfo(""); 
        yInfo("Options:");
        yInfo("");
        yInfo("   --context    path:  where to find the called resource");
        yInfo("   --from       from:  the name of the .ini file.");
        yInfo("   --name       name:  the name of the module (default cartControlReachAvoid).");
        yInfo("   --robot      robot: the name of the robot. Default icubSim.");
        yInfo("   --rate       rate:  the period used by the thread. Default 100ms.");
        yInfo("   --verbosity  int:   verbosity level (default 0).");
           
        yInfo("");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    cartControlReachAvoid cCRA;
    return cCRA.runModule(rf);
}
// empty line to make gcc happy
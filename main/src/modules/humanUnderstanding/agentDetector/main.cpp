/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Stéphane Lallée, moved from EFAA by Maxime Petit
 * email:   stephane.lallee@gmail.com
 * website: http://wysiwyd.upf.edu/ 
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
\defgroup agentDetector agentDetector

@ingroup efaa_modules

Module based on \ref kinectClient "Kinect Wrapper Client" that track and recognize people.

\section intro_sec Description
This module receive information from the Kinect wrapper, performs facial identification and convert joints
to the iCub referential.

It requires the \ref kinectServer an OPC and RFH running.

\section lib_sec Libraries
- YARP libraries.
- \ref kinect "KinectWrapper" library.

\section parameters_sec Parameters
--verbosity \e verbosity
- specify the verbosity level of the client print-outs.

--carrier \e carrier
- specify the protocol used to connect to the server ports.

--remote \e remote
- specify the kinectServer name to connect to.

--opc \e opc
- specify the OPC name to connect to.

--rfh \e rfh
- specify the RFH name to connect to.

--name \e name
- specify the kinectClient stem-name.

\section tested_os_sec Tested OS
Windows, Linux

\author Stéphane Lallée
*/

#include <yarp/os/Network.h>
#include "AgentDetector.h"
using namespace yarp::os;
int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stdout, "Yarp network not available\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("agentDetector");
    rf.setDefaultConfigFile("agentDetector.ini");
    rf.configure(argc,argv);

    AgentDetector mod;

    mod.runModule(rf);

    return 0;
}


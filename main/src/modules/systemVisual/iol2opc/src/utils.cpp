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

#include <yarp/os/Time.h>

#include "utils.h"
#include "module.h"


/**********************************************************/
RtLocalization::RtLocalization() : RateThread(30)
{
    bridge=NULL;
}


/**********************************************************/
void RtLocalization::setBridge(IOL2OPCBridge *bridge)
{
    this->bridge=bridge;
}


/**********************************************************/
bool RtLocalization::threadInit()
{
    return ((bridge!=NULL) && (getRate()!=0.0));
}


/**********************************************************/
void RtLocalization::run()
{
    bridge->doLocalization();
}


/**********************************************************/
MemoryUpdater::MemoryUpdater() : RateThread(100)
{
    bridge=NULL;
}


/**********************************************************/
void MemoryUpdater::setBridge(IOL2OPCBridge *bridge)
{
    this->bridge=bridge;
}


/**********************************************************/
bool MemoryUpdater::threadInit()
{
    return ((bridge!=NULL) && (getRate()!=0.0));
}


/**********************************************************/
void MemoryUpdater::run()
{
    bridge->updateMemory();
}




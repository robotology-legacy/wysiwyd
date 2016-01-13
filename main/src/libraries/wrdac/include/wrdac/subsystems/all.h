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

#ifndef __EFAA_SUBSYSTEM_ALL_H__
#define __EFAA_SUBSYSTEM_ALL_H__

#define SUBSYSTEM               "abstract"

#include "subSystem.h"

namespace wysiwyd{
    namespace wrdac{
        class SubSystem_ABM;
        class SubSystem_agentDetector;
        class SubSystem_ARE;
        class SubSystem_Attention;
        class SubSystem_babbling;
        class SubSystem_Expression;
        class SubSystem_iKart;
        class SubSystem_IOL2OPC;
        class SubSystem_Postures;
        class SubSystem_Reactable;
        class SubSystem_Speech;
        class SubSystem_Recog;
        class SubSystem_LRH;
        class SubSystem_SlidingController;
    }
}

#endif

/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Ugo Pattacini
 * email:   ugo.pattacini@iit.it
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

#ifndef __wysiwydIDLClients_H__
#define __wysiwydIDLClients_H__

// whenever a new service (e.g. new_service_IDL) becomes available,
// you have to add up two more lines to this file:
// 1. #include "new_service_IDL.h"
// 2. @ref new_service_IDL

#include "slidingController_IDL.h"
#include "interpersonalDistanceRegulator_IDL.h"
#include "iol2opc_IDL.h"
#include "perspectiveTaking_IDL.h"

/**
*
* @defgroup wysiwyd_wysiwydIDLClients IDL Client Interfaces
* @ingroup wysiwyd_libraries
*
* This library contains all the definitions for a client to
* interface to the IDL services made available by WYSIWYD
* software. \n
* Available services are listed below: \n
* - @ref interpersonalDistanceRegulator_IDL
* - @ref slidingController_IDL
* - @ref cvz_IDL
* - @ref cvzMmcm_IDL
*/

#endif

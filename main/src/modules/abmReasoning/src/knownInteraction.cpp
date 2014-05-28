// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2012 EFAA Consortium, European Commission FP7 Project IST-270490
* Authors: Grégoire Pointeau, Maxime Petit
* email:   gregoire.pointeau@inserm.fr, maxime.petit@inserm.fr
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

\section tested_os_sec Tested OS
Windows

\author Grégoire Pointeau, Maxime Petit
*/ 

#include <efaa/knownInteraction.h>


void knownInteraction::addInteraction(tuple<string, int, string, string> tInput)
{
	bool bFound = false;
	for (vector<tuple<string, int, string, string>>::iterator itTuple = listInteraction.begin() ; itTuple != listInteraction.end() ; itTuple++)
	{
		bool test1 = get<0>(tInput).c_str() == get<0>(*itTuple).c_str();
		bool test2 = get<0>(tInput) == get<0>(*itTuple).c_str();
		bool test3 = get<0>(tInput) == get<0>(*itTuple);
		bool test4 = get<0>(tInput).c_str() == get<0>(*itTuple);

		if (get<0>(tInput) == get<0>(*itTuple) && get<2>(tInput) == get<2>(*itTuple) && get<3>(tInput) == get<3>(*itTuple) && !bFound) 
		{
			get<1>(*itTuple)++;
			bFound =true;
		}
	}
	if (!bFound)
	{
		listInteraction.push_back(tInput);
	}
}
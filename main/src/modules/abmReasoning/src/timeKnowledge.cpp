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

#include <timeKnowledge.h>


/*
* Transform a shared plan in a timeKnowledge
* input format : 
* <string name> <string timeArg1> <string timeArg2>
*/
void timeKnowledge::fromBottle(Bottle bInput)
{
	if (bInput.size() !=3)
	{
		std::cout << "Error in timeKnowledge : fromBottle. Wrong number of inputs (need 3 inputs : <string name> <string timeArg1> <string timeArg2>)" << endl;
		return;
	}
	if ( !(bInput.get(0).isString() && bInput.get(1).isString() && bInput.get(2).isString()) )
	{
		std::cout << "Error in timeKnowledge : fromBottle. Wrong format of inputs (need 3 inputs : <string name> <string timeArg1> <string timeArg2>)" << endl;
		return;
	}
	
	sTemporal = bInput.get(0).asString();
	struct tm	tmTimeArg1 = abmReasoningFunction::string2Time(bInput.get(1).asString().c_str()),
				tmTimeArg2 = abmReasoningFunction::string2Time(bInput.get(2).asString().c_str()) ;

	timeArg1.push_back(tmTimeArg1);
	timeArg2.push_back(tmTimeArg2);
}

/**
* Add a new entity to the timeKnowledge
* input format : 
* <string name> <string timeArg1> <string timeArg2>
*/
void timeKnowledge::addKnowledge(Bottle bInput)
{
	if (bInput.size() !=3)
	{
		std::cout << "Error in addKnowledge : fromBottle. Wrong number of inputs (need 3 inputs : <string name> <string timeArg1> <string timeArg2>)" << endl;
		return;
	}
	if ( !(bInput.get(0).isString() && bInput.get(1).isString() && bInput.get(2).isString()) )
	{
		std::cout << "Error in addKnowledge : fromBottle. Wrong format of inputs (need 3 inputs : <string name> <string timeArg1> <string timeArg2>)" << endl;
		return;
	}
	
	struct tm	tmTimeArg1 = abmReasoningFunction::string2Time(bInput.get(1).asString().c_str()),
				tmTimeArg2 = abmReasoningFunction::string2Time(bInput.get(2).asString().c_str()) ;

	timeArg1.push_back(tmTimeArg1);
	timeArg2.push_back(tmTimeArg2);
}


struct tm timeKnowledge::timeDiff(struct tm tm1, struct tm tm2)
{
//	struct tm diffTime;
	time_t myTime;
	time(&myTime);					// get unix time
	tm *diffTime = localtime(&myTime);		// conversion in local time
	return *diffTime;
}

/**
* Return the percentage of action where T1 is inferior to T2
*/
double timeKnowledge::T1inferiorT2percent()
{
	if (timeArg1.size() == timeArg2.size())
	{
		iSize = timeArg1.size();
	}
	else
	{
		std::cout << "Error in timeKnowledge::T1inferiorT2percent() | wrong size of timeKnowledge" << endl;
		return 0.5;
	}

	int inferior = 0,
		superior = 0;

	for (int i = 0; i < iSize; i++)
	{
		if (abmReasoningFunction::timeDiff(timeArg1[i], timeArg2[i]))
		{
			inferior++;
		}
		else
		{
			superior++;
		}
	}

	return ((inferior*1.)/(inferior*1.+superior*1.));
}
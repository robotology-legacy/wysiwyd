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

#include <efaa/grammarKnowledge.h>


bool pronom::AddInstance(Bottle bInput)
{
	string	sSpeaker = bInput.get(0).toString().c_str(),
			sAddressee = bInput.get(1).toString().c_str(),
			sAgent = bInput.get(3).toString().c_str();

	m3Data.incr(sSpeaker, sAddressee, sAgent);


	if (sSubject == sSpeaker)	pSubject_Is_Speaker.first++;
	pSubject_Is_Speaker.second++;

	if (sSubject == sAddressee)	pSubject_Is_Addressee.first++;
	pSubject_Is_Addressee.second++;

	if (sSubject == sAgent) pSubject_Is_Agent.first++;
	pSubject_Is_Agent.second++;

	


	// TODO : TO BE REMOVE

	// speaker = addressee
	if (sSpeaker == sAddressee)
		pSpeaker_Is_Addressee.first++;
	pSpeaker_Is_Addressee.second++;

	// speaker = subject
	if (sSpeaker == sSubject)
		pSpeaker_Is_Subject.first++;
	pSpeaker_Is_Subject.second++;

	// speaker = agent
	if (sSpeaker == sAgent)
		pSpeaker_Is_Agent.first++;
	pSpeaker_Is_Agent.second++;

	// addressee = subject
	if (sAddressee == sSubject)
		pAddressee_Is_Subject.first++;
	pAddressee_Is_Subject.second++;

	// addressee = agent
	if (sAgent == sAddressee)
		pAddressee_Is_Agent.first++;
	pAddressee_Is_Agent.second++;

	// subject = agent
	if (sSubject == sAgent)
		pSubject_Is_Agent.first++;
	pSubject_Is_Agent.second++;
	
	// addressee is
	bool fAddressee = false;
	for (vector<pair<string, int> >::iterator it_RE = vAddresseeIs.begin() ; it_RE != vAddresseeIs.end() ; it_RE++)
	{
		if (it_RE->first == sAddressee && !fAddressee)
		{
			fAddressee = true;
			it_RE->second++;
		}
	}
	if (!fAddressee)
	{
		pair<string, int> pTemp(sAddressee,1);
		vAddresseeIs.push_back(pTemp);
	}

	// speaker is
	bool fSpeaker = false;
	for (vector<pair<string, int> >::iterator it_SP = vSpeakerIs.begin() ; it_SP != vSpeakerIs.end() ; it_SP++)
	{
		if (it_SP->first == sSpeaker && !fSpeaker)
		{
			fSpeaker = true;
			it_SP->second++;
		}
	}
	if (!fSpeaker)
	{
		pair<string, int> pTemp(sSpeaker,1);
		vSpeakerIs.push_back(pTemp);
	}

	// agent is
	bool fAgent = false;
	for (vector<pair<string, int> >::iterator it_Ag = vAgentIs.begin() ; it_Ag != vAgentIs.end() ; it_Ag++)
	{
		if (it_Ag->first == sAgent && !fAgent)
		{
			fAgent = true;
			it_Ag->second++;
		}
	}
	if (!fAgent)
	{
		pair<string, int> pTemp(sAgent,1);
		vAgentIs.push_back(pTemp);
	}

	return true;
}
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

#include <behavior.h>

Bottle behavior::getConsequence()
{
    Bottle bOutput;
    bOutput.addString(sName.c_str());

    vector <pair <string, vector <double> > >   vDrive;

    // for each time the behavior has been seen
    for (vector < vector< pair <string, double> > >::iterator it_occurence = vEffect.begin() ; it_occurence !=vEffect.end() ; it_occurence++ )
    {
        // for each drive changed
        for (vector< pair <string, double> >::iterator it_drive = it_occurence->begin() ; it_drive != it_occurence->end() ; it_drive++)
        {

            bool driveKnown = false;

            // for each known effect
            for (vector <pair <string, vector <double> > >::iterator it_effect = vDrive.begin() ; it_effect != vDrive.end() ; it_effect ++)
            {

                // if the drive is already know
                if (it_effect->first == it_drive->first && !driveKnown)
                {
                    it_effect->second.push_back(it_drive->second);
                    driveKnown = true;
                }
            }
            // if first time we see this drive
            if (!driveKnown)
            {
                pair<string, vector <double> > newDrive;
                newDrive.first = it_drive->first;
                newDrive.second.push_back(it_drive->second);
                vDrive.push_back(newDrive);
            }
        }
    }

    Bottle bDrive;
    
    for (int i = 0 ; i < vDrive.size() ; i++)
    {
        bDrive.clear();
        bDrive.addString(vDrive[i].first.c_str());
        double sum = 0.;
        for (int j = 0 ; j < vDrive[i].second.size() ; j++)
        {
            sum += vDrive[i].second[j];
        }
        bDrive.addDouble(sum/(1.*vDrive[i].second.size()));
        bOutput.addList() = bDrive;
    }

    return bOutput;
}

Bottle behavior::getConsequence(int last)
{
    Bottle bOutput;
    bOutput.addString(sName.c_str());
    if (last < 1)
    {
        std::cout << "Error in behavior::getConsequence | number of last action to low" << endl;
        bOutput.addString("Error in behavior::getConsequence | number of last action to low");
        return bOutput;
    }

    vector <pair <string, vector <double> > >   vDrive;
    int loop = 0,
        legal = vEffect.size() - last;
    // for each time the behavior has been seen
    for (vector < vector< pair <string, double> > >::iterator it_occurence = vEffect.begin() ; it_occurence !=vEffect.end() ; it_occurence++ )
    {
        if (loop >= legal)
        {
            // for each drive changed
            for (vector< pair <string, double> >::iterator it_drive = it_occurence->begin() ; it_drive != it_occurence->end() ; it_drive++)
            {

                bool driveKnown = false;

                // for each known effect
                for (vector <pair <string, vector <double> > >::iterator it_effect = vDrive.begin() ; it_effect != vDrive.end() ; it_effect ++)
                {

                    // if the drive is already know
                    if (it_effect->first == it_drive->first && !driveKnown)
                    {
                        it_effect->second.push_back(it_drive->second);
                        driveKnown = true;
                    }
                }
                // if first time we see this drive
                if (!driveKnown)
                {
                    pair<string, vector <double> > newDrive;
                    newDrive.first = it_drive->first;
                    newDrive.second.push_back(it_drive->second);
                    vDrive.push_back(newDrive);
                }
            }
        }
        loop++;
    }

    Bottle bDrive;
    for (int i = 0 ; i < vDrive.size() ; i++)
    {
        bDrive.clear();
        bDrive.addString(vDrive[i].first.c_str());
        double sum = 0.;
        for (int j = 0 ; j < vDrive[i].second.size() ; j++)
        {
            sum += vDrive[i].second[j];
        }
        bDrive.addDouble(sum/(1.*vDrive[i].second.size()));
        bOutput.addList() = bDrive;
    }
    return bOutput;
}


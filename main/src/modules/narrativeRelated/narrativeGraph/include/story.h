/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau
 * email:   gregoire.pointeau@inserm.fr
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

#ifndef _STORY_H_
#define _STORY_H_


#include <evtStory.h>

class myTimeStruct{
public:
    struct tm m_tm;
    long int iMilliSec;

    std::string toString(){
        std::ostringstream osOut;
        osOut << m_tm.tm_mday << "/"
            << m_tm.tm_mon + 1 << "/"
            << m_tm.tm_year << " "
            << m_tm.tm_hour << ":"
            << m_tm.tm_min << ":"
            << m_tm.tm_sec;
        return osOut.str();
    }
};


class story{
public:

    int counter;

    int iBasedOn;   // indicate the counter of the story where it comes from if it is an imagined story. if real: -1

    void addOCW(std::vector<std::string> _OCW);
    void updateMapScore();
    unsigned int  iThresholdSentence; // threshold of the number of sentence in a story

    // list of the OCW inside the event in the ABM (naive story)
    std::vector<std::string>     vOCWEvt;

    //list of the OCW get from the human narration
    std::vector<std::string>     vOCWNarration;

    // matrix of the score of each OCW from vOCWEvt
    std::map<std::string, std::vector<double> > mapScoreEvent;

    // matrix of the score of each OCW from vOCWNarration;
    std::map<std::string, std::vector<double> > mapScoreNarration;


    std::vector<std::tuple<double, double, double, double> > vtPAOR;

    myTimeStruct    timeBegin;
    myTimeStruct    timeEnd;

    yarp::os::Bottle unfoldGoal(std::string);

    std::vector<std::string>    sentenceStory;
    std::vector<std::string>    humanNarration;
    std::vector<std::string>    meaningStory;       // human narration

    bool displayNarration(bool displayAll = true);

    std::vector<int>        viInstances;
    std::vector<evtStory>   vEvents;

    std::string toString();


};

#endif
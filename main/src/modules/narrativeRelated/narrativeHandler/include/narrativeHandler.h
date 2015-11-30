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



#include <story.h>


class narrativeHandler : public yarp::os::RFModule {
private:

    wysiwyd::wrdac::ICubClient  *iCub;

    double      period;
    bool abm;

    double dThresholdDiffStory; // threshold in second between to action from two different stories.
    unsigned int  iThresholdSizeStory; // threshold of the number of instance in a story
    yarp::os::Port  rpcPort;

    std::vector<story> listStories;

    void tellingStory(story st);


public:
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    void findStories(int iInstance = 0);

    double timeDiff(myTimeStruct tm1, myTimeStruct tm2, bool bPrint = false);
    myTimeStruct string2Time(std::string sTime);

    //RPC & scenarios
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
};

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
#include "wrdac/clients/icubClient.h"
#include "wrdac/subsystems/subSystem_recog.h"
#include "wrdac/subsystems/subSystem_LRH.h"
#include "wrdac/subsystems/subSystem_speech.h"


class narrativeHandler : public yarp::os::RFModule {
private:

    wysiwyd::wrdac::ICubClient  *iCub;
    int counter;

    double      period;
    bool lrh;

    double dThresholdDiffStory; // threshold in second between to action from two different stories.
    unsigned int  iThresholdSizeStory; // threshold of the number of instance in a story
	unsigned int  iThresholdSentence; // threshold of the number of sentence in a story
	unsigned int  iMinInstance; // threshold of the number of sentence in a story
	yarp::os::Port  rpcPort;

    std::vector<story> listStories;

    bool tellingStoryFromMeaning(story st);
    void initializeStories();

    yarp::os::Bottle unfoldGoal(std::string goal);
    void updateScoreStory(story &st);
    std::string narrator;
    std::vector<std::string> initializeEVT(evtStory &evt, int _instance, yarp::os::Bottle bActivity, yarp::os::Bottle bArguments, yarp::os::Bottle _bRelations);

    std::vector<std::pair<std::string, std::string> >  comparator;

    bool checkListPAOR(std::vector<std::string> vOriginal, std::vector<std::string> vCopy);
    std::string adaptMeaning(std::string oriMeaning);

    void addNarrationToStory(story &target, bool overWrite = false);
    std::string grammarToString(std::string sPath);

    std::string GrammarNarration; // the file for the grammar narration
    std::string GrammarYesNo;

	bool narrate();
	bool askNarrate();
	bool narrationToSpeech(story sto);

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
    void compareNarration(story &target); // try to tell the story target based on the other known stories
    void sayNarrationSimple(story target);
    void createNarration(story &sto);
    std::string createMeaning(std::string agent, std::string predicate, std::string object = "", std::string recipient = "");


};

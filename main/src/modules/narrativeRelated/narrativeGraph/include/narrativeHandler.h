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


#ifndef _NARRATIVEHANDLER_H_
#define _NARRATIVEHANDLER_H_

#include <storygraph.h>

class narrativeHandler : public yarp::os::RFModule {
private:

    wysiwyd::wrdac::ICubClient  *iCub;
    wysiwyd::wrdac::OPCClient   *mentalOPC;
    int counter;
    int cursorStories;
    void cleanMental();

    std::map<int, std::vector<std::string>>  listAutoScenarios;
    std::map<int, std::vector<std::string>>  listAutoMeaning;
    std::map<int, std::vector<std::string>>  listAutoNaives;
    void initializeScenarios(yarp::os::Bottle bNarration, yarp::os::ResourceFinder &rf);
    void initializeMeaning(yarp::os::Bottle bMeaning, yarp::os::ResourceFinder &rf);
    void initializeNaives(yarp::os::Bottle bNaives, yarp::os::ResourceFinder &rf);
    void NaiveToPAOR();
    void checkScenarios(int iSce = -1);


    double      period;
    bool lrh;

    double dThresholdDiffStory; // threshold in second between to action from two different stories.
    unsigned int  iThresholdSizeStory; // threshold of the number of instance in a story
    unsigned int  iThresholdSentence; // threshold of the number of sentence in a story
    unsigned int iThresholdScoreIGARFPAOR;  // threshold of confidence or relation between an IGARF and a PAOR

    unsigned int  instanceStart; // min range of the research of story
    unsigned int  instanceStop;  // max range of the research of story
    int            storyToNarrate; // first instance of the story to narrate
    int            scenarioToRecall;

    bool researchWindows;   // if the research of sotry is by window (min*max) or by n-back
    int nBackSize;              // number of nBack instance to look at
    bool bInitial;          // if is the first research

    yarp::os::Port  rpcPort;
    yarp::os::Port  Port2abmReasoning;

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
    evtStory adaptMeaning(evtStory& evt);

    void enrichMeaning(std::string &meaning, std::string sentence);
    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    double getPeriod()
    {
        return period;
    }

    bool updateModule();

    void findStories();
    void findNarration();

    double timeDiff(myTimeStruct tm1, myTimeStruct tm2, bool bPrint = false);
    myTimeStruct string2Time(std::string sTime);

    // RPC & scenarios
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
    void compareNarration(story &target); // try to tell the story target based on the other known stories
    void sayNarrationSimple(story target);
    void createNarration(story &sto);
    std::string createMeaning(std::string agent, std::string predicate, std::string object = "", std::string recipient = "");
    void linkNarrationScenario(int iNarration, int iScenario);
    std::string linkMeaningScenario(int iMeaning, int iScenario);

    // SituationModel
    storygraph::SituationModel sm; ///< Current Situation Model
    void loadSM(int iScenario);
    std::ofstream fTrainOutput; ///< File stream for LRH train
    std::string sNarrativeFolderName;
    std::string sNarrativeFileName;
    std::string sSVGFolderName;
    std::string sSVGFileName;

    // Meanings
    void addLink(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
    void LRHtoSM(std::string m, yarp::os::Bottle& reply, bool create = false);
    void sentenceToTrain(std::string s, yarp::os::Bottle& reply);
    discourseform::meaningSentence sentenceToEvent(std::string level1); ///< transform a meaning from lrh to an properly formatted event
    discourseform::meaningSentence evtToMeaning(std::string sIGARF, int iIGARF);
    void removeDoubleMeaning(std::vector<std::tuple <discourseform::meaningSentence, double, discourseform::PAOR > > &vMeaningScore);
    void removeDoubleMeaning(std::vector<discourseform::hriResponse > &vMeaningScore);

    ///< DFW related
    void analyseDFW();      ///< run a set of analysis on the DFW
    void exportDFW();       ///< export data of the DFW in a file csv
    std::vector <storygraph::DFW> vDFW;
    void displayDFW();
    std::vector < discourseform::hriResponse > useDFW(yarp::os::Bottle bInput); ///< Create a sentence using the specified DFW
    storygraph::DFW foundDFW(std::string sdfw);
    std::string prepareMeaningForLRH(std::string dfw, discourseform::meaningSentence M1);
    std::string prepareMeaningForLRH(std::string dfw, discourseform::meaningSentence M1, discourseform::meaningSentence M2, bool DFWAB);



    ///< HRI Related   
    std::string GrammarNarration; // the file for the grammar narration
    std::string GrammarQuestionDFW; // file for HRI interaction with DFW
    std::string GrammarYesNo;

    bool addHumanNarration(int iIns, std::string sModality = "speech", int iScena = 1);   // add narration through speech
    bool narrationToSpeech(story sto);
    bool narrationToMeaning(story& target);
    bool speechConfirmation();
    void imagineStory(story& target);
    void addNarrationToStory(story &target, bool overWrite = false);
    void addTextNarration(story &target, bool overWrite = false);    // add narration through text
    void addAutoNarration(story &target, int iScena, bool overWrite = false);    // add a full narration (# iScena) to the story iIns

    void recordNarrationABM(story &target);
    void listeningStory();

    bool shouldSpeak;
    bool narrate(int iIns);

    yarp::os::Bottle questionHRI_DFW();
    std::vector < discourseform::hriResponse > what_DFW_Simple(yarp::os::Bottle bInput, int iScenario = 5);
    std::vector < discourseform::hriResponse > what_DFW_Double(yarp::os::Bottle bInput, discourseform::PAOR  &sPAOR, int iScenario = 5);
    std::vector < discourseform::hriResponse > whyPAOR(yarp::os::Bottle bInput, discourseform::PAOR  &sPAOR, int iScenario = 5);
    bool doYouRemember(std::string sInput);
    bool createNarration(std::vector< std::tuple <yarp::os::Bottle, discourseform::PAOR > > vQuestion, int iScenario, std::vector < discourseform::hriResponse > vResponses);
    
    std::string pickResponse(std::vector < discourseform::hriResponse > &vResponses, std::vector<discourseform::PAOR>   &vSaid);


    std::string lowerKey(std::string input);
};



#endif

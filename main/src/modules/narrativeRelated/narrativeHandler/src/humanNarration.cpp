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

#include "narrativeHandler.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;


/*
*  Add through the speechRecog a human spoken narration to an existing story
*
*/
void narrativeHandler::addNarrationToStory(story &target, bool overWrite){

    if (target.meaningStory.size() != 0){
        if (overWrite){
            yWarning(" in narrativeHandler::addNarrationToStory - already an existing narration - overWriting is on");
        }
        else{
            yWarning(" in narrativeHandler::addNarrationToStory - already an existing narration - overWriting is off | aborting");
            return;
        }
    }

    bool storyOnGoing = true;
    ostringstream osError;
    Bottle bOutput;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning; // send the information of recall to the abmReasoning

    vector<string> vNewStory;


    while (storyOnGoing){
        // while the narration is not finished
        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarNarration), 20, false, true);

        if (bRecognized.get(0).asInt() == 0)
        {
            yError() << " error in narrativeHandler::addNarrationToStory | Error in speechRecog";
            return;
        }

        bAnswer = *bRecognized.get(1).asList();
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)
        string sSentence = bAnswer.get(0).asString();

        if (bAnswer.get(0).asString() == "stop")
        {
            yInfo("stop called");
            storyOnGoing = false;
        }
        else{
            cout << "confirmation: " << sSentence << endl;

            bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarYesNo), 20, false, false);

            if (bRecognized.get(1).asList()->get(0).toString() == "yes"){
                vNewStory.push_back(sSentence);
            }
        }
    }

    cout << endl << "human said: " << endl;
    for (auto &phrase : vNewStory){
        cout << phrase << endl;
    }

    target.humanNarration = vNewStory;

    yInfo(" End of human narration, starting to create meaning with LRH");
    if (true){   // to change to lrh
        for (auto sentence : target.humanNarration){
            removeUnderscoreString(sentence);
            string meaning = iCub->getLRH()->SentenceToMeaning(sentence);
            enrichMeaning(meaning, sentence);
            target.meaningStory.push_back(meaning);
        }
    }
    yInfo(" human narration meanings are:");
    for (auto meani : target.meaningStory){
        cout << "\t" << meani << endl;
    }

    // recording narration
    recordNarrationABM(target);

    yInfo(" END add narration");

}



// take a meaning under the format: OCW, OCW OCW, OCW OCW, P1 P2 A2 O3 ... and return: , OCW1 OCW2 , OCW3 OCW4 OCW5 <o> [_-_-_-_-_-_-_-_][A-P-_-_-_-_-_-_][A-_-P-O-_-_-_-_] <o>
void narrativeHandler::enrichMeaning(string &meaning, string sentence){
    //if,could I,ask I John,give John it me,P1 P2 A2 P3 A3 O3 P4 A4 O4 R4
    //if I could ask John to give it to me
    //[P-_-_-_-_-_-_-_][_ - A - P - _ - _ - _ - _ - _][_ - A - _ - P - R - _ - _ - _][_ - A - _ - _ - _ - P - O - R]


    // decompose the sentence into words
    istringstream iss(sentence);
    vector<string> wordsSentence;
    copy(istream_iterator<string>(iss),
        istream_iterator<string>(),
        back_inserter(wordsSentence));

    //    cout << "         BEGIN ENRICH         " << endl;

    //yInfo() << " sentence: " << sentence;
    //yInfo() << " meaning: " << meaning;

    // This is to print the vector

    //yInfo(" sentence decomposed: ");
    //for (auto iter : wordsSentence)
    //{
    //    cout << iter << " ";
    //}
    //cout << endl;

    string delimiter = ",";
    size_t pos = 0;
    string token;
    string s = meaning;
    ostringstream osMeaning;
    int iPAORWordsInProp = 0;
    bool isFirst = true;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        if (!isFirst) { osMeaning << " , "; }
        osMeaning << token;
        iPAORWordsInProp++;
        s.erase(0, pos + delimiter.length());
        //cout << "\t meaning is: " << osMeaning.str() << " - iPAORWordsInProp " << iPAORWordsInProp << endl;
        isFirst = false;
    }
    //cout << s << endl;

    vector<string>  meaningParsed;
    // meaning parsed by spaces and colon
    stringstream stringStream(meaning);
    string line;
    while (std::getline(stringStream, line))
    {
        std::size_t prev = 0, pos;
        while ((pos = line.find_first_of(" ,", prev)) != std::string::npos)
        {
            if (pos > prev)
                meaningParsed.push_back(line.substr(prev, pos - prev));
            prev = pos + 1;
        }
        if (prev < line.length())
            meaningParsed.push_back(line.substr(prev, std::string::npos));
    }

    // remove PAOR from meaningParsed:
    vector<string> meaningWords = vector<string>(meaningParsed.begin(), meaningParsed.begin() + meaningParsed.size() / 2);
    vector<string> meaningPAOR = vector<string>(meaningParsed.begin() + meaningParsed.size() / 2, meaningParsed.end());

    //yInfo() << " meaning meaningWords is:";
    //for (auto it : meaningWords){
    //    cout << it << " ";
    //}
    //cout << endl;
    //yInfo() << " meaning meaningPAOR is:";
    //for (auto it : meaningPAOR){
    //    cout << it << " ";
    //}


    // get the number of OCW:
    vector<string> OCW;
    for (auto word : wordsSentence){
        if (isIn(meaningParsed, word)){
            OCW.push_back(word);
        }
    }
    //cout << endl;
    //yInfo() << " OCW is:";
    //for (auto it : OCW){
    //    cout << it << " ";
    //}
    int iPAOR = meaningPAOR.size();
    osMeaning << " <o> ";


    int iCurrentPAOR = 1;
    vector<string> currentProp;
    for (auto kk : OCW){
        currentProp.push_back("_");
    }

    for (int ii = 0; ii < iPAOR; ii++){

        // if we change the proposition
        string current = meaningPAOR[ii];
        if (int(current.at(1) - '0') != iCurrentPAOR){

            // Fill the meaning
            osMeaning << "[";
            bool isFirst = true;
            for (auto kk : currentProp){
                if (!isFirst) osMeaning << "-";
                osMeaning << kk;
                isFirst = false;
            }
            osMeaning << "]";

            // clear the current one
            currentProp.clear();
            for (auto kk : OCW){
                currentProp.push_back("_");
            }
        }
        // Find the position in the OCW and add the role in the current prop
        for (unsigned int jj = 0; jj < OCW.size(); jj++){
            if (meaningWords[ii] == OCW[jj]){
                currentProp[jj] = current.at(0);
            }
        }
        iCurrentPAOR = int(current.at(1) - '0');
        //yInfo() << " osMeaning: " << osMeaning.str();
    }
    osMeaning << "[";
    isFirst = true;
    for (auto kk : currentProp){
        if (!isFirst) osMeaning << "-";
        osMeaning << kk;
        isFirst = false;
    }

    osMeaning << "] <o>";


    // if no narrative word
    if (iPAORWordsInProp == iCurrentPAOR - 1){
        //yInfo("   adding a coma");
        meaning = "," + osMeaning.str();
    }
    else{
        meaning = osMeaning.str();
    }


    //cout << "meaning is: " << meaning << endl;
    //cout << "result is: " <<  iCub->getLRH()->meaningToSentence(meaning) << endl;

    //meaning = osMeaning.str();
    //cout << endl;

}


/*
* Get the human narration and record it into ABM and link it to the corresponding story
*/
void narrativeHandler::recordNarrationABM(story &target){

    // each sentence is recorded with the corresponding instance from the target and the corresponding order

    yInfo(" BEGIN recording in ABM");
    if (target.humanNarration.size() != target.meaningStory.size()){
        yWarning(" in narrativeHandler::recordNarrationABM : meaning and narration have different size. Won't be recorded");
        return;
    }

    for (unsigned int ii = 0; ii < target.humanNarration.size(); ii++){
        if (iCub->getABMClient()->Connect())
        {
            std::list<std::pair<string, string> > lArgument;
            lArgument.push_back(pair<string, string>(target.meaningStory[ii], "meaning"));
            lArgument.push_back(pair<string, string>(target.humanNarration[ii], "sentence"));
            lArgument.push_back(pair<string, string>(to_string(target.viInstances[0]), "story"));
            lArgument.push_back(pair<string, string>(to_string(ii), "rank"));
            iCub->getABMClient()->sendActivity("action",
                "narration",
                "narration",
                lArgument,
                true);
        }
    }
    yInfo(" END recording in ABM");

}








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

    cout << endl;
    for (auto &phrase : vNewStory){
        cout << phrase << endl;
    }

    target.humanNarration = vNewStory;

}



string narrativeHandler::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    cout << "path is: " << sPath << endl;

    if (!isGrammar)
    {
        cout << "Error in narrativeHandler::grammarToString. Couldn't open file : " << sPath << "." << endl;
        return "Error in narrativeHandler::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    cout << " grammar to string finished" << endl;

    return sOutput;
}
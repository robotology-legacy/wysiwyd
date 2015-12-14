#include <story.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;


void story::createNarration()
{
    bool VERBOSE = false;

    vector<string>  vsOutput;

    ostringstream  osCurrent;

    //Bottle bPreviousRelation;

    osCurrent << "story: " << timeBegin.toString() << " to " << timeEnd.toString() << endl;

    vsOutput.push_back(osCurrent.str());

    for (unsigned int ii = 0; ii != vEvents.size(); ii++){

        osCurrent.str("");
        if (VERBOSE) cout << ii << "...";

        // initial situation
        if (ii == 0){
            if (vEvents[ii].bRelations.toString() != "NULL"){
                osCurrent << "\t\t\t" << "At the beggining, ";
                for (int jj = 0; jj < vEvents[ii].bRelations.size(); jj++){
                    if (jj != 0){
                        osCurrent << " and ";
                    }
                    osCurrent << vEvents[ii].bRelations.get(jj).toString();
                }
                osCurrent << endl;
            }
        }
        // end initial situation

        // if it is an ACTION
        if (vEvents[ii].activity_type == "action"){
            // if the action begin
            if (vEvents[ii].begin){

                if (ii == 0){
                    osCurrent << "\t\t\t" << vEvents[ii].agent << " tries to " << vEvents[ii].predicate << " the " << vEvents[ii].object;
                    if (vEvents[ii].recipient != "none" && vEvents[ii].recipient != "")  osCurrent << " " << vEvents[ii].recipient;
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "adv1" || iarg->first == "adv2") osCurrent << " " << iarg->second;
                    }
                    osCurrent << endl;
                }
                else{
                    // if the previous instance wasn't already an action
                    if (vEvents[ii].activity_type != vEvents[ii - 1].activity_type || vEvents[ii].begin != vEvents[ii - 1].begin){
                        osCurrent << "\t\t\t" << vEvents[ii].agent << " tries to " << vEvents[ii].predicate << " the " << vEvents[ii].object;
                        if (vEvents[ii].recipient != "none" && vEvents[ii].recipient != "")  osCurrent << " " << vEvents[ii].recipient;
                        for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                            if (iarg->first == "adv1" || iarg->first == "adv2") osCurrent << " " << iarg->second;
                        }
                        osCurrent << endl;
                    }
                }
            }
            // the action ends
            else{
                if (ii == 0){
                    bool bStatusFound = false;
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "status" && iarg->second == "failed"){
                            osCurrent << "\t\t\t" << "But it failed." << endl;
                            bStatusFound = true;
                        }
                    }
                    if (!bStatusFound){
                        osCurrent << "\t\t\t" << "And it worked." << endl;
                    }
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "reason"){
                            osCurrent << " because " << iarg->second << "." << endl;
                        }
                    }
                }
                else{
                    if (VERBOSE) cout << endl;
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (VERBOSE) cout << iarg->first << " " << iarg->second << " - " << endl;
                    }
                    // if previous instance was not a beggining of action
                    if (vEvents[ii - 1].begin || vEvents[ii - 1].activity_type != "action"){
                        bool bStatusFound = false;
                        for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                            if (iarg->first == "status" && iarg->second == "failed"){
                                osCurrent << "\t\t\t" << "But it failed." << endl;
                                bStatusFound = true;
                            }
                        }
                        if (!bStatusFound){
                            osCurrent << "\t\t\t" << "And it worked." << endl;
                        }
                    }
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "reason"){
                            osCurrent << "\t\t\t" << "Because " << iarg->second << "." << endl;
                        }
                    }
                }
            }
        }
        else if (vEvents[ii].activity_name == "sentence") {
            string speaker = "none",
                addressee = "none",
                sentence = "none";
            for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                if (iarg->first == "speaker") speaker = iarg->second;
                else if (iarg->first == "addressee")     addressee = iarg->second;
                else if (iarg->first == "sentence")    sentence = iarg->second;
            }

            osCurrent << "\t\t\t" << speaker << " says to " << addressee << ": " << sentence << endl;
        }
        // if not actino or sentence
        else if (vEvents[ii].activity_type == "reasoning"){
            /* for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                 osCurrent << iarg->second << " ";
                 }*/
            // if the action begin
            if (vEvents[ii].begin){

                // if the previous instance wasn't already an action
                //if (current_activitytype != previous_activitytype || previous_begin != current_begin){
                osCurrent << "\t\t\t" << vEvents[ii].agent << " tries to " << vEvents[ii].activity_name << endl;

                for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                    if (iarg->first == "goal"){
                        Bottle bUnfolded = unfoldGoal(iarg->second);
                        osCurrent << "\t\t\t" << "The goal was that: " << bUnfolded.find("agent").toString() << " tries to " << bUnfolded.find("predicate").toString() << " the " << bUnfolded.find("object").toString();
                        if (bUnfolded.find("recipient").toString() != "")  osCurrent << " " << bUnfolded.find("recipient").toString();
                        osCurrent << endl;
                    }
                }
            }
            // the action ends
            else{
                if (ii == 0){
                    bool bStatusFound = false;
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "status" && iarg->second == "failed"){
                            osCurrent << "\t\t\t" << "But it failed";
                            bStatusFound = true;
                        }
                    }
                    if (!bStatusFound){
                        osCurrent << "\t\t\t" << "And it worked." << endl;
                    }
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "reason"){
                            osCurrent << " because " << iarg->second;
                        }
                    }
                    osCurrent << "." << endl;
                }
                else if (vEvents[ii - 1].begin || vEvents[ii - 1].activity_type != "action"){// if previous instance was not a beggining of action
                    bool bStatusFound = false;
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "status" && iarg->second == "failed"){
                            osCurrent << "\t\t\t" << "But it failed." << endl;
                            bStatusFound = true;
                        }
                    }
                    if (!bStatusFound){
                        osCurrent << "\t\t\t" << "And it worked." << endl;
                    }
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "reason"){
                            osCurrent << " because " << iarg->second << "." << endl;
                        }
                    }
                }
                else{
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "reason"){
                            osCurrent << " because " << iarg->second << "." << endl;;
                        }
                    }
                }
            }
        }
        // nor action/reasoning/sentence
        else {
            //for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
            //    osCurrent << iarg->first << " " << iarg->second << "; ";
            //    //                osCurrent << bTemp.toString() << endl;
            //}
            //osCurrent << endl;
            // if the action begin
            if (vEvents[ii].begin){
                if (ii == 0){
                    osCurrent << "\t\t\t" << vEvents[ii].agent << " tries to " << vEvents[ii].activity_name << endl;
                }
                // if the previous instance wasn't already an action
                else if (vEvents[ii].activity_type != vEvents[ii - 1].activity_type || vEvents[ii].begin != vEvents[ii - 1].begin){
                    osCurrent << "\t\t\t" << vEvents[ii].agent << " tries to " << vEvents[ii].activity_name << endl;
                }
            }
            // the action ends
            else {
                if (ii == 0){
                    bool bStatusFound = false;
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "status" && iarg->second == "failed"){
                            osCurrent << "\t\t\t" << "But it failed";
                            bStatusFound = true;
                        }
                    }
                    if (!bStatusFound){
                        osCurrent << "\t\t\t" << "And it worked." << endl;
                    }
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "reason"){
                            osCurrent << " because " << iarg->second << "." << endl;
                        }
                    }
                }
                else if (vEvents[ii - 1].begin || vEvents[ii - 1].activity_type != "action"){// if previous instance was not a beggining of action
                    bool bStatusFound = false;
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "status" && iarg->second == "failed"){
                            osCurrent << "\t\t\t" << "But it failed";
                            bStatusFound = true;
                        }
                    }
                    if (!bStatusFound){
                        osCurrent << "\t\t\t" << "And it worked." << endl;
                    }
                    for (auto iarg = vEvents[ii].vArgument.begin(); iarg != vEvents[ii].vArgument.end(); iarg++){
                        if (iarg->first == "reason"){
                            osCurrent << " because " << iarg->second << "." << endl;
                        }
                    }
                }
            }
        }


        // changes in the relations:
        if (ii != vEvents.size() && ii != 0){

            if (vEvents[ii].bRelations != vEvents[ii - 1].bRelations)
            {
                if (vEvents[ii].bRelations.toString() != "NULL"){
                    osCurrent << "\t\t\t" << "And now, ";
                    for (int jj = 0; jj < vEvents[ii].bRelations.size(); jj++){
                        if (jj != 0){
                            osCurrent << " and ";
                        }
                        osCurrent << vEvents[ii].bRelations.get(jj).toString();
                    }
                    osCurrent << endl;
                }
            }
        }

        // final situation
        if (ii == vEvents.size() - 1){
            osCurrent << "\t\t\t" << "In the end, ";
            for (int jj = 0; jj < vEvents[ii].bRelations.size(); jj++){
                if (jj != 0){
                    osCurrent << " and ";
                }
                osCurrent << vEvents[ii].bRelations.get(jj).toString();
            }
            osCurrent << "." << endl;
        }


        if (VERBOSE) cout << osCurrent.str();

        vsOutput.push_back(osCurrent.str());

    }

    if (VERBOSE) cout << endl << endl;
    sentenceStory = vsOutput;
}


Bottle story::unfoldGoal(string goal)
{
    bool bVerbose = false;
    Bottle bOutput;

    if (bVerbose) cout << endl << "Starting to unfold: " << goal << endl;

    bool isRole = true;
    bool bIsFirst = true;
    istringstream iss(goal);
    Bottle bTemp;
    do
    {
        string sub;
        iss >> sub;
        if (sub[0] == '(') sub = sub.erase(0, 1);
        if (sub[sub.size() - 1] == ')')   sub = sub.erase(sub.size() - 1);
        if (bVerbose) cout << "Substring: " << sub << endl;
        if (isRole){
            if (!bIsFirst) bOutput.addList() = bTemp;
            bTemp.clear();
            bTemp.addString(sub);
        }
        else{
            bTemp.addString(sub);
        }
        bIsFirst = false;
        isRole = !isRole;

    } while (iss);

    if (bVerbose) cout << "bOutput: " << bOutput.toString() << endl;

    if (bVerbose) cout << "find is: " << bOutput.find("predicate").toString() << endl;

    return bOutput;
}


void story::displayNarration()
{
    createNarration();

    cout << "begin display narration:" << endl;

    for (auto itSt = sentenceStory.begin(); itSt != sentenceStory.end(); itSt++){
        cout << *itSt;
    }

    cout << "OCW are: ";
    for (auto itS = vOCW.begin(); itS != vOCW.end(); itS++){
        cout << *itS << "  ";
    }

    cout << endl;
}


void story::addOCW(string OCW){
    if (OCW == "none")    return;

    for (auto itS = vOCW.begin(); itS != vOCW.end(); itS++){
        if (*itS == OCW){
            return;
        }
    }
    vOCW.push_back(OCW);
}


void story::addInstance(int _instance, Bottle bActivity, Bottle bArguments, Bottle _bRelations){
    
    vector<string>    _instanceOCW;
    
    evtStory evt;

    _instanceOCW = evt.initialize(_instance, bActivity, bArguments, _bRelations);

    vEvents.push_back(evt);

    for (auto itS = _instanceOCW.begin(); itS != _instanceOCW.end(); itS++){
        addOCW(*itS);
    }

}



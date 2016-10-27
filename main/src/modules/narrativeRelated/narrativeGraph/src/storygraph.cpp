#include "storygraph.h"
#include <regex>
#include <map>
#include <algorithm>

using namespace storygraph;
using namespace std;

const int histoSize = 10;

template<typename Cont, typename It>
auto ToggleIndices(Cont &cont, It beg, It end) -> decltype(std::end(cont))
{
    int helpIndx(0);
    return std::stable_partition(std::begin(cont), std::end(cont),
        [&](typename Cont::value_type const& val) -> bool {
        return std::find(beg, end, helpIndx++) != end;
    });
}

void margin(int level) {
    for (int i = 0; i < level; i++)
        cout << "| ";
}

void marginInFile(int level, ofstream &IGARFfile) {
    for (int i = 0; i < level; i++)
        IGARFfile << "| ";
}

bool storygraph::operator==(const sKeyMean& A, const sKeyMean& B) {
    return (A.iIGARF == B.iIGARF &&
        A.cPart == B.cPart &&
        A.iRel == B.iRel);
}

SituationModel::SituationModel() { clear(); }

void SituationModel::clear() {
    vRelations.clear();
    vActionEvts.clear();
    vIGARF.clear();
    vDiscourseLinks.clear();
    vChronoEvent.clear();
    vChronoIgarf.clear();
    endSentence();
}

/*---------*
 * Display *
 *---------*/

string SituationModel::getSentenceEvt(int i) {
    if (i < 0 || i >= (int)vActionEvts.size())
        return "";

    return vActionEvts.at(i).agent + " " +
        vActionEvts.at(i).predicate + " " +
        vActionEvts.at(i).object + " " +
        vActionEvts.at(i).recipient;
}

string SituationModel::getSentenceRel(int i) {
    if (i < 0 || i >= (int)vRelations.size())
        return "";

    return vRelations.at(i).subject + " " + vRelations.at(i).verb + " " + vRelations.at(i).object;
}

string SituationModel::dispRelations(const vector < int >& rels) {
    string s = "";
    for (int i : rels) {
        s += " [" + getSentenceRel(i) + "] ";
    }
    return s;
}

void line(int level) {
    char prevC = cout.fill();
    margin(level);
    cout.fill('-');
    cout.width(79 - 2 * level);
    cout << right << "-" << endl;
    cout.fill(prevC);
}

void SituationModel::showIGARF(int i, ofstream &IGARFfile, int level) {
    vChronoIgarf.push_back(i);
    if (i < 0 || i >= (int)vIGARF.size())
        return;

    const sIGARF &evt = vIGARF.at(i);
    vIGARF.at(i).iLevel = level;
    margin(level);
    marginInFile(level, IGARFfile);
    cout << "[" << i << "] " << endl;
    IGARFfile << endl;
    line(level);
    margin(level);
    marginInFile(level, IGARFfile);
    cout << "+-INIT: " << dispRelations(evt.vInitState) << endl;
    IGARFfile << "INIT " << dispRelations(evt.vInitState) << endl;
    vChronoEvent.push_back(pair<int, string>(i, "I"));
    margin(level);
    marginInFile(level, IGARFfile);
    cout << "+-GOAL: " << dispRelations(evt.vGoal) << endl;
    IGARFfile << "GOAL " << dispRelations(evt.vGoal) << endl;
    vChronoEvent.push_back(pair<int, string>(i, "G"));
    margin(level);
    marginInFile(level, IGARFfile);
    cout << "+-ACTION: ";
    IGARFfile << "ACTION ";
    vChronoEvent.push_back(pair<int, string>(i, "A"));
    if (evt.tAction == ACTION_EVT) {
        cout << "[" << evt.iAction << "] " << getSentenceEvt(evt.iAction) << endl;
        IGARFfile << getSentenceEvt(evt.iAction) << endl;
    }
    else if (evt.tAction == IGARF_EVT) {
        cout << "[" << evt.iAction << "]" << endl;
        IGARFfile << endl;
        showIGARF(evt.iAction, IGARFfile, level + 1);
    }
    else
        cout << endl;
    margin(level);
    marginInFile(level, IGARFfile);
    cout << "+-RESULT: ";
    IGARFfile << "RESULT ";
    vChronoEvent.push_back(pair<int, string>(i, "R"));
    if (evt.tResult == ACTION_EVT) {
        cout << "[" << evt.iResult << "] " << getSentenceEvt(evt.iResult) << endl;
        IGARFfile << getSentenceEvt(evt.iResult) << endl;
    }
    else if (evt.tResult == IGARF_EVT) {
        cout << "[" << evt.iResult << "]" << endl;
        IGARFfile << endl;
        showIGARF(evt.iResult, IGARFfile, level + 1);
    }
    else{
        cout << endl;
        IGARFfile << endl;
    }
    margin(level);
    marginInFile(level, IGARFfile);
    cout << "+-FINAL: " << dispRelations(evt.vFinalState) << endl;
    IGARFfile << "FINAL " << dispRelations(evt.vFinalState) << endl;
    vChronoEvent.push_back(pair<int, string>(i, "F"));
    margin(level);
    marginInFile(level, IGARFfile);
    cout << "+-NEXT: ";
    IGARFfile << "NEXT ";
    if (evt.iNext != -1) {
        cout << "[" << evt.iNext << "]" << endl;
        IGARFfile << endl;
        line(level);
        showIGARF(evt.iNext, IGARFfile, level);
    }
    else {
        cout << endl;
        IGARFfile << endl;
        line(level);
    }
}

// Creation - Modification

/*----------*
 * Creation *
 *----------*/

int SituationModel::addNewActionEvt(const sActionEvt& a) {
    vActionEvts.push_back(a);
    return vActionEvts.size() - 1;
}

int SituationModel::findRelation(const sRelation& rel, bool create) {
    for (int i = 0; i < (int)vRelations.size(); i++) {
        if (vRelations.at(i) == rel)
            return i;
    }
    // Not found
    if (create) {
        vRelations.push_back(rel);
        return vRelations.size() - 1;
    }
    else {
        return -1;
    }
}


int SituationModel::createIGARF() {
    sIGARF newEvent;
    newEvent.tAction = UNDEF;
    newEvent.tResult = UNDEF;
    newEvent.iAction = -1;
    newEvent.iResult = -1;
    newEvent.iNext = -1;
    vIGARF.push_back(newEvent);
    return vIGARF.size() - 1;
}

/*--------------*
 * Modification *
 *--------------*/

void SituationModel::modifEventIGARF(int iIGARF, char cPart, int iActEvt) {
    if (iIGARF < 0 || iIGARF >= (int)vIGARF.size())
        return;
    if (iActEvt < 0 || iActEvt >= (int)vActionEvts.size())
        return;

    if (cPart == 'A') {
        vIGARF.at(iIGARF).tAction = ACTION_EVT;
        vIGARF.at(iIGARF).iAction = iActEvt;
    }
    else if (cPart == 'R') {
        vIGARF.at(iIGARF).tResult = ACTION_EVT;
        vIGARF.at(iIGARF).iResult = iActEvt;
    }
}

void SituationModel::modifContentIGARF(int iIGARF, char cPart, int jIGARF) {
    if (iIGARF < 0 || iIGARF >= (int)vIGARF.size())
        return;
    if (jIGARF < 0 || jIGARF >= (int)vIGARF.size())
        return;

    if (cPart == 'A') {
        vIGARF.at(iIGARF).tAction = IGARF_EVT;
        vIGARF.at(iIGARF).iAction = jIGARF;
    }
    else if (cPart == 'R') {
        vIGARF.at(iIGARF).tResult = IGARF_EVT;
        vIGARF.at(iIGARF).iResult = jIGARF;
    }
    else if (cPart == 'N') {
        vIGARF.at(iIGARF).tResult = IGARF_EVT;
        vIGARF.at(iIGARF).iResult = jIGARF;
    }
}

void SituationModel::remContentIGARF(int iIGARF, char cPart) {
    if (iIGARF < 0 || iIGARF >= (int)vIGARF.size())
        return;

    if (cPart == 'A') {
        vIGARF.at(iIGARF).tAction = UNDEF;
        vIGARF.at(iIGARF).iAction = -1;
    }
    else if (cPart == 'R') {
        vIGARF.at(iIGARF).tResult = UNDEF;
        vIGARF.at(iIGARF).iResult = -1;
    }
    else if (cPart == 'N') {
        vIGARF.at(iIGARF).iNext = -1;
    }
}

int find(vector < int > &vInt, int elt) {
    for (int i = 0; i < (int)vInt.size(); i++) {
        if (vInt.at(i) == elt)
            return i;
    }
    return -1;
}

int SituationModel::addRelationIGARF(int iIGARF, char cPart, int iRel) {
    if (iIGARF < 0 || iIGARF >= (int)vIGARF.size())
        return -1;
    if (iRel < 0 || iRel >= (int)vRelations.size())
        return -1;

    if (cPart == 'I' && find(vIGARF.at(iIGARF).vInitState, iRel) == -1) {
        vIGARF.at(iIGARF).vInitState.push_back(iRel);
        return vIGARF.at(iIGARF).vInitState.size() - 1;
    }
    else if (cPart == 'G' && find(vIGARF.at(iIGARF).vGoal, iRel) == -1) {
        vIGARF.at(iIGARF).vGoal.push_back(iRel);
        return vIGARF.at(iIGARF).vGoal.size() - 1;
    }
    else if (cPart == 'F' && find(vIGARF.at(iIGARF).vFinalState, iRel) == -1) {
        vIGARF.at(iIGARF).vFinalState.push_back(iRel);
        return vIGARF.at(iIGARF).vFinalState.size() - 1;
    }
    return -1;
}

void SituationModel::removeRelationIGARF(int iIGARF, char cPart, int iRel) {
    if (iIGARF < 0 || iIGARF >= (int)vIGARF.size())
        return;
    if (iRel < 0 || iRel >= (int)vRelations.size())
        return;

    sIGARF &ig = vIGARF.at(iIGARF);
    if (cPart == 'I' && find(ig.vInitState, iRel) != -1) {
        ig.vInitState.erase(ig.vInitState.begin() + find(ig.vInitState, iRel));
    }
    else if (cPart == 'G' && find(ig.vGoal, iRel) != -1) {
        ig.vGoal.erase(ig.vGoal.begin() + find(ig.vGoal, iRel));
    }
    else if (cPart == 'F' && find(ig.vFinalState, iRel) != -1) {
        ig.vFinalState.erase(ig.vFinalState.begin() + find(ig.vFinalState, iRel));
    }
}

string findValue(const vector<pair<string, string> > &vArgs, string role) {
    // Arguments
    for (auto iarg = vArgs.begin(); iarg != vArgs.end(); iarg++){
        if (iarg->first == role)
            return iarg->second;
    }
    return "";
}

bool isRelationsBInA(vector < int > a, vector < int > b) {
    bool cont = true;
    for (int j = 0; j < (int)b.size() && cont; j++) {
        bool found = false;
        for (int i = 0; i < (int)a.size() && !found; i++) {
            if (a.at(i) == b.at(j)) {
                found = true;
            }
        }
        cont = found;
    }
    return cont;
}


/*--------------------*
 * Links and sKeyMean *
 *--------------------*/

void storygraph::SituationModel::cleanLinks() {
    vDiscourseLinks.clear();
    lastFocus = createKey(-1, 'A', -1);
}

sKeyMean SituationModel::createKey(int iIGARF, char cPart, int iRel) {
    sKeyMean k;
    if (iIGARF < 0 || iIGARF >= (int)vIGARF.size())
        k.iIGARF = -1;
    else
        k.iIGARF = iIGARF;

    if (cPart == 'I' || cPart == 'G' || cPart == 'R' || cPart == 'F')
        k.cPart = cPart;
    else
        k.cPart = 'A';

    if (cPart == 'I' || cPart == 'G' || cPart == 'F') {
        if (iRel < 0 ||
            (cPart == 'I' && iRel >= (int)vIGARF.at(k.iIGARF).vInitState.size()) ||
            (cPart == 'G' && iRel >= (int)vIGARF.at(k.iIGARF).vGoal.size()) ||
            (cPart == 'F' && iRel >= (int)vIGARF.at(k.iIGARF).vFinalState.size()))
            k.iRel = -1;
        else
            k.iRel = iRel;
    }
    else {
        k.iRel = -1;
    }

    return k;
}

sActionEvt SituationModel::getEvent(const sKeyMean& km) {
    sActionEvt a;
    if (km.iIGARF < 0 || km.iIGARF >= (int)vIGARF.size()) {
        return a; // Blank
    }
    const sIGARF &igarf = vIGARF.at(km.iIGARF);
    if (km.cPart == 'A' || km.cPart == 'R') {
        return (km.cPart == 'A') ? vActionEvts.at(igarf.iAction) : vActionEvts.at(igarf.iResult);
    }
    else if (km.cPart == 'I') {
        const sRelation& r = vRelations.at(igarf.vInitState.at(km.iRel));
        return relToAct(r);
    }
    else if (km.cPart == 'G') {
        const sRelation& r = vRelations.at(igarf.vGoal.at(km.iRel));
        return relToAct(r);
    }
    else if (km.cPart == 'F') {
        const sRelation& r = vRelations.at(igarf.vFinalState.at(km.iRel));
        return relToAct(r);
    }
    return a; // Blank
}

void SituationModel::createLink(sKeyMean from, sKeyMean to, string word) {
    sDiscourseLink newLink;
    newLink.fromEvt = from;
    newLink.toEvt = to;
    newLink.word = word;
    vDiscourseLinks.push_back(newLink);
}

/*---------*
 * ABMtoSM *
 *---------*/
void SituationModel::ABMtoSM(const story &sto, ofstream &IGARFfile) {
    // Clearing
    clear();

    // Get events
    vector < evtStory > vEvents = sto.vEvents;

    instanceBegin = sto.viInstances[0];

    // Step 1 : Considering begin and end of activities, create events and relations
    for (unsigned int i = 0; i < vEvents.size(); i++) {
        const evtStory &currentEvt = vEvents.at(i);
        //        cout <<"=======================================================================\n"<< "Current event is: \n" << vEvents.at(i).toString();


        if (currentEvt.begin) { // For each begin of action
            evtStory endCurrent;
            bool hasEnd = false;
            if (i + 1 < vEvents.size()
                && !vEvents.at(i + 1).begin
                && currentEvt.activity_name == vEvents.at(i + 1).activity_name) { // Is the next event the end event ?
                endCurrent = vEvents.at(i + 1);
                //                cout << " level " << i +1 << " in an end." << endl;
                //                cout << " has end TRUE " << endl;
                hasEnd = true;
            }
            // New Action events and relations
            sIGARF newEvent;
            newEvent.iNext = -1;
            // > Action
            newEvent.tAction = ACTION_EVT;
            storygraph::sActionEvt action;
            action.predicate = currentEvt.predicate;
            action.agent = currentEvt.agent;
            action.object = currentEvt.object;
            action.recipient = currentEvt.recipient;
            newEvent.iAction = addNewActionEvt(action);

            // > Init State
            for (int i = 0; i < currentEvt.bRelations.size(); i++) {
                if (currentEvt.bRelations.get(i).isList())
                    newEvent.vInitState.push_back(findRelation(fromValueToRelation(currentEvt.bRelations.get(i)), true));
            }

            newEvent.iLevel = -1;

            // > Result
            newEvent.tResult = UNDEF;
            newEvent.iResult = -1;
            if (hasEnd) {
                sActionEvt result;
                result.predicate = endCurrent.predicate;
                result.agent = endCurrent.agent;
                result.object = endCurrent.object;
                result.recipient = endCurrent.recipient;
                if (findValue(endCurrent.vArgument, "status") == "failed") {
                    result.predicate = "fail";
                    result.object = currentEvt.predicate;
                    result.recipient = "";

                    if (findValue(endCurrent.vArgument, "reason") == "outofreach") {
                        sRelation cause;
                        cause.subject = "it";
                        cause.verb = "is";
                        cause.object = "out-of-reach";
                        newEvent.vInitState.push_back(findRelation(cause, true));
                        newEvent.vFinalState.push_back(findRelation(cause, true));
                    }
                }
                newEvent.tResult = ACTION_EVT;
                newEvent.iResult = addNewActionEvt(result);

                // > Goal
                if (findValue(endCurrent.vArgument, "goal") != "") {
                    // Extract the goal relation
                    string goal = findValue(endCurrent.vArgument, "goal");
                    size_t pos = goal.find("predicate") + 10;
                    string pred = goal.substr(pos, goal.find(")", pos) - pos);
                    pos = goal.find("agent") + 6;
                    string agent = goal.substr(pos, goal.find(")", pos) - pos);
                    pos = goal.find("object") + 7;
                    string object = goal.substr(pos, goal.find(")", pos) - pos);
                    sRelation g;
                    if (agent == "icub") agent = "iCub";
                    g.subject = agent;
                    g.verb = pred;
                    g.object = object;

                    if (g.verb == "have") {
                        g.verb = "want";
                    }
                    //vRelations.push_back(g);
                    int i = findRelation(g, true);
                    newEvent.vGoal.push_back(i);
                }

                // > Goal
                if (findValue(endCurrent.vArgument, "want") != "") {
                    // Extract the goal relation
                    string goal = findValue(endCurrent.vArgument, "want");
                    size_t pos = goal.find("predicate") + 10;
                    string pred = goal.substr(pos, goal.find(")", pos) - pos);
                    pos = goal.find("agent") + 6;
                    string agent = goal.substr(pos, goal.find(")", pos) - pos);
                    pos = goal.find("object") + 7;
                    string object = goal.substr(pos, goal.find(")", pos) - pos);
                    sRelation g;
                    if (agent == "icub") agent = "iCub";
                    g.subject = agent;
                    g.verb = pred;
                    if (g.verb == "have"){
                        g.verb = "want";
                    }
                    g.object = object;
                    //vRelations.push_back(g);
                    int i = findRelation(g, true);
                    newEvent.vGoal.push_back(i);
                }

                // > Final State
                for (int i = 0; i < endCurrent.bRelations.size(); i++) {
                    if (endCurrent.bRelations.get(i).isList()) {
                        newEvent.vFinalState.push_back(findRelation(fromValueToRelation(endCurrent.bRelations.get(i)), true));
                    }
                }
            }
            else { // No end event
                //                cout << "No end event" << endl;
                newEvent.vFinalState = newEvent.vInitState;
            }

            // Stock it
            //            cout << "Pushing back: " << newEvent.toString() << endl;
            //            cout << "Event is: " << currentEvt.predicate << " " << currentEvt.agent << " " << currentEvt.object << " " << currentEvt.recipient << " " << currentEvt.begin << endl;
            vIGARF.push_back(newEvent);
        }
    }

    //displayEvent();
    makeStructure(IGARFfile);
}

void SituationModel::makeStructure(ofstream &IGARFfile) {
    // Step 2: Make chains
    // Each chain ends iff (1. Init and Final State are different) OR (2. Result is a failure) OR (3. End of the story)
    vector < int > rep; // Indexes of IGARF which packed chains (used for next step)
    int firstOfChain = -1;
    int lastOfChain = -1;
    int N = vIGARF.size();

    for (int i = 0; i < N; i++) {
        // End chain?
        sIGARF currentIGARF = vIGARF.at(i);

        //        cout << " current igarf is: " << i << "/" << N << " " << currentIGART.toString() << endl;
        if ((currentIGARF.tResult == ACTION_EVT && VocabularyHandler::sameMeaning(vActionEvts.at(currentIGARF.iResult).predicate, "fail")) ||
            !(isRelationsBInA(currentIGARF.vInitState, currentIGARF.vFinalState) && isRelationsBInA(currentIGARF.vFinalState, currentIGARF.vInitState)) ||
            i == N - 1) {
            //cout << "1/";
            if (!(currentIGARF.tResult == ACTION_EVT &&
                VocabularyHandler::sameMeaning(vActionEvts.at(currentIGARF.iResult).predicate, "fail")) && // Not a failure
                !(isRelationsBInA(currentIGARF.vInitState, currentIGARF.vFinalState) &&
                isRelationsBInA(currentIGARF.vFinalState, currentIGARF.vInitState))) { // Final and init state are differents
                // if a final staéte should be set as goal, change have by "want"
                for (auto fin : vIGARF.at(i).vFinalState){
                    sRelation newR = vRelations[fin];
                    if (vRelations[fin].verb == "have"){                        
                        newR.verb = "want";                        
                    }
                    int kk = findRelation(newR, true);
                    vIGARF.at(i).vGoal.push_back(kk);
                }
            }
            if (lastOfChain == -1){
                rep.push_back(i);
                //cout << " last of chain is -1" << endl;
            }
            else {
                vIGARF.at(lastOfChain).iNext = i;
                // New IGARF, pack the chain
                sIGARF newEvent;
                newEvent.vInitState = vIGARF.at(firstOfChain).vInitState;
                newEvent.vGoal = vIGARF.at(firstOfChain).vGoal;
                newEvent.tAction = IGARF_EVT;
                newEvent.iAction = firstOfChain;
                newEvent.tResult = currentIGARF.tResult;
                newEvent.iResult = currentIGARF.iResult;
                newEvent.vFinalState = currentIGARF.vFinalState;
                newEvent.iNext = -1;
                newEvent.iLevel = -1;
                vIGARF.push_back(newEvent);
                //                cout << " add IGARF BEFORE: " << newEvent.toString() << endl;

                rep.push_back(vIGARF.size() - 1);
                firstOfChain = -1;
                lastOfChain = -1;
            }
        }
        else {
            //cout << "2/";
            if (lastOfChain == -1) {
                //cout << "last of chain is -1" << endl;
                firstOfChain = i;
                lastOfChain = i;
            }
            else {
                //cout << " last chain not -1" << endl;
                vIGARF.at(lastOfChain).iNext = i;
                lastOfChain = i;
            }
        }
    }

    if (rep.size() == 0)
        return;
    int head = rep.at(0);

    //displayEvent();

    // Step 3: Assembling chains
    int j;
    for (unsigned int i = 0; i < rep.size(); i++) {
        j = rep.at(i);
        if (vIGARF.at(j).tResult == ACTION_EVT && VocabularyHandler::sameMeaning(vActionEvts.at(vIGARF.at(j).iResult).predicate, "fail") && i + 1 < rep.size()) {
            // New IGARF, pack current and next
            sIGARF newEvent;
            newEvent.vInitState = vIGARF.at(j).vInitState;
            newEvent.vGoal = vIGARF.at(j).vGoal;
            newEvent.tAction = IGARF_EVT;
            newEvent.iAction = j;
            newEvent.tResult = IGARF_EVT;
            newEvent.iResult = rep.at(i + 1);
            newEvent.vFinalState = vIGARF.at(rep.at(i + 1)).vFinalState;
            newEvent.iNext = -1;
            newEvent.iLevel = -1;

            //            cout << " add IGARF AFTER: " << newEvent.toString() << endl;

            vIGARF.push_back(newEvent);
            if (head == j)
                head = vIGARF.size() - 1;
            rep.at(i + 1) = vIGARF.size() - 1;
        }
        else if (i + 1 < rep.size()) {
            vIGARF.at(j).iNext = rep.at(i + 1);
        }
    }

    // Step 4: Spreading goals
    // (Naïve)
    // From Result and Next to Current
    bool change = true;
    while (change) {
        change = false;
        for (int i = 0; i < (int)vIGARF.size(); i++) {
            if (vIGARF.at(i).vGoal.empty()) {
                if (vIGARF.at(i).tResult == IGARF_EVT && !vIGARF.at(vIGARF.at(i).iResult).vGoal.empty()) {
                    vIGARF.at(i).vGoal = vIGARF.at(vIGARF.at(i).iResult).vGoal;
                    change = true;
                }
                else if (vIGARF.at(i).iNext != -1 && !vIGARF.at(vIGARF.at(i).iNext).vGoal.empty()) {
                    vIGARF.at(i).vGoal = vIGARF.at(vIGARF.at(i).iNext).vGoal;
                    change = true;
                }
            }
        }
    }

    // From Current to Action, Result and Next
    change = true;
    while (change) {
        change = false;
        for (int i = 0; i < (int)vIGARF.size(); i++) {
            if (!vIGARF.at(i).vGoal.empty()) {
                if (vIGARF.at(i).tAction == IGARF_EVT && vIGARF.at(vIGARF.at(i).iAction).vGoal.empty()) {
                    vIGARF.at(vIGARF.at(i).iAction).vGoal = vIGARF.at(i).vGoal;
                    change = true;
                }
                if (vIGARF.at(i).tResult == IGARF_EVT && vIGARF.at(vIGARF.at(i).iResult).vGoal.empty()) {
                    vIGARF.at(vIGARF.at(i).iResult).vGoal = vIGARF.at(i).vGoal;
                    change = true;
                }
                if (vIGARF.at(i).iNext != -1 && vIGARF.at(vIGARF.at(i).iNext).vGoal.empty()) {
                    vIGARF.at(vIGARF.at(i).iNext).vGoal = vIGARF.at(i).vGoal;
                    change = true;
                }
            }
        }
    }


    cout << "Story from instance: " << instanceBegin << "; Head is: " << head << endl << endl;

    vChronoIgarf.clear();
    showIGARF(head, IGARFfile);

    cout << endl;

}





/*-------------------------------------*
 * SMtoTrain and SMandNarrativeToTrain *
 *-------------------------------------*/
int SituationModel::proximityScoreAction(int i, const vector<string>& ocw) {
    if (i < 0 || i >= (int)vActionEvts.size())
        return -1;
    const sActionEvt& e = vActionEvts.at(i);
    int score = 0;

    //cout << "comparing: ";
    //for (int ii = 0; ii < ocw.size(); ii++){
    //    cout << ocw[ii] << " ";
    //}
    //cout << " <-> " << e.predicate << " " << e.agent << " " << e.object << " " << e.recipient<<" ; score: ";


    // if ocw has only 1 element:
    // check if predicate is predicate
    if (VocabularyHandler::shareMeaning(e.predicate, ocw[0]))
    {
        //cout << "IDEM: " << e.predicate << " = " << ocw[0] << endl;
        score += 10;
    }

    // if at least 2 element
    if (ocw.size() > 1){
        if (VocabularyHandler::shareMeaning(e.agent, ocw[1]))
        {
            //cout << "IDEM: " << e.agent << " = " << ocw[1] << endl;
            score += 8;
        }
    }

    // if at least 3 element
    if (ocw.size() > 2){
        if (VocabularyHandler::shareMeaning(e.object, ocw[2]))
        {
            //cout << "IDEM: " << e.object << " = " << ocw[2] << endl;
            score += 6;
        }
    }

    // if at least 4 element
    if (ocw.size() > 3){
        if (VocabularyHandler::shareMeaning(e.recipient, ocw[3]))
            score += 4;
    }

    if (VocabularyHandler::shareMeaning(e.predicate, ocw))
        score += 5;
    if (VocabularyHandler::shareMeaning(e.agent, ocw))
        score += 4;
    if (VocabularyHandler::shareMeaning(e.object, ocw))
        score += 2;
    if (VocabularyHandler::shareMeaning(e.recipient, ocw))
        score += 1;
    //cout << score << endl;


    return score;// (score == 12) ? 1 : 0; // Binary return, if all the words in the sActionEvt don't need to be in ocw, it is best to return score
}

int SituationModel::proximityScoreRelation(int i, const vector <string>& ocw) {
    if (i < 0 || i >= (int)vRelations.size())
        return -1;
    const sRelation& r = vRelations.at(i);
    int score = 0;
    /*
        cout << "comparing: ";
        for (int ii = 0; ii < ocw.size(); ii++){
        cout << ocw[ii] << " ";
        }
        cout << " <-> " << r.verb << " " << r.subject << " " << r.object << " ; score: ";*/

    // if ocw has only 1 element:
    // check if predicate is predicate
    if (VocabularyHandler::shareMeaning(r.verb, ocw[0]))
    {
        //       cout << "IDEM: " << r.verb << " = " << ocw[0] << endl;
        score += 10;
    }

    // if at least 2 element
    if (ocw.size() > 1){
        if (VocabularyHandler::shareMeaning(r.subject, ocw[1]))
        {
            //            cout << "IDEM: " << r.subject << " = " << ocw[1] << endl;
            score += 8;
        }
    }

    // if at least 3 element
    if (ocw.size() > 2){
        if (VocabularyHandler::shareMeaning(r.object, ocw[2]))
        {
            //            cout << "IDEM: " << r.object << " = " << ocw[2] << endl;
            score += 6;
        }
    }

    if (VocabularyHandler::shareMeaning(r.verb, ocw))
        score += 5;
    if (VocabularyHandler::shareMeaning(r.subject, ocw))
        score += 4;
    if (VocabularyHandler::shareMeaning(r.object, ocw))
        score += 2;

    //    cout << score << "; relation: " << i << endl;

    return score;// (score == 11) ? 1 : 0; // See proximityScoreAction(..)
}

vector<sKeyMean> SituationModel::findBest(const vector<string>& ocw, int &iScore) {
    int score_max = 0;
    vector<sKeyMean>  vkmBest;
    vector<int>   vScore;
    bool equal = false;
    sKeyMean km = createKey(-1, 'Z', -1);
    for (int i = 0; i < (int)vActionEvts.size(); i++) {
        int s = proximityScoreAction(i, ocw);
        if (s >= score_max && s > 0) {
            equal = (s == score_max);
            score_max = s;
            bool hasClear = false;
            // Search this event in the IGARF
            for (int j = 0; j < (int)vIGARF.size(); j++) {
                if (vIGARF.at(j).tAction == ACTION_EVT && vIGARF.at(j).iAction == i) {
                    km = createKey(j, 'A', -1);
                    if (!equal && !hasClear) {
                        hasClear = true;
                        vkmBest.clear();
                        vScore.clear();
                    }
                    vkmBest.push_back(km);
                    vScore.push_back(s);
                }
                else if (vIGARF.at(j).tResult == ACTION_EVT && vIGARF.at(j).iResult == i) {
                    km = createKey(j, 'R', -1);
                    if (!equal && !hasClear) {
                        vkmBest.clear();
                        vScore.clear();
                    }
                    vkmBest.push_back(km);
                    vScore.push_back(s);
                }
            }
        }
    }
    for (int i = 0; i < (int)vRelations.size(); i++) {
        int s = proximityScoreRelation(i, ocw);
        if (s >= score_max && s > 0) {
            equal = (s == score_max);
            score_max = s;
            bool hasClear = false;
            for (int j = 0; j < (int)vIGARF.size(); j++) {
                for (int k = 0; k < (int)vIGARF.at(j).vGoal.size(); k++) {
                    if (vIGARF.at(j).vGoal.at(k) == i) {
                        km = createKey(j, 'G', k);
                        if (!equal && !hasClear) {
                            hasClear = true;
                            vkmBest.clear();
                            vScore.clear();
                        }
                        vkmBest.push_back(km);
                        vScore.push_back(s);
                    }
                }
                for (int k = 0; k < (int)vIGARF.at(j).vFinalState.size(); k++) {
                    if (vIGARF.at(j).vFinalState.at(k) == i) {
                        km = createKey(j, 'F', k);
                        if (!equal && !hasClear) {
                            hasClear = true;
                            vkmBest.clear();
                            vScore.clear();
                        }
                        vkmBest.push_back(km);
                        vScore.push_back(s);
                    }
                }
                for (int k = 0; k < (int)vIGARF.at(j).vInitState.size(); k++) {
                    if (vIGARF.at(j).vInitState.at(k) == i) {
                        km = createKey(j, 'I', k);
                        if (!equal && !hasClear) {
                            vkmBest.clear();
                            vScore.clear();
                            hasClear = true;
                        }
                        vkmBest.push_back(km);
                        vScore.push_back(s);
                    }
                }
            }
        }
    }


    removeDoubleEvt(vkmBest);

    //if (vkmBest.size() > 1){// && score_max != 0) {
    //    cout << "several best target: " << vkmBest.size() << endl;
    //}
    iScore = score_max;
    //cout << " **score: " << score_max << "** " << endl;
    return vkmBest;
}

///< check in a vector of sKeyMean, if there are several identical states, if so, keep only the first
void SituationModel::removeDoubleEvt(vector<sKeyMean> &vkmBest){

    int ii = 0;
    int iRange = vChronoEvent.size();
    int toKeep = 0;
    vector<int>  toRemove;

    // check if several "goal" states.
    ii = 0;
    iRange = vChronoEvent.size();
    toKeep = 0;
    toRemove.clear();
    for (auto km : vkmBest){
        ostringstream ss;
        ss << km.cPart;
        pair<int, string> pTmp(km.iIGARF, ss.str());
        if (km.cPart == 'G'){
            int pos = find(vChronoEvent.begin(), vChronoEvent.end(), pTmp) - vChronoEvent.begin();
            if (pos < iRange){
                iRange = pos;
                toKeep = toRemove.size();
            }
            toRemove.push_back(ii);
        }
        ii++;
    }
    if (toRemove.size() > 1){
        toRemove.erase(toRemove.begin() + toKeep);
        sort(toRemove.rbegin(), toRemove.rend());
        for (auto er : toRemove){
            vkmBest.erase(vkmBest.begin() + er);
        }
    }

    // check if several "initial" states.
    ii = 0;
    iRange = vChronoEvent.size();
    toKeep = 0;
    toRemove.clear();
    for (auto km : vkmBest){
        ostringstream ss;
        ss << km.cPart;
        pair<int, string> pTmp(km.iIGARF, ss.str());
        if (km.cPart == 'I'){
            int pos = find(vChronoEvent.begin(), vChronoEvent.end(), pTmp) - vChronoEvent.begin();
            if (pos < iRange){
                iRange = pos;
                toKeep = toRemove.size();
            }
            toRemove.push_back(ii);
        }
        ii++;
    }
    if (toRemove.size() > 1){
        toRemove.erase(toRemove.begin() + toKeep);
        sort(toRemove.rbegin(), toRemove.rend());
        for (auto er : toRemove){
            vkmBest.erase(vkmBest.begin() + er);
        }
    }


    // check if several "final" states.
    ii = 0;
    iRange = vChronoEvent.size();
    toKeep = 0;
    toRemove.clear();
    for (auto km : vkmBest){
        ostringstream ss;
        ss << km.cPart;
        pair<int, string> pTmp(km.iIGARF, ss.str());
        if (km.cPart == 'F'){
            int pos = find(vChronoEvent.begin(), vChronoEvent.end(), pTmp) - vChronoEvent.begin();
            if (pos < iRange){
                iRange = pos;
                toKeep = toRemove.size();
            }
            toRemove.push_back(ii);
        }
        ii++;
    }
    if (toRemove.size() > 1){
        toRemove.erase(toRemove.begin() + toKeep);
        sort(toRemove.rbegin(), toRemove.rend());
        for (auto er : toRemove){
            vkmBest.erase(vkmBest.begin() + er);
        }
    }

    // check if several "final" states.
    ii = 0;
    iRange = vChronoEvent.size();
    toKeep = 0;
    toRemove.clear();
    for (auto km : vkmBest){
        ostringstream ss;
        ss << km.cPart;
        pair<int, string> pTmp(km.iIGARF, ss.str());
        if (km.cPart == 'R'){
            int pos = find(vChronoEvent.begin(), vChronoEvent.end(), pTmp) - vChronoEvent.begin();
            if (pos < iRange){
                iRange = pos;
                toKeep = toRemove.size();
            }
            toRemove.push_back(ii);
        }
        ii++;
    }
    if (toRemove.size() > 1){
        toRemove.erase(toRemove.begin() + toKeep);
        sort(toRemove.rbegin(), toRemove.rend());
        for (auto er : toRemove){
            vkmBest.erase(vkmBest.begin() + er);
        }
    }


}




string SituationModel::SMtoTrain(string sentence) {
    //to do
    //if (sentence != "") {
    //    Meaning m(sentence);
    //    m.setContext(getEvent(lastFocus));
    //    sKeyMean km;// = findBest(m.ocwSet());
    //    if (km.iIGARF != -1) {
    //        m.extractFocus(getEvent(km));
    //        m.extractOthers();
    //        lastFocus = km;
    //        return m.getMeaning();
    //    }
    //    else
    //        return "NoMatch";
    //}
    //else
    return "";
}

/*--------------------------*
 * LRHtoSM and LRHtoBlankSM *
 *--------------------------*/
sActionEvt SituationModel::extractAction(const string& meaning) {
    sActionEvt a;
    // Extract meaning (PAOR)
    vector <string> words = split(meaning, ' ');
    // Verb
    a.predicate = words.at(0);
    // Subject
    a.agent = words.at(1);
    // Object
    a.object = (words.size() > 2) ? words.at(2) : "";
    // Recipient
    a.recipient = (words.size() > 3) ? words.at(3) : "";
    return a;
}

sKeyMean SituationModel::findEventOrRelation(sActionEvt a) {
    for (int i = 0; i < (int)vActionEvts.size(); i++) {
        if (vActionEvts.at(i) == a) {
            // Search this event in the IGARF
            for (int j = 0; j < (int)vIGARF.size(); j++) {
                if (vIGARF.at(j).tAction == ACTION_EVT && vIGARF.at(j).iAction == i) {
                    return createKey(j, 'A', -1);
                }
                else if (vIGARF.at(j).tResult == ACTION_EVT && vIGARF.at(j).iResult == i) {
                    return createKey(j, 'R', -1);
                }
            }
        }
    }
    if (a.recipient == "") {
        sRelation r;
        r.verb = a.predicate;
        r.subject = a.agent;
        r.object = a.object;
        int i = findRelation(r);
        if (i != -1) {
            for (int j = 0; j < (int)vIGARF.size(); j++) {
                for (int k = 0; k < (int)vIGARF.at(j).vInitState.size(); k++) {
                    if (vIGARF.at(j).vInitState.at(k) == i) {
                        return createKey(j, 'I', k);
                    }
                }
                for (int k = 0; k < (int)vIGARF.at(j).vGoal.size(); k++) {
                    if (vIGARF.at(j).vGoal.at(k) == i) {
                        return createKey(j, 'G', k);
                    }
                }
                for (int k = 0; k < (int)vIGARF.at(j).vFinalState.size(); k++) {
                    if (vIGARF.at(j).vFinalState.at(k) == i) {
                        return createKey(j, 'F', k);
                    }
                }
            }
        }
    }
    sKeyMean km = createKey(-1, 'A', -1);
    return km;
}

void SituationModel::LRHtoSM(const string& meaning, bool create) {
    // Does the first line of the meaning gives the narrative semantic word used to create a link ?
    vector <string> lines = split(meaning, ',');
    // Extract each narrative words
    vector <string> dfws = split(lines.at(0), ' ');
    int meaningLine = 0; // Where is the meaning containing Action or Relation
    if (lines.size() > 1 &&
        ((dfws.size() > 0 &&
        VocabularyHandler::isDFW(dfws.at(0))) || // VocabularyHandler know this DFW...
        dfws.size() < 2)) { // ... or there is just one word so it can't be a verbal meaning (Predicate Agent is a minimum)
        meaningLine = 1; // First line contains DFW
    }

    // Get Current Key Mean
    sActionEvt a = extractAction(lines.at(meaningLine));

    vector<string> vTmp = { a.predicate, a.agent, a.object, a.recipient };
    int iScore = 0;
    sKeyMean current = findBest(vTmp, iScore)[0];
    if (current.iIGARF == -1) {
        // Test with pronouns
        const sActionEvt& context = getEvent(lastFocus);
        if (context.agent != "")
            VocabularyHandler::replacePronouns(context, a);
        vector<string> vTmp = { a.predicate, a.agent, a.object, a.recipient };
        iScore = 0;
        current = findBest(vTmp, iScore)[0];
        //        current = findEventOrRelation(a);
    }
    vector <string> words = split(lines.at(meaningLine), ' ');
    if (create && current.iIGARF == -1) { // Neither event nor relation found
        // Find out if relation or event is best
        if (VocabularyHandler::isActionVoc(words.at(0)) || words.size() < 3) { // Relation need at least three words
            // Create an event
            current.cPart = 'A';
            if (lastFocus.iIGARF != -1 &&
                (vIGARF.at(lastFocus.iIGARF).tAction == UNDEF))
                current.iIGARF = lastFocus.iIGARF;
            else
                current.iIGARF = createIGARF();
            const sActionEvt a = a;
            int i = addNewActionEvt(a);
            current.iRel = -1;
            modifEventIGARF(current.iIGARF, current.cPart, i);
        }
        else {
            // Add a relation to lastFocus
            if (lastFocus.iIGARF != -1 && (vIGARF.at(lastFocus.iIGARF).tAction == UNDEF)) // Setting the init Relations
                current.iIGARF = lastFocus.iIGARF;
            else
                current.iIGARF = createIGARF();
            current.cPart = 'I';
            sRelation r;
            r.verb = a.predicate;
            r.subject = a.agent;
            r.object = a.object;
            int instanceRelation = findRelation(r, true);
            current.iRel = addRelationIGARF(current.iIGARF, current.cPart, instanceRelation);
        }
    }

    if (meaningLine == 0) {
        dfws.clear();
        dfws.push_back(""); // Mute link: no DFW
    }
    // Extract each narrative words
    for (string w : dfws) {
        createLink(lastFocus, current, w);
    }
    lastFocus = current; // Move focus
}

void SituationModel::endSentence() {
    sentenceEnd = true;
    lastFocus = createKey(-1, 'A', -1);
}

/*---------*
 * SMtoLRH *
 *---------*/

void SituationModel::AUXautoLink(int iIGARF) {
    if (iIGARF < 0 || iIGARF >= (int)vIGARF.size())
        return;
    const sIGARF &igarf = vIGARF.at(iIGARF);
    sKeyMean last = createKey(-1, 'A', -1);
    int where = -1;
    while (where < (int)igarf.vInitState.size() - 1) {
        if (vRelSaid.find(igarf.vInitState.at(last.iRel + 1)) == vRelSaid.end()) { // This relation has not been said before
            sKeyMean next = createKey(iIGARF, 'I', last.iRel + 1);
            string word;
            if (last.cPart == 'I')
                word = "and";
            createLink(last, next, word);
            last = next;
            vRelSaid.insert(igarf.vInitState.at(last.iRel));
            where = last.iRel;
        }
        else {
            where++;
        }
    }
    if (igarf.tAction == ACTION_EVT) {
        if (vActSaid.find(igarf.iAction) == vActSaid.end()) {
            sKeyMean next = createKey(iIGARF, 'A', -1);
            createLink(last, next, "so");
            last = next;
            vActSaid.insert(igarf.iAction);
        }
    }
    else if (igarf.tAction == IGARF_EVT) {
        AUXautoLink(igarf.iAction);
    }
    if (igarf.tResult == ACTION_EVT) {
        if (vActSaid.find(igarf.iResult) == vActSaid.end() && !(vActionEvts.at(igarf.iAction) == vActionEvts.at(igarf.iResult))) {
            sKeyMean next = createKey(iIGARF, 'R', -1);
            createLink(last, next, "and");
            last = next;
            vActSaid.insert(igarf.iResult);
        }
        else if (vActionEvts.at(igarf.iAction) == vActionEvts.at(igarf.iResult)) {
            vActSaid.insert(igarf.iResult);
        }
    }
    else if (igarf.tResult == IGARF_EVT) {
        AUXautoLink(igarf.iResult);
    }
    last = createKey(-1, 'A', -1);
    where = -1;
    while (where < (int)igarf.vFinalState.size() - 1) {
        if (vRelSaid.find(igarf.vFinalState.at(last.iRel + 1)) == vRelSaid.end()) { // This relation has not been said before
            sKeyMean next = createKey(iIGARF, 'F', last.iRel + 1);
            string word;
            if (last.cPart == 'F')
                word = "and";
            else {
                word = "finally";
            }
            createLink(last, next, word);
            last = next;
            vRelSaid.insert(igarf.vFinalState.at(last.iRel));
            where = last.iRel;
        }
        else {
            where++;
        }
    }
    if (igarf.iNext != -1) {
        AUXautoLink(igarf.iNext);
    }
}

void SituationModel::autoLink(int iIGARF) {
    vRelSaid.clear();
    vActSaid.clear();
    AUXautoLink(iIGARF);
    vRelSaid.clear();
    vActSaid.clear();
}

void SituationModel::SMtoLRH(string lang) {
    sKeyMean last = createKey(-1, 'A', -1);
    for (int i = 0; i < (int)vDiscourseLinks.size(); i++) {
        // Getting all links that goes from and to the same events at this moment of the narrative
        int j = i;
        bool cont = true;
        while (j < (int)vDiscourseLinks.size() && cont) {
            if (!(vDiscourseLinks.at(j).fromEvt == vDiscourseLinks.at(i).fromEvt &&
                vDiscourseLinks.at(j).toEvt == vDiscourseLinks.at(i).toEvt)) {
                cont = false;
                j--;
            }
            j++;
        }
        j--;
        // Use the last one to have meaning
        const sDiscourseLink& lk = vDiscourseLinks.at(j);
        if (lk.fromEvt.iIGARF != -1) {
            if (last.iIGARF != lk.fromEvt.iIGARF ||
                last.cPart != lk.fromEvt.cPart ||
                last.iRel != lk.fromEvt.iRel) {
                // Tell the from event
                Meaning m("");
                m.addEvent(getEvent(lk.fromEvt));
                m.evtToMeaning(lang);
                cout << m.getMeaning() << endl;
            }
        }
        Meaning m("");
        m.setContext(getEvent(lk.fromEvt));
        // Adds all the DFW of the links collected
        for (int k = i; k <= j; k++) {
            if (vDiscourseLinks.at(k).word != "")
                m.addDFW(vDiscourseLinks.at(k).word);
        }
        m.addEvent(getEvent(lk.toEvt));
        m.evtToMeaning(lang);
        cout << m.getMeaning() << endl;
        last = lk.toEvt;
        i = j;
    }
}


/*-----------*
 * Rendering *
 *-----------*/

void SituationModel::initSizes(int _rendering_wEvtBox, int _rendering_hEvtBox, int _rendering_hOffset, int _rendering_vOffset) {
    rendering_wEvtBox = _rendering_wEvtBox;
    rendering_wIGARFBox = 20 + (rendering_wEvtBox + 10) * 5;
    rendering_hEvtBox = _rendering_hEvtBox;
    rendering_hIGARFBox = _rendering_hEvtBox + 20;
    rendering_hOffset = _rendering_hOffset;
    rendering_vOffset = _rendering_vOffset;
}

void SituationModel::calculateSize(int currentIGARF, vector < int > &IGARFlevels, int level) {
    if (currentIGARF < 0 || currentIGARF >(int)vIGARF.size() - 1)
        return;
    const sIGARF &evt = vIGARF.at(currentIGARF);
    if (level > (int)IGARFlevels.size() - 1)
        IGARFlevels.push_back(1);
    else
        IGARFlevels.at(level)++;
    if (evt.tAction == IGARF_EVT) {
        calculateSize(evt.iAction, IGARFlevels, level + 1);
    }
    if (evt.tResult == IGARF_EVT) {
        calculateSize(evt.iResult, IGARFlevels, level + 1);
    }
    calculateSize(evt.iNext, IGARFlevels, level);
}

pair <int, int> SituationModel::calculateSize(int nIGARF) {
    vector < int > IGARFlevels;
    calculateSize(nIGARF, IGARFlevels, 0);
    int width = 0;
    int depth = IGARFlevels.size();
    for (int m : IGARFlevels) {
        if (m > width)
            width = m;
    }
    return pair <int, int>(width*(rendering_wIGARFBox + rendering_hOffset) + 40, depth*(rendering_hIGARFBox + rendering_vOffset) + 40);
}

void SituationModel::writeIGARFdef(ofstream &fOutput, int nIGARF) {
    string outColour = "#16A086", inColour = "#9BBB58", lineColour = "#297FB8";
    string boxSize = "height=\"" + to_string(rendering_hEvtBox) + "px\" width=\"" + to_string(rendering_wEvtBox) + "px\"";
    pair <int, int> size = calculateSize(nIGARF);
    fOutput << "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
        << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 20010904//EN\"" << endl
        << "\"http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd\">" << endl
        << "<svg width=\"" << size.first << "px\" height=\"" << size.second << "px\" xml:lang=\"en\"" << endl
        << "xmlns=\"http://www.w3.org/2000/svg\"" << endl
        << "xmlns:xlink=\"http://www.w3.org/1999/xlink\">" << endl
        << "  <title>Test IGARF</title>" << endl
        << "  <desc>Automatically IGARF representation</desc>" << endl
        << "  " << endl
        << "  <defs>" << endl
        << "   <style type=\"text/css\"><![CDATA[" << endl
        << "   .box {" << endl
        << "     font-family: sans-serif;" << endl
        << "     font-size: 16px;" << endl
        << "   }" << endl
        << "   .arrow {" << endl
        << "     stroke: " << lineColour << ";" << endl
        << "     stroke-width: 4px;" << endl
        << "     marker-end: url(#Triangle);" << endl
        << "     fill: none;" << endl
        << "   }" << endl
        << "   .igarf {" << endl
        << "     stroke: " << outColour << ";" << endl
        << "     stroke-width: 10px;" << endl
        << "     fill: none;" << endl
        << "   }" << endl
        << "   .zoom {" << endl
        << "     fill: " << outColour << ";" << endl
        << "     fill-opacity: 0.3;" << endl
        << "   }" << endl
        << "   .event {" << endl
        << "     stroke: " << outColour << ";" << endl
        << "     stroke-width: 3px;" << endl
        << "     fill: " << inColour << ";" << endl
        << "     fill-opacity: 0.8;" << endl
        << "   }" << endl
        << "   .next {" << endl
        << "     fill: " << inColour << ";" << endl
        << "     fill-opacity: 0.8;" << endl
        << "   }" << endl
        << "   ]]></style>" << endl
        << "    <marker id=\"Triangle\"" << endl
        << "      viewBox=\"0 0 10 10\" refX=\"2\" refY=\"5\" " << endl
        << "      markerUnits=\"strokeWidth\"" << endl
        << "      markerWidth=\"4\" markerHeight=\"3\"" << endl
        << "      orient=\"auto\"" << endl
        << "      fill=\"" << lineColour << "\">" << endl
        << "      <path d=\"M 0 0 L 10 5 L 0 10 L 2 5 z\" />" << endl
        << "    </marker>" << endl
        << "    <g id=\"igarfBox\">" << endl
        << "      <rect x=\"0px\" y=\"0px\" height=\"" << rendering_hIGARFBox << "px\" width=\"" << rendering_wIGARFBox << "px\" class=\"igarf\"/>" << endl
        << "      <rect x=\"10px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
        << "      <rect x=\"" << rendering_wEvtBox + 20 << "px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
        << "      <rect x=\"" << (rendering_wEvtBox + 10) * 2 + 10 << "px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
        << "      <rect x=\"" << (rendering_wEvtBox + 10) * 3 + 10 << "px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
        << "      <rect x=\"" << (rendering_wEvtBox + 10) * 4 + 10 << "px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
        << "      <rect x=\"" << rendering_wIGARFBox - 10 << "px\" y=\"" << rendering_hIGARFBox / 2 - 30 << "px\" height=\"60px\" width=\"20px\" class=\"next\"/>" << endl
        << "    </g>" << endl
        << "  </defs>" << endl;
}

void SituationModel::writeSVGIGARF(ofstream &fOutput, int nIGARF, int x, int y) {
    if (nIGARF < 0 || nIGARF >(int)vIGARF.size() - 1)
        return;
    const sIGARF &evt = vIGARF.at(nIGARF);

    fOutput << "  <g transform=\"translate(" << x << " " << y << ")\" class=\"box\">" << endl
        << "    <use xlink:href=\"#igarfBox\"/>" << endl;

    // InitState
    int offset = 10;
    fOutput << "    <text transform=\"translate(" << offset << " 7)\">" << endl;
    for (int iRel : evt.vInitState) {
        const sRelation& r = vRelations.at(iRel);
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "      " << r.subject << " " << r.verb << " " << r.object << endl
            << "      </tspan>" << endl;
    }
    fOutput << "    </text>" << endl;

    // Goal
    offset += rendering_wEvtBox + 10;
    fOutput << "    <text transform=\"translate(" << offset << " 7)\">" << endl;
    for (int iRel : evt.vGoal) {
        const sRelation& r = vRelations.at(iRel);
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "      " << r.subject << " " << r.verb << " " << r.object << endl
            << "      </tspan>" << endl;
    }
    fOutput << "    </text>" << endl;

    // Action
    offset += rendering_wEvtBox + 10;
    if (evt.tAction == ACTION_EVT) {
        fOutput << "    <text transform=\"translate(" << offset << " 7)\">" << endl;
        const sActionEvt& a = vActionEvts.at(evt.iAction);
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "        A: " << a.agent << endl
            << "      </tspan>" << endl;
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "        P: " << a.predicate << endl
            << "      </tspan>" << endl;
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "        O: " << a.object << endl
            << "      </tspan>" << endl;
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "        R: " << a.recipient << endl
            << "      </tspan>" << endl;
        fOutput << "    </text>" << endl;
    }

    // Result
    offset += rendering_wEvtBox + 10;
    if (evt.tResult == ACTION_EVT) {
        fOutput << "    <text transform=\"translate(" << offset << " 7)\">" << endl;
        const sActionEvt& a = vActionEvts.at(evt.iResult);
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "        A: " << a.agent << endl
            << "      </tspan>" << endl;
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "        P: " << a.predicate << endl
            << "      </tspan>" << endl;
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "        O: " << a.object << endl
            << "      </tspan>" << endl;
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "        R: " << a.recipient << endl
            << "      </tspan>" << endl;
        fOutput << "    </text>" << endl;
    }

    // FinalState
    offset += rendering_wEvtBox + 10;
    fOutput << "    <text transform=\"translate(" << offset << " 7)\">" << endl;
    for (int iRel : evt.vFinalState) {
        const sRelation& r = vRelations.at(iRel);
        fOutput << "      <tspan x=\"10px\" dy=\"18px\">" << endl
            << "      " << r.subject << " " << r.verb << " " << r.object << endl
            << "      </tspan>" << endl;
    }
    fOutput << "    </text>" << endl;

    fOutput << "  </g>" << endl;
}

int SituationModel::addIGARFtoGrid(ofstream &fOutput, vector < int > &IGARFgrid, int currentIGARF, int level) {
    if (currentIGARF < 0 || currentIGARF >(int)vIGARF.size() - 1)
        return -1;
    if (level > (int)IGARFgrid.size() - 1)
        IGARFgrid.push_back(0);
    else
        IGARFgrid.at(level)++;
    int where = IGARFgrid.at(level);
    writeSVGIGARF(fOutput, currentIGARF, 20 + where*(rendering_wIGARFBox + rendering_hOffset), 20 + level*(rendering_hIGARFBox + rendering_vOffset));
    const sIGARF &evt = vIGARF.at(currentIGARF);
    if (evt.tAction == IGARF_EVT) {
        int beginAt = 0;
        if (level < (int)IGARFgrid.size() - 1)
            beginAt = IGARFgrid.at(level + 1) + 1;
        int endAt = addIGARFtoGrid(fOutput, IGARFgrid, evt.iAction, level + 1);
        int xactLeft = 20 + where*(rendering_wIGARFBox + rendering_hOffset) + (rendering_wEvtBox + 10) * 2 + 10;
        int yactBottom = 30 + rendering_hEvtBox + level*(rendering_hIGARFBox + rendering_vOffset);
        int xactRight = xactLeft + rendering_wEvtBox;
        int xigarfLeft = 20 + beginAt*(rendering_wIGARFBox + rendering_hOffset);
        int yigarfTop = 20 + (level + 1)*(rendering_hIGARFBox + rendering_vOffset);
        int xigarfRight = 20 + endAt*(rendering_wIGARFBox + rendering_hOffset) + rendering_wIGARFBox;
        fOutput << "  <path d=\"M " << xactLeft << "," << yactBottom << " L " << xigarfLeft << "," << yigarfTop << " L " << xigarfRight << "," << yigarfTop << " L " << xactRight << "," << yactBottom << "\" class=\"zoom\"/>" << endl
            << "  <path d=\"M " << xactLeft << "," << yactBottom << " L " << xigarfLeft << "," << yigarfTop << "\" class=\"arrow\"/>" << endl
            << "  <path d=\"M " << xactRight << "," << yactBottom << " L " << xigarfRight << "," << yigarfTop << "\" class=\"arrow\"/>" << endl;
    }
    if (evt.tResult == IGARF_EVT) {
        int beginAt = 0;
        if (level < (int)IGARFgrid.size() - 1)
            beginAt = IGARFgrid.at(level + 1) + 1;
        int endAt = addIGARFtoGrid(fOutput, IGARFgrid, evt.iResult, level + 1);
        int xactLeft = 20 + where*(rendering_wIGARFBox + rendering_hOffset) + (rendering_wEvtBox + 10) * 3 + 10;
        int yactBottom = 30 + rendering_hEvtBox + level*(rendering_hIGARFBox + rendering_vOffset);
        int xactRight = xactLeft + rendering_wEvtBox;
        int xigarfLeft = 20 + beginAt*(rendering_wIGARFBox + rendering_hOffset);
        int yigarfTop = 20 + (level + 1)*(rendering_hIGARFBox + rendering_vOffset);
        int xigarfRight = 20 + endAt*(rendering_wIGARFBox + rendering_hOffset) + rendering_wIGARFBox;
        fOutput << "  <path d=\"M " << xactLeft << "," << yactBottom << " L " << xigarfLeft << "," << yigarfTop << " L " << xigarfRight << "," << yigarfTop << " L " << xactRight << "," << yactBottom << "\" class=\"zoom\"/>" << endl
            << "  <path d=\"M " << xactLeft << "," << yactBottom << " L " << xigarfLeft << "," << yigarfTop << "\" class=\"arrow\"/>" << endl
            << "  <path d=\"M " << xactRight << "," << yactBottom << " L " << xigarfRight << "," << yigarfTop << "\" class=\"arrow\"/>" << endl;
    }
    if (evt.iNext != -1) {
        int x = 20 + where*(rendering_wIGARFBox + rendering_hOffset) + rendering_wIGARFBox;
        int y = 20 + level*(rendering_hIGARFBox + rendering_vOffset) + rendering_hIGARFBox / 2;
        fOutput << "  <path d=\"M " << x << "," << y << " l " << rendering_hOffset - 5 << ",0\" class=\"arrow\"/>" << endl;
    }
    addIGARFtoGrid(fOutput, IGARFgrid, evt.iNext, level);
    return IGARFgrid.at(level);
}

void SituationModel::writeSVG(ofstream &fOutput, int nIGARF) {
    writeIGARFdef(fOutput, nIGARF);
    // For each depth level, the number of IGARF already at that level is stored in IGARFgrid
    vector < int > IGARFgrid;
    IGARFgrid.push_back(-1);
    addIGARFtoGrid(fOutput, IGARFgrid, nIGARF, 0);
    fOutput << "</svg>\n";
    fOutput.flush();
}



void SituationModel::displayEvent(){
    int doku = 0;
    cout << endl;
    for (auto ig : vIGARF){
        cout << doku << ": " << ig.toString() << endl;
        if (ig.tAction == 1){
            cout << "\t [" << vActionEvts[ig.iAction].agent
                << "-" << vActionEvts[ig.iAction].predicate
                << "-" << vActionEvts[ig.iAction].object
                << "-" << vActionEvts[ig.iAction].recipient << "]" << endl;
        }
        else{
            cout << "\t [" << vActionEvts[vIGARF.at(ig.iAction).iAction].agent
                << "-" << vActionEvts[vIGARF.at(ig.iAction).iAction].predicate
                << "-" << vActionEvts[vIGARF.at(ig.iAction).iAction].object
                << "-" << vActionEvts[vIGARF.at(ig.iAction).iAction].recipient << "]" << endl;
        }

        doku++;
    }
}




void SituationModel::displayGoals(){
    int doku = 0;
    cout << endl;
    for (auto ig : vIGARF){
        cout << doku << ": " << ig.toString() << endl;
        for (auto goal : ig.vGoal){
            cout << "\t" << vRelations[goal].subject << " "
                << vRelations[goal].verb << " "
                << vRelations[goal].object << endl;
        }
        doku++;
    }
}


void SituationModel::checkEVTIGARF(EVT_IGARF &IGA_Input){

    IGA_Input.rangeIGARF = find(vChronoIgarf.begin(), vChronoIgarf.end(), IGA_Input.iIgarf) - vChronoIgarf.begin();

    ostringstream ss;
    ss << IGA_Input.km.cPart;
    pair<int, string> pTmp(IGA_Input.km.iIGARF, ss.str());

    int pos = find(vChronoEvent.begin(), vChronoEvent.end(), pTmp) - vChronoEvent.begin();

    double dPos = (pos*1.0) / (vChronoEvent.size() *1.0);
    //cout << "Event: " << IGA_Input.km.iIGARF << "-" << pTmp.first << "-" << pTmp.second << " is at pos: " << pos << ", -> " << dPos << endl;

    IGA_Input.dIGARF = dPos;

}



DFW::DFW(string name){
    sName = name;

    clearCorIG();
}



void DFW::analyseCorr(){
    clearCorIG();

    map<char, int>  dict = {
        { 'I', 0 },
        { 'G', 1 },
        { 'A', 2 },
        { 'R', 3 },
        { 'F', 4 }
    };


    // fill correlation map
    for (auto doIG : vDoubleIGARF){
        corIGARF[dict.find(doIG.first.km.cPart)->second][dict.find(doIG.second.km.cPart)->second]++;
    }

    // fill simple vector
    for (auto doIG : vSingleIGARF){
        simpleIGARF[dict.find(doIG.km.cPart)->second]++;
    }


    // Normalize
    double sumS = 0;
    double sumD = 0;
    for (int ii = 0; ii < 5; ii++){
        for (int jj = 0; jj < 5; jj++){
            sumD += corIGARF[ii][jj];
        }
        sumS += simpleIGARF[ii];
    }

    for (int ii = 0; ii < 5; ii++){
        for (int jj = 0; jj < 5; jj++){
            if (sumD != 0){
                corIGARF[ii][jj] /= (1.0*sumD);
            }
        }
        if (sumS != 0){
            simpleIGARF[ii] /= (1.0*sumS);
        }
    }
}


/// clear the correlation matrix (fill it with 0)
void DFW::clearCorIG(){
    for (int ii = 0; ii < 5; ii++){
        for (int jj = 0; jj < 5; jj++){
            corIGARF[ii][jj] = 0;
        }
        simpleIGARF[ii] = 0;
    }
}



void DFW::printCorMatrix(){

    cout << "Correlation Matrix " << sName << endl << endl;

    cout << " " << "\t" << "I" << "\t" << "G" << "\t" << "A" << "\t" << "R" << "\t" << "F" << endl;
    cout << "I";
    cout.precision(3);
    for (int ii = 0; ii < 5; ii++){
        cout << "\t" << corIGARF[0][ii];
    }
    cout << endl;
    cout << "G";
    for (int ii = 0; ii < 5; ii++){
        cout << "\t" << corIGARF[1][ii];
    }
    cout << endl;
    cout << "A";
    for (int ii = 0; ii < 5; ii++){
        cout << "\t" << corIGARF[2][ii];
    }
    cout << endl;
    cout << "R";
    for (int ii = 0; ii < 5; ii++){
        cout << "\t" << corIGARF[3][ii];
    }
    cout << endl;
    cout << "F";
    for (int ii = 0; ii < 5; ii++){
        cout << "\t" << corIGARF[4][ii];
    }
    cout << endl;
}



void DFW::createHistSimple(){
    // take the time data of simple events and ordinate them in an histogram

    vTimeSimple.clear();
    for (int ii = 0; ii < histoSize; ii++){
        vTimeSimple.push_back(0);
    }

    double stepSize = 1.0 / (histoSize*1.);

    for (auto single : vSingleIGARF){

        for (int step = 0; step < histoSize; step++){
            double lower = step*stepSize;
            double upper = (step + 1)*stepSize;
            if (single.dIGARF >= lower && single.dIGARF < upper){
                vTimeSimple[step]++;
            }
        }
    }

    double sum = 0;
    for (auto elt : vTimeSimple){
        sum += elt;
    }
    if (sum != 0){
        for (auto &elt : vTimeSimple){
            elt /= (1.0*sum);
        }
    }
}


void DFW::createHistDouble(){
    // take the time data of simple events and ordinate them in an histogram

    vTimeDouble.clear();
    for (int ii = 0; ii < histoSize; ii++){
        vTimeDouble.push_back(0);
    }

    double stepSize = 2.0 / (histoSize*1.);

    for (auto doubleIG : vDoubleIGARF){
        for (int step = 0; step < histoSize; step++){
            if (doubleIG.second.dIGARF - doubleIG.first.dIGARF >= (step*stepSize - 1)
                && doubleIG.second.dIGARF - doubleIG.first.dIGARF < ((step + 1)*stepSize - 1)){
                vTimeDouble[step]++;
            }
        }
    }

    double sum = 0;
    for (auto elt : vTimeDouble){
        sum += elt;
    }

    if (sum != 0){
        for (auto &elt : vTimeDouble){
            elt /= (1.0*sum);
        }
    }
}
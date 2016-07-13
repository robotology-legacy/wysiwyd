#include "storygraph.h"
#include <regex>
#include <map>

using namespace storygraph;
using namespace std;

void margin(int level) {
    for(int i = 0; i < level; i++)
        cout << "| ";
}

SituationModel::SituationModel() {clear();}

void SituationModel::clear() {
    vRelations.clear();
    vActionEvts.clear();
    vIGARF.clear();
    vDiscourseLinks.clear();
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
    cout.width(79 - 2*level);
    cout << right << "-" << endl;
    cout.fill(prevC);
}

void SituationModel::showIGARF(int i, int level) {
    if (i < 0 || i >= (int)vIGARF.size())
        return;

    const sIGARF &evt = vIGARF.at(i);
    margin(level);
    cout << "[" << i << "] " << endl;
    line(level);
    margin(level);
    cout << "+-INIT: " << dispRelations(evt.vInitState) << endl;
    margin(level);
    cout << "+-GOAL: " << dispRelations(evt.vGoal) << endl;
    margin(level);
    cout << "+-ACTION: ";
    if (evt.tAction == ACTION_EVT) {
        cout << "[" << evt.iAction << "] " << getSentenceEvt(evt.iAction) << endl;
    }
    else if (evt.tAction == IGARF_EVT) {
        cout << "[" << evt.iAction << "]" << endl;
        showIGARF(evt.iAction, level + 1);
    }
    else
        cout << endl;
    margin(level);
    cout << "+-RESULT: ";
    if (evt.tResult == ACTION_EVT) {
        cout << "[" << evt.iResult << "] " << getSentenceEvt(evt.iResult) << endl;
    }
    else if (evt.tResult == IGARF_EVT) {
        cout << "[" << evt.iResult << "]" << endl;
        showIGARF(evt.iResult, level + 1);
    }
    else
        cout << endl;
    margin(level);
    cout << "+-FINAL: " << dispRelations(evt.vFinalState) << endl;
    margin(level);
    cout << "+-NEXT: ";
    if (evt.iNext != -1) {
        cout << "[" << evt.iNext << "]" << endl;
        line(level);
        showIGARF(evt.iNext, level);
    }
    else {
        cout << endl;
        line(level);
    }
}

// Creation - Modification

/*----------*
 * Creation *
 *----------*/

int SituationModel::addNewActionEvt(std::string predicate, std::string agent, std::string object, std::string recipient) {
    sActionEvt action;
    action.predicate = predicate;
    action.agent     = agent;
    action.object    = object;
    action.recipient = recipient;
    vActionEvts.push_back(action);
    return vActionEvts.size() - 1;
}

int SituationModel::findRelation(sRelation rel) {
    for (int i = 0; i < (int)vRelations.size(); i++) {
        if (vRelations.at(i) == rel)
            return i;
    }
    return -1;
}

int SituationModel::addOrFindRelation(sRelation rel) {
    int i = findRelation(rel);
    if (i != -1)
        return i;
    vRelations.push_back(rel);
    return vRelations.size() - 1;
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

int find(std::vector < int > &vInt, int elt) {
    for(int i = 0; i < (int)vInt.size(); i++) {
        if (vInt.at(i) == elt)
            return i;
    }
    return -1;
}

void SituationModel::addRelationIGARF(int iIGARF, char cPart, int iRel) {
    if (iIGARF < 0 || iIGARF >= (int)vIGARF.size())
        return;
    if (iRel < 0 || iRel >= (int)vRelations.size())
        return;

    if (cPart == 'I' && find(vIGARF.at(iIGARF).vInitState, iRel) == -1) {
        vIGARF.at(iIGARF).vInitState.push_back(iRel);
    }
    else if (cPart == 'G' && find(vIGARF.at(iIGARF).vGoal, iRel) == -1) {
        vIGARF.at(iIGARF).vGoal.push_back(iRel);
    }
    else if (cPart == 'F' && find(vIGARF.at(iIGARF).vFinalState, iRel) == -1) {
        vIGARF.at(iIGARF).vFinalState.push_back(iRel);
    }
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

void SituationModel::createFromStory(const story &sto) {
    // Clearing
    clear();

    // Get events
    vector < evtStory > vEvents = sto.vEvents;

    // Step 1 : Considering begin and end of activities, create events and relations
    for (unsigned int i = 0; i < vEvents.size(); i++) {
        const evtStory &currentEvt = vEvents.at(i);
        if (currentEvt.begin) { // For each begin of action
            evtStory endCurrent;
            bool hasEnd = false;
            if (i + 1 < vEvents.size() && !vEvents.at(i + 1).begin && currentEvt.activity_name == vEvents.at(i + 1).activity_name) { // Is the next event the end event ?
                endCurrent = vEvents.at(i + 1);
                hasEnd = true;
            }
            // New Action events and relations
            sIGARF newEvent;
            newEvent.iNext = -1;
            // > Action
            newEvent.tAction = ACTION_EVT;
            newEvent.iAction = addNewActionEvt(currentEvt.predicate, currentEvt.agent, currentEvt.object, currentEvt.recipient);

            // > Init State
            for (int i = 0; i < currentEvt.bRelations.size(); i++) {
                if (currentEvt.bRelations.get(i).isList())
                    newEvent.vInitState.push_back(addOrFindRelation(fromValueToRelation(currentEvt.bRelations.get(i))));
            }

            // > Result
            newEvent.tResult = UNDEF;
            newEvent.iResult = -1;
            if (hasEnd) {
                sActionEvt result;
                result.predicate = endCurrent.predicate;
                result.agent     = endCurrent.agent;
                result.object    = endCurrent.object;
                result.recipient = endCurrent.recipient;
                if (findValue(endCurrent.vArgument, "status") == "failed") {
                    result.predicate = "fail";
                    result.object    = currentEvt.predicate;
                    result.recipient = "";

                    if (findValue(endCurrent.vArgument, "reason") == "outofreach") {
                        sRelation cause;
                        cause.subject = "it";
                        cause.verb = "is";
                        cause.object = "out-of-reach";
                        newEvent.vInitState.push_back(addOrFindRelation(cause));
                        newEvent.vFinalState.push_back(addOrFindRelation(cause));
                    }
                }
                vActionEvts.push_back(result);
                newEvent.tResult = ACTION_EVT;
                newEvent.iResult = vActionEvts.size() - 1;

                // > Goal
                if (findValue(endCurrent.vArgument, "goal") != "") {
                    // Extract the goal relation
                    std::string goal = findValue(endCurrent.vArgument, "goal");
                    size_t pos = goal.find("predicate") + 10;
                    std::string pred = goal.substr(pos, goal.find(")", pos) - pos);
                    pos = goal.find("agent") + 6;
                    std::string agent = goal.substr(pos, goal.find(")", pos) - pos);
                    pos = goal.find("object") + 7;
                    std::string object = goal.substr(pos, goal.find(")", pos) - pos);
                    sRelation g;
                    g.subject = agent;
                    g.verb = pred;
                    g.object = object;
                    vRelations.push_back(g);
                    int i = addOrFindRelation(g);
                    newEvent.vGoal.push_back(i);
                }

                // > Final State
                for (int i = 0; i < endCurrent.bRelations.size(); i++) {
                    if (endCurrent.bRelations.get(i).isList()) {
                        newEvent.vFinalState.push_back(addOrFindRelation(fromValueToRelation(endCurrent.bRelations.get(i))));
                    }
                }
            }
            else { // No end event
                newEvent.vFinalState = newEvent.vInitState;
            }

            // Stock it
            vIGARF.push_back(newEvent);
        }
    }

    // Step 2: Make chains
    // Each chain ends iff (1. Init and Final State are different) OR (2. Result is a failure) OR (3. End of the story)
    vector < int > rep; // Indexes of IGARF which packed chains (used for next step)
    int firstOfChain = -1;
    int lastOfChain = -1;
    int N = vIGARF.size();
    for (int i = 0; i < N; i++) {
        // End chain?
        if ((vIGARF.at(i).tResult == ACTION_EVT && vActionEvts.at(vIGARF.at(i).iResult).predicate == "fail") ||
            !(isRelationsBInA(vIGARF.at(i).vInitState, vIGARF.at(i).vFinalState) && isRelationsBInA(vIGARF.at(i).vFinalState, vIGARF.at(i).vInitState)) ||
            i == N - 1) {
            if (!(vIGARF.at(i).tResult == ACTION_EVT &&
                  vActionEvts.at(vIGARF.at(i).iResult).predicate == "fail") && // Not a failure
                !(isRelationsBInA(vIGARF.at(i).vInitState, vIGARF.at(i).vFinalState) &&
                  isRelationsBInA(vIGARF.at(i).vFinalState, vIGARF.at(i).vInitState))) { // Final and init state are differents
                //vIGARF.at(i).vGoal = vIGARF.at(i).vFinalState;
            }
            if (lastOfChain == -1)
                rep.push_back(i);
            else {
                vIGARF.at(lastOfChain).iNext = i;
                // New IGARF, pack the chain
                sIGARF newEvent;
                newEvent.vInitState = vIGARF.at(firstOfChain).vInitState;
                newEvent.vGoal = vIGARF.at(firstOfChain).vGoal;
                newEvent.tAction = IGARF_EVT;
                newEvent.iAction = firstOfChain;
                newEvent.tResult = vIGARF.at(i).tResult;
                newEvent.iResult = vIGARF.at(i).iResult;
                newEvent.vFinalState = vIGARF.at(i).vFinalState;
                newEvent.iNext = -1;
                vIGARF.push_back(newEvent);

                rep.push_back(vIGARF.size() - 1);
                firstOfChain = -1;
                lastOfChain = -1;
            }
        }
        else {
            if (lastOfChain == -1) {
                firstOfChain = i;
                lastOfChain = i;
            }
            else {
                vIGARF.at(lastOfChain).iNext = i;
                lastOfChain = i;
            }
        }
    }
    int head = rep.at(0);

    // Step 3: Assembling chains
    int j;
    for (unsigned int i = 0; i < rep.size(); i++) {
        j = rep.at(i);
        if (vIGARF.at(j).tResult == ACTION_EVT && vActionEvts.at(vIGARF.at(j).iResult).predicate == "fail" && i + 1 < rep.size()) {
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
        for(int i = 0; i < (int)vIGARF.size(); i++) {
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
        for(int i = 0; i < (int)vIGARF.size(); i++) {
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

    showIGARF(head);
}

/*---------*
 * Meaning *
 *---------*/

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

    if (iRel < 0 || iRel >= (int)vRelations.size())
        k.iRel = -1;
    else
        k.iRel = iRel;

    return k;
}

void SituationModel::createLink(sKeyMean from, sKeyMean to, string word) {
    sDiscourseLink newLink;
    newLink.fromEvt = from;
    newLink.toEvt = to;
    newLink.word = word;
    vDiscourseLinks.push_back(newLink);
}

pair <string, string> SituationModel::meaningFromKeyMean(const sKeyMean &key, int beginAt) {
    pair <string, string> ans;
    if (key.iIGARF < 0 || key.iIGARF >= (int)vIGARF.size()) {
        return ans;
    }
    ans.second = "[";
    int restartAt = beginAt + 3;
    for (int i = 0; i < beginAt; i++)
        ans.second += "_-";
    const sIGARF &igarf = vIGARF.at(key.iIGARF);
    if (key.cPart == 'A' || key.cPart == 'R') {
        const sActionEvt& e = (key.cPart == 'A') ? vActionEvts.at(igarf.iAction):vActionEvts.at(igarf.iResult);
        ans.first = e.predicate + " " + e.agent;
        ans.second += "A-P-";
        if (e.object != "") {
            ans.second += "O-";
            ans.first += " " + e.object;
        }
        else
            ans.second += "_-";
        if (e.recipient != "") {
            ans.second += "R";
            ans.first += " " + e.recipient;
        }
        else
            ans.second += "_";
        restartAt = beginAt + 4;
    }
    else if (key.cPart == 'I') {
        const sRelation& r = vRelations.at(igarf.vInitState.at(key.iRel));
        ans.first = r.verb + " " + r.subject + " " + r.object;
        ans.second += "A-P-O";
    }
    else if (key.cPart == 'G') {
        const sRelation& r = vRelations.at(igarf.vGoal.at(key.iRel));
        ans.first = r.verb + " " + r.subject + " " + r.object;
        ans.second += "A-P-O";
    }
    else if (key.cPart == 'F') {
        const sRelation& r = vRelations.at(igarf.vFinalState.at(key.iRel));
        ans.first = r.verb + " " + r.subject + " " + r.object;
        ans.second += "A-P-O";
    }
    for (int i = restartAt; i < 8; i++)
        ans.second += "-_";
    ans.second += "]";
    return ans;
}

void SituationModel::produceBasicMeaning() {
    sKeyMean last;
    last.iIGARF = -1;
    last.cPart = 'A';
    last.iRel = -1;
    for (sDiscourseLink lk : vDiscourseLinks) {
        if (last.iIGARF != lk.fromEvt.iIGARF ||
            last.cPart != lk.fromEvt.cPart ||
            last.iRel != lk.fromEvt.iRel) {
            // Tell the from event
            pair <string, string> ans = meaningFromKeyMean(lk.fromEvt);
            cout << " , " << ans.first << " <o> [_-_-_-_-_-_-_-_]" << ans.second << " <o> " << endl;
        }
        cout << lk.word << " , ";
        // Tell the to_event
        sActionEvt e;
        pair <string, string> ans = meaningFromKeyMean(lk.toEvt, (lk.word != "") ? 1 : 0);
        cout << ans.first << " <o> ";
        if (lk.word != "")
            cout << "[P-_-_-_-_-_-_-_]";
        else
            cout << "[_-_-_-_-_-_-_-_]";
        cout << ans.second << " <o> " << endl;
        last = lk.toEvt;
    }
}

sKeyMean SituationModel::findEventOrRelation(string meaning) {
    meaning = meaning.substr(0, meaning.find('<'));
    // Extract meaning (PAOR)
    vector <string> words = split(meaning, ' ');
    // Verb
    string predicate = words.at(0);
    // Subject
    string agent = words.at(1);
    // Object
    string object = (words.size() > 2) ? words.at(2) : "";
    // Recipient
    string recipient = (words.size() > 3) ? words.at(3) : "";
    // Find it
    return findEventOrRelation(predicate, agent, object, recipient);
}

sKeyMean SituationModel::findEventOrRelation(std::string predicate, std::string agent, std::string object, std::string recipient) {
    for (int i = 0; i < (int)vActionEvts.size(); i++) {
        if (vActionEvts.at(i).predicate == predicate &&
            vActionEvts.at(i).agent     == agent     &&
            vActionEvts.at(i).object    == object    &&
            vActionEvts.at(i).recipient == recipient) {
            // Search this event in the IGARF
            for (int j = 0; j < (int)vIGARF.size(); j++) {
                if (vIGARF.at(j).tAction == ACTION_EVT && vIGARF.at(j).iAction == i) {
                    sKeyMean km;
                    km.iIGARF = j;
                    km.cPart = 'A';
                    km.iRel = -1;
                    return km;
                }
                else if (vIGARF.at(j).tResult == ACTION_EVT && vIGARF.at(j).iResult == i) {
                    sKeyMean km;
                    km.iIGARF = j;
                    km.cPart = 'R';
                    km.iRel = -1;
                    return km;
                }
            }
        }
    }
    if (recipient == "") {
        sRelation r;
        r.verb = predicate;
        r.subject = agent;
        r.object = object;
        int i = findRelation(r);
        if (i != -1) {
            for (int j = 0; j < (int)vIGARF.size(); j++) {
                for (int k = 0; k < (int)vIGARF.at(j).vInitState.size(); k++) {
                    if (vIGARF.at(j).vInitState.at(k) == i) {
                        sKeyMean km;
                        km.iIGARF = j;
                        km.cPart = 'I';
                        km.iRel = k;
                        return km;
                    }
                }
                for (int k = 0; k < (int)vIGARF.at(j).vGoal.size(); k++) {
                    if (vIGARF.at(j).vGoal.at(k) == i) {
                        sKeyMean km;
                        km.iIGARF = j;
                        km.cPart = 'G';
                        km.iRel = k;
                        return km;
                    }
                }
                for (int k = 0; k < (int)vIGARF.at(j).vFinalState.size(); k++) {
                    if (vIGARF.at(j).vFinalState.at(k) == i) {
                        sKeyMean km;
                        km.iIGARF = j;
                        km.cPart = 'F';
                        km.iRel = k;
                        return km;
                    }
                }
            }
        }
    }
    sKeyMean km;
    km.iIGARF = -1;
    km.cPart = 'A';
    km.iRel = -1;
    return km;
}

void SituationModel::endSentence() {
    sentenceEnd = true;
    lastFocus = createKey(-1, 'A', -1);
}

int SituationModel::extractRel(string meaning) {
    meaning = meaning.substr(0, meaning.find('<'));
    // Extract meaning (PAOR)
    vector <string> words = split(meaning, ' ');
    // Verb
    string predicate = words.at(0);
    // Subject
    string agent = words.at(1);
    // Object
    string object = (words.size() > 2) ? words.at(2) : "";
    sRelation r;
    r.verb = predicate;
    r.subject = agent;
    r.object = object;
    return addOrFindRelation(r);
}

int SituationModel::extractAction(string meaning) {
    meaning = meaning.substr(0, meaning.find('<'));
    // Extract meaning (PAOR)
    vector <string> words = split(meaning, ' ');
    // Verb
    string predicate = words.at(0);
    // Subject
    string agent = words.at(1);
    // Object
    string object = (words.size() > 2) ? words.at(2) : "";
    // Recipient
    string recipient = (words.size() > 3) ? words.at(3) : "";
    return addNewActionEvt(predicate, agent, object, recipient);
}

void SituationModel::addLinkFromMeaning(string meaning, bool create) {
    // The first line of the meaning gives the narrative semantic word used to create a link
    vector <string> lines = split(meaning, ',');

    // Get Current Key Mean
    sKeyMean current = findEventOrRelation(lines.at(1));
    vector <string> words = split(lines.at(1), ' ');

    // Extract each narrative words
    vector <string> dfws = split(lines.at(0), ' ');
    for (string w : dfws) {
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
                if (lastFocus.iIGARF != -1 &&
                    lastFocus.iIGARF != current.iIGARF &&
                    vIGARF.at(lastFocus.iIGARF).iNext == -1) // Link to last Focus
                    vIGARF.at(lastFocus.iIGARF).iNext = current.iIGARF;
                int i = extractAction(lines.at(1));
                current.iRel = -1;
                modifEventIGARF(current.iIGARF, current.cPart, i);
            }
            else {
                // Add a relation to lastFocus
                if (lastFocus.iIGARF != -1 && (vIGARF.at(lastFocus.iIGARF).tAction == UNDEF)) // Setting the init Relations
                    current.iIGARF = lastFocus.iIGARF;
                else
                    current.iIGARF = createIGARF();
                if (lastFocus.iIGARF != -1 &&
                    lastFocus.iIGARF != current.iIGARF &&
                    vIGARF.at(lastFocus.iIGARF).iNext == -1) // Link to last Focus
                    vIGARF.at(lastFocus.iIGARF).iNext = current.iIGARF;
                current.cPart = 'I';
                current.iRel = extractRel(lines.at(1));
                addRelationIGARF(current.iIGARF, current.cPart, current.iRel);
            }
        }
        createLink(lastFocus, current, w);
    }
    lastFocus = current;
}

int SituationModel::proximityScoreAction(int i, set <string> ocw) {
    if (i < 0 || i >= (int)vActionEvts.size())
        return -1;
    const sActionEvt& e = vActionEvts.at(i);
    int score = 0;
    if (VocabularyHandler::shareMeaning(e.predicate, ocw))
        score += 5;
    if (VocabularyHandler::shareMeaning(e.agent, ocw))
        score += 4;
    if (VocabularyHandler::shareMeaning(e.object, ocw))
        score += 2;
    if (VocabularyHandler::shareMeaning(e.recipient, ocw))
        score += 1;
    return (score = 12) ? 1 : 0;
}

int SituationModel::proximityScoreRelation(int i, set <string> ocw) {
    if (i < 0 || i >= (int)vRelations.size())
        return -1;
    const sRelation& r = vRelations.at(i);
    int score = 0;
    if (VocabularyHandler::shareMeaning(r.verb, ocw))
        score += 5;
    if (VocabularyHandler::shareMeaning(r.subject, ocw))
        score += 4;
    if (VocabularyHandler::shareMeaning(r.object, ocw))
        score += 2;
    return (score = 9) ? 1 : 0;
}

sKeyMean SituationModel::findBest(set<string> ocw) {
    int score_max = 0;
    sKeyMean km;
    km.iIGARF = -1;
    km.cPart = 'A';
    km.iRel = -1;
    for (int i = 0; i < (int)vActionEvts.size(); i++) {
        int s = proximityScoreAction(i, ocw);
        if (s > score_max) {
            score_max = s;
            // Search this event in the IGARF
            for (int j = 0; j < (int)vIGARF.size(); j++) {
                if (vIGARF.at(j).tAction == ACTION_EVT && vIGARF.at(j).iAction == i) {
                    km.iIGARF = j;
                    km.cPart = 'A';
                    km.iRel = -1;
                }
                else if (vIGARF.at(j).tResult == ACTION_EVT && vIGARF.at(j).iResult == i) {
                    km.iIGARF = j;
                    km.cPart = 'R';
                    km.iRel = -1;
                }
            }
        }
    }
    for (int i = 0; i < (int)vRelations.size(); i++) {
        int s = proximityScoreRelation(i, ocw);
        if (s > score_max) {
            score_max = s;
            for (int j = 0; j < (int)vIGARF.size(); j++) {
                for (int k = 0; k < (int)vIGARF.at(j).vInitState.size(); k++) {
                    if (vIGARF.at(j).vInitState.at(k) == i) {
                        km.iIGARF = j;
                        km.cPart = 'I';
                        km.iRel = k;
                    }
                }
                for (int k = 0; k < (int)vIGARF.at(j).vGoal.size(); k++) {
                    if (vIGARF.at(j).vGoal.at(k) == i) {
                        km.iIGARF = j;
                        km.cPart = 'G';
                        km.iRel = k;
                    }
                }
                for (int k = 0; k < (int)vIGARF.at(j).vFinalState.size(); k++) {
                    if (vIGARF.at(j).vFinalState.at(k) == i) {
                        km.iIGARF = j;
                        km.cPart = 'F';
                        km.iRel = k;
                    }
                }
            }
        }
    }
    return km;
}

void SituationModel::copyOCW(sKeyMean km, string &predicate, string &agent, string &object, string &recipient) {
    if (km.iIGARF == -1) {
        return;
    }
    const sIGARF &igarf = vIGARF.at(km.iIGARF);
    if (km.cPart == 'A' || km.cPart == 'R') {
        const sActionEvt& e = (km.cPart == 'A') ? vActionEvts.at(igarf.iAction) : vActionEvts.at(igarf.iResult);
        predicate = e.predicate;
        agent = e.agent;
        object = e.object;
        recipient = e.recipient;
    }
    else if (km.cPart == 'I') {
        const sRelation& r = vRelations.at(igarf.vInitState.at(km.iRel));
        predicate = r.verb;
        agent = r.subject;
        object = r.object;
        recipient = "";
    }
    else if (km.cPart == 'G') {
        const sRelation& r = vRelations.at(igarf.vGoal.at(km.iRel));
        predicate = r.verb;
        agent = r.subject;
        object = r.object;
        recipient = "";
    }
    else if (km.cPart == 'F') {
        const sRelation& r = vRelations.at(igarf.vFinalState.at(km.iRel));
        predicate = r.verb;
        agent = r.subject;
        object = r.object;
        recipient = "";
    }
}

/*-----------*
 * Rendering *
 *-----------*/

void SituationModel::initSizes(int _rendering_wEvtBox, int _rendering_hEvtBox, int _rendering_hOffset, int _rendering_vOffset) {
    rendering_wEvtBox   = _rendering_wEvtBox;
    rendering_wIGARFBox = 20 + (rendering_wEvtBox + 10)*5;
    rendering_hEvtBox   = _rendering_hEvtBox;
    rendering_hIGARFBox = _rendering_hEvtBox + 20;
    rendering_hOffset   = _rendering_hOffset;
    rendering_vOffset   = _rendering_vOffset;
}

void SituationModel::calculateSize(int currentIGARF, vector < int > &IGARFlevels, int level) {
    if (currentIGARF < 0 || currentIGARF > (int)vIGARF.size() - 1)
        return;
    const sIGARF &evt = vIGARF.at(currentIGARF);
    if (level > (int)IGARFlevels.size() - 1)
        IGARFlevels.push_back(1);
    else
        IGARFlevels.at(level)++;
    if (evt.tAction == IGARF_EVT) {
        calculateSize(evt.iAction, IGARFlevels, level+1);
    }
    if (evt.tResult == IGARF_EVT) {
        calculateSize(evt.iResult, IGARFlevels, level+1);
    }
    calculateSize(evt.iNext, IGARFlevels, level);
}

pair <int, int> SituationModel::calculateSize(int nIGARF) {
    vector < int > IGARFlevels;
    calculateSize(nIGARF, IGARFlevels, 0);
    int width = 0;
    int depth = IGARFlevels.size();
    for(int m : IGARFlevels) {
        if (m > width)
            width = m;
    }
    return pair <int, int>(width*(rendering_wIGARFBox + rendering_hOffset) + 40, depth*(rendering_hIGARFBox + rendering_vOffset) + 40);
}

void SituationModel::writeIGARFdef(std::ofstream &fOutput, int nIGARF) {
    string outColour = "#16A086", inColour ="#9BBB58", lineColour = "#297FB8";
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
            << "      <rect x=\"0px\" y=\"0px\" height=\""<< rendering_hIGARFBox <<"px\" width=\""<< rendering_wIGARFBox <<"px\" class=\"igarf\"/>" << endl
            << "      <rect x=\"10px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
            << "      <rect x=\"" << rendering_wEvtBox + 20 << "px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
            << "      <rect x=\"" << (rendering_wEvtBox + 10)*2 + 10 << "px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
            << "      <rect x=\"" << (rendering_wEvtBox + 10)*3 + 10 << "px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
            << "      <rect x=\"" << (rendering_wEvtBox + 10)*4 + 10 << "px\" y=\"10px\" " << boxSize << " class=\"event\"/>" << endl
            << "      <rect x=\"" << rendering_wIGARFBox - 10 << "px\" y=\"" << rendering_hIGARFBox/2 - 30 << "px\" height=\"60px\" width=\"20px\" class=\"next\"/>" << endl
            << "    </g>" << endl
            << "  </defs>" << endl;
}

void SituationModel::writeSVGIGARF(ofstream &fOutput, int nIGARF, int x, int y) {
    if (nIGARF < 0 || nIGARF > (int)vIGARF.size() - 1)
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
    if (currentIGARF < 0 || currentIGARF > (int)vIGARF.size() - 1)
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
        int xactLeft    = 20 + where*(rendering_wIGARFBox + rendering_hOffset) + (rendering_wEvtBox + 10)*2 + 10;
        int yactBottom  = 30 + rendering_hEvtBox + level*(rendering_hIGARFBox + rendering_vOffset);
        int xactRight   = xactLeft + rendering_wEvtBox;
        int xigarfLeft  = 20 + beginAt*(rendering_wIGARFBox + rendering_hOffset);
        int yigarfTop   = 20 + (level + 1)*(rendering_hIGARFBox + rendering_vOffset);
        int xigarfRight = 20 + endAt*(rendering_wIGARFBox + rendering_hOffset) + rendering_wIGARFBox;
        fOutput << "  <path d=\"M " << xactLeft  << "," << yactBottom << " L " << xigarfLeft  << "," << yigarfTop << " L " << xigarfRight << "," << yigarfTop << " L " << xactRight << "," << yactBottom << "\" class=\"zoom\"/>" << endl
                << "  <path d=\"M " << xactLeft  << "," << yactBottom << " L " << xigarfLeft  << "," << yigarfTop << "\" class=\"arrow\"/>" << endl
                << "  <path d=\"M " << xactRight << "," << yactBottom << " L " << xigarfRight << "," << yigarfTop << "\" class=\"arrow\"/>" << endl;
    }
    if (evt.tResult == IGARF_EVT) {
        int beginAt = 0;
        if (level < (int)IGARFgrid.size() - 1)
            beginAt = IGARFgrid.at(level + 1) + 1;
        int endAt = addIGARFtoGrid(fOutput, IGARFgrid, evt.iResult, level + 1);
        int xactLeft    = 20 + where*(rendering_wIGARFBox + rendering_hOffset) + (rendering_wEvtBox + 10)*3 + 10;
        int yactBottom  = 30 + rendering_hEvtBox + level*(rendering_hIGARFBox + rendering_vOffset);
        int xactRight   = xactLeft + rendering_wEvtBox;
        int xigarfLeft  = 20 + beginAt*(rendering_wIGARFBox + rendering_hOffset);
        int yigarfTop   = 20 + (level + 1)*(rendering_hIGARFBox + rendering_vOffset);
        int xigarfRight = 20 + endAt*(rendering_wIGARFBox + rendering_hOffset) + rendering_wIGARFBox;
        fOutput << "  <path d=\"M " << xactLeft  << "," << yactBottom << " L " << xigarfLeft  << "," << yigarfTop << " L " << xigarfRight << "," << yigarfTop << " L " << xactRight << "," << yactBottom << "\" class=\"zoom\"/>" << endl
                << "  <path d=\"M " << xactLeft  << "," << yactBottom << " L " << xigarfLeft  << "," << yigarfTop << "\" class=\"arrow\"/>" << endl
                << "  <path d=\"M " << xactRight << "," << yactBottom << " L " << xigarfRight << "," << yigarfTop << "\" class=\"arrow\"/>" << endl;
    }
    if (evt.iNext != -1) {
        int x = 20 + where*(rendering_wIGARFBox + rendering_hOffset) + rendering_wIGARFBox;
        int y = 20 + level*(rendering_hIGARFBox + rendering_vOffset) + rendering_hIGARFBox/2;
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


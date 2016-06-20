#include <storygraph.h>
#include <regex>
#include <map>

using namespace storygraph;
using namespace std;

void margin(int level) {
    for(int i = 0; i < level; i++)
        cout << "| ";
}

situationModel::situationModel() {}

void situationModel::clear() {
    vRelations.clear();
    vActionEvts.clear();
    vIGARF.clear();
    vDiscourseLinks.clear();
    endSentence();
}

bool situationModel::sameRelation(const sRelation &r1, const sRelation &r2) {
    return (r1.verb    == r2.verb    &&
            r1.subject == r2.subject &&
            r1.object  == r2.object);
}

// Display

string situationModel::getSentenceEvt(int i) {
    if (i < 0 || i >= (int)vActionEvts.size())
        return "";

    return vActionEvts.at(i).agent + " " +
           vActionEvts.at(i).predicate + " " +
           vActionEvts.at(i).object + " " +
           vActionEvts.at(i).recipient;
}

string situationModel::getSentenceRel(int i) {
    if (i < 0 || i >= (int)vRelations.size())
        return "";

    return vRelations.at(i).subject + " " + vRelations.at(i).verb + " " + vRelations.at(i).object;
}

string situationModel::dispRelations(const vector < int >& rels) {
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

void situationModel::showIGARF(int i, int level) {
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

// Creation
int situationModel::addNewActionEvt(std::string predicate, std::string agent, std::string object, std::string recipient) {
    sActionEvt action;
    action.predicate = predicate;
    action.agent     = agent;
    action.object    = object;
    action.recipient = recipient;
    vActionEvts.push_back(action);
    return vActionEvts.size() - 1;
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

int situationModel::findRelation(sRelation rel) {
    for (int i = 0; i < (int)vRelations.size(); i++) {
        if (sameRelation(vRelations.at(i), rel))
            return i;
    }
    return -1;
}

int situationModel::addOrFindRelation(sRelation rel) {
    int i = findRelation(rel);
    if (i != -1)
        return i;
    vRelations.push_back(rel);
    return vRelations.size() - 1;
}

int situationModel::createIGARF() {
    sIGARF newEvent;
    newEvent.tAction = UNDEF;
    newEvent.tResult = UNDEF;
    newEvent.iAction = -1;
    newEvent.iResult = -1;
    newEvent.iNext = -1;
    vIGARF.push_back(newEvent);
    return vIGARF.size() - 1;
}

// Modification
void situationModel::modifEventIGARF(int iIGARF, char cPart, int iActEvt) {
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

void situationModel::modifContentIGARF(int iIGARF, char cPart, int jIGARF) {
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

void situationModel::remContentIGARF(int iIGARF, char cPart) {
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

void situationModel::addRelationIGARF(int iIGARF, char cPart, int iRel) {
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

void situationModel::removeRelationIGARF(int iIGARF, char cPart, int iRel) {
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

sRelation situationModel::fromValueToRelation(const yarp::os::Value& b) {
    sRelation n;
    if (b.isList()) {
        n.subject   = b.asList()->get(0).toString();
        n.verb      = b.asList()->get(1).toString();
        n.object    = b.asList()->get(2).toString();
    }
    return n;
}

void situationModel::createFromStory(const story &sto) {
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
            if (i + 1 < vEvents.size() && !vEvents.at(i+1).begin && currentEvt.activity_name == vEvents.at(i + 1).activity_name) { // Is the next event the end event ?
                endCurrent = vEvents.at(i+1);
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
                    result.object    = "to_" + currentEvt.predicate;
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

                // > Final State
                for (int i = 0; i < endCurrent.bRelations.size(); i++) {
                    if (endCurrent.bRelations.get(i).isList()) {
                        newEvent.vFinalState.push_back(addOrFindRelation(fromValueToRelation(endCurrent.bRelations.get(i))));
                    }
                }

                // > Goal
                if (!isRelationsBInA(newEvent.vInitState, newEvent.vFinalState)) {
                    for (int j = 0; j < (int)newEvent.vFinalState.size(); j++) {
                        bool found = false;
                        for (int i = 0; i < (int)newEvent.vInitState.size() && !found; i++) {
                            if (sameRelation(vRelations.at(newEvent.vInitState.at(i)), vRelations.at(newEvent.vFinalState.at(j)))) {
                                found = true;
                            }
                        }
                        if (!found) {
                            newEvent.vGoal.push_back(newEvent.vFinalState.at(j));
                        }
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
        if ((vIGARF.at(i).tResult == ACTION_EVT && vActionEvts.at(vIGARF.at(i).iResult).predicate == "fail_to") ||
            !(isRelationsBInA(vIGARF.at(i).vInitState, vIGARF.at(i).vFinalState) && isRelationsBInA(vIGARF.at(i).vFinalState, vIGARF.at(i).vInitState)) ||
            i == N - 1) {
            if (!(vIGARF.at(i).tResult == ACTION_EVT &&
                  vActionEvts.at(vIGARF.at(i).iResult).predicate == "fail") && // Not a failure
                !(isRelationsBInA(vIGARF.at(i).vInitState, vIGARF.at(i).vFinalState) &&
                  isRelationsBInA(vIGARF.at(i).vFinalState, vIGARF.at(i).vInitState))) { // Final and init state are differents
                vIGARF.at(i).vGoal = vIGARF.at(i).vFinalState;
            }
            if (lastOfChain == -1)
                rep.push_back(i);
            else {
                vIGARF.at(lastOfChain).iNext = i;
                // New IGARF, pack the chain
                sIGARF newEvent;
                newEvent.vInitState = vIGARF.at(firstOfChain).vInitState;
                newEvent.vGoal = vIGARF.at(firstOfChain).vGoal; // < Todo: Where goal should be taken?
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
        if (vIGARF.at(j).tResult == ACTION_EVT && vActionEvts.at(vIGARF.at(j).iResult).predicate == "fail_to" && i + 1 < rep.size()) {
            // New IGARF, pack current and next
            sIGARF newEvent;
            newEvent.vInitState = vIGARF.at(j).vInitState;
            newEvent.vGoal = vIGARF.at(j).vGoal; // < Todo: Where goal should be taken?
            newEvent.tAction = IGARF_EVT;
            newEvent.iAction = j;
            newEvent.tResult = IGARF_EVT;
            newEvent.iResult = rep.at(i + 1);
            newEvent.vFinalState = vIGARF.at(rep.at(i + 1)).vFinalState;
            newEvent.iNext = -1;
            vIGARF.push_back(newEvent);
            if (head == j)
                head = vIGARF.size() - 1;
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

// Meaning
sKeyMean situationModel::createKey(int iIGARF, char cPart, int iRel) {
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

void situationModel::createLink(sKeyMean from, sKeyMean to, string word) {
    sDiscourseLink newLink;
    newLink.fromEvt = from;
    newLink.toEvt = to;
    newLink.word = word;
    vDiscourseLinks.push_back(newLink);
}

sKeyMean situationModel::findEventOrRelation(string meaning) {
    meaning = meaning.substr(0, meaning.find('<'));
    meaning += " ";
    // Extract meaning (PAOR)
    string rest = "";
    // Predicate
    size_t posEnd = meaning.find(' ');
    string predicate = meaning.substr(0, posEnd);
    // Agent
    string agent = meaning.substr(posEnd + 1, meaning.find(' ', posEnd + 1) - posEnd - 1);
    posEnd = meaning.find(' ', posEnd + 1);
    // Object
    string object = meaning.substr(posEnd + 1, meaning.find(' ', posEnd + 1) - posEnd - 1);
    posEnd = meaning.find(' ', posEnd + 1);
    // Recipient
    string recipient = meaning.substr(posEnd + 1, meaning.find(' ', posEnd + 1) - posEnd - 1);
    // Find it
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
        }
    }
    sKeyMean km;
    km.iIGARF = -1;
    km.cPart = 'A';
    km.iRel = -1;
    return km;
}

bool situationModel::pointToEvent(std::string word) {
    int evtCount = 0;
    int relCount = 0;
    for(sDiscourseLink lk : vDiscourseLinks) {
        if (lk.word == word) {
            if (lk.toEvt.iRel != -1)
                relCount++;
            else
                evtCount++;
        }
    }
    return evtCount >= relCount;
}

char situationModel::pointToState(std::string word) {
    int initCount = 0;
    int goalCount = 0;
    int finalCount = 0;
    for(sDiscourseLink lk : vDiscourseLinks) {
        if (lk.word == word) {
            switch (lk.toEvt.cPart) {
            case 'I':
                initCount++;
                break;
            case 'G':
                goalCount++;
                break;
            case 'F':
                finalCount++;
                break;
            }
        }
    }
    if (goalCount >= initCount && goalCount >= finalCount)
        return 'G';
    else if (finalCount >= goalCount && finalCount >= initCount)
        return 'F';
    else
        return 'I';
}

char situationModel::pointToAct(std::string word) {
    int actCount = 0;
    int resCount = 0;
    for(sDiscourseLink lk : vDiscourseLinks) {
        if (lk.word == word) {
            switch (lk.toEvt.cPart) {
            case 'A':
                actCount++;
                break;
            case 'R':
                resCount++;
                break;
            }
        }
    }
    if (resCount > actCount)
        return 'R';
    else
        return 'A';
}
int situationModel::extractRel(string meaning) {
    meaning = meaning.substr(0, meaning.find('<'));
    meaning += " ";
    // Extract meaning (PAOR)
    string rest = "";
    // Verb
    size_t posEnd = meaning.find(' ');
    string predicate = meaning.substr(0, posEnd);
    // Subject
    string agent = meaning.substr(posEnd + 1, meaning.find(' ', posEnd + 1) - posEnd - 1);
    posEnd = meaning.find(' ', posEnd + 1);
    // Object
    string object = meaning.substr(posEnd + 1, meaning.find(' ', posEnd + 1) - posEnd - 1);
    sRelation r;
    r.verb = predicate;
    r.subject = agent;
    r.object = object;
    return addOrFindRelation(r);
}

int situationModel::extractAction(string meaning) {
    meaning = meaning.substr(0, meaning.find('<'));
    meaning += " ";
    // Extract meaning (PAOR)
    string rest = "";
    // Verb
    size_t posEnd = meaning.find(' ');
    string predicate = meaning.substr(0, posEnd);
    // Subject
    string agent = meaning.substr(posEnd + 1, meaning.find(' ', posEnd + 1) - posEnd - 1);
    posEnd = meaning.find(' ', posEnd + 1);
    // Object
    string object = meaning.substr(posEnd + 1, meaning.find(' ', posEnd + 1) - posEnd - 1);
    posEnd = meaning.find(' ', posEnd + 1);
    // Recipient
    string recipient = meaning.substr(posEnd + 1, meaning.find(' ', posEnd + 1) - posEnd - 1);
    return addNewActionEvt(predicate, agent, object, recipient);
}

sKeyMean situationModel::addMeaningAndLink(string meaning, sKeyMean previous, bool create) {
    if (!sentenceEnd && previous.iIGARF == -1 && !vDiscourseLinks.empty())
        previous = vDiscourseLinks.back().toEvt;
    sentenceEnd = false;
    // The first line of the meaning gives the narrative semantic word used to create a link
    size_t posEnd = meaning.find(',');
    string firstLine = meaning.substr(0, posEnd);
    if (meaning[posEnd + 1] == ' ')
        posEnd++; // Remove useless begin space
    string secondLine = meaning.substr(posEnd + 1, meaning.find(',', posEnd + 1));

    // Get Current Key Mean
    sKeyMean current = findEventOrRelation(secondLine);

    // Extract each narrative words
    size_t endWrd = -1;
    string w = "";
    do {
        w = firstLine.substr(endWrd + 1, firstLine.find(' ', endWrd + 1) - endWrd - 1);
        endWrd = firstLine.find(' ', endWrd + 1);
        if (create && current.iIGARF == -1) { // Neither event nor relation found
            if (create) { // Still not found
                // Find out if relation or event is best
                if (pointToEvent(w)) {
                    // Create an event
                    current.cPart = pointToAct(w);
                    if (previous.iIGARF != -1 &&
                       ((current.cPart == 'A' && vIGARF.at(previous.iIGARF).tAction == UNDEF) ||
                        (current.cPart == 'R' && vIGARF.at(previous.iIGARF).tResult == UNDEF)))
                        current.iIGARF = previous.iIGARF;
                    else
                        current.iIGARF = createIGARF();
                    int i = extractAction(secondLine);
                    current.iRel = -1;
                    modifEventIGARF(current.iIGARF, current.cPart, i);
                }
                else {
                    // Add a relation to previous
                    if (previous.iIGARF != -1)
                        current.iIGARF = previous.iIGARF;
                    else
                        current.iIGARF = createIGARF();
                    current.cPart = pointToState(w);
                    current.iRel = extractRel(secondLine);
                    addRelationIGARF(current.iIGARF, current.cPart, current.iRel);
                }
            }
        }
        createLink(previous, current, w);
    } while (w != "" && firstLine.find(' ', endWrd + 1) != string::npos);
    vMeanings.push_back(meaning);

    return current;
}

// TEST
void situationModel::TESTwhenIsUsed(string word) {
    for(sDiscourseLink lk : vDiscourseLinks) {
        if (lk.word == word) {
            yInfo() << "From" << lk.fromEvt.cPart << "of IGARF" << lk.fromEvt.iIGARF << "to" << lk.toEvt.cPart << "of IGARF" << lk.toEvt.iIGARF;
        }
    }
}


void situationModel::endSentence() {
    sentenceEnd = true;
}

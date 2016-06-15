#include <storygraph.h>
#include <regex>
#include <map>

using namespace storygraph;

storyGraph::storyGraph() {}

void storyGraph::initializeStory(const story& sto) {
    vDGAR.clear();
    vNarrativeLinks.clear();
    vEvents.clear();

    vEvents = sto.vEvents;
}

// -- Display functions --

std::string storyGraph::expressDGAR(int i, int details) {
    std::string exp = "";
    // Checks
    if (i < 0 || i >= (int)vDGAR.size())
        return exp;
    sDGAR story = vDGAR.at(i);
    if (details == 0)
        return story.label;
    // Drive and its links
    std::string d = getDescription(story.tDrive, story.iDrive, details);
    // Goal
    std::string g = getDescription(story.tGoal, story.iGoal, details);
    // Action
    std::string a = getDescription(story.tAction, story.iAction, details);
    // Result
    std::string r = getDescription(story.tResult, story.iResult, details);
    // Next
    std::string n = getDescription(story.tNext, story.iNext, details);
    if (d != "") {
        return d;
    }
    if (a != "") {
        return a;
    }
    if (g != "") {
        return g;
    }
    if (r != "") {
        return r;
    }
    if (n != "") {
        return n;
    }
    return "";
}

std::string storyGraph::evtRelations(int i) {
    std::string s = "";
    if (i < 0 || i >= (int)vEvents.size())
        return s;
    evtStory evt = vEvents.at(i);

    // Relation
    s += " [";
    for (int i = 0; i < evt.bRelations.size(); i++) {
        if (evt.bRelations.get(i).isList()) {
            s += evt.bRelations.get(i).asList()->get(1).toString();
            s += "(" + evt.bRelations.get(i).asList()->get(0).toString();
            s += "," + evt.bRelations.get(i).asList()->get(2).toString();
            s += "),";
        }
    }
    s += "]";

    return s;
}

std::string storyGraph::evtArguments(int i) {
    std::string s = "";
    if (i < 0 || i >= (int)vEvents.size())
        return s;
    evtStory evt = vEvents.at(i);

    // Arguments
    for (auto iarg = evt.vArgument.begin(); iarg != evt.vArgument.end(); iarg++){
        s += " [" + iarg->first + ": " + iarg->second + "]";
    }

    return s;
}

std::string storyGraph::evtDetails(int i) {
    std::string s = "";
    if (i < 0 || i >= (int)vEvents.size())
        return s;
    evtStory evt = vEvents.at(i);
    s += evt.predicate + "(" + evt.agent + ", " + evt.object + ", " + evt.recipient + ")";

    // Relation
    s += evtRelations(i);

    // Arguments
    s += evtArguments(i);

    return s;
}

std::string storyGraph::getDescription(cell_type ct, int i, int details) {
    if (ct == EVENT) {
        return evtToSentence(i);
    }
    else if (ct == DGAR_CELL) {
        return expressDGAR(i, details - 1);
    }
    return "";
}

void margin(int level) {
    for(int i = 0; i < level; i++)
        std::cout << "| ";
}

void storyGraph::show_tree(int start, int level, bool withNext) {
    // Checks
    if (start < 0 || start >= (int)vDGAR.size())
        return;

    sDGAR main = vDGAR.at(start);
    std::cout << "[" << start << "] " << main.label << std::endl;
    // Drive
    margin(level);
    std::cout << "+-DRIVE: ";
    if (main.tDrive == EVENT) {
        std::cout << "[" << main.iDrive << "] " << evtRelations(main.iDrive) << std::endl;
    }
    else if (main.tDrive == DGAR_CELL) {
        show_tree(main.iDrive, level + 1, withNext);
    }
    else {
        std::cout << std::endl;
    }
    // Goal
    margin(level);
    std::cout << "+-GOAL: ";
    if (main.tGoal == EVENT) {
        std::cout << "[" << main.iGoal << "] " << evtRelations(main.iGoal) << std::endl;
    }
    else if (main.tGoal == DGAR_CELL) {
        show_tree(main.iGoal, level + 1, withNext);
    }
    else {
        std::cout << std::endl;
    }
    // Action
    margin(level);
    std::cout << "+-ACTION: ";
    if (main.tAction == EVENT) {
        std::cout << "[" << main.iAction << "] " << evtDetails(main.iAction) << std::endl;
    }
    else if (main.tAction == DGAR_CELL) {
        show_tree(main.iAction, level + 1, withNext);
    }
    else {
        std::cout << std::endl;
    }
    // Result
    margin(level);
    std::cout << "+-RESULT: ";
    if (main.tResult == EVENT) {
        std::cout << "[" << main.iResult << "] " << evtRelations(main.iResult) << evtArguments(main.iResult) << std::endl;
    }
    else if (main.tResult == DGAR_CELL) {
        show_tree(main.iResult, level + 1, withNext);
    }
    else {
        std::cout << std::endl;
    }
    // Next
    margin(level);
    std::cout << "+-NEXT: ";
    if (main.tNext == EVENT) {
        std::cout << evtDetails(main.iNext) << std::endl;
    }
    else if (main.tNext == DGAR_CELL) {
        if (withNext) {
            std::cout << std::endl;
            margin(level);
            std::cout << "| ";
            show_tree(main.iNext, level, withNext);
        }
        else {
            std::cout << main.iNext << std::endl;
        }
    }
    else {
        std::cout << std::endl;
    }
}

// -- Access --

std::string storyGraph::whatIs(int i, std::string role) {
    std::string s = "";
    if (i < 0 || i >= (int)vEvents.size())
        return s;
    evtStory evt = vEvents.at(i);
    // Arguments
    for (auto iarg = evt.vArgument.begin(); iarg != evt.vArgument.end(); iarg++){
        if (iarg->first == role)
            return iarg->second;
    }
    return "";
}

// -- DGAR construction --

bool sameValue(const yarp::os::Value& v1, const yarp::os::Value& v2) {
    if (v1 == v2)
        return true;
    if (v1.isList() && v2.isList()) {
        return (v1.asList()->get(0).asString() == v2.asList()->get(0).asString()) &&
               (v1.asList()->get(1).asString() == v2.asList()->get(1).asString()) &&
               (v1.asList()->get(2).asString() == v2.asList()->get(2).asString());
    }
    return false;
}

bool storyGraph::satisfies(int a, int b) {
    if (a < 0 || a >= (int)vEvents.size())
        return false;
    evtStory evt1 = vEvents.at(a);
    if (b < 0 || b >= (int)vEvents.size())
        return false;
    evtStory evt2 = vEvents.at(b);

    bool cont = true;
    for (int i = 0; i < evt2.bRelations.size() && cont; i++) {
        bool found = false;
        for (int j = 0; j < evt1.bRelations.size() && !found; j++) {
            if (sameValue(evt2.bRelations.get(i), evt1.bRelations.get(j))) {
                found = true;
            }
        }
        cont = found;
    }
    return cont;
}

sDGAR storyGraph::addDGAR(const sDGAR &dgarToAdd) {
    sDGAR copy = dgarToAdd;
    // Drive
    if (copy.tDrive == NONE || copy.iDrive == -1) {
        copy.tDrive = NONE;
        copy.iDrive = -1;
    }
    else if (copy.tDrive == EVENT) {
        if (copy.iDrive < 0 || copy.iDrive >= (int)vEvents.size()) {
            copy.tDrive = NONE;
            copy.iDrive = -1;
        }
    }
    else {
        if (copy.iDrive < 0 || copy.iDrive >= (int)vDGAR.size()) {
            copy.tDrive = NONE;
            copy.iDrive = -1;
        }
    }
    // Goal
    if (copy.tGoal == NONE || copy.iGoal == -1) {
        copy.tGoal = NONE;
        copy.iGoal = -1;
    }
    else if (copy.tGoal == EVENT) {
        if (copy.iGoal < 0 || copy.iGoal >= (int)vEvents.size()) {
            copy.tGoal = NONE;
            copy.iGoal = -1;
        }
    }
    else {
        if (copy.iGoal < 0 || copy.iGoal >= (int)vDGAR.size()) {
            copy.tGoal = NONE;
            copy.iGoal = -1;
        }
    }
    // Action
    if (copy.tAction == NONE || copy.iAction == -1) {
        copy.tAction = NONE;
        copy.iAction = -1;
    }
    else if (copy.tAction == EVENT) {
        if (copy.iAction < 0 || copy.iAction >= (int)vEvents.size()) {
            copy.tAction = NONE;
            copy.iAction = -1;
        }
    }
    else {
        if (copy.iAction < 0 || copy.iAction >= (int)vDGAR.size()) {
            copy.tAction = NONE;
            copy.iAction = -1;
        }
    }
    // Result
    if (copy.tResult == NONE || copy.iResult == -1) {
        copy.tResult = NONE;
        copy.iResult = -1;
    }
    else if (copy.tResult == EVENT) {
        if (copy.iResult < 0 || copy.iResult >= (int)vEvents.size()) {
            copy.tResult = NONE;
            copy.iResult = -1;
        }
    }
    else {
        if (copy.iResult < 0 || copy.iResult >= (int)vDGAR.size()) {
            copy.tResult = NONE;
            copy.iResult = -1;
        }
    }
    // Next
    if (copy.tNext == NONE || copy.iNext == -1) {
        copy.tNext = NONE;
        copy.iNext = -1;
    }
    else if (copy.tNext == EVENT) {
        if (copy.iNext < 0 || copy.iNext >= (int)vEvents.size()) {
            copy.tNext = NONE;
            copy.iNext = -1;
        }
    }
    else {
        if (copy.iNext < 0 || copy.iNext >= (int)vDGAR.size()) {
            copy.tNext = NONE;
            copy.iNext = -1;
        }
    }
    // Adding it
    vDGAR.push_back(copy);
    return copy;
}

void storyGraph::createAndAddEvt(std::string predicate, std::string agent, std::string object, std::string recipient) {
    evtStory e;
    e.instance = -1;
    e.activity_name = predicate;
    e.activity_type = "narrated";

    e.predicate = predicate;
    e.agent = agent;
    e.object = object;
    e.recipient = recipient;
    e.isNarration = true;
    e.begin = true;

    vEvents.push_back(e);
}

//  --- Naïve Narration ---

std::string storyGraph::evtToSentence(int i) {
        // -- Todo
    std::string s = "";
    if (i < 0 || i >= (int)vEvents.size())
        return s;
    evtStory evt = vEvents.at(i);
    s += evt.agent + " " + evt.predicate + (evt.object !=""?" ":"") + evt.object + (evt.recipient!=""?" to ":"") + evt.recipient;
    return s;
}

std::string storyGraph::argumentToSentence(int i, int j) {
    if (i < 0 || i >= (int)vEvents.size())
        return "";
    evtStory evt = vEvents.at(i);
    if (j < 0 || j >= (int)evt.vArgument.size())
        return "";
    std::string what = evt.vArgument.at(j).first;
    std::string value = evt.vArgument.at(j).second;
    return what + " is " + value;
}

std::string storyGraph::relationToSentence(int i, int j) {
    if (i < 0 || i >= (int)vEvents.size())
        return "";
    evtStory evt = vEvents.at(i);
    if (j < 0 || j >= (int)evt.bRelations.size())
        return "";
    return evt.bRelations.get(j).toString();
}

// -- Semantic Narration

sKeyEvt newKey(int iDGAR, char cellCat, int iRel) {
    sKeyEvt n;
    n.iDGAR = iDGAR;
    n.cellCat = cellCat;
    n.iRel = iRel;
    return n;
}

void storyGraph::addLink(sKeyEvt from, sKeyEvt to, std::string word) {
    sLink newLink;
    newLink.fromEvt = from;
    newLink.toEvt = to;
    newLink.word = word;
    vNarrativeLinks.push_back(newLink);
}

void storyGraph::addMeaningAndLink(sKeyEvt from, sKeyEvt to, std::string meaning) {
    /* The first line of the meaning gives the narrative semantic word used to
     * creates a link between the fromEvt and the toEvt */
    std::string firstLine = meaning.substr(0, meaning.find(','));
    // Extract each narrative words
    size_t endWrd = firstLine.find(' ');
    while (endWrd != std::string::npos && firstLine != "") {
        std::string w = firstLine.substr(0, endWrd);
        firstLine = firstLine.substr(endWrd + 1);
        addLink(from, to, w);
        endWrd = firstLine.find(' ');
    }
    vMeanings.push_back(meaning);
}

/*std::string storyGraph::tagsToOCW(int i) {
    if (i < 0 || i >= (int)vNarraMeanings.size())
        return "";
    sNarrativeMeaning lk = vNarraMeanings.at(i);
    std::string s = lk.meaning;
    std::smatch m;
    std::regex e ("<([FT]):([DGAR])(:r[0-9]*)?(:a[a-zA-Z0-9_]*)?(:[PAOR])?>");

    std::string next = "";
    do { // Replace each tag
        std::regex_search(s, m, e);
        if (!m.empty()) {
            next = m.prefix();
            // Replacing event
            // Select next tag
            std::string tag = m[0];
            std::smatch caract;
            std::regex evtExp ("<([FT]):([DGAR])(:r[0-9]*)?(:a[a-zA-Z0-9_]*)?(:[PAOR])?>");
            std::regex_search(tag, caract, evtExp);
            // Select relation
            std::string relation = caract[3];
            std::smatch rn;
            std::regex relExp (":r([0-9]*)");
            std::regex_search(relation, rn, relExp);
            // Select arguments
            std::string argument = caract[4];
            std::smatch an;
            std::regex argExp (":a([a-zA-Z0-9_]*)");
            std::regex_search(argument, an, argExp);
            // PAOR is the 5th
            if (caract[1] != "" && caract[2] != "") { // If DGAR number and type of cell are precised
                int nDGAR = (caract[1] == "F") ? lk.fromDGAR : lk.toDGAR;
                int nEvt = -1;
                if (nDGAR >= 0 && nDGAR < (int) vDGAR.size()) {
                    if (caract[2] == "D")
                        nEvt = vDGAR.at(nDGAR).iDrive;
                    else if (caract[2] == "G")
                        nEvt = vDGAR.at(nDGAR).iGoal;
                    else if (caract[2] == "R")
                        nEvt = vDGAR.at(nDGAR).iResult;
                    else
                        nEvt = vDGAR.at(nDGAR).iAction;
                    if (nEvt != -1) {
                        // Is there focus on a relation?
                        if (rn.size() > 1 && rn[1] != "") {
                            int i = stoi(rn[1]);
                            if (caract[5] == ":P")
                                next += vEvents.at(nEvt).bRelations.get(i).asList()->get(1).toString();
                            else if (caract[5] == ":A")
                                next += vEvents.at(nEvt).bRelations.get(i).asList()->get(0).toString();
                            else if (caract[5] == ":O")
                                next += vEvents.at(nEvt).bRelations.get(i).asList()->get(2).toString();
                            else {
                                next += vEvents.at(nEvt).bRelations.get(i).asList()->get(1).toString() + " " +
                                        vEvents.at(nEvt).bRelations.get(i).asList()->get(0).toString() + " " +
                                        vEvents.at(nEvt).bRelations.get(i).asList()->get(2).toString();
                            }
                        }
                        // Is there focus on a argument?
                        else if (an.size() > 1 && an[1] != "") {
                            if (caract[5] == ":P")
                                next += whatIs(nEvt, an[1]);
                            else if (caract[5] == ":A")
                                next += an[1];
                        }
                        else {
                            // Only the event
                            if (caract[5] == ":P")
                                next += vEvents.at(nEvt).predicate;
                            else if (caract[5] == ":A")
                                next += vEvents.at(nEvt).agent;
                            else if (caract[5] == ":O")
                                next += vEvents.at(nEvt).object;
                            else if (caract[5] == ":R")
                                next += vEvents.at(nEvt).recipient;
                            else {
                                next += vEvents.at(nEvt).predicate + " " +
                                        vEvents.at(nEvt).agent + " " +
                                        vEvents.at(nEvt).object + " " +
                                        vEvents.at(nEvt).predicate;
                            }
                        }
                    }
                }
            }
            next += m.suffix();
            s = next;
        }
    } while (!m.empty());
    return s;
}*/

int storyGraph::isKnown(std::string predicate, std::string agent, std::string object, std::string recipient) {
    for(unsigned int i = 0; i < vEvents.size(); i++) {
        evtStory e = vEvents.at(i);
        if (e.predicate == predicate && e.agent == agent && e.object == object && e.recipient == recipient)
            return i;
    }
    return -1;
}

void storyGraph::addRelation(int i, std::string predicate, std::string agent, std::string object) {
    if (i < 0 || i >= (int)vEvents.size())
        return;
    evtStory& e = vEvents.at(i);
    yarp::os::Bottle aimedRelation;
    aimedRelation.addString(agent);
    aimedRelation.addString(predicate);
    aimedRelation.addString(object);
    e.bRelations.addList() = aimedRelation;
}

void storyGraph::addArgument(int i, std::string key, std::string value){
    if (i < 0 || i >= (int)vEvents.size())
        return;
    evtStory& e = vEvents.at(i);
    e.vArgument.push_back(std::pair < std::string, std::string > (key, value));
}

void storyGraph::TESTwhenIsUsed(std::string word) {
    for(sLink lk : vNarrativeLinks) {
        if (lk.word == word) {
            std::cout << "From " << lk.fromEvt.cellCat << " of DGAR " << lk.fromEvt.iDGAR << " to " << lk.toEvt.cellCat << " of DGAR " << lk.toEvt.iDGAR << std::endl;
        }
    }
}

#include <storygraph.h>
#include <regex>

using namespace storygraph;

storyGraph::storyGraph() {}

void storyGraph::initializeStory(const story& sto) {
    vDGAR.clear();
    vNarrativeLinks.clear();
    vEvents.clear();

    vEvents = sto.vEvents;
}

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

int storyGraph::findFrom(int fromEvt, int startSearch) {
    for (unsigned int i = startSearch; i < vNarrativeLinks.size(); i++) {
        if (vNarrativeLinks.at(i).fromEvt == fromEvt) {
            return i;
        }
    }
    return -1;
}

void storyGraph::extract(const evtStory &evt) {
    for (int i = 0; i < evt.bRelations.size(); i++) {
        if (evt.bRelations.get(i).isList()) {
            evtStory newEvt;
            newEvt.agent = evt.bRelations.get(i).asList()->get(0).toString();
            newEvt.predicate = evt.bRelations.get(i).asList()->get(1).toString();
            newEvt.object = evt.bRelations.get(i).asList()->get(2).toString();
            vEvents.push_back(newEvt);
        }
    }
}

void margin(int level) {
    for(int i = 0; i < level; i++)
        std::cout << "| ";
}

void storyGraph::show_arbor(int start, int level, bool withNext) {
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
        show_arbor(main.iDrive, level + 1, withNext);
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
        show_arbor(main.iGoal, level + 1, withNext);
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
        show_arbor(main.iAction, level + 1, withNext);
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
        show_arbor(main.iResult, level + 1, withNext);
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
            show_arbor(main.iNext, level, withNext);
        }
        else {
            std::cout << main.iNext << std::endl;
        }
    }
    else {
        std::cout << std::endl;
    }
}

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

//  --- Narrate ---

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

std::string storyGraph::linkToSentence(int i) {
    if (i < 0 || i >= (int)vNarrativeLinks.size())
        return "";
    std::string s = vNarrativeLinks.at(i).label;
    std::smatch m;
    std::regex e ("<e:[0-9]*(;a:[a-zA-Z0-9_]*)?(;r:[0-9]*)?>");

    std::string next = "";
    do { // Replace each balise
        std::regex_search(s, m, e);
        if (!m.empty()) {
            next = m.prefix();
            // Replacing event
            std::string balise = m[0];
            std::smatch m2;
            std::regex evtExp ("e:([0-9]*)");
            std::regex_search(balise, m2, evtExp);
            if (m2.size() > 1 && m2[1] != "") {
                int nEvt = stoi(m2[1]);
                // Is there focus on an argument?
                std::smatch m3;
                std::regex argExp (";a:([a-zA-Z0-9_]*)");
                std::regex_search(balise, m3, argExp);
                if (m3.size() > 1 && m3[1] != "") {
                    next += whatIs(nEvt, m3[1]);
                }
                else {
                    // Is there focus on a relation?
                    std::smatch m3;
                    std::regex relExp (";r:([0-9]*)");
                    std::regex_search(balise, m3, relExp);
                    if (m3.size() > 1 && m3[1] != "") {
                        int nRel = stoi(m3[1]);
                        next += relationToSentence(nEvt, nRel);
                    }
                    else {
                        next += evtToSentence(nEvt);
                    }
                }
            }
            next += m.suffix();
            s = next;
        }
    } while (!m.empty());
    return s;
}

void storyGraph::tellStory() {
    for (unsigned int i = 0; i < vNarrativeLinks.size(); i++) {
        std::cout << linkToSentence(i) << std::endl;
    }
}


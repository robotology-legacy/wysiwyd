#include "semanticscanner.h"
#include <set>
#include <map>

using namespace yarp::os;
using namespace storygraph;
using namespace std;

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        if (item != "")
            elems.push_back(item);
    }
    return elems;
}

bool storygraph::operator==(const sActionEvt& A, const sActionEvt& B)  {
    return (A.predicate == B.predicate
         && A.agent     == B.agent
         && A.object    == B.object
         && A.recipient == B.recipient);
}

bool storygraph::operator==(const sRelation& A, const sRelation& B)  {
    return (A.subject == B.subject
         && A.verb    == B.verb
         && A.object  == B.object);
}

sRelation storygraph::fromValueToRelation(const yarp::os::Value& b) {
    sRelation n;
    if (b.isList()) {
        n.subject   = b.asList()->get(0).toString();
        n.verb      = b.asList()->get(1).toString();
        n.object    = b.asList()->get(2).toString();
    }
    return n;
}

bool VocabularyHandler::shareMeaning(string word, std::set<string> ocw) {
    if (ocw.find(word) != ocw.end())
        return true;
    for(set<string> wordClass : VocabularyHandler::vSynonyms) {
        if (wordClass.find(word) != wordClass.end()) { // If the current classWord is of our word
            for(string s : wordClass) {
                if (ocw.find(s) != ocw.end())
                    return true;
            }
        }
    }
    return false;
}

bool VocabularyHandler::sameMeaning(string word1, string word2) {
    if (word1 == word2)
        return true;
    for(set<string> wordClass : VocabularyHandler::vSynonyms) {
        if (wordClass.find(word1) != wordClass.end() && wordClass.find(word2) != wordClass.end()) {
            return true;
        }
    }
    return false;
}

void VocabularyHandler::enrichSynonyms(std::set<std::string> setWords) {
    VocabularyHandler::vSynonyms.push_back(setWords);
}

void VocabularyHandler::enrichDFW(std::string dfw) {
    VocabularyHandler::vDFW.insert(dfw);
}

void VocabularyHandler::setPronouns(string agent, string object, string recipient) {
    VocabularyHandler::agentPronoun     = agent;
    VocabularyHandler::objectPronoun    = object;
    VocabularyHandler::recipientPronoun = recipient;
}

void VocabularyHandler::initVoc(std::vector<story> listStories) {
    VocabularyHandler::actionPredicates.clear();
    VocabularyHandler::relationPredicates.clear();
    for(story &st : listStories) {
        for (evtStory &e : st.vEvents) {
            VocabularyHandler::actionPredicates.insert(e.predicate);
            VocabularyHandler::actionPredicates.insert(e.activity_name);
            VocabularyHandler::actionPredicates.insert(e.activity_type);
            for (int i = 0; i < e.bRelations.size(); i++) {
                if (e.bRelations.get(i).isList())
                    VocabularyHandler::relationPredicates.insert(storygraph::fromValueToRelation(e.bRelations.get(i)).verb);
            }
        }
    }
}

bool VocabularyHandler::isActionVoc(string predicate) {
    return (storygraph::VocabularyHandler::shareMeaning(predicate, VocabularyHandler::actionPredicates));
}

Meaning::Meaning() {}

Meaning::Meaning(string _sentence, sActionEvt previousContext) {
    sentence = _sentence;
    currentContext = previousContext;
}

void Meaning::extractOCW(const set<string> &setCCW) {
    vector < string > words = split(sentence, ' ');
    for (string w : words) {
        if (setCCW.find(w) == setCCW.end()) {
            vOCW.push_back(w);
        }
    }
}

set<string> Meaning::ocwSet() {
    set < string > ocw(vOCW.begin(), vOCW.end());
    for (string word : VocabularyHandler::vDFW) {
        ocw.erase(word);
    }
    return ocw;
}

void Meaning::extractFocus(const string &predicate, const string &agent, const string &object, const string &recipient) {
    meaningLine newMean;
    for (string &w : vOCW) {
        if (w != "" && VocabularyHandler::sameMeaning(w, predicate)) {
            newMean.focus.push_back('P');
            if (newMean.ocw.predicate == "")
                newMean.ocw.predicate = w;
        }
        else if (w != "" && (VocabularyHandler::sameMeaning(w, agent) ||
                             (w == VocabularyHandler::agentPronoun && VocabularyHandler::sameMeaning(currentContext.agent, agent)))) {
            newMean.focus.push_back('A');
            if (newMean.ocw.agent == "")
                newMean.ocw.agent = w;
        }
        else if (w != "" && (VocabularyHandler::sameMeaning(w, object) ||
                             (w == VocabularyHandler::objectPronoun && VocabularyHandler::sameMeaning(currentContext.object, object)))) {
            newMean.focus.push_back('O');
            if (newMean.ocw.object == "")
                newMean.ocw.object = w;
        }
        else if (w != "" && (VocabularyHandler::sameMeaning(w, recipient) ||
                             (w == VocabularyHandler::recipientPronoun && VocabularyHandler::sameMeaning(currentContext.recipient, recipient)))) {
            newMean.focus.push_back('R');
            if (newMean.ocw.recipient == "")
                newMean.ocw.recipient = w;
        }
        else {
            newMean.focus.push_back('_');
        }
    }
    currentContext.predicate   = newMean.ocw.predicate;
    if (newMean.ocw.agent != VocabularyHandler::agentPronoun && newMean.ocw.agent != "")
        currentContext.agent   = newMean.ocw.agent;
    if (newMean.ocw.object != VocabularyHandler::objectPronoun && newMean.ocw.object != "")
        currentContext.object  = newMean.ocw.object;
    if (newMean.ocw.recipient != VocabularyHandler::recipientPronoun && newMean.ocw.recipient != "")
        currentContext.recipient   = newMean.ocw.recipient;
    vMeanings.push_back(newMean);
}

void Meaning::DFWLine() {
    for (meaningLine lm : vMeanings) {
        for (int i = 0; i < (int)lm.focus.size(); i++) {
            if (lm.focus.at(i) != '_') {
                vOCW.at(i) = "";
            }
        }
    }
    // First pass extract DFW, second extract the others
    bool dfwTaken = false, otherTaken = false;
    do {
        meaningLine newMean;
        int i = 0;
        string letters = "PAOR";
        for (string& w : vOCW) {
            if (w != "" &&
                ((!dfwTaken && VocabularyHandler::vDFW.find(w) != VocabularyHandler::vDFW.end()) ||
                 dfwTaken)) {
                newMean.focus.push_back(letters[i]);
                if (letters[i] == 'P') {
                    newMean.ocw.predicate = w;
                    i++;
                }
                else if (letters[i] == 'A') {
                    newMean.ocw.agent = w;
                    i++;
                }
                else if (letters[i] == 'O') {
                    newMean.ocw.object = w;
                    i++;
                }
                else if (letters[i] == 'R') {
                    newMean.ocw.recipient = w;
                    i = 0;
                    vMeanings.insert(vMeanings.begin(), newMean);
                    newMean.ocw.predicate = "";
                    newMean.ocw.agent = "";
                    newMean.ocw.object = "";
                    newMean.ocw.recipient = "";
                }
                w = "";
            }
            else {
                newMean.focus.push_back('_');
            }
        }
        if (!dfwTaken)
            vMeanings.insert(vMeanings.begin(), newMean);
        else
            vMeanings.push_back(newMean);
        if (dfwTaken)
            otherTaken = true;
        else
            dfwTaken = true;
    } while (!(dfwTaken && otherTaken));
}

string Meaning::getMeaning(int minFocus) {
    string sOCW, sFocus;
    for (const meaningLine &mean : vMeanings) {
        sOCW += (mean.ocw.predicate != "") ? (mean.ocw.predicate + " ") : "";
        sOCW += (mean.ocw.agent != "") ? (mean.ocw.agent + " ") : "";
        sOCW += (mean.ocw.object != "") ? (mean.ocw.object + " ") : "";
        sOCW += (mean.ocw.recipient != "") ? (mean.ocw.recipient + " ") : "";
        sOCW += ", ";
        sFocus += "[";
        int nbFoc = 0;
        for (char f : mean.focus) {
            sFocus += f;
            sFocus += "-";
            nbFoc++;
        }
        for (int k = nbFoc; k < minFocus; k++)
            sFocus += "_-";
        sFocus = sFocus.substr(0, sFocus.length() - 1) + "]";
    }
    return sOCW + "<o> " + sFocus + " <o>; " + sentence;
}

sActionEvt Meaning::getContext() {
    return currentContext;
}

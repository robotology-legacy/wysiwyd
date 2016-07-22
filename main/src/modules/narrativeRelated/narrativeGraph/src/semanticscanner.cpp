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
    return (VocabularyHandler::sameMeaning(A.predicate, B.predicate)
         && VocabularyHandler::sameMeaning(A.agent    , B.agent)
         && VocabularyHandler::sameMeaning(A.object   , B.object)
         && VocabularyHandler::sameMeaning(A.recipient, B.recipient));
}

bool storygraph::operator==(const sRelation& A, const sRelation& B)  {
    return (VocabularyHandler::sameMeaning(A.subject, B.subject)
         && VocabularyHandler::sameMeaning(A.verb   , B.verb)
         && VocabularyHandler::sameMeaning(A.object , B.object));
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
sActionEvt storygraph::relToAct(const sRelation& rel) {
    sActionEvt a;
    a.agent = rel.subject;
    a.predicate = rel.verb;
    a.object = rel.object;
    a.recipient = "";
    return a;
}

void VocabularyHandler::addCCW(string word) {
    VocabularyHandler::setCCW.insert(word);
}

vector <string> VocabularyHandler::extractOCW(const vector <string>& words) {
    vector <string> vOCW;
    for (string w : words) {
        if (VocabularyHandler::setCCW.find(w) == VocabularyHandler::setCCW.end()) {
            vOCW.push_back(w);
        }
    }
    return vOCW;
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
    if (VocabularyHandler::vDFW.find(dfw) == VocabularyHandler::vDFW.end()) {
        sDFWUse use;
        VocabularyHandler::vDFW[dfw] = use;
    }
}

bool VocabularyHandler::isDFW(std::string word) {
    return (VocabularyHandler::vDFW.find(word) != VocabularyHandler::vDFW.end());
}

void VocabularyHandler::setPronouns(string agent, string object, string recipient) {
    VocabularyHandler::agentPronoun     = agent;
    VocabularyHandler::objectPronoun    = object;
    VocabularyHandler::recipientPronoun = recipient;
}
void VocabularyHandler::replacePronouns(const sActionEvt& context, sActionEvt& toReplace) {
    if (toReplace.agent == VocabularyHandler::agentPronoun) {
        toReplace.agent = context.agent;
    }
    if (toReplace.agent == VocabularyHandler::objectPronoun) {
        toReplace.agent = context.object;
    }
    if (toReplace.object == VocabularyHandler::objectPronoun) {
        toReplace.object = context.object;
    }
    if (toReplace.recipient == VocabularyHandler::objectPronoun) {
        toReplace.recipient = context.object;
    }
    if (toReplace.recipient == VocabularyHandler::recipientPronoun) {
        toReplace.recipient = context.recipient;
    }
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
    actionPredicates.insert("fail");
}

bool VocabularyHandler::isActionVoc(string predicate) {
    if (storygraph::VocabularyHandler::shareMeaning(predicate, VocabularyHandler::actionPredicates))
        return true;
    for(set<string> wordClass : VocabularyHandler::vSynonyms) {
        if (wordClass.find(predicate) != wordClass.end()) {
            for (string w : wordClass) {
                if (storygraph::VocabularyHandler::shareMeaning(w, VocabularyHandler::actionPredicates))
                    return true;
            }
        }
    }
    return false;
}

Meaning::Meaning() {
    lineWithDFW = false;
}

void Meaning::setContext(sActionEvt context) {
    currentContext = context;
}

// Sentence -> Meaning + PAOR
Meaning::Meaning(string _sentence) {
    sentence = _sentence;
    lineWithDFW = false;
}

set<string> Meaning::ocwSet() {
    vector < string > words = split(sentence, ' ');
    vOCW = VocabularyHandler::extractOCW(words);
    set < string > ocw(vOCW.begin(), vOCW.end());
    for (auto it = VocabularyHandler::vDFW.begin(); it != VocabularyHandler::vDFW.end(); it++) {
        ocw.erase(it->first);
    }
    if (ocw.find(VocabularyHandler::agentPronoun) != ocw.end()) {
        ocw.insert(currentContext.agent);
    }
    if (ocw.find(VocabularyHandler::objectPronoun) != ocw.end()) {
        ocw.insert(currentContext.object);
    }
    if (ocw.find(VocabularyHandler::recipientPronoun) != ocw.end()) {
        ocw.insert(currentContext.recipient);
    }
    return ocw;
}

void Meaning::extractFocus(const sActionEvt &a) {
    meaningLine newMean;
    bool empty = true;
    for (string &w : vOCW) {
        if (w != "" && VocabularyHandler::sameMeaning(w, a.predicate)) {
            empty = false;
            newMean.focus.push_back('P');
            if (newMean.ocw.predicate == "") {
                newMean.ocw.predicate = w;
            }
        }
        else if (w != "" && (VocabularyHandler::sameMeaning(w, a.agent) ||
                             (w == VocabularyHandler::agentPronoun && VocabularyHandler::sameMeaning(currentContext.agent, a.agent)))) {
            newMean.focus.push_back('A');
            if (newMean.ocw.agent == "") {
                newMean.ocw.agent = w;
            }
        }
        else if (w != "" && (VocabularyHandler::sameMeaning(w, a.object) ||
                             (w == VocabularyHandler::objectPronoun && VocabularyHandler::sameMeaning(currentContext.object, a.object)))) {
            newMean.focus.push_back('O');
            if (newMean.ocw.object == "") {
                newMean.ocw.object = w;
            }
        }
        else if (w != "" && (VocabularyHandler::sameMeaning(w, a.recipient) ||
                             (w == VocabularyHandler::recipientPronoun && VocabularyHandler::sameMeaning(currentContext.recipient, a.recipient)))) {
            newMean.focus.push_back('R');
            if (newMean.ocw.recipient == "") {
                newMean.ocw.recipient = w;
            }
        }
        else {
            newMean.focus.push_back('_');
        }
    }
    currentContext.predicate = newMean.ocw.predicate;
    if (newMean.ocw.agent   != VocabularyHandler::agentPronoun && newMean.ocw.agent != "")
        currentContext.agent = newMean.ocw.agent;
    if (newMean.ocw.object   != VocabularyHandler::objectPronoun && newMean.ocw.object != "")
        currentContext.object = newMean.ocw.object;
    if (newMean.ocw.recipient   != VocabularyHandler::recipientPronoun && newMean.ocw.recipient != "")
        currentContext.recipient = newMean.ocw.recipient;
    if (!empty)
        vMeanings.push_back(newMean);
}

void Meaning::extractOthers() {
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
        bool empty = true;
        int i = 0;
        string letters = "PAOR";
        for (string& w : vOCW) {
            if (w != "" &&
                ((!dfwTaken && VocabularyHandler::vDFW.find(w) != VocabularyHandler::vDFW.end()) ||
                 dfwTaken)) {
                newMean.focus.push_back(letters[i]);
                if (letters[i] == 'P') {
                    empty = false;
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
                    empty = true;
                }
                w = "";
            }
            else {
                newMean.focus.push_back('_');
            }
        }
        if (!dfwTaken && !empty)
            vMeanings.insert(vMeanings.begin(), newMean);
        else if (!empty)
            vMeanings.push_back(newMean);
        if (dfwTaken)
            otherTaken = true;
        else
            dfwTaken = true;
    } while (!(dfwTaken && otherTaken));
}

// sActionEvent -> Meaning + PAOR

void Meaning::addDFW(string word) {
    if (!lineWithDFW) {
        meaningLine newLine;
        newLine.ocw.predicate = word;
        vMeanings.insert(vMeanings.begin(), newLine);
        lineWithDFW = true;
    }
    else {
        if (vMeanings.at(0).ocw.agent == "")
            vMeanings.at(0).ocw.agent = word;
        else if (vMeanings.at(0).ocw.object == "")
            vMeanings.at(0).ocw.object = word;
        else if (vMeanings.at(0).ocw.recipient == "")
            vMeanings.at(0).ocw.recipient = word;
        else
            cerr << "Meaning::addDFW: Can't add a new DFW (line is full)" << endl;
    }
}

void Meaning::addEvent(sActionEvt a) {
    meaningLine newLine;
    newLine.ocw = a;
    vMeanings.push_back(newLine);
}

void Meaning::evtToMeaning(string lang) {
    // First the DFW line:
    int beginAt = 0;
    if (lineWithDFW) {
        beginAt += 1;
        vMeanings.at(0).focus.push_back('P');
        if (vMeanings.at(0).ocw.agent != "") {
            vMeanings.at(0).focus.push_back('A');
            beginAt += 1;
        }
        if (vMeanings.at(0).ocw.object != "") {
            vMeanings.at(0).focus.push_back('O');
            beginAt += 1;
        }
        if (vMeanings.at(0).ocw.recipient != "") {
            vMeanings.at(0).focus.push_back('R');
            beginAt += 1;
        }
    }
    // Then the others
    string basicRelationFocus = "APOR";
    if (lang == "jap") {
        basicRelationFocus = "AROP";
    }
    for (int mline = (lineWithDFW?1:0) ; mline < (int)vMeanings.size() ; mline++) {
        // Leave spaces
        for (int i = 0; i < beginAt; i++)
            vMeanings.at(mline).focus.push_back('_');
        for (char letter : basicRelationFocus) {
            if (letter == 'P' && vMeanings.at(mline).ocw.predicate != "") {
                vMeanings.at(mline).focus.push_back(letter);
                beginAt += 1;
            }
            else if (letter == 'A' && vMeanings.at(mline).ocw.agent != "") {
                vMeanings.at(mline).focus.push_back(letter);
                beginAt += 1;
            }
            else if (letter == 'O' && vMeanings.at(mline).ocw.object != "") {
                vMeanings.at(mline).focus.push_back(letter);
                beginAt += 1;
            }
            else if (letter == 'R' && vMeanings.at(mline).ocw.recipient != "") {
                vMeanings.at(mline).focus.push_back(letter);
                beginAt += 1;
            }
        }
    }
}

//

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
    sOCW = sOCW.substr(0, sOCW.length() - 2); // Remove last ", "
    return sOCW + "<o> " + sFocus + " <o>; " + sentence;
}

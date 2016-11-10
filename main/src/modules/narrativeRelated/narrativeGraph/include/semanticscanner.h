/*
 * Copyright (C) 2016 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Solène Mirliaz
 * email:   solene.mirliaz@ens-rennes.fr
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

#ifndef _SEMSCAN_H_
#define _SEMSCAN_H_

#include <story.h>
#include <set>

std::vector<std::string> split(const std::string &s, char delim);

namespace storygraph {
    struct PAOR{
        std::string P;
        std::string A;
        std::string O;
        std::string R;

        int nbElm(){
            int ii = 0;
            if (P != " " && P != "") ii++;
            if (A != " " && A != "") ii++;
            if (O != " " && O != "") ii++;
            if (R != " " && R != "") ii++;
            return ii;
        }

        bool operator==(const PAOR& P2){
            return (P == P2.P && A == P2.A && O == P2.O && R == P2.R);
        }

        PAOR(){
            P = "";
            A = "";
            O = "";
            R = "";
        }
        PAOR(std::string p, std::string a, std::string o = "", std::string r = ""){
            P = p;
            A = a;
            O = o;
            R = r;
        }
        std::string toString(){
            return (P + " " + A + " " + O + " " + R + " ");
        }
    };
    /// \struct sActionEvt
    /// \brief Describes one atomic event with PAOR elements
    struct sActionEvt {
        std::string predicate;
        std::string agent;
        std::string object;
        std::string recipient;
    };
    bool operator==(const sActionEvt& A, const sActionEvt& B);

    /// \struct sRelation
    /// \brief Describes one atomic relation with subject, verb and object
    struct sRelation {
        std::string subject;
        std::string verb;
        std::string object;
    };
    bool operator==(const sRelation& A, const sRelation& B);
    sRelation fromValueToRelation(const yarp::os::Value& b);
    sActionEvt relToAct(const sRelation& rel);

    /// \namespace VocabularyHandler
    /// Encapsulates the functions which process synonyms, dfw and pronouns
    namespace VocabularyHandler {
        // -- CCW
        static std::set <std::string> setCCW; ///< Set of Closed Class Words
        void addCCW(std::string word);
        std::vector <std::string> extractOCW(const std::vector <std::string>& words);

        // -- Synonyms
        static std::vector < std::set <std::string> > vSynonyms;         ///< A container for synonym classes
        bool shareMeaning(std::string word, std::vector <std::string> ocw); ///< Checks if the word or one of its synonyms is in the ocw set provides
        bool shareMeaning(std::string word, std::set <std::string> ocw); ///< Checks if the word or one of its synonyms is in the ocw set provides
        bool shareMeaning(std::string word, std::string ocw); ///< Checks if the word or one of its synonyms is the ocw
        bool sameMeaning(std::string word1, std::string word2);          ///< Checks if two words are in the same synonym class
        void enrichSynonyms(std::set<std::string> setWords);             ///< Adds a synonym class

        // -- Pronouns
        static std::string agentPronoun, objectPronoun, recipientPronoun;
        void setPronouns(std::string agent, std::string object, std::string recipient);
        void replacePronouns(const sActionEvt& context, sActionEvt& toReplace);

        // -- DFW
        struct sDFWUse {
        };

        static std::map <std::string, sDFWUse> vDFW; ///< A container for DFW
        void enrichDFW(std::string dfw); ///< Adds a DFW to the known ones
        bool isDFW(std::string word);

        // -- Vocabulary
        static std::set <std::string> relationPredicates;
        static std::set <std::string> actionPredicates;
        void initVoc(std::vector<story> listStories); ///< Scans the data extracted from the ABM to sort predicates as Relation or Action ones
        bool isActionVoc(std::string predicate);      ///< @returns true if this predicate (or one of its synonyms) is a predicate for an action in the ABM
    }

    /// \struct meaningLine
    /// Describes one line of meaning (open-class words and focus)
    struct meaningLine {
        sActionEvt ocw;
        std::vector <char> focus;
    };

    /// \class Meaning
    /// Contains a sentence and constructs its meaning(s) (+ focus) if ocw with role are given.
    /// Or contains an sActionEvt and construct a focus.
    class Meaning {
    private:
        std::string sentence;
        std::vector <std::string> vOCW;
        std::vector <meaningLine> vMeanings;
        sActionEvt currentContext;
        bool lineWithDFW;
    public:
        Meaning();
        Meaning(std::string _sentence);
        // Context
        void setContext(sActionEvt context); ///< Set the context of this Meaning
        // Sentence -> Meaning + PAOR
        std::set < std::string > ocwSet(); ///< Removes all ccw of the sentence, creating the vector vOCW
                                           ///< Returns the ocw in a set to search best event in the Situation Model. DFW are removed.
        void extractFocus(const sActionEvt &a); ///< Adds a meaning line, replace ocw of action by '_' in vOCW
        void extractOthers(); ///< Adds lines for the other ocw (dfw are added in the first line)
        // sActionEvent -> Meaning + PAOR
        void addDFW(std::string word);
        void addEvent(sActionEvt a);
        void evtToMeaning(std::string lang="en"); ///< Complete each meaningLine with naïve focus (according to the lang).
        //
        std::string getMeaning(int minFocus = 8); ///< Returns the whole train line: "meaning1, meaning2, ... <o> focus1, focus2, ... <o>; sentence"
    };
}

#endif

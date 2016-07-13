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

    /// \namespace VocabularyHandler
    /// Encapsulates the functions which process synonyms, dfw and pronouns
    namespace VocabularyHandler {
        // -- Synonyms
        static std::vector < std::set <std::string> > vSynonyms; /// A container for synonym classes
        bool shareMeaning(std::string word, std::set <std::string> ocw); /// Checks if the word or one of its synonyms is in the ocw set provides
        bool sameMeaning(std::string word1, std::string word2); /// Checks if two words are in the same synonym class
        void enrichSynonyms(std::set<std::string> setWords); /// Adds a synonym class

        // -- Pronouns
        static std::string agentPronoun, objectPronoun, recipientPronoun;
        void setPronouns(std::string agent, std::string object, std::string recipient);

        // -- DFW
        static std::set <std::string> vDFW; /// A container for DFW @todo Improve to add the usage of the dfw
        void enrichDFW(std::string dfw); /// Adds a DFW to the known ones

        // -- Vocabulary
        static std::set <std::string> relationPredicates;
        static std::set <std::string> actionPredicates;
        void initVoc(std::vector<story> listStories);
        bool isActionVoc(std::string predicate); /// Returns true if this predicate (or one of its synonyms) is a predicate for an action in the ABM
    }

    /// \struct meaningLine
    /// Describes one line of meaning (open-class words and focus)
    struct meaningLine {
        sActionEvt ocw;
        std::vector <char> focus;
    };

    /// \class Meaning
    /// Contains a sentence and constructs its meaning (+ focus) if ocw with role are given.
    class Meaning {
    private:
        std::string sentence;
        std::vector <meaningLine> vMeanings;
        std::vector <std::string> vOCW;
        sActionEvt currentContext;
    public:
        Meaning();
        Meaning(std::string _sentence, sActionEvt previousContext);
        void extractOCW(const std::set <std::string> &setCCW); ///< Remove all ccw of the sentence, creating the vector of ocw
        std::set < std::string > ocwSet(); ///< Returns the ocw in a set to search best event in the Situation Model: DFW are thus removed
        void extractFocus(const std::string &predicate, const std::string &agent,
                          const std::string &object, const std::string &recipient); ///< Adds a meaning line, replace ocw of action by '_' in vOCW
        void DFWLine(); ///< Adds lines in the beginning for dfw
        std::string getMeaning(int minFocus = 8); ///< Returns the whole train line: "meaning1, meaning2, ... <o> focus1, focus2, ... <o>; sentence"
        sActionEvt getContext(); ///< Returns the context (a sActionEvt) for next sentence
    };
}

#endif

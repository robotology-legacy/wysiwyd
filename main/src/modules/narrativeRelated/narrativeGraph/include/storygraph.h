/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
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

#include <story.h>

namespace storygraph {
    struct sKeyEvt { // For narrative link: which part of which DGAR is looked at?
        int iDGAR;
        char cellCat; // Drive, Goal, Action or Result ?
        int iRel; // if looking at a relation, -1 else
    };

    struct sLink {
        sKeyEvt fromEvt;
        sKeyEvt toEvt;
        std::string word;
    };

    enum cell_type {
        NONE, EVENT, DGAR_CELL
    };

    struct sDGAR {
        std::string label; // <- Not used in narration
        cell_type tDrive;
        cell_type tGoal;
        cell_type tAction;
        cell_type tResult;
        cell_type tNext;
        int iDrive;
        int iGoal;
        int iAction;
        int iResult;
        int iNext;
    };

    class storyGraph {
    public:
        std::vector < evtStory > vEvents;
        std::vector < sDGAR >    vDGAR;
        std::vector < sLink >    vNarrativeLinks;

        std::vector < std::string > vMeanings; // NarrativeSemanticWords, PAOR1, PAOR2, PAOR3 <o> [_-_-_-L-_], [A-P-O-_-_], [A-_-_-_-P], [_-_-_-_-_] <o>

        storyGraph();

        void initializeStory(const story& base); // Imports events

        // Display functions
        std::string expressDGAR(int i, int details = -1); // details decrease when going in a sub DGAR, stops at 0 to give a simple label (-1 -> all sub DGARs)
        std::string evtRelations(int i); // A string to display relations.
        std::string evtArguments(int i);
        std::string evtDetails(int i); // A string to display PAOR, relations and arguments.
        std::string getDescription(cell_type ct, int i, int details = -1); // Given a cell type and an index, returns details about the event.
        void show_tree(int start, int level = 0, bool withNext = false); // Display a tree view of a DGAR

        // Access
        std::string whatIs(int i, std::string role); // Given an event number, find the argument with label 'role'

        // DGAR construction
        bool satisfies(int a, int b); // Does the a-th event satifies the relations of the b-th event?
        sDGAR addDGAR(const sDGAR& dgarToAdd); // Adds a DGAR, checking for validity of parameters. Returns the DGAR added, possibly different from the entry.
        void createAndAddEvt(std::string predicate, std::string agent, std::string object = "", std::string recipient = "");
        void addRelation(int i, std::string predicate, std::string agent, std::string object); // Adds a relation to the i-th event

        // Naïve Narration (! Used for debug only !)
        std::string evtToSentence(int i); // Naïve sentence corresponding to the event
        std::string argumentToSentence(int i, int j); // Naïve sentence corresponding to the j-th argument of the i-th event
        std::string relationToSentence(int i, int j);
        //std::string linkToSentence(int i); // Replace the <X> in the i-th link by the corresponding sentences of the events.
        //void tellStory(); // Using the narrative links vector, tell the story.

        // Semantic Narration
        // From a meaning and link, enrich DGAR
        sKeyEvt newKey(int iDGAR, char cellCat, int iRel);
        void addLink(sKeyEvt from, sKeyEvt to, std::string word);
        void addMeaningAndLink(sKeyEvt from, sKeyEvt to, std::string meaning);
        //std::string tagsToOCW(int i);

        int isKnown(std::string predicate, std::string agent, std::string object = "", std::string recipient = ""); // Return Evt number if exists, -1 else
        void TESTwhenIsUsed(std::string word); // Temporary tool function. Displays all links made with the word
    };
}

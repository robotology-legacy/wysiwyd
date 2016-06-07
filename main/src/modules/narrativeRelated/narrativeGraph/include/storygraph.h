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
    struct link {
        int fromEvt;
        int toEvt;
        std::string label; // contains spaces
    };

    enum cell_type {
        NONE, EVENT, DGAR_CELL
    };

    struct sDGAR {
        std::string label; // Todo : Associate with an evtStory so that it can be narrated like the other events
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
        std::vector< evtStory > vEvents;
        std::vector< link >     vNarrativeLinks;
        std::vector< sDGAR >    vDGAR;

        storyGraph();

        void initializeStory(const story& base); // Imports events
        std::string expressDGAR(int i, int details = -1); // details decrease when going in a sub DGAR, stops at 0 to give a simple label (-1 -> all sub DGARs)
        std::string evtRelations(int i); // A string to display relations.
        std::string evtArguments(int i); // A string to display relations.
        std::string evtDetails(int i); // A string to display PAOR, relations and arguments.
        std::string getDescription(cell_type ct, int i, int details = -1); // Given a cell type and an index, returns details about the event.

        int findFrom(int fromEvt, int startSearch = 0); // Find the first link from the fromEvt, returns its index
        void extract(const evtStory& evt); // From an event, creates events from the relations, and adds them to the vEvents

        void show_arbor(int start, int level = 0, bool withNext = false);

        std::string whatIs(int i, std::string role); // Given an event number, find the argument with label 'role'
        bool satisfies(int a, int b); // Does the a-th event satifies the relations of the b-th event

        sDGAR addDGAR(const sDGAR& dgarToAdd); // Adds a DGAR, checking for validity of parameters. Returns the DGAR added, possibly different from the entry.

        // Narrate
        std::string evtToSentence(int i); // Naïve sentence corresponding to the event
        std::string argumentToSentence(int i, int j); // Naïve sentence corresponding to the j-th argument of the i-th event
        std::string relationToSentence(int i, int j); // Naïve sentence corresponding to the j-th relation of the i-th event
        std::string linkToSentence(int i); // Replace the <X> in the i-th link by the corresponding sentences of the events.
        void tellStory(); // Using the narrative links vector, tell the story.
    };
}

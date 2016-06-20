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

    struct sActionEvt {
        std::string predicate;
        std::string agent;
        std::string object;
        std::string recipient;
    };

    struct sRelation {
        std::string subject;
        std::string verb;
        std::string object;
    };

    enum evtType {
        UNDEF, ACTION_EVT, IGARF_EVT
    };

    struct sIGARF {
        std::vector < int > vInitState;
        std::vector < int > vGoal;
        evtType tAction;
        evtType tResult;
        int iAction;
        int iResult;
        std::vector < int > vFinalState;
        int iNext; // -1 if none
    };

    struct sKeyMean {
        int iIGARF;
        char cPart; // I (InitState) G (Goal) A (Action) R (Result) F (FinalState)
        int iRel; // Relation index
    };

    struct sDiscourseLink {
        sKeyMean fromEvt;
        sKeyMean toEvt;
        std::string word;
    };

    class situationModel {
    public:
        std::vector < sRelation >      vRelations;
        std::vector < sActionEvt >     vActionEvts;
        std::vector < sIGARF >         vIGARF;
        std::vector < sDiscourseLink > vDiscourseLinks;

        std::vector < std::string > vMeanings; // NarrativeSemanticWords, PAOR1, PAOR2, PAOR3 <o> [_-_-_-L-_], [A-P-O-_-_], [A-_-_-_-P], [_-_-_-_-_] <o>

        situationModel();

        void initializeStory(const story& base); // Imports events

        // -- IGARF approach
        void clear(); // Remove all IGARF and ActionEvt

        bool sameRelation(const sRelation &r1, const sRelation &r2);

        // Display
        std::string getSentenceEvt(int i); // Produces a naïve sentence from the i-th event
        std::string getSentenceRel(int i); // Produces a naïve sentence from the i-th relations
        std::string dispRelations(const std::vector < int >& rels);
        void showIGARF(int i, int level = 0); // Displays a tree view of the i-th IGARF

        // Creation - Modification
        // Create (or find) Relation, ActionEvt or IGARF event and stock them in the class vectors. Return their index.
        int addNewActionEvt(std::string predicate, std::string agent, std::string object = "", std::string recipient = "");
        int addOrFindRelation(sRelation rel);
        int createIGARF();
        sRelation fromValueToRelation(const yarp::os::Value& b);
        // Modify
        void modifEventIGARF(int iIGARF, char cPart, int iActEvt); // Modify Action (cPart = 'A') or Result (cPart = 'R')
        void modifContentIGARF(int iIGARF, char cPart, int jIGARF); // Action or Result of iIGARF-th IGARF became jIGARF-th IGARF
        void remContentIGARF(int iIGARF, char cPart);
        void addRelationIGARF(int iIGARF, char cPart, int iRel); // Add the iRel-th relation to the iIGARF-th IGARF event
        void removeRelationIGARF(int iIGARF, char cPart, int iRel); // iRel index is the index in the class vector (not IGARF vector)
        void createFromStory(const story &s);
        // Meaning
        sKeyMean createKey(int iIGARF, char cPart, int iRel);
        void createLink(sKeyMean from, sKeyMean to, std::string word);
        sKeyMean findEventOrRelation(std::string meaning); // Return a (-1 'A' -1) sKeyMean if not found
        sKeyMean addMeaningAndLink(std::string meaning, sKeyMean previous); // From a meaning extract discourse function words and events
                                                                            // then create a link from previous to current event. Return current keyMean.
        void TESTwhenIsUsed(std::string word); // Temporary tool function. Displays all links made with the word

    };
}

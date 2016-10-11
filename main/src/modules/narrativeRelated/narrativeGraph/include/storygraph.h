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

#ifndef _STORYGRAPH_H_
#define _STORYGRAPH_H_

#include <semanticscanner.h>
#include <discourseform.h>
#include <iostream>
#include <fstream>

namespace storygraph {

    /// An enum to distinguish type of event for Action or Result in the IGARF
    enum evtType {
        UNDEF, ACTION_EVT, IGARF_EVT
    };

    /// Describes an IGARF Event
    struct sIGARF {
        std::vector < int > vInitState; ///< The instance numbers of the relations describing the Initial State
        std::vector < int > vGoal; ///< The instance numbers of the relations describing the Goal
        evtType tAction; ///< The type of the Action cell
        evtType tResult; ///< The type of the Result cell
        int iAction; ///< The instance number of the Action cell
        int iResult; ///< The instance number of the Result cell
        std::vector < int > vFinalState; ///< The instance numbers of the relations describing the Final State
        int iNext;  ///< The instance number of the Next IGARF (in temporal order). -1 if there is no next event
        int iLevel; /// level of depth of the IGARF
        std::string toString() {
            std::ostringstream os;
            os << "vInitState: ";
            for (auto ini : vInitState){
                os << ini << " ";
            }
            os << "; vGoal: ";
            for (auto ini : vGoal){
                os << ini << " ";
            }
            os << "; vFinalState: ";
            for (auto ini : vFinalState){
                os << ini << " ";
            }
            os << "; tAction: " << tAction << "; tResult: " << tResult << "; iAction: " << iAction << "; iResult: " << iResult << "; iNext: " << iNext << "; iLevel: " << iLevel;
            return os.str();
        }
    };

    /// Describes where a link come from or go to in the Situation Model
    struct sKeyMean {
        int iIGARF; ///< Instance number of the IGARF pointed at
        char cPart; ///< I (InitState) G (Goal) A (Action) R (Result) F (FinalState)
        int iRel; ///< Relation index, -1 if no relation is pointed at
        ///< @warning It is the index in the part pointed at, not in SituationModel::vRelations.

        std::string toString(){
            std::ostringstream os;
            //            std::cout << "IGARF: " << iIGARF << " char: " << cPart << " iRel: " << iRel << std::endl;
            os << "IGARF: " << iIGARF << " char: " << cPart << " iRel: " << iRel;
            return os.str();
        }
    };
    bool operator==(const sKeyMean& A, const sKeyMean& B);

    /// Describes a dicourse link
    struct sDiscourseLink {
        sKeyMean fromEvt; ///< The left part of the link in discourse
        sKeyMean toEvt; ///< The right part of the link in discourse  (can be null)
        std::string word; ///< Discourse Function Word attached to this link
    };

    class EVT_IGARF{
    public:
        sKeyMean km;
        int iIgarf;
        int iLevel;
        double dIGARF; ///< fraction of the IGARF (# divided par total)
        int rangeIGARF;

        std::string toString(){
            std::ostringstream os;
            os << km.iIGARF << " " << km.cPart << " " << km.iRel << " - " << iIgarf << " / " << iLevel << " / " << dIGARF;
            return os.str();
        }


        EVT_IGARF(){}

        EVT_IGARF(sKeyMean _km, int ig, int il){
            km = _km;
            iIgarf = ig;
            iLevel = il;
        }
    };


    class SituationModel {
    private:
        int rendering_wEvtBox; // Width of the Atom Event boxes
        int rendering_hEvtBox;
        int rendering_wIGARFBox;
        int rendering_hIGARFBox;
        int rendering_hOffset; // Horizontal Offset between IGARF boxes
        int rendering_vOffset;

        bool sentenceEnd;
        sKeyMean lastFocus;

        std::set < int > vRelSaid;
        std::set < int > vActSaid;

    public:
        int instanceBegin; /// ABM instance of the first event

        std::vector < sRelation >      vRelations;
        std::vector < sActionEvt >     vActionEvts;
        std::vector < sIGARF >         vIGARF;
        std::vector < sDiscourseLink > vDiscourseLinks;
        std::vector < int > vChronoIgarf; ///< Vector of the apparition of the IGARF in term of chronology
        std::vector < std::pair<int, std::string > > vChronoEvent; ///< Vector of the apparition of the events in term of chronology: first element is the chronolog of the IGARF, second element is I,G,A,R or F

        SituationModel();

        void initializeStory(const story& base); ///< Imports events
        void clear(); ///< Removes all IGARF, Action Events, Relations and Discourse Links

        // -- Display
        std::string getSentenceEvt(int i); ///< Produces a naïve sentence from the i-th event
        std::string getSentenceRel(int i); ///< Produces a naïve sentence from the i-th relations
        std::string dispRelations(const std::vector < int >& rels); ///< Displays all relations in a [Subject Verb Object] format
        void showIGARF(int i, std::ofstream &IGARFfile, int level = 0); ///< Displays a tree view of the i-th IGARF
        /**< @param level Used to set the margin recursively. You don't need to use it. **/


        // -- Creation - Modification
        // Create (or find) Relation, ActionEvt or IGARF event and stock them in the class vectors. Return their index.
        int addNewActionEvt(const sActionEvt& act); ///< Creates a new Action Event, returns its index in vActionEvts
        int findRelation(const sRelation& rel, bool create = false); ///< @param create When true, it creates the relation when it doesn't exist.
        ///< @return Index of the relation in vRelations or -1 if it doesn't exist
        int createIGARF(); ///< Creates a blank new IGARF and returns its index in vIGARF
        // Modify
        void modifEventIGARF(int iIGARF, char cPart, int iActEvt); ///< Modifies index of Action Event for Action (cPart = 'A') or Result (cPart = 'R') in an IGARF
        void modifContentIGARF(int iIGARF, char cPart, int jIGARF); ///< Action or Result of iIGARF-th IGARF became jIGARF-th IGARF
        void remContentIGARF(int iIGARF, char cPart); ///< Sets Action or Result to None
        int addRelationIGARF(int iIGARF, char cPart, int iRel); ///< Adds the iRel-th relation to the iIGARF-th IGARF event
        ///< Return index of the relation in the state vector (for sKeyMean)
        void removeRelationIGARF(int iIGARF, char cPart, int iRel); ///< @param iRel index in vRelations
        // Links and sKeyMean
        void cleanLinks(); ///< Removes all links and lose focus
        sKeyMean createKey(int iIGARF, char cPart, int iRel);
        sActionEvt getEvent(const sKeyMean& km);
        void createLink(sKeyMean from, sKeyMean to, std::string word); ///< Creates and adds a link in the vDiscourseLinks
        // -- ABMtoSM
        void ABMtoSM(const story &s, std::ofstream &IGARFfile); ///< Uses a story (its vector of evtStory) to automatically generate a Situation Model
        void makeStructure(std::ofstream &IGARFfile); ///< From all the IGARF, makes a structure with story arc, failure and consequence, etc...
        // -- SMtoTrain and SMandNarrativeToTrain
        int proximityScoreAction(int i, const std::vector <std::string>& ocw); ///< Mesures vocabulary coherence for sActionEvt
        ///< @param i Index of the sActionEvt in SituationModel::vActionEvts.
        ///< @return The coherence score. It may be weighted
        /// (Predicate is more important thant Recipient) or binary (threshold of acceptance)
        int proximityScoreRelation(int i, const std::vector <std::string>& ocw); ///< @see proximityScoreAction
        std::vector<sKeyMean> findBest(const std::vector <std::string>& ocw, int& iScore); ///< Find the sActionEvt or sRelation that share the most vocabulary
        ///< @return Returns a sKeyMean to the best event or relation or a (-1 'A' -1) sKeyMean if none has been found
        std::string SMtoTrain(std::string sentence);
        // -- LRHtoSM and LRHtoBlankSM
        sActionEvt extractAction(const std::string& meaning); ///< Returns the sActionEvt describe by the meaning
        sKeyMean findEventOrRelation(sActionEvt a); // Return a (-1 'A' -1) sKeyMean if not found
        void LRHtoSM(const std::string& meaning, bool create); ///< Adds a link in the vDiscourseLinks from lastFocus to the event or relation describe in the meaning.
        ///< @param meaning Contains a meaning with format: "meaning1, meaning2, ... ". If the first meaning contains DFW,
        /// then they are used to make links.
        ///< @param create If true and if no known event or relation has been recognized in the meaning,
        /// then a new sActionEvt or sRelation is created and added to the SituationModel (it is integrated in an IGARF).
        std::vector <std::pair <int, int> >  SMtoStructure(int head);   /// return the list of IGARF with their levels

        void endSentence(); ///< Ends a sentence, avoid next event to be automatically link to last one
        // -- SMtoLRH
        void AUXautoLink(int iIGARF); // Auxiliary
        void autoLink(int iIGARF);
        void SMtoLRH(std::string lang = "en"); ///< Use discourse links to create a story.
        ///< Reads the link in order of input (FIFO). Use SituationModel::meaningFromKeyMean to produce meanings.

        // -- Rendering
        void initSizes(int _rendering_wEvtBox, int _rendering_hEvtBox, int _rendering_hOffset, int _rendering_vOffset); ///< Calculates all sizes for rendering
        void calculateSize(int currentIGARF, std::vector < int > &IGARFlevels, int level); ///< Auxiliary function to std::pair <int, int> calculateSize(int nIGARF)
        std::pair <int, int> calculateSize(int nIGARF); ///< Returns the size of the canvas to display the IGARF
        void writeIGARFdef(std::ofstream &fOutputint, int nIGARF); ///< Defines the IGARF appearance in the SVG file

        void writeSVGIGARF(std::ofstream &fOutput, int nIGARF, int x, int y); ///< Auxiliary function: Draws the nIGARF-th at the given position in the SVG file
        int addIGARFtoGrid(std::ofstream &fOutput, std::vector < int > &IGARFgrid, int currentIGARF, int level); ///< Auxiliary function: Draws the IGARF and its sons recursively at the corresponding level
        void writeSVG(std::ofstream &fOutput, int nIGARF); ///< Draws the IGARF by calling the appropriate auxiliary function

        void displayEvent(); // display all event in the IGARF
        void checkEVTIGARF(EVT_IGARF &evtKM);

        void removeDoubleEvt(std::vector<sKeyMean> &vkmBest);

    };


    // class of Discourse Function Words
    class DFW {
    public:
        std::string  sName;
        DFW(std::string name);

        std::vector <EVT_IGARF>   vSingleIGARF;
        std::vector <std::pair <EVT_IGARF, EVT_IGARF> >   vDoubleIGARF;
    };

}

#endif

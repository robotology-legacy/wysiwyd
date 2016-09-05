/*
 * Copyright (C) 2016 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau
 * email:   gregoire.pointeau@inserm.fr
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

#ifndef _DISCOURSEFORM_H_
#define _DISCOURSEFORM_H_

#include <story.h>
#include <storygraph.h>
#include <set>


namespace discourseform {


    /// struct discourse
    /// Matrix 3D:
    /// D1: number of sentence in the discourse
    /// D2: number of proposition in the sentence
    /// D3: number of OCW in the proposition

    struct meaningProposition{
        std::vector<std::string> vRole; // P A O R
        std::vector<std::string> vOCW;  // open class words
//        std::vector<typename storygraph::sKeyMean> kmLinkEvt; // score (double) of relation to an event of a SM (int)
    };

    struct meaningSentence{
        std::vector<meaningProposition> vSentence;
    };

    struct discourse {
        std::vector < meaningSentence > vDiscourse;
    };



    // meaning of a discourse in term of speech understanding
    class meaningDiscourse{

    public:
        discourseform::discourse meanings;

        bool meaningToDiscourseForm(std::vector<std::string> vMeaning); // goes from a vector of meaning from lrh, to a discourse form
        void print();
    };


}




#endif

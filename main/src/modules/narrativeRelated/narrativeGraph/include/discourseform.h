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


    // Structure of possible sentence responded by the robot
    // contain the sentence, a double (score) and a PAOR of the related evt
    struct hriResponse{
        std::string sentence;
        double      score;
        storygraph::PAOR        paor;

        hriResponse(){
            sentence = "";
            score = 0;
            paor = storygraph::PAOR();
        }

        hriResponse(std::string s, double d, storygraph::PAOR pa){
            sentence = s;
            score = d;
            paor = pa;
        }

        bool operator==(const hriResponse& hr){
            return sentence == hr.sentence;
        }

        std::string toString(){
            std::ostringstream os;
            os << sentence << " " << score << " - " << paor.toString();
            return os.str();
        }





        //        std::vector<typename storygraph::sKeyMean> kmLinkEvt; // score (double) of relation to an event of a SM (int)
    };

    struct meaningSentence{
        std::vector<storygraph::PAOR> vSentence;

        // only get the first proposition as PAOR

        std::string getSentence(){
            std::ostringstream os;
            for (auto prop : vSentence){
                os << " " << prop.toString();
            }
            return os.str();
        }
    };

    struct discourse {
        std::vector < meaningSentence > vDiscourse;
    };



    // meaning of a discourse in term of speech understanding
    class meaningDiscourse{

    public:
        discourseform::discourse meanings;

        std::string meaningToDiscourseForm(std::vector<std::string> vMeaning); // goes from a vector of meaning from lrh, to a discourse form
        void print();
    };


}




#endif

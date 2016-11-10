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


    struct PAOR{
        std::string P;
        std::string A;
        std::string O;
        std::string R;
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


    // Structure of possible sentence responded by the robot
    // contain the sentence, a double (score) and a PAOR of the related evt
    struct hriResponse{
        std::string sentence;
        double      score;
        PAOR        paor;

        hriResponse(){
            sentence = "";
            score = 0;
            paor = PAOR();
        }

        hriResponse(std::string s, double d, PAOR pa){
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

    };




    /// struct discourse
    /// Matrix 3D:
    /// D1: number of sentence in the discourse
    /// D2: number of proposition in the sentence
    /// D3: number of OCW in the proposition

    struct meaningProposition{
        std::vector<std::string> vRole; // P A O R
        std::vector<std::string> vOCW;  // open class words

        discourseform::PAOR toPAOR(){
            discourseform::PAOR paor;

            if (vOCW.size() > 0){
                paor.P = vOCW[0];
            }
            if (vOCW.size() > 1){
                paor.A = vOCW[1];
            }
            if (vOCW.size() > 2){
                paor.O = vOCW[2];
            }
            if (vOCW.size() > 3){
                paor.R = vOCW[3];
            }

            return paor;
        }

//        std::vector<typename storygraph::sKeyMean> kmLinkEvt; // score (double) of relation to an event of a SM (int)
    };

    struct meaningSentence{
        std::vector<meaningProposition> vSentence;

        // only get the first proposition as PAOR
        discourseform::PAOR toPAOR(){
            discourseform::PAOR paor;
            if (vSentence.size() != 0){
                return (vSentence[0].toPAOR());
            }
            return paor;
        }


        std::string getSentence(){
            std::ostringstream os;
            for (auto prop : vSentence){
                for (auto OCW : prop.vOCW){
                    os << " " << OCW;
                }
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

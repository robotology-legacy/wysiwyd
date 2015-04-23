/* 
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Ugo Pattacini, Tobias Fischer
 * email:   ugo.pattacini@iit.it, t.fischer@imperial.ac.uk
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

#ifndef __CLASSIFIERHANDLING_H__
#define __CLASSIFIERHANDLING_H__

#include <string>
#include <deque>
#include <map>

#include <yarp/os/Bottle.h>

#define OBJECT_UNKNOWN  "?"

using namespace std;
using namespace yarp::os;


/**********************************************************/
class Classifier
{
protected:
    string name;
    double threshold;
    double newScore;

    struct Score
    {
        bool   isPositive;
        double score;
    };

    deque<Score> window;
    size_t winLen;

    void   init(const double thres=0.5);
    void   push(const Score &s);
    void   push(const bool isPositive);
    void   declare(const bool isPositive);
    double update();

public:
    Classifier(const double thres=0.5);
    Classifier(const Classifier &classifier);
    Classifier(const string &name, const double thres=0.5);
    Classifier(const Bottle &options);
    string getName() const { return name; }
    bool   isThis(const double val) const;
    void   prepare(const double newScore);
    void   negative();
    void   positive();
    void   fromBottle(const Bottle &options);
    Bottle toBottle();
};


/**********************************************************/
class ClassifiersDataBase : public map<string,Classifier*>
{
public:
    ~ClassifiersDataBase();
    void   clear();
    void   erase(iterator it);
    string findName(const Bottle &scores, const string &tag);
};

#endif


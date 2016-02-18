/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
 * website: http://wysiwyd.upf.edu/
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

#ifndef TOUCHINGORDER_H
#define TOUCHINGORDER_H

#include <yarp/os/all.h>

#include "behavior.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;



class TouchingOrder: public Behavior
{
private:
    void run(Bottle args=Bottle());
    std::string babblingLimb ;

public:
    TouchingOrder(Mutex* mut): Behavior(mut) {
        ;
    }

    void configure();

    bool handleTouch(string type, string target);
    bool handleSearch(string type, string target);

    void close_extra_ports() {
        ;
    }
};



#endif // TOUCHINGORDER_H

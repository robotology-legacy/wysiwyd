/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
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

#ifndef ABMAUGMENTIONEXAMPLE
#define ABMAUGMENTIONEXAMPLE

#include <vector>

#include <yarp/os/RFModule.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>

class ABMAugmentionExample: public yarp::os::RFModule {
protected:
    yarp::os::Port abm;
    yarp::os::Port handlerPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > augmentedImageIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > augmentedImageOut;

    std::vector<yarp::sig::ImageOf<yarp::sig::PixelRgb> > vRawImages;
    std::vector<yarp::sig::ImageOf<yarp::sig::PixelRgb> > vAugmentedImages;
    std::vector<yarp::os::Bottle> vEnvelopes;

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();

    bool receiveImages(int instance);
    void augmentImages();
    void sendAugmentedImages();
};

#endif

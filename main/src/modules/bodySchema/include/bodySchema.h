/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Martina Zambelli
 * email:   m.zambelli13@imperial.ac.uk
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

#ifndef _BODYSCHEMA_H_
#define _BODYSCHEMA_H_

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream> 
#include <sstream>
#include <string>

#include <vector>
#include <math.h>
#include <time.h>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <wrdac/clients/icubClient.h>
#include <otl.h>
#include <otl_oesgp.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::sig::draw;
using namespace wysiwyd::wrdac;
using namespace OTL;
using namespace cv;

typedef enum {idle, learning, babbling} State;

class bodySchema : public RFModule {
private:
    string moduleName;
    int isVerbose;

    Port handlerPort;// a port to handle messages

    ofstream encodersData;
    ofstream velocitiesData;
    ofstream commandData;

    BufferedPort< ImageOf<PixelRgb> > imgPortIn;
    BufferedPort< ImageOf<PixelRgb> > imgPortOut;
    BufferedPort<Bottle> portVelocityOut;
    BufferedPort<Bottle> portPredictionErrors;

    IPositionControl* pos;
    IVelocityControl* vel;
    IEncoders* encs;
    IControlMode2 *ictrl;
    IInteractionMode *iint;

    IPositionControl* posHead;
    IVelocityControl* velHead;
    IEncoders* encsHead;

    PolyDriver* armDev;
    PolyDriver* headDev;

    yarp::sig::Vector encoders, cmd, command, new_command, tmpSpeed, tmpAcc;
    yarp::sig::Vector encodersHead, commandHead;

    string ports[4];
    string video[2];

    int MAX_COUNT;
    bool needToInit;
    string videoName;
    int fps;

    string part;
    string robot;
    string mode;

    int nInputs;
    int nOutputs;

    double initTime;

    int capseq;

    string foldername;
    string foldernamepoints;
    string foldernamevideo;
    string foldernameframes;
    string fileEncData;
    string fileCmdData;
    
    double freq1, freq2, freq3, amp, ampcos2;
    double amp0,amp1,amp2,amp3,amp4;
    double train_duration;
    double test_duration;
    
    double start_commandHead[5];
    double start_command[7];
    
    int frame_idx;
    int num_init_points;
    vector<int> points_idx;
    Mat gray, prevGray, image;
    vector<Point2f> points[2];
    string source_window;

    OESGP oesgp;
    OESGP oesgp2;
    VectorXd prediction;
    VectorXd prediction_variance;

    int p_input_dim;
    int p_output_dim;
    int p_reservoir_size;
    double p_input_weight;
    double p_output_feedback_weight;
    double p_leak_rate;
    double p_connectivity;
    double p_spectral_radius;
    int p_use_inputs_in_state;
    double p_noise;
    double p_epsilon;
    int p_capacity;
    int p_random_seed;

    State state;
    bool endTrain;

public:
    bool configure(ResourceFinder &rf); // configure
    bool interruptModule();        // interrupt, e.g., the ports
    bool close();               // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod();
    bool updateModule();
    
    bool learnAbsPos();

    const std::string trail_string();
    bool create_folders();
    yarp::sig::Vector babblingExecution(double &t);
    bool goStartPos();
    bool writeEncoderData(Point2f &lost_indic,ofstream& fs_enc,ofstream& fs_cmd);
    bool findFeatures(TermCriteria &termcrit, Size &subPixWinSize, Size &winSize);
    bool getBabblingImages();
    bool init_iCub(string &part);
    bool init_oesgp_learner();
    
private:
    string portImageLeftName;
    string portImageTargetLeftName;
    BufferedPort<ImageOf<PixelRgb> > portImageLeft;
    BufferedPort<yarp::sig::Vector> portImageTargetLeft;
    
    double prev_x;
    double prev_y;
    double prev_xMean;
    double prev_yMean;
    yarp::sig::Vector setpoints;
    
    void find_image();
};

#endif // __BODYSCHEMA_H__

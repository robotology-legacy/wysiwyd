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

#ifndef _MODELOTL_H_
#define _MODELOTL_H_

#include <cv.h>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <wrdac/clients/icubClient.h>
#include <otl.h>
#include <otl_oesgp.h>

using namespace OTL;

class ModelOTL : public yarp::os::RFModule {
private:
    std::string moduleName;

    yarp::os::Port handlerPort;
    yarp::os::BufferedPort<yarp::os::Bottle> portVelocityOut;
    yarp::os::BufferedPort<yarp::os::Bottle> portPredictionErrors;
    yarp::os::BufferedPort<yarp::os::Bottle> portInputsfromBabbling;
    yarp::os::BufferedPort<yarp::os::Bottle> portOutputsfromSensoryProc;

    std::string part;

    int nInputs;
    int nOutputs;

    double initTime;

    double train_duration,test_duration;
    bool endTrain;

    OESGP oesgp,oesgp2;
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

public:
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();

private:

    bool learnOTLModel();
    bool init_oesgp_learner();
};

#endif // __MODELOTL_H__

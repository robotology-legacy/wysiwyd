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

#include "modelOTL.h"

#include <cmath>
#include <signal.h>
#include <vector>
#include <iostream>

using namespace OTL;
using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

bool ModelOTL::configure(yarp::os::ResourceFinder &rf) {
    bool bEveryThingisGood = true;

    moduleName = rf.check("name",Value("bodySchema"),"module name (string)").asString();

    part = rf.check("part",Value("left_arm")).asString();

    Bottle &babbl_par = rf.findGroup("babbling_param");
    train_duration = babbl_par.check("train_duration", Value(20.0)).asDouble();
    test_duration = babbl_par.check("test_duration", Value(10.0)).asDouble();

    Bottle &oesgp_par = rf.findGroup("oesgp_learner");
    p_input_dim = oesgp_par.check("input_dim", Value(2)).asInt();
    p_output_dim = oesgp_par.check("output_dim", Value(1)).asInt();
    p_reservoir_size = oesgp_par.check("reservoir_size", Value(100)).asDouble();
    p_input_weight = oesgp_par.check("input_weight", Value(1.0)).asDouble();
    p_output_feedback_weight = oesgp_par.check("output_feedback_weight", Value(0.0)).asDouble();
    p_leak_rate = oesgp_par.check("leak_rate", Value(0.9)).asDouble();
    p_connectivity = oesgp_par.check("connectivity", Value(0.1)).asDouble();
    p_spectral_radius = oesgp_par.check("spectral_radius", Value(0.90)).asDouble();
    p_use_inputs_in_state = oesgp_par.check("use_inputs_in_state", Value(1)).asInt();
    p_noise = oesgp_par.check("noise", Value(0.000001)).asDouble();
    p_epsilon = oesgp_par.check("epsilon", Value(1e-3)).asDouble();
    p_capacity = oesgp_par.check("capacity", Value(10)).asInt();
    p_random_seed = oesgp_par.check("random_seed", Value(0)).asInt();

    setName(moduleName.c_str());

    // Open handler port
    if (!handlerPort.open("/" + getName() + "/rpc")) {
        cout << getName() << ": Unable to open port " << "/" << getName() << "/rpc" << endl;
        bEveryThingisGood = false;
    }

    if (!portVelocityOut.open("/" + getName() + "/" + part + "/velocity:o")) {
        cout << getName() << ": Unable to open port " << "/" << getName() << "/" << part << "/velocity:o" << endl;
        bEveryThingisGood = false;
    }

    if (!portPredictionErrors.open("/" + getName() + "/portPredictionErrors")) {
        cout << getName() << ": Unable to open port " << "/" + getName() + "/portPredictionErrors" << endl;
        bEveryThingisGood = false;
    }

    if (!portInputsfromBabbling.open("/" + getName() + "/portInputs")) {
        cout << getName() << ": Unable to open port " << "/" + getName() + "/portInputs" << endl;
        bEveryThingisGood = false;
    }

    if (!portOutputsfromSensoryProc.open("/" + getName() + "/portOutputsfromSensoryProc")) {
        cout << getName() << ": Unable to open port " << "/" + getName() + "/portOutputsfromSensoryProc" << endl;
        bEveryThingisGood = false;
    }


    attach(handlerPort);

    return bEveryThingisGood;
}

bool ModelOTL::interruptModule() {

    portVelocityOut.interrupt();
    portPredictionErrors.interrupt();
    portInputsfromBabbling.interrupt();
    portOutputsfromSensoryProc.interrupt();
    handlerPort.interrupt();

    yInfo() << "Bye!";

    return true;

}

bool ModelOTL::close() {
    cout << "Closing module, please wait ... " <<endl;

    portVelocityOut.interrupt();
    portVelocityOut.close();

    portPredictionErrors.interrupt();
    portPredictionErrors.close();

    portInputsfromBabbling.interrupt();
    portInputsfromBabbling.close();

    portOutputsfromSensoryProc.interrupt();
    portOutputsfromSensoryProc.close();

    handlerPort.interrupt();
    handlerPort.close();

    yInfo() << "Bye!";

    return true;
}

bool ModelOTL::respond(const Bottle& command, Bottle& reply) {
    yInfo() << "Model learning via OESGP";
    reply.addString("nack");

    return true;
}

bool ModelOTL::updateModule() {
    return true;
}

double ModelOTL::getPeriod() {
    return 0.1;
}


bool ModelOTL::learnOTLModel()
{
    init_oesgp_learner();

    VectorXd input(1);
    VectorXd output(1);
    VectorXd input_test(1);

    yarp::sig::Vector babCmd;
    Bottle *inputCmd;
    Bottle *outputSens;

    double startTime = yarp::os::Time::now();
    while (Time::now() < startTime + train_duration){
        double t = Time::now() - startTime;
        yInfo() << " Time: " << t << " / " << train_duration ;

        inputCmd = portInputsfromBabbling.read(true);
        input(0) = inputCmd->get(0).asDouble();

        //update learner
        oesgp.update(input);

        //predict the next state
        oesgp.predict(prediction, prediction_variance);

        outputSens = portOutputsfromSensoryProc.read(true);
        output(0) = outputSens->get(0).asDouble();

        double error[1];
        error[0] = (prediction(0) - output(0));
        yInfo() << "Error: " << error[0] << ", |BV|: " << oesgp.getCurrentSize() ;

        //train with the true next state
        oesgp.train(output);
    }

    /* Validation */
    yInfo() << "Testing saving and loading model " ;

    oesgp.save("oesgptest");
    oesgp2.load("oesgptest");

    int sum=0;
    int Sqno=0;
    int count=0;

    double startTime_test = yarp::os::Time::now();

    while (Time::now() < startTime_test  + test_duration) {
        double t = Time::now() - startTime_test;
        yInfo() << " Time: " << t << " / " << test_duration ;

        inputCmd = portInputsfromBabbling.read(true);
        input_test(0) = inputCmd->get(0).asDouble();

        //update
        oesgp2.update(input_test);

        //predict
        oesgp2.predict(prediction, prediction_variance);

        VectorXd output(1);
        outputSens = portOutputsfromSensoryProc.read(true);
        output(0) = outputSens->get(0).asDouble();

        double error[1], rmse[1];
        error[0] = (prediction(0) - output(0));

        count++;
        Sqno=pow(error[0],2);
        sum=sum+Sqno;

        rmse[0] = sqrt(sum/count); //rmse(output(0),prediction(0));//
        yInfo() << "Error: " << error[0]  ;
        yInfo() << "RMSE: " << rmse[0]  ;
        Bottle& errB = portPredictionErrors.prepare(); // Get the object
        errB.clear();
        errB.addDouble(rmse[0]);
        portPredictionErrors.write(); // Now send it on its way
    }

    return true;
}


bool ModelOTL::init_oesgp_learner()
{
    //Create our OESGP object
    int activation_function = Reservoir::TANH;
    VectorXd kernel_parameters(2);
    kernel_parameters << 1.0, 1.0;

    oesgp.init( p_input_dim, p_output_dim, p_reservoir_size,
                p_input_weight, p_output_feedback_weight,
                activation_function,
                p_leak_rate,
                p_connectivity, p_spectral_radius,
                p_use_inputs_in_state,
                kernel_parameters,
                p_noise, p_epsilon, p_capacity, p_random_seed);

    return true;
}

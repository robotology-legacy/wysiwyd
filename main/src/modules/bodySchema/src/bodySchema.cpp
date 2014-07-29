/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau
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

#include <bodySchema.h>

#include <cmath>

using namespace OTL;
using namespace std;
using namespace yarp::os;

bodySchema::bodySchema()
{
}

bodySchema::~bodySchema()
{
}



/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the
 * equivalent of the "open" method.
 */

bool bodySchema::configure(yarp::os::ResourceFinder &rf) {
    
    bool    bEveryThingisGood = true;
    moduleName            = rf.check("name",
                                     Value("bodySchema"),
                                     "module name (string)").asString();


    part = rf.find("part").asString();
    robot = rf.check("robot",Value("icubSim")).asString();
    mode = rf.check("mode",Value("online")).asString();

    state = idle;



    /*
    * before continuing, set the module name before getting any other parameters,
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());


    // Open handler port
    handlerPortName = "/" + getName() + "/rpc";         // use getName() rather than a literal

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        bEveryThingisGood = false;
    }

    // Open ports
    portImageInRightName = "/" + getName() + "/portImageInRight";

    if (!portImageInRight.open(portImageInRightName.c_str())) {
        cout << getName() << ": Unable to open port " << portImageInRightName << endl;
        bEveryThingisGood = false;
    }

    portImageInLeftName = "/" + getName() + "/portImageInLeft";

    if (!portImageInLeft.open(portImageInLeftName.c_str())) {
        cout << getName() << ": Unable to open port " << portImageInLeftName << endl;
        bEveryThingisGood = false;
    }

    portRightArmStateName = "/" + getName() + "/portRightArmState";

    if (!portRightArmState.open(portRightArmStateName.c_str())) {
        cout << getName() << ": Unable to open port " << portRightArmStateName << endl;
        bEveryThingisGood = false;
    }

    portLeftArmStateName = "/" + getName() + "/portLeftArmState";

    if (!portLeftArmState.open(portLeftArmStateName.c_str())) {
        cout << getName() << ": Unable to open port " << portLeftArmStateName << endl;
        bEveryThingisGood = false;
    }


    // Predictions/Errors ports
    portPredictionsName = "/" + getName() + "/portPredictions";

    if (!portPredictions.open(portPredictionsName.c_str())) {
        cout << getName() << ": Unable to open port " << portPredictionsName << endl;
        bEveryThingisGood = false;
    }

    portPredictionErrorsName = "/" + getName() + "/portPredictionErrors";

    if (!portPredictionErrors.open(portPredictionErrorsName.c_str())) {
        cout << getName() << ": Unable to open port " << portPredictionErrorsName << endl;
        bEveryThingisGood = false;
    }

    portReadPredictionsName = "/" + getName() + "/portReadPredictions";

    if (!portReadPredictions.open(portReadPredictionsName.c_str())) {
        cout << getName() << ": Unable to open port " << portReadPredictionsName << endl;
        bEveryThingisGood = false;
    }

    portReadPredictionErrorsName = "/" + getName() + "/portReadPredictionErrors";

    if (!portReadPredictionErrors.open(portReadPredictionErrorsName.c_str())) {
        cout << getName() << ": Unable to open port " << portReadPredictionErrorsName << endl;
        bEveryThingisGood = false;
    }

    // Inputs ports: InData -> inputs of the model to build; OutData -> observed outputs of the model to build.
    portInDataName = "/" + getName() + "/portInData";

    if (!portInData.open(portInDataName.c_str())) {
        cout << getName() << ": Unable to open port " << portInDataName << endl;
        bEveryThingisGood = false;
    }

    portOutDataName = "/" + getName() + "/portOutData";

    if (!portOutData.open(portOutDataName.c_str())) {
        cout << getName() << ": Unable to open port " << portOutDataName << endl;
        bEveryThingisGood = false;
    }

    portReadInDataName = "/" + getName() + "/portReadInData";

    if (!portReadInData.open(portReadInDataName.c_str())) {
        cout << getName() << ": Unable to open port " << portReadInDataName << endl;
        bEveryThingisGood = false;
    }

    portReadOutDataName = "/" + getName() + "/portReadOutData";

    if (!portReadOutData.open(portReadOutDataName.c_str())) {
        cout << getName() << ": Unable to open port " << portReadOutDataName << endl;
        bEveryThingisGood = false;
    }



    if (mode=="online")
    {
        init_iCub(part);
    }
    else
    {
        cout << endl << "Open dataSetPlayer!" <<endl<<endl;
        inPort = "/" + part + "/vel:o" ;
        outPort =  "/icub/"+ part + "/state:o";

        cout << inPort <<endl;
        cout <<outPort <<endl;

        while(!Network::connect(outPort,portReadOutDataName))
        {
            cout << "Trying to connect input data port..." << endl;
            Time::delay(0.1);
        }
        while(!Network::connect(inPort,portReadInDataName))
        {
            cout << "Trying to connect output data port..." << endl;
            Time::delay(0.1);
        }

    }



    attach(handlerPort);                  // attach to port

    return bEveryThingisGood ;
}

bool bodySchema::interruptModule() {

    portReadOutData.interrupt();
    portReadInData.interrupt();
    return true;
}

bool bodySchema::close() {
    handlerPort.close();
    portImageInRight.close();
    portImageInLeft.close();
    portRightArmState.close();
    portLeftArmState.close();
    portPredictions.close();
    portPredictionErrors.close();
    portInData.close();
    portOutData.close();
    portReadInData.close();
    portReadOutData.close();

    return true;
}

bool bodySchema::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) +
            " commands are: \n" +
            "babblingLearning \n" +
            "learning \n" +
            "help \n" +
            "quit \n";

    reply.clear();

    if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="babblingLearning") {

        if (state==idle)
        {
            state = babbling;

//            learn();

            cout << helpMessage;
            reply.addString("ack");
        }
        else
            reply.addString("nack");
    }
    else if (command.get(0).asString()=="learning") {

        if (state==idle)
        {
            state = learning;

//            inPort = "/" + part + "/vel:o" ;
//            outPort =  "/icub/"+ part + "/state:o";
//            learn(inPort,outPort);

            cout << helpMessage;
            reply.addString("ok");

        }
        else
            reply.addString("nack");


    }
    else
        RFModule::respond(command,reply);


    return true;
}

/* Called periodically every getPeriod() seconds */
bool bodySchema::updateModule() {


    if (state==learning)
    {
        learn(inPort,outPort);
        state=idle;
    }
    else if (state==babbling)
    {
        learn();
        state=idle;
    }

    return true;
}

double bodySchema::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}





/*
*   Learning
*
*/

bool bodySchema::learn(string &dataIn, string &dataOut)
{

    cout << "Learning from recorded data (dataSetPlayer required)" <<endl;



    //Create our OESGP object
    OESGP oesgp;

    //Problem parameters
    //we want to predict a simple sine wave
    int input_dim = 3;
    int output_dim = 3;

    //Reservoir Parameters
    //you can change these to see how it affects the predictions
    int reservoir_size = 300;
    double input_weight = 1.0;
    double output_feedback_weight = 0.0;
    int activation_function = Reservoir::TANH;
    double leak_rate = 0.9;
    double connectivity = 0.1;
    double spectral_radius = 0.90;
    bool use_inputs_in_state = false;

    VectorXd kernel_parameters(2); //gaussian kernel parameters
    kernel_parameters << 1.0, 1.0; //l = 1.0, alpha = 1.0

    //SOGP parameters
    double noise = 0.000001;
    double epsilon = 1e-3;
    int capacity = 300;

    int random_seed = 0;


    try {


//        for(int kk=0;kk<10;kk++)
//        {
//            cout <<"?????"<<endl;
//            Time::yield();
//        }


        //Initialise our OESGP
        oesgp.init( input_dim, output_dim, reservoir_size,
                    input_weight, output_feedback_weight,
                    activation_function,
                    leak_rate,
                    connectivity, spectral_radius,
                    use_inputs_in_state,
                    kernel_parameters,
                    noise, epsilon, capacity, random_seed);


        cout << "?????"<<endl;

        //now we loop using a sine wave
        unsigned int max_itr = 10000; // TO MODIFY!!!!
        unsigned int iters = max_itr;

        //note that we use Eigen VectorXd objects
        //look at http://eigen.tuxfamily.org for more information about Eigen
        VectorXd input(3);  //the 1 is the size of the vector
        VectorXd output(3);

        VectorXd state;
        VectorXd prediction;
        VectorXd prediction_variance;

        VectorXd in(1);  //the 1 is the size of the vector
        VectorXd out(1);



        double prev_pos0;
        double prev_pos1;
        double prev_pos2;
        Bottle *prevPosB = portReadOutData.read();
        if (!prevPosB->isNull())
        {
            prev_pos0 = prevPosB->get(0).asDouble();//cout << "prev pos 0:" << prev_pos0 << endl; // 0 is the joint number!
            prev_pos1 = prevPosB->get(1).asDouble(); // 1 is the joint number!
            prev_pos2 = prevPosB->get(2).asDouble(); // 2 is the joint number!
        }
        else
            cout << "NULL Bottle" << endl;



        for (unsigned int i=1; i<iters; i++) {

            //create the input and output
            Bottle *inB = portReadInData.read();

            if (!inB->isNull())
            {
                in(0) = inB->get(0).asDouble(); //cout << "in 0:" << in(0) << endl;
                in(1) = inB->get(1).asDouble(); // 1 is the joint number!
                in(2) = inB->get(2).asDouble(); // 2 is the joint number!
            }
            else
                cout << "NULL Bottle" << endl;


            Bottle *outB = portReadOutData.read();

            if (!outB->isNull())
            {
                output(0) = outB->get(0).asDouble() - prev_pos0; // 0 is the joint number!
                output(1) = outB->get(1).asDouble() - prev_pos1;
                output(2) = outB->get(2).asDouble() - prev_pos2;
            }
            else
                cout << "NULL Bottle" << endl;

            prev_pos0 = outB->get(0).asDouble();
            prev_pos1 = outB->get(1).asDouble();
            prev_pos2 = outB->get(2).asDouble();

            input(0) = in(0);
            input(1) = in(1);
            input(2) = in(2);

            cout << "Output.:" << output << endl;

            Bottle& outDataB = portOutData.prepare(); // Get the object
            outDataB.clear();
            outDataB.addDouble(output(0));
            outDataB.addDouble(output(1));
            outDataB.addDouble(output(2));
            portOutData.write(); // Now send it on its way



            cout << "Written to port portOutData..." << endl;

            //update the OESGP with the input
            oesgp.update(input);

            //predict the next state
            oesgp.predict(prediction, prediction_variance);
            cout << "Prediction:" << prediction << endl;
//            cout << "Prediction:" << prediction.value() << endl;

            //print the error
//            double error = (prediction - output).norm();
            double error[3];
            error[0] = (prediction(0) - output(0));//.norm();
            error[1] = (prediction(1) - output(1));//.norm();
            error[2] = (prediction(2) - output(2));//.norm();

//            cout << "Error: " << error << ", |BV|: "
//                 << oesgp.getCurrentSize() <<  endl;

            cout << "Error: " << error[0] << " " << error[1] << " " << error[2] << ", |BV|: "
                    << oesgp.getCurrentSize() <<  endl;

            //train with the true next state
            oesgp.train(output);

            Bottle& predB = portPredictions.prepare(); // Get the object
            predB.clear();
            predB.addDouble(prediction(0)); //cout << "Prediction 0:" << prediction(0) << endl;
            predB.addDouble(prediction(1)); //cout << "Prediction 1:" << prediction(1) << endl;
            predB.addDouble(prediction(2)); //cout << "Prediction 2:" << prediction(2) << endl;
            portPredictions.write(); // Now send it on its way

            Bottle& errB = portPredictionErrors.prepare(); // Get the object
            errB.clear();
            errB.addDouble(error[0]);
            errB.addDouble(error[1]);
            errB.addDouble(error[2]);
            portPredictionErrors.write(); // Now send it on its way


            Time::yield();


        }

        cout << "Testing saving and loading model " << std::endl;

        //Here, we demonstrate how to save and load a model
        //we save the original model and create a new oesgp object to load the
        //saved model into

        oesgp.save("oesgptest");

        OESGP oesgp2;
        oesgp2.load("oesgptest");

        //prediction test
        for (unsigned int i=max_itr; i<max_itr+50; i++) {

            Bottle *inB = portReadInData.read();

            if (!inB->isNull())
            {
                in(0) = inB->get(0).asDouble(); //cout << "in 0:" << in(0) << endl;
                in(1) = inB->get(1).asDouble(); // 1 is the joint number!
                in(2) = inB->get(2).asDouble(); // 2 is the joint number!
            }
            else
                cout << "NULL Bottle" << endl;


            Bottle *outB = portReadOutData.read();

            if (!outB->isNull())
            {
                output(0) = outB->get(0).asDouble() - prev_pos0; // 0 is the joint number!
                output(1) = outB->get(1).asDouble() - prev_pos1;
                output(2) = outB->get(2).asDouble() - prev_pos2;
            }
            else
                cout << "NULL Bottle" << endl;

            prev_pos0 = outB->get(0).asDouble();
            prev_pos1 = outB->get(1).asDouble();
            prev_pos2 = outB->get(2).asDouble();

            input(0) = in(0);
            input(1) = in(1);
            input(2) = in(2);

            Bottle& outDataB = portOutData.prepare(); // Get the object
            outDataB.addDouble(output(0));
            outDataB.addDouble(output(1));
            outDataB.addDouble(output(2));
            portOutData.write(); // Now send it on its way



            //update
            oesgp2.update(input);

            //predict
            oesgp2.predict(prediction, prediction_variance);
//            double error = (prediction - output).norm();
            double error[3];
            error[0] = (prediction(0) - output(0));//.norm();
            error[1] = (prediction(1) - output(1));//.norm();
            error[2] = (prediction(2) - output(2));//.norm();

            cout << "Error: " << error[0] << " " << error[1] << " " << error[2] << " " << endl;


            Bottle& predB = portPredictions.prepare(); // Get the object
            predB.addDouble(prediction(0));
            predB.addDouble(prediction(1));
            predB.addDouble(prediction(2));
            portPredictions.write(); // Now send it on its way

            Bottle& errB = portPredictionErrors.prepare(); // Get the object
            errB.addDouble(error[0]);
            errB.addDouble(error[1]);
            errB.addDouble(error[2]);
            portPredictionErrors.write(); // Now send it on its way



            Time::yield();
        }

    } catch (OTLException &e) {
        cout << "CATCH????"<<endl;
        e.showError();
    }


    return true;
}



bool bodySchema::learn()
{

    cout << "Learning from babbling; iCub required." << endl;

    //Create our OESGP object
    OESGP oesgp;

    //Problem parameters
    //we want to predict a simple sine wave
    int input_dim = 4;
    int output_dim = 4;

    //Reservoir Parameters
    //you can change these to see how it affects the predictions
    int reservoir_size = 300;
    double input_weight = 1.0;
    double output_feedback_weight = 0.0;
    int activation_function = Reservoir::TANH;
    double leak_rate = 0.9;
    double connectivity = 0.1;
    double spectral_radius = 0.90;
    bool use_inputs_in_state = false;

    VectorXd kernel_parameters(2); //gaussian kernel parameters
    kernel_parameters << 10.0, 1.0; //l = 1.0, alpha = 1.0

    //SOGP parameters
    double noise = 0.0001;
    double epsilon = 1e-3;
    int capacity = 150;

    int random_seed = 0;

    try {
        //Initialise our OESGP
        oesgp.init( input_dim, output_dim, reservoir_size,
                    input_weight, output_feedback_weight,
                    activation_function,
                    leak_rate,
                    connectivity, spectral_radius,
                    use_inputs_in_state,
                    kernel_parameters,
                    noise, epsilon, capacity, random_seed);


        //now we loop using a sine wave
        unsigned int max_itr = 1000; // TO MODIFY!!!!
        unsigned int iters = max_itr;

        //note that we use Eigen VectorXd objects
        //look at http://eigen.tuxfamily.org for more information about Eigen
        VectorXd input(4);  //the 1 is the size of the vector
        VectorXd output(4);

        VectorXd state;
        VectorXd prediction;
        VectorXd prediction_variance;

        VectorXd in(1);  //the 1 is the size of the vector
        VectorXd out(1);


        double prev_pos0;
        double prev_pos1;
        double prev_pos2;
        double prev_pos3;
        pos->positionMove(0,-70);
        pos->positionMove(1,-70);
        pos->positionMove(2,-20);
        pos->positionMove(3,-50);
        Bottle *prevPosB = portReadOutData.read();
        if (!prevPosB->isNull())
        {
            prev_pos0 = prevPosB->get(0).asDouble(); // 0 is the joint number!
            prev_pos1 = prevPosB->get(1).asDouble(); // 1 is the joint number!
            prev_pos2 = prevPosB->get(2).asDouble(); // 2 is the joint number!
            prev_pos3 = prevPosB->get(3).asDouble(); // 3 is the joint number!
        }
        else
            cout << "NULL Bottle" << endl;



        for (unsigned int i=1; i<iters; i++) {

            //create the input and output
            in(0) = 50*sin(i*0.1)+sin(i*0.001)+5*sin(i*0.0001); //-70+10*sin(i*0.1)
            in(1) = 70*sin(i*0.1)+sin(i*0.001)+7*sin(i*0.0001);
            in(2) = 60*sin(i*0.1)+sin(i*0.001)+6*sin(i*0.0001);
            in(3) = 80*sin(i*0.1)+sin(i*0.001)+8*sin(i*0.0001);

            vel->velocityMove(0,in(0));//cmdRightArm);//cmd.data());
            vel->velocityMove(1,in(1));
            vel->velocityMove(2,in(2));
            vel->velocityMove(3,in(3));

            Bottle *outB = portReadOutData.read();

            if (!outB->isNull())
            {
                output(0) = outB->get(0).asDouble() - prev_pos0; // 0 is the joint number!
                output(1) = outB->get(1).asDouble() - prev_pos1;
                output(2) = outB->get(2).asDouble() - prev_pos2;
                output(3) = outB->get(3).asDouble() - prev_pos3;
            }
            else
                cout << "NULL Bottle" << endl;

            prev_pos0 = outB->get(0).asDouble();
            prev_pos1 = outB->get(1).asDouble();
            prev_pos2 = outB->get(2).asDouble();
            prev_pos3 = outB->get(3).asDouble();

            input(0) = in(0);
            input(1) = in(1);
            input(2) = in(2);
            input(3) = in(3);

            cout << "Output.:" << output << endl;

            Bottle& outDataB = portOutData.prepare(); // Get the object
            outDataB.clear();
            outDataB.addDouble(output(0));
            outDataB.addDouble(output(1));
            outDataB.addDouble(output(2));
            outDataB.addDouble(output(3));
            portOutData.write(); // Now send it on its way


            //update the OESGP with the input
            oesgp.update(input);

            //predict the next state
            oesgp.predict(prediction, prediction_variance);
            cout << "Prediction:" << prediction << endl;
//            cout << "Prediction:" << prediction.value() << endl;

            //print the error
//            double error = (prediction - output).norm();
            double error[4];
            error[0] = (prediction(0) - output(0));//.norm();
            error[1] = (prediction(1) - output(1));//.norm();
            error[2] = (prediction(2) - output(2));//.norm();
            error[3] = (prediction(3) - output(3));//.norm();

//            cout << "Error: " << error << ", |BV|: "
//                 << oesgp.getCurrentSize() <<  endl;

            cout << "Error: " << error[0] << " " << error[1] << " " << error[2]<< " " << error[3] << ", |BV|: "
                    << oesgp.getCurrentSize() <<  endl;

            //train with the true next state
            oesgp.train(output);

            Bottle& predB = portPredictions.prepare(); // Get the object
            predB.clear();
            predB.addDouble(prediction(0)); //cout << "Prediction 0:" << prediction(0) << endl;
            predB.addDouble(prediction(1)); //cout << "Prediction 1:" << prediction(1) << endl;
            predB.addDouble(prediction(2)); //cout << "Prediction 2:" << prediction(2) << endl;
            predB.addDouble(prediction(3));
            portPredictions.write(); // Now send it on its way

            Bottle& errB = portPredictionErrors.prepare(); // Get the object
            errB.clear();
            errB.addDouble(error[0]);
            errB.addDouble(error[1]);
            errB.addDouble(error[2]);
            errB.addDouble(error[3]);
            portPredictionErrors.write(); // Now send it on its way


            Time::yield();

        }

        cout << "Testing saving and loading model " << std::endl;

        //Here, we demonstrate how to save and load a model
        //we save the original model and create a new oesgp object to load the
        //saved model into

        oesgp.save("oesgptest");

        OESGP oesgp2;
        oesgp2.load("oesgptest");

        //prediction test
        for (unsigned int i=max_itr; i<max_itr+50; i++) {

            in(0) = 50*sin(i*0.1)+sin(i*0.001)+5*sin(i*0.0001); //-70+10*sin(i*0.1)
            in(1) = 70*sin(i*0.1)+sin(i*0.001)+7*sin(i*0.0001);
            in(2) = 60*sin(i*0.1)+sin(i*0.001)+6*sin(i*0.0001);
            in(3) = 80*sin(i*0.1)+sin(i*0.001)+8*sin(i*0.0001);


            vel->velocityMove(0,in(0));//cmd.data());
            vel->velocityMove(1,in(1));
            vel->velocityMove(2,in(2));
            vel->velocityMove(3,in(3));

            Bottle *outB = portReadOutData.read();

            if (!outB->isNull())
            {
                output(0) = outB->get(0).asDouble() - prev_pos0; // 0 is the joint number!
                output(1) = outB->get(1).asDouble() - prev_pos1;
                output(2) = outB->get(2).asDouble() - prev_pos2;
                output(3) = outB->get(3).asDouble() - prev_pos3;
            }
            else
                cout << "NULL Bottle" << endl;

            prev_pos0 = outB->get(0).asDouble();
            prev_pos1 = outB->get(1).asDouble();
            prev_pos2 = outB->get(2).asDouble();
            prev_pos3 = outB->get(3).asDouble();

            input(0) = in(0);
            input(1) = in(1);
            input(2) = in(2);
            input(3) = in(3);

            Bottle& outDataB = portOutData.prepare(); // Get the object
            outDataB.addDouble(output(0));
            outDataB.addDouble(output(1));
            outDataB.addDouble(output(2));
            outDataB.addDouble(output(3));
            portOutData.write(); // Now send it on its way



            //update
            oesgp2.update(input);

            //predict
            oesgp2.predict(prediction, prediction_variance);
            double error[4];
            error[0] = (prediction(0) - output(0));//.norm();
            error[1] = (prediction(1) - output(1));//.norm();
            error[2] = (prediction(2) - output(2));//.norm();
            error[3] = (prediction(3) - output(3));//.norm();
//            cout << "Error: " << error << endl;

            cout << "Error: " << error[0] << " " << error[1] << " " << error[2]<< " " << error[2] << endl;

            Bottle& predB = portPredictions.prepare(); // Get the object
            predB.addDouble(prediction(0));
            predB.addDouble(prediction(1));
            predB.addDouble(prediction(2));
            predB.addDouble(prediction(3));
            portPredictions.write(); // Now send it on its way

            Bottle& errB = portPredictionErrors.prepare(); // Get the object
            errB.addDouble(error[0]);
            errB.addDouble(error[1]);
            errB.addDouble(error[2]);
            errB.addDouble(error[3]);
            portPredictionErrors.write(); // Now send it on its way

            Time::yield();
        }

    } catch (OTLException &e) {
        e.showError();
    }
    return true;


}



bool bodySchema::init_iCub(string &part)
{

        /* Create PolyDriver */

        Property option;

        string portnameArm = part;//"left_arm";
        cout << part << endl;
        option.put("robot", robot.c_str());
        option.put("device", "remote_controlboard");
        Value& robotnameArm = option.find("robot");

        string sA("/");
        sA += robotnameArm.asString();
        sA += "/";
        sA += portnameArm.c_str();
        sA += "/control";
        option.put("local", sA.c_str());

        sA.clear();
        sA += "/";
        sA += robotnameArm.asString();
        sA += "/";
        sA += portnameArm.c_str();
        option.put("remote", sA.c_str());

        cout<<"DEBUG "<< option.toString().c_str() <<endl;


        rightArmDev = new PolyDriver(option);
        if (!rightArmDev->isValid()) {
            printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            Network::fini();
            return false;
        }

        rightArmDev->view(pos);
        rightArmDev->view(vel);
        rightArmDev->view(encs);
        rightArmDev->view(ictrl);
        rightArmDev->view(iint);
        if (pos==NULL || encs==NULL || vel==NULL || ictrl==NULL || iint==NULL ){
            printf("Cannot get interface to robot device\n");
            rightArmDev->close();
        }
        int nj = 0;
//        pos->getAxes(&nj);
//        vel->getAxes(&nj);
        encs->getAxes(&nj);
        encoders.resize(nj);
        cmd.resize(nj);

        ictrl->setControlMode(0,VOCAB_CM_VELOCITY);
        iint->setInteractionMode(0,VOCAB_IM_COMPLIANT);

        ictrl->setControlMode(1,VOCAB_CM_VELOCITY);
        iint->setInteractionMode(1,VOCAB_IM_COMPLIANT);

        ictrl->setControlMode(2,VOCAB_CM_VELOCITY);
        iint->setInteractionMode(2,VOCAB_IM_COMPLIANT);

        ictrl->setControlMode(3,VOCAB_CM_VELOCITY);
        iint->setInteractionMode(3,VOCAB_IM_COMPLIANT);

//        for (int i=3; i<nj; i++)
//        {
//            ictrl->setControlMode(i,VOCAB_CM_IDLE);
//            iint->setInteractionMode(i,VOCAB_IM_COMPLIANT);
//        }

        printf("Wait for encoders");
        while (!encs->getEncoders(encoders.data()))
        {
            Time::delay(0.1);
            printf(".");
        }
        printf("\n");

        printf("Right Arm initialized.\n");

        string portName = "/" + robot+"/" + part + "/command:i";
        while (!Network::connect(portInDataName,portName));
        {
            cout << "Connecting left arm port..." << endl;
            Time::delay(0.2);
        }

        portName = "/" + robot+"/" + part + "/state:o";
        while (!Network::connect(portName,portReadOutDataName))
            {
                cout << "Connecting right arm port..." << endl;
                Time::delay(0.2);
            }


        return true;
}




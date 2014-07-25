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

bodySchema::bodySchema(ResourceFinder &rf)
{
/*    iCurrentInstance = -1;
    sCurrentPronom = "none";
    sCurrentActivity = "none";
    psCurrentComplement.first = "none";
    psCurrentComplement.second = "none";
    */
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

    fileIn = rf.findFile("inputs.txt").c_str();
    fileOut = rf.findFile("outputs.txt").c_str();

    nInputs = 1;
    nOutputs = 2;



    /* Create PolyDriver */

    Property option;

    string portnameArm = "right_arm";
    option.put("robot", "icubSim"); // typically from the command line.
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
    pos->getAxes(&nj);
    vel->getAxes(&nj);
    encoders.resize(nj);
    cmd.resize(nj);

    for (int i=0; i<nj; i++)
    {
        ictrl->setPositionMode(i);
        iint->setInteractionMode(i,VOCAB_IM_COMPLIANT);
    }

    printf("Wait for encoders");
    while (!encs->getEncoders(encoders.data()))
    {
        Time::delay(0.1);
        printf(".");
    }
    printf("\n");

    printf("Right Arm initialized.\n");




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


    // Connect to /icub/right_arm/state:o
    while (!Network::connect(portInDataName,"/icubSim/right_arm/command:i"))
    {
        cout << "Connecting right arm port..." << endl;
        Time::delay(0.2);
    }
    while (!Network::connect("/icubSim/right_arm/state:o",portReadInDataName))
    {
        cout << "Connecting right arm port..." << endl;
        Time::delay(0.2);
    }


//    Network::connect(portInDataName,portReadInDataName);
//    Network::connect(portOutDataName,portReadOutDataName);
    Network::connect(portReadInDataName,portReadOutDataName);






    attach(handlerPort);                  // attach to port

/*
    //------------------------//
    //      iCub Client
    //------------------------//
    iCub = new ICubClient(moduleName.c_str(),"bodySchema/conf","client.ini",true);
    iCub->say("test!",false);
    //  iCub->opc->isVerbose = false;

    // Connect iCub Client, and ports
    //      bEveryThingisGood &= !iCub->connect()

    bEveryThingisGood &= Network::connect(port2abmName.c_str(), "/autobiographicalMemory/request:i");

    if (!bEveryThingisGood)
        cout << endl << "Some dependencies are not running (ICubClient or port(s) connections)"<<endl<<endl;
    else
        cout << endl << endl << "----------------------------------------------" << endl << endl << "bodySchema ready !" << endl << endl;

*/
    return bEveryThingisGood ;
}

bool bodySchema::interruptModule() {
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

//    iCub->close();
//    delete iCub;

    return true;
}

bool bodySchema::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) +
            " commands are: \n" +
            "learn \n" +
            "help \n" +
            "quit \n";

    reply.clear();

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="learn") {
        isLearning = true;
//        learn(fileIn,fileOut);
        initTime = Time::now();
        cout << helpMessage;
        reply.addString("ok");
    }


    return true;
}

/* Called periodically every getPeriod() seconds */
bool bodySchema::updateModule() {

    if(isLearning)
        learn(fileIn,fileOut);
////        learn(nInputs,nOutputs);

    return true;
}

double bodySchema::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}



//bool bodySchema::generateMovs(int& iters)
//{

//    VectorXd input(1);  //the 1 is the size of the vector
//    VectorXd output(1);

//    for (unsigned int i=0; i<iters; i++) {

//        //create the input and output
//        input(0) = sin(i*0.01);
//        output(0) = sin((i+1)*0.01);

//        Bottle& inputB = portInData.prepare(); // Get the object
//        inputB.addDouble(input(0));
//        portInData.write(); // Now send it on its way

//        Bottle& outputB = portOutData.prepare(); // Get the object
//        outputB.addDouble(output(0));
//        portOutData.write(); // Now send it on its way

//    }

//    return true;
//}




/*
*   Learning
*
*/


bool bodySchema::learn(string& fileNameIn, string& fileNameOut)
{

    cout << "\n\nGoing to the learn method...\n\n" <<endl;




    //Create our OESGP object
    OESGP oesgp;

    //Problem parameters
    //we want to predict a simple sine wave
    int input_dim = 1;
    int output_dim = 1;

    //Reservoir Parameters
    //you can change these to see how it affects the predictions
    int reservoir_size = 100;
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
    double noise = 0.01;
    double epsilon = 1e-3;
    int capacity = 200;

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
        unsigned int max_itr = 10000000; // TO MODIFY!!!!
        unsigned int iters = max_itr;

        //note that we use Eigen VectorXd objects
        //look at http://eigen.tuxfamily.org for more information about Eigen
        VectorXd input(1);  //the 1 is the size of the vector
        VectorXd output(1);

        VectorXd state;
        VectorXd prediction;
        VectorXd prediction_variance;

        VectorXd in(1);  //the 1 is the size of the vector
        VectorXd out(1);

        for (unsigned int i=0; i<iters; i++) {

            //create the input and output
            in(0) = -90+10*sin(i*0.01);
            out(0) = -90+10*sin((i+1)*0.01);

            Bottle& inputB = portInData.prepare(); // Get the object
            inputB.addDouble(in(0));
            portInData.write(); // Now send it on its way

            cmd[0]=in(0);
            cmdRightArm=in(0);
            cout << "Moving right arm" << endl;
            cout << cmdRightArm << endl;
            pos->positionMove(0,cmdRightArm);//cmd.data());

//            Bottle& outputB = portOutData.prepare(); // Get the object
//            outputB.addDouble(out(0));
//            portOutData.write(); // Now send it on its way

////        }

////        for (unsigned int i=0; i<max_itr; i++) {

////            //create the input and output
////            input(0) = sin(i*0.01);
////            output(0) = sin((i+1)*0.01);

//            // Read input and output from ports
//            Bottle *inB = portReadInData.read();
//            Bottle *outB = portReadOutData.read();

//            if (!inB->isNull())
//                input(0) = inB->get(i).asDouble();
//            if (!outB->isNull())
//                output(0) = outB->get(i).asDouble();

//            cout << "Going to update OESGP..." << endl;
//            //update the OESGP with the input
//            oesgp.update(input);

//            cout << "Going to predict OESGP..." << endl;
//            //predict the next state
//            oesgp.predict(prediction, prediction_variance);

//            //print the error
//            double error = (prediction - output).norm();
//            cout << "Error: " << error << ", |BV|: "
//                 << oesgp.getCurrentSize() <<  endl;

//            //train with the true next state
//            oesgp.train(output);
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
            input(0) = -90+10*sin(i*0.01);
            output(0) = -90+10*sin((i+1)*0.01);

            //update
            oesgp2.update(input);

            //predict
            oesgp2.predict(prediction, prediction_variance);
            double error = (prediction - output).norm();
            cout << "Error: " << error << endl;


            Bottle& predB = portPredictions.prepare(); // Get the object
            predB.addDouble(prediction.value());
            portPredictions.write(); // Now send it on its way

            Bottle& errB = portPredictionErrors.prepare(); // Get the object
            errB.addDouble(error);
            portPredictionErrors.write(); // Now send it on its way

        }

    } catch (OTLException &e) {
        e.showError();
    }





    return true;


}




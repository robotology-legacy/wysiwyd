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

#include "bodySchema.h"

#include <cmath>
#include <signal.h>
#include <vector>
#include <iostream>


using namespace OTL;
using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

bool bodySchema::configure(yarp::os::ResourceFinder &rf) {
    bool bEveryThingisGood = true;

    moduleName = rf.check("name",Value("bodySchema"),"module name (string)").asString();
    state = idle;

    part = rf.check("part",Value("left_arm")).asString();
    robot = rf.check("robot",Value("icubSim")).asString();
    fps = rf.check("fps",Value(30)).asInt(); //30;

    Bottle &start_pos = rf.findGroup("start_position");
    start_commandHead[0] = start_pos.check("cmdH0", Value(-25.0)).asDouble();
    start_commandHead[1] = start_pos.check("cmdH1", Value(-15.0)).asDouble();
    start_commandHead[2] = start_pos.check("cmdH2", Value(0.0)).asDouble();
    start_commandHead[3] = start_pos.check("cmdH3", Value(0.0)).asDouble();
    start_commandHead[4] = start_pos.check("cmdH4", Value(-20.0)).asDouble();

    start_command[0] = start_pos.check("cmdL0", Value(-40.0)).asDouble();
    start_command[1] = start_pos.check("cmdL1", Value(20.0)).asDouble();
    start_command[2] = start_pos.check("cmdL2", Value(20.0)).asDouble();
    start_command[3] = start_pos.check("cmdL3", Value(60.0)).asDouble();
    start_command[4] = start_pos.check("cmdL4", Value(-60.0)).asDouble();
    start_command[5] = start_pos.check("cmdL5", Value(-10.0)).asDouble();
    start_command[6] = start_pos.check("cmdL6", Value(10.0)).asDouble();
    start_command[7] = start_pos.check("cmdL7", Value(18.0)).asDouble();
    start_command[8] = start_pos.check("cmdL8", Value(30.0)).asDouble();
    start_command[9] = start_pos.check("cmdL9", Value(4.0)).asDouble();
    start_command[10] = start_pos.check("cmdL10", Value(2.0)).asDouble();
    start_command[11] = start_pos.check("cmdL11", Value(4.0)).asDouble();
    start_command[12] = start_pos.check("cmdL12", Value(7.0)).asDouble();
    start_command[13] = start_pos.check("cmdL13", Value(14.0)).asDouble();
    start_command[14] = start_pos.check("cmdL14", Value(5.0)).asDouble();
    start_command[15] = start_pos.check("cmdL15", Value(33.0)).asDouble();

    Bottle &babbl_par = rf.findGroup("babbling_param");
    freq1 = babbl_par.check("freq1", Value(0.3)).asDouble();
    freq2 = babbl_par.check("freq2", Value(0.5)).asDouble();
    freq3 = babbl_par.check("freq3", Value(0.2)).asDouble();
    freq4 = babbl_par.check("freq4", Value(0.1)).asDouble();
    amp0 = babbl_par.check("amp0", Value(6)).asDouble();
    amp1 = babbl_par.check("amp1", Value(4)).asDouble();
    amp2 = babbl_par.check("amp2", Value(7)).asDouble();
    amp3 = babbl_par.check("amp3", Value(6)).asDouble();
    amp4 = babbl_par.check("amp4", Value(12)).asDouble();
    amp8 = babbl_par.check("amp8", Value(-40)).asDouble();
    amp9 = babbl_par.check("amp9", Value(-40)).asDouble();
    amp11 = babbl_par.check("amp11", Value(-20)).asDouble();
    amp12 = babbl_par.check("amp12", Value(-20)).asDouble();
    amp13 = babbl_par.check("amp13", Value(-20)).asDouble();
    amp14 = babbl_par.check("amp14", Value(-20)).asDouble();
    amp15 = babbl_par.check("amp15", Value(-20)).asDouble();
    ampcos2 = babbl_par.check("ampcos2", Value(0.0)).asDouble();
    train_duration = babbl_par.check("train_duration", Value(10.0)).asDouble();
    test_duration = babbl_par.check("test_duration", Value(5.0)).asDouble();

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

    // keep this part if you want to use both cameras
    // if you only want to use want, delete accordingly
    if (robot=="icub")
    {
        ports[0] = "/"+robot+"/camcalib/left/out";
        ports[1] = "/"+robot+"/camcalib/right/out";
    }
    else //if (robot=='icubSim')
    {
        ports[0] = "/"+robot+"/cam/left";
        ports[1] = "/"+robot+"/cam/right";
    }
    ports[2] = "/"+moduleName+"/camleft";
    ports[3] = "/"+moduleName+"/camright";

    video[0] = "videoL.avi";
    video[1] = "videoR.avi";

    state = idle;
    shouldQuit = false;

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

    if (!portToABM.open("/" + getName() + "/toABM")) {
        cout << getName() << ": Unable to open port " << "/" + getName() + "/toABM" << endl;
        bEveryThingisGood = false;
    }


//    if (!portToSFM.open("/" + getName() + "/toSFM")) {
//        cout << getName() << ": Unable to open port " << "/" + getName() + "/toSFM" << endl;
//        bEveryThingisGood = false;
//    }





    Network::connect(portToABM.getName(), "/autobiographicalMemory/rpc");
    if(!Network::isConnected(portToABM.getName(), "/autobiographicalMemory/rpc")){
        cout << endl << "WARNING : Cannot connect to ABM, storing data into it will not be possible unless manual connection" << endl << endl;
    } else {
        cout << "Connected to ABM : Data (especially in Winter) are coming!" << endl;
    }




    // Initialize iCub and Vision
    while (!init_iCub(part)) {
        cout << getName() << ": initialising iCub... please wait... " << endl;
        bEveryThingisGood = false;
    }

    cout << "End configuration..." <<endl;
    cout << "Current state : " << state <<endl;

    attach(handlerPort);

    string welcomeMessage =
            "\n + + + + + + + + + + + + + + + + + + + + + + + + + \n"
            "This is " + getName() + " module. With this module, the iCub will perform\n"
            "motor babbling with its left arm, while learning the corresponding\n"
            "forward model online.\n\n"
            "Then, in another terminal run `yarp rpc /bodySchema/rpc' to open the bodySchema rpc port.\n"
            "Launch the `babbling arm' command to perform learning from motor babbling. \n"
            "The complete list of commands follows: \n"
            "babbling arm \n"
            "babbling hand \n"
            "help \n"
            "quit \n";

    cout << welcomeMessage;

    return bEveryThingisGood;
}

bool bodySchema::interruptModule() {

    bool homeStart = goStartPos();
    if(!homeStart) {
        cout << "I got lost going home!" << endl;
    }

    imgPortIn.interrupt();
    imgPortOut.interrupt();
    portVelocityOut.interrupt();
    portPredictionErrors.interrupt();
    portToABM.interrupt();
    handlerPort.interrupt();

    cout << "Bye!" << endl;

    return true;

}

bool bodySchema::close() {
    cout << "Closing module, please wait ... " <<endl;

    bool homeStart = goStartPos();
    if(!homeStart) {
        cout << "I got lost going home!" << endl;
    }

    armDev->close();
    headDev->close();

    imgPortIn.interrupt();
    imgPortIn.close();

    imgPortOut.interrupt();
    imgPortOut.close();

    portVelocityOut.interrupt();
    portVelocityOut.close();

    portPredictionErrors.interrupt();
    portPredictionErrors.close();

    portToABM.interrupt();
    portToABM.close();

    handlerPort.interrupt();
    handlerPort.close();

    cout << "Bye!" << endl;

    return true;
}

bool bodySchema::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =
            "\n + + + + + + + + + + + + + + + + + + + + + + + + + \n"
            "This is " + getName() + " module. With this module, the iCub will perform\n"
            "motor babbling with its left arm, while learning the corresponding\n"
            "forward model online.\n\n"
            "Launch the `babbling arm' command to perform learning from\n"
            "motor babbling. \n\n"
            "All commands are: \n"
            "babbling arm \n"
            "babbling hand \n"
            "help \n"
            "quit \n";

    reply.clear();

    if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    } else if (command.get(0).asString()=="quit") {
        shouldQuit = true;
        reply.addString("closing");
    }
    else if (command.get(0).asString()=="babbling") {
        if (state==idle)
        {
            if (command.size()==2)
            {

                if (command.get(1).asString()=="arm")
                {
                    state = babblingArm;
                    cout << "... state : " << state <<endl;
                    reply = dealABM(command,1);

                    //check ABM reply
                    if (reply.isNull()) {
                        cout << "Reply from ABM is null : NOT connected?" << endl;
                    } else if (reply.get(0).asString()!="ack"){
                        cout << reply.toString() << endl;
                    }

                    reply.clear();
                    learnAbsPos(state); //or learn(), or learnChangePos()
                    reply = dealABM(command,0);

                    //check ABM reply
                    if (reply.isNull()) {
                        cout << "Reply from ABM is null : NOT connected?" << endl;
                    } else if (reply.get(0).asString()!="ack"){
                        cout << reply.toString() << endl;
                    }
                    cout << "Finish babbling arm..." << endl;
                    state = idle;
                    cout << "... state : " << state <<endl;
                    cout << helpMessage;
                    reply.addString("ack");
                }
                else if (command.get(1).asString()=="hand")
                {
                    state = babblingHand;
                    cout << "... state : " << state <<endl;
                    reply = dealABM(command,1);
                    cout << reply.toString() << endl;
                    reply.clear();
                    learnAbsPos(state); //or learn(), or learnChangePos()
                    reply = dealABM(command,0);
                    cout << reply.toString() << endl;

                    cout << "Finish babbling hand..." << endl;
                    state = idle;
                    cout << "... state : " << state <<endl;
                    cout << helpMessage;
                    reply.addString("ack");
                }
                else
                    cout << "Argument 2 not appropriate!...Cup of tea?" << endl;
            }
            else {
                cout << "Argument missing!" << endl;
            }

        }
        else {
            reply.addString("nack");
        }
    }
    else if (command.get(0).asString()=="singleJointBabbling") {
        if (state==idle)
        {
            if (command.size()==2)
            {
                int joint_index = command.get(1).asInt();

                state = vvv2015;
                yInfo() << "... state: " << state;
                Bottle abmReply = dealABM(command,1);

                //check ABM reply
                if (abmReply.isNull()) {
                    yWarning() << "Reply from ABM is null NOT connected?";
                } else if (abmReply.get(0).asString()!="ack"){
                    yWarning() << "Response from ABM: " << abmReply.toString();
                }

                abmReply.clear();
                singleJointBabbling(joint_index);
                abmReply = dealABM(command,0);

                //check ABM reply
                if (abmReply.isNull()) {
                    yWarning() << "Reply from ABM is null NOT connected?";
                } else if (abmReply.get(0).asString()!="ack"){
                    yWarning() << "Response from ABM: " << abmReply.toString();
                }

                yInfo() << "Finish singleJointBabbling";
                state = idle;
                yDebug() << "... state : " << state;
                yInfo() << helpMessage;
                reply.addString("ack");
            }
            else {
                yError() << "Argument missing! singleJointBabbling jointNumber";
                reply.addString("nack");
            }
        }
        else {
            yError() << "Still busy with previous babbling action, try again later.";
            reply.addString("nack");
        }
    }
    else {
        yError() << "Unknown command!";
        yInfo() << helpMessage;
        reply.addString("nack");
    }

    return true;
}

bool bodySchema::updateModule() {
    if(shouldQuit && state == idle) {
        return false;
    } else {
//        joint_index = 11; // 13; // 9; // 15;
//        VVV2015(joint_index);
//        close();
        return true;
    }
}

double bodySchema::getPeriod() {
    return 0.1;
}


/*
 *
 *
 */
Bottle bodySchema::dealABM(const Bottle& command, int begin)
{
    yDebug() << "Dealing with ABM";
    if (begin<0 || begin>1)
    {
        yError() << "begin parameter must be 1 or 0.";
        Bottle bError;
        bError.addString("nack");
        bError.addString("Error: begin item should be either 1 or 0.");
        return bError;
    }

    Bottle bABM, bABMreply;
    bABM.addString("snapshot");
    Bottle bSubMain;
    bSubMain.addString("action");
    bSubMain.addString("babbling");
    bSubMain.addString("action");
    Bottle bSubArgument;
    bSubArgument.addString("arguments");
    Bottle bSubSubArgument;
    bSubSubArgument.addString(command.get(1).asString());
    bSubSubArgument.addString("limb");
    Bottle bSubSubArgument2;
    bSubSubArgument2.addString(part);
    bSubSubArgument2.addString("side");
    Bottle bSubSubArgument3;
    bSubSubArgument3.addString(robot);
    bSubSubArgument3.addString("agent1");
    Bottle bBegin;
    bBegin.addString("begin");
    bBegin.addInt(begin);

    bABM.addList() = bSubMain;
    bSubArgument.addList() = bSubSubArgument;
    bSubArgument.addList() = bSubSubArgument2;
    bSubArgument.addList() = bSubSubArgument3;
    bABM.addList() = bSubArgument;
    bABM.addList() = bBegin;

    portToABM.write(bABM,bABMreply);

    return bABMreply;
}

/*
 * Learning while babbling
 * This learns absolute positions
 */
bool bodySchema::learnAbsPos(State &state)
{
    cout << "Learning from babbling; iCub required." << endl;

    // First go to home position
    bool homeStart = goStartPos();
    if(!homeStart) {
        cout << "I got lost going home!" << endl;
    }

    while(!Network::isConnected(ports[0], imgPortIn.getName())) {
        Network::connect(ports[0], imgPortIn.getName());
        cout << "Waiting for port " << ports[0] << " to connect to " << imgPortIn.getName() << endl;
        Time::delay(1.0);
    }

    // this needs to be a configuration option
    double startTime = yarp::os::Time::now();

    videoName = "video";
    MAX_COUNT = 150;

    bool createfolders = create_folders();
    if(!createfolders) {
        cout << "Error creating folders" << endl;
    }
    ofstream fs_enc(fileEncData.c_str());
    if(!fs_enc) {
        std::cerr<<"Cannot open the output file 'encData' ."<<std::endl;
        return 0;
    }
    ofstream fs_cmd(fileCmdData.c_str());
    if(!fs_cmd) {
        std::cerr<<"Cannot open the output file 'cmdData' ."<<std::endl;
        return 0;
    }

    capseq = 1;

    Point2f lost_indic(-1.0f, -1.0f);

    num_init_points = 0;

    source_window = "image";
    frame_idx = 0;

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,50, 0.3);//75,0.0001);//20, 0.03);//
    Size subPixWinSize(10,10), winSize(31,31);

    init_oesgp_learner();

    VectorXd input(1);
    VectorXd output(1);
    VectorXd input_test(1);

    double AOD = rand() / double(RAND_MAX);
    yarp::sig::Vector babCmd;


    while (Time::now() < startTime + train_duration){
        double t = Time::now() - startTime;
        cout << endl << " Time: " << t << " / " << train_duration << endl;


        if (state == babblingArm) {
            babCmd = babblingExecution(t,AOD);
        } else if (state == babblingHand) {
            babCmd = babblingHandExecution(t);
        } else {
            cout << "Error: state not recognised." << endl;
        }

        input(0) = babCmd[0];

        bool babImg = getBabblingImages();
        if(!babImg) {
            cout << "Error getting images while babbling" << endl;
        }
        bool findFeat = findFeatures(termcrit, subPixWinSize, winSize);
        if(!findFeat) {
            cout << "Error finding features" << endl;
        }

        //        cout << "Write encoder data" << endl;
        bool writeEncData = writeEncoderData(lost_indic, fs_enc, fs_cmd);
        if(!writeEncData) {
            cout << "Error writing encoder data" << endl;
        }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);

        //update learner
        oesgp.update(input);

        //predict the next state
        oesgp.predict(prediction, prediction_variance);

        output(0) = encoders[0];

        double error[1];
        error[0] = (prediction(0) - output(0));
        //        cout << "Error: " << error[0] << ", |BV|: " << oesgp.getCurrentSize() <<  endl;

        //train with the true next state
        oesgp.train(output);

        capseq++;
    }

    /* Validation */
    cout << "Testing saving and loading model " << std::endl;

    oesgp.save("oesgptest");
    oesgp2.load("oesgptest");

    int sum=0;
    int Sqno=0;
    int count=0;

    double startTime_test = yarp::os::Time::now();

    while (Time::now() < startTime_test  + test_duration) {
        double t = Time::now() - startTime_test;
        cout << endl << " Time: " << t << " / "<< test_duration << endl;

        double AOD = 1;
        yarp::sig::Vector babCmd_test = babblingExecution(t,AOD);
        input_test(0) = babCmd_test[0];

        //update
        oesgp2.update(input_test);

        //predict
        oesgp2.predict(prediction, prediction_variance);

        VectorXd output(1);
        output(0) = encoders[0];

        double error[1], rmse[1];
        error[0] = (prediction(0) - output(0));

        count++;
        Sqno=pow(error[0],2);
        sum=sum+Sqno;

        rmse[0] = sqrt(sum/count); //rmse(output(0),prediction(0));//
        cout << "Error: " << error[0] << endl;
        cout << "RMSE: " << rmse[0] << endl;
        Bottle& errB = portPredictionErrors.prepare(); // Get the object
        errB.clear();
        errB.addDouble(rmse[0]);
        portPredictionErrors.write(); // Now send it on its way
    }

    bool homeEnd = goStartPos();
    if(!homeEnd) {
        cout << "I got lost going home!" << endl;
    }

    Network::disconnect(ports[0], imgPortIn.getName());

    destroyWindow(source_window);

    return true;
}

const std::string bodySchema::trail_string()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M", &tstruct);

    return buf;
}

bool bodySchema::create_folders()
{
    string tr = trail_string();
    foldername = "data"+tr+"/";
    yarp::os::mkdir(foldername.c_str());
    foldernamepoints = foldername + "points/";
    yarp::os::mkdir(foldernamepoints.c_str());
    foldernamevideo = foldernamepoints + videoName;
    yarp::os::mkdir(foldernamevideo.c_str());
    foldernameframes = foldername + "frames";
    yarp::os::mkdir(foldernameframes.c_str());
    fileEncData = foldername + "encData.txt";
    fileCmdData = foldername + "cmdData.txt";

    return true;
}

yarp::sig::Vector bodySchema::babblingExecution(double &t, double &AOD)
{


    for(int i=0; i<16; i++)
    {
        ictrl->setControlMode(i,VOCAB_CM_VELOCITY);
    }

    double w1 = freq1*t;
    double w2 = freq2*t;
    double w3 = freq3*t;
    double w4 = freq4*t;



    for (unsigned int l=0; l<command.size(); l++) {
        command[l]=0;
    }
    command[0]=cos(w1 * 2 * M_PI)+ampcos2*cos(w4 * 2 * M_PI);//amp0*AOD;//
    command[1]=cos(w1 * 2 * M_PI)+ampcos2*cos(w4 * 2 * M_PI);//amp1*AOD;//
    command[2]=cos(w1 * 2 * M_PI)+ampcos2*cos(w4 * 2 * M_PI);//amp2*AOD;//
    command[3]=cos(w2 * 2 * M_PI)+ampcos2*cos(w4 * 2 * M_PI);//amp3*AOD;//
    command[6]=cos(w3 * 2 * M_PI)+ampcos2*cos(w4 * 2 * M_PI);//amp4*AOD;//



    //cout << "Sending bottle" << endl;
    Bottle& inDataB = portVelocityOut.prepare(); // Get the object
    inDataB.clear();
    for  (unsigned int l=0; l<command.size(); l++)
    {
        inDataB.addDouble(command[l]);
    }
    portVelocityOut.write();

    //cout << "Do velocity move" << endl;

    vel->velocityMove(command.data());

    //cout << "Move done" << endl;
    return command;
}

yarp::sig::Vector bodySchema::babblingHandExecution(double &t)
{


    for(int i=0; i<16; i++)
    {
        ictrl->setControlMode(i,VOCAB_CM_VELOCITY);
    }


        double w1 = freq1*t;
        double w2 = freq2*t;
        double w3 = freq3*t;
        double w4 = freq4*t;

        for (unsigned int l=0; l<command.size(); l++) {
            command[l]=0;
        }
        command[6]=amp4*cos(w3 * 2 * M_PI)+ampcos2*cos(w4 * 2 * M_PI);
        command[8]=amp8*cos(w1 * 2 * M_PI);//-40*amp*cos(0.05 * t * 2 * M_PI)+0*cos(1*t * 2 * M_PI);
        command[9]=amp9*cos(w1 * 2 * M_PI);//-40*amp*cos(0.05 * t * 2 * M_PI)+0*cos(1*t * 2 * M_PI);
        command[11]=amp11*cos(w4 * 2 * M_PI);//-20*amp*cos(0.05 * t * 2 * M_PI)+0*cos(1*t * 2 * M_PI);
        command[13]=amp13*cos(w4 * 2 * M_PI);//-20*amp*cos(0.05 * t * 2 * M_PI)+0*cos(1*t * 2 * M_PI);
        command[15]=amp15*cos(w4 * 2 * M_PI);//-20*amp*cos(0.05 * t * 2 * M_PI)+0*cos(1*t * 2 * M_PI);
        command[12]=amp12*cos(w2 * 2 * M_PI);//-20*amp*cos(0.05 * t * 2 * M_PI)+0*cos(1*t * 2 * M_PI);
        command[14]=amp14*cos(w2 * 2 * M_PI);//-20*amp*cos(0.05 * t * 2 * M_PI)+0*cos(1*t * 2 * M_PI);

        //cout << "Sending bottle" << endl;
        Bottle& inDataB = portVelocityOut.prepare(); // Get the object
        inDataB.clear();
        for  (unsigned int l=0; l<command.size(); l++)
        {
            inDataB.addDouble(command[l]);
        }
        portVelocityOut.write();

        //cout << "Do velocity move" << endl;

        vel->velocityMove(command.data());


    /*
     * ****************************************************************************
    cout << "Set control mode" << endl;
    for(int i=0; i<16; i++) {
        ictrl->setControlMode(i,VOCAB_CM_POSITION);
        pos->setRefSpeed(i, 20);
    }

    // increase speed for finger joints
    for(int i=8; i<16; i++) {
        pos->setRefSpeed(i, 35);
    }
    pos->setRefSpeed(15, 45);

    int tmp = (int)round(t/3)%3;
    for (int i=0; i<=6; i++) {
        command[i]=start_command[i];
    }
    //command[5]=10;

    switch (tmp)
    {
    case 0:
    {
        command[6]=-2; 	command[7]=35;  command[8]=20; 	command[9]=20; 	command[10]=20; command[11]=20;
        command[12]=40; command[13]=40; command[14]=60; command[15]=100;
        break;
    }
    case 1:
    {
        command[6]=2; 	command[7]=0; 	command[8]=30; 	command[9]=0; 	command[10]=50; command[11]=40;
        command[12]=60; command[13]=20; command[14]=10; command[15]=225;
        break;
    }
    case 2:
    {
        command[6]=5; 	command[7]=10; 	command[8]=25; 	command[9]=10; 	command[10]=15; command[11]=10;
        command[12]=20; command[13]=30; command[14]=30; command[15]=180;
        break;
    }
    }

    cout << command.toString() << endl;

    pos->positionMove(command.data());
    * ****************************************************************************
    */

    return command;
}




bool bodySchema::goStartPos()
{
    /* Move head to start position */
    commandHead = encodersHead;
    commandHead[0]=start_commandHead[0];
    commandHead[1]=start_commandHead[1];
    commandHead[2]=start_commandHead[2];
    commandHead[3]=start_commandHead[3];
    commandHead[4]=start_commandHead[4];
    posHead->positionMove(commandHead.data());

    /* Go to start position */
    //cout << "Start position move" << endl;

    //cout << "Set position control mode" << endl;
    command = encoders;
    for(int i=0; i<16; i++)
    {
        ictrl->setControlMode(i,VOCAB_CM_POSITION);
        command[i]=start_command[i];
    }
    pos->positionMove(command.data());

    bool done_head=false;
    bool done_arm=false;
    while (!done_head || !done_arm) {
        cout << "Wait for position moves to finish" << endl;
        posHead->checkMotionDone(&done_head);
        pos->checkMotionDone(&done_arm);
        Time::delay(0.04);
    }

    Time::delay(1.0);

    return true;
}

bool bodySchema::writeEncoderData(Point2f &lost_indic,ofstream& fs_enc,ofstream& fs_cmd)
{
    bool okEnc = encs->getEncoders(encoders.data());
    if(!okEnc) {
        cerr << "Error receiving encoders" << endl;
    }
    else
    {
        double time = Time::now();
        fs_enc << fixed << time << "\t ";
        fs_cmd << fixed << time << "\t ";

        for (unsigned int kk=0; kk<command.length(); kk++){
            fs_enc << encoders[kk] <<"\t ";
            fs_cmd << command[kk] <<"\t ";
        }

        fs_enc <<"\n ";
        fs_cmd <<"\n ";

        char of_str_p[300];
        frame_idx += 1;

        sprintf(of_str_p,"%s/point_seq_%d.txt",foldernamepoints.c_str(),frame_idx);
        ofstream fs_p(of_str_p);
        if(!fs_p) {
            std::cerr<<"Cannot open the output file for 'points' ."<<std::endl;
            return 1;
        }
        int j, k;
        for(j=k=0 ; j < num_init_points; j++ ) {
            if(j == points_idx[k])
            {
                fs_p << points[1][k] << endl;
                k++;
            } else {
                fs_p << lost_indic << endl;
            }
        }
        fs_p.close();
    }

    //cout << "Write encoder data done" << endl;
    return true;
}

bool bodySchema::findFeatures(TermCriteria &termcrit, Size &subPixWinSize, Size &winSize)
{
    if (capseq==1) //or points[0].empty() )
    {
        Mat copy;
        copy = image.clone();

        // automatic initialization
        goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.003, 3, Mat(), 3, 0, 0.04);
        cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
        needToInit = false;
        /// Draw corners detected
        int radius = 4;
        for(unsigned int i = 0; i < points[1].size(); i++ ) {
            circle( copy, points[1][i], radius, Scalar(0,255,0), -1, 8);
        }
        namedWindow( source_window, CV_WINDOW_AUTOSIZE );
        imshow( source_window, copy );

        for(size_t i=0; i<points[1].size(); i++)
        {
            points_idx.push_back(i);
        }
        num_init_points = points[1].size();

        //cout << "Number of points " << num_init_points << endl;
    } else {
        Mat copy;
        copy = image.clone();

        vector<uchar> status;
        vector<float> err;
        if(prevGray.empty())
            gray.copyTo(prevGray);
        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                3, termcrit, 0, 0.001);

        size_t i, k;
        for( i = k = 0; i < points[1].size(); i++ )
        {
            if( !status[i] ) {
                continue;
            }

            points_idx[k] = points_idx[i];
            points[1][k++] = points[1][i];

            circle( copy, points[1][i], 2, Scalar(0,255,0), -1, 8);
        }
        points[1].resize(k);
        points_idx.resize(k);

        imshow( source_window, copy );

        // convert Mat to YARP image
        Mat toYarp(copy);
        cvtColor(toYarp, toYarp, CV_BGR2RGB);
        IplImage* imageIpl = new IplImage(toYarp);
        ImageOf<PixelRgb> &imageYarp = imgPortOut.prepare();
        imageYarp.resize(imageIpl->width, imageIpl->height);
        cvCopyImage(imageIpl, (IplImage *)imageYarp.getIplImage());

        // send YARP image to port
        imgPortOut.write();
    }

    return true;
}

bool bodySchema::getBabblingImages()
{
    //    cout << "Start babbling images" << endl;
    cvWaitKey(1000/fps);

    ImageOf<PixelRgb> *yarpImage = imgPortIn.read();
    IplImage *cvImage = cvCreateImage(cvSize(yarpImage->width(), yarpImage->height()), IPL_DEPTH_8U, 3);
    cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage, CV_RGB2BGR);
    //    cout << "Conversion done" << endl;

    char framecap[256];
    sprintf(framecap, "%s/left%05d.tif", foldernameframes.c_str(), capseq);
    cvSaveImage(framecap, cvImage);

    cvReleaseImage(&cvImage);

    //cout << "Write image done" << endl;

    image = imread( framecap, 1 );
    if(!image.data) {
        cout << "Error!!!" << endl;
        return false;
    }

    cvtColor( image, gray, CV_BGR2GRAY );

    return true;
}

bool bodySchema::init_oesgp_learner()
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

bool bodySchema::init_iCub(string &part)
{
    /* Create PolyDriver for left arm */
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
    cout << sA << endl;
    option.put("remote", sA.c_str());

    cout<<"DEBUG "<< option.toString().c_str() <<endl;

    armDev = new PolyDriver(option);
    if (!armDev->isValid()) {
        cout << "Device not available.  Here are the known devices:" << endl;
        cout << Drivers::factory().toString() << endl;
        Network::fini();
        return false;
    }

    armDev->view(pos);
    armDev->view(vel);
    armDev->view(encs);
    armDev->view(ictrl);
    armDev->view(iint);
    armDev->view(itrq);

    if (pos==NULL || encs==NULL || vel==NULL || ictrl==NULL || iint==NULL  || itrq==NULL ){
        cout << "Cannot get interface to robot device" << endl;
        armDev->close();
    }

    if (encs==NULL)
        cout << "...ENCS NULL..." << encs << endl;
    cout << "......" << armDev << endl;

    int nj = 0;
    pos->getAxes(&nj);
    vel->getAxes(&nj);
    encs->getAxes(&nj);
    encoders.resize(nj);
    cmd.resize(nj);

    cout << "Wait for arm encoders" << endl;
    while (!encs->getEncoders(encoders.data()))
    {
        Time::delay(0.1);
        cout << "." << endl;
    }
    cout << endl << endl;

    cout << "Arm initialized." << endl;

    /* Init. head */
    string portnameHead = "head";
    option.put("robot", robot.c_str()); //"icub"); // typically from the command line.
    option.put("device", "remote_controlboard");
    Value& robotnameHead = option.find("robot");

    string sH("/");
    sH += robotnameHead.asString();
    sH += "/";
    sH += portnameHead.c_str();
    sH += "/control";
    option.put("local", sH.c_str());

    sH.clear();
    sH += "/";
    sH += robotnameHead.asString();
    sH += "/";
    sH += portnameHead.c_str();
    option.put("remote", sH.c_str());

    headDev = new PolyDriver(option);
    if (!headDev->isValid()) {
        cout << "Device not available.  Here are the known devices:" << endl;
        cout << Drivers::factory().toString() << endl;
        Network::fini();
        return false;
    }

    headDev->view(posHead);
    headDev->view(velHead);
    headDev->view(encsHead);
    if (posHead==NULL || encsHead==NULL || velHead==NULL ){
        cout << "Cannot get interface to robot head" << endl;
        headDev->close();
    }
    int jnts = 0;

    posHead->getAxes(&jnts);
    velHead->getAxes(&jnts);
    encodersHead.resize(jnts);
    commandHead.resize(jnts);

    cout << "Wait for encoders (HEAD)" << endl;
    while (!encsHead->getEncoders(encodersHead.data()))
    {
        Time::delay(0.1);
        cout << ".";
    }
    cout << endl;

    cout << "Head initialized." << endl;

    /* Set velocity control for arm */
    cout << "Set velocity control mode" << endl;
    for(int i=0; i<=6; i++)
    {
        ictrl->setControlMode(i,VOCAB_CM_VELOCITY);
    }

    cout << "> Initialisation done." << endl;

    imgPortIn.open("/" + getName() + "/img:i");
    imgPortOut.open("/" + getName() + "/featureImg:o");

    Network::connect("/" + getName() + "/featureImg:o", "/yarpview/bodySchema/featureImg:i");

    return true;
}


/*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Babbling with a single joint
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

bool bodySchema::singleJointBabbling(int j_idx)
{
    // First go to home position
    bool homeStart = goStartPos();
    if(!homeStart) {
        yError() << "I got lost going home! Abort!";
        return false;
    }

    for(int i=0; i<16; i++)
    {
        ictrl->setControlMode(i,VOCAB_CM_VELOCITY);
    }

    while(!Network::isConnected(ports[0], imgPortIn.getName())) {
        Network::connect(ports[0], imgPortIn.getName());
        cout << "Waiting for port " << ports[0] << " to connect to " << imgPortIn.getName() << endl;
        Time::delay(1.0);
    }

    // this needs to be a configuration option
    double startTime = yarp::os::Time::now();

    MAX_COUNT = 150;

    capseq = 1;

    num_init_points = 0;

    source_window = "image";

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,50, 0.3);//75,0.0001);//20, 0.03);//
    Size subPixWinSize(10,10), winSize(31,31);

    while (Time::now() < startTime + train_duration){
        double t = Time::now() - startTime;
        cout << endl << " Time: " << t << " / " << train_duration << endl;

        // move the joint j_idx
        for (unsigned int l=0; l<command.size(); l++) {
            command[l]=0;
        }
        command[j_idx]=10*cos(0.1*t * 2 * M_PI);
        vel->velocityMove(command.data());

        bool babImg = getBabblingImages();
        if(!babImg) {
            yError() << "Error getting images while babbling";
        }
        bool findFeat = findFeatures(termcrit, subPixWinSize, winSize);
        if(!findFeat) {
            yError() << "Error finding features";
        }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);

        capseq++;
    }

    bool homeEnd = goStartPos();
    if(!homeEnd) {
        yError() << "I got lost going home!";
    }

    Network::disconnect(ports[0], imgPortIn.getName());

    destroyWindow(source_window);

    return true;
}

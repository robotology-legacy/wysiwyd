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

#include "babbling.h"

#include <vector>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

bool Babbling::configure(yarp::os::ResourceFinder &rf) {
    bool bEveryThingisGood = true;

    moduleName = rf.check("name",Value("babbling"),"module name (string)").asString();

//    part = rf.check("part",Value("left_arm")).asString();
    robot = rf.check("robot",Value("icubSim")).asString();
    fps = rf.check("fps",Value(30)).asInt(); //30;
    cmd_source = rf.check("cmd_source",Value("C")).asString();
    single_joint = rf.check("single_joint",Value(-1)).asInt();

    Bottle &start_pos = rf.findGroup("start_position");
    Bottle *b_start_commandHead = start_pos.find("head").asList();
    Bottle *b_start_command = start_pos.find("arm").asList();

    if ((b_start_commandHead->isNull()) | (b_start_commandHead->size()<5))
    {
        yWarning("Something is wrong in ini file. Default value is used");
        start_commandHead[0] = -25.0;
        start_commandHead[1]= -15.0;
        start_commandHead[2]= 0.0;
        start_commandHead[3]= 0.0;
        start_commandHead[4]= -20.0;
    }
    else
    {
        for(int i=0; i<b_start_commandHead->size(); i++)
            start_commandHead[i] = b_start_commandHead->get(i).asDouble();
    }

    if ((b_start_command->isNull()) | (b_start_command->size()<16))
    {
        yWarning("Something is wrong in ini file. Default value is used");
        start_command[0] = -40.0;
        start_command[1] = 25.0;
        start_command[2] = 20.0;
        start_command[3] = 85.0;
        start_command[4] = -50.0;
        start_command[5] = 0.0;
        start_command[6] = 8.0;
        start_command[7] = 15.0;
        start_command[8] = 30.0;
        start_command[9] = 4.0;
        start_command[10] = 2.0;
        start_command[11] = 0.0;
        start_command[12] = 7.0;
        start_command[13] = 14.0;
        start_command[14] = 5.0;
        start_command[15] = 14.0;
    }
    else
    {
        for(int i=0; i<b_start_command->size(); i++)
            start_command[i] = b_start_command->get(i).asDouble();
    }


    Bottle &babbl_par = rf.findGroup("babbling_param");
    freq = babbl_par.check("freq", Value(0.2)).asDouble();
    amp = babbl_par.check("amp", Value(5)).asDouble();
    train_duration = babbl_par.check("train_duration", Value(20.0)).asDouble();

    // keep this part if you want to use both cameras
    // if you only want to use want, delete accordingly
    if (robot=="icub")
    {
        leftCameraPort = "/"+robot+"/camcalib/left/out";
        rightCameraPort = "/"+robot+"/camcalib/right/out";
    }
    else //if (robot=='icubSim')
    {
        leftCameraPort = "/"+robot+"/cam/left";
        rightCameraPort = "/"+robot+"/cam/right";
    }

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

    if (!portToABM.open("/" + getName() + "/toABM")) {
        yError() << getName() << ": Unable to open port " << "/" + getName() + "/toABM";
        bEveryThingisGood = false;
    }

    if(!Network::connect(portToABM.getName(), "/autobiographicalMemory/rpc")){
        yWarning() << "Cannot connect to ABM, storing data into it will not be possible unless manual connection";
    } else {
        yInfo() << "Connected to ABM : Data are coming!";
    }


    // Initialize iCub and Vision
    while (!init_iCub(part)) {
        cout << getName() << ": initialising iCub... please wait... " << endl;
        bEveryThingisGood = false;
    }

    for (int l=0; l<16; l++)
        ref_command[l] = 0;


    yDebug() << "End configuration...";

    attach(handlerPort);

    return bEveryThingisGood;
}

bool Babbling::interruptModule() {

    portVelocityOut.interrupt();
    handlerPort.interrupt();
    portToABM.interrupt();

    yInfo() << "Bye!";

    return true;

}

bool Babbling::close() {
    cout << "Closing module, please wait ... " <<endl;

    leftArmDev->close();
    headDev->close();

    portVelocityOut.interrupt();
    portVelocityOut.close();

    handlerPort.interrupt();
    handlerPort.close();

    portToABM.interrupt();
    portToABM.close();

    yInfo() << "Bye!";

    return true;
}

bool Babbling::respond(const Bottle& command, Bottle& reply) {

    string helpMessage =  string(getName().c_str()) +
            " commands are: \n " +
            "babbling arm: motor commands sent to all the arm joints \n " +
            "babbling joint <int joint_number>: motor commands sent to joint_number only \n " +
            "help \n " +
            "quit \n" ;

    reply.clear();

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        yInfo() << helpMessage;
        reply.addString(helpMessage);
    }
    else if (command.get(0).asString()=="babbling") {
        if (command.get(1).asString()=="arm")
        {
            single_joint = -1;
            part_babbling = command.get(1).asString();

            if (command.get(2).asString()=="left")
            {
                part = "left_arm";
                yInfo() << "Babbling "+command.get(2).asString()+" arm...";
                doBabbling();
                reply.addString("ack");
            }
            else if (command.get(2).asString()=="right")
            {
                part = "right_arm";
                yInfo() << "Babbling "+command.get(2).asString()+" arm...";
                doBabbling();
                reply.addString("ack");
            }
            else
            {
                yError("Invalid babbling part: specify LEFT or RIGHT after 'arm'.");
                reply.addString("nack");
            }


        }
        else if (command.get(1).asString()=="joint")
        {
            single_joint = command.get(2).asInt();
            if(single_joint < 16 && single_joint>=0)
            {

                if (command.get(3).asString()=="left")
                {
                    part = "left_arm";
                    yInfo() << "Babbling joint " << single_joint << "...";
                    doBabbling();
                    reply.addString("ack");
                }
                else if (command.get(3).asString()=="right")
                {
                    part = "right_arm";
                    yInfo() << "Babbling joint " << single_joint << "...";
                    doBabbling();
                    reply.addString("ack");
                }
                else
                {
                    yError("Invalid babbling part: specify LEFT or RIGHT after joint number.");
                    reply.addString("nack");
                }


            }
            else
            {
                yError("Invalid joint number.");
                reply.addString("nack");
            }

        }
        else if (command.get(1).asString()=="hand")
        {
            single_joint = -1;
            part_babbling = command.get(1).asString();

            if (command.get(2).asString()=="left")
            {
                part = "left_arm";
                yInfo() << "Babbling "+command.get(2).asString()+" hand...";
                doBabbling();
                reply.addString("ack");
            }
            else if (command.get(2).asString()=="right")
            {
                part = "right_arm";
                yInfo() << "Babbling "+command.get(2).asString()+" hand...";
                doBabbling();
                reply.addString("ack");
            }
            else
            {
                yError("Invalid babbling part: specify LEFT or RIGHT after 'arm'.");
                reply.addString("nack");
            }


        }
        else
        {
            yInfo() << "Command not found\n" << helpMessage;
            reply.addString("nack");
        }
    }

    return true;
}

bool Babbling::updateModule() {
    return true;
}

double Babbling::getPeriod() {
    return 0.1;
}


bool Babbling::doBabbling()
{
    // First go to home position
    bool homeStart = gotoStartPos();
    if(!homeStart) {
        cout << "I got lost going home!" << endl;
    }
    for(int i=0; i<16; i++)
    {
        if(part=="right_arm"){
            ictrlRightArm->setControlMode(i,VOCAB_CM_VELOCITY);
        }
        else if(part=="left_arm"){
            ictrlLeftArm->setControlMode(i,VOCAB_CM_VELOCITY);
        }
        else
            yError() << "Don't know which part to move to do babbling." ;


    }

    Bottle reply;
    Bottle abmCommand;
    abmCommand.addString("babbling");
    abmCommand.addString("arm");
    abmCommand.addString(part);
    reply = dealABM(abmCommand,1);

    //check ABM reply
    if (reply.isNull()) {
        cout << "Reply from ABM is null : NOT connected?" << endl;
    } else if (reply.get(0).asString()!="ack"){
        cout << reply.toString() << endl;
    }

    reply.clear();

    if (cmd_source == "C")
    {
        double startTime = yarp::os::Time::now();
        while (Time::now() < startTime + train_duration){
//            yInfo() << Time::now() << "/" << startTime + train_duration;
            double t = Time::now() - startTime;
            yInfo() << "t = " << t << "/ " << train_duration;

            babblingCommands(t,single_joint);
        }
    }
    else
    {
        babblingCommandsMatlab();
    }

    reply = dealABM(abmCommand,0);
    //check ABM reply
    if (reply.isNull()) {
        cout << "Reply from ABM is null : NOT connected?" << endl;
    } else if (reply.get(0).asString()!="ack"){
        cout << reply.toString() << endl;
    }


    bool homeEnd = gotoStartPos();
    if(!homeEnd) {
        cout << "I got lost going home!" << endl;
    }

    return true;
}

yarp::sig::Vector Babbling::babblingCommands(double &t, int j_idx)
{
    for (unsigned int l=0; l<command.size(); l++)
        command[l]=0;

    for (unsigned int l=0; l<16; l++)
        ref_command[l]=start_command[l] + amp*sin(freq*t * 2 * M_PI);

    if(j_idx != -1)
    {
        if(part=="right_arm"){
            bool okEncArm = encsRightArm->getEncoders(encodersRightArm.data());
            if(!okEncArm) {
                cerr << "Error receiving encoders";
                command[j_idx] = 0;
            } else {
                command[j_idx] = 10 * (ref_command[j_idx] - encodersRightArm[j_idx]);
                if(command[j_idx] > 50)
                    command[j_idx] = 50;
                if(command[j_idx] < -50)
                    command[j_idx] = -50;
            }
        } else if(part=="left_arm"){
            bool okEncArm = encsLeftArm->getEncoders(encodersLeftArm.data());
            if(!okEncArm) {
                cerr << "Error receiving encoders";
                command[j_idx] = 0;
            } else {
                command[j_idx] = 10 * (ref_command[j_idx] - encodersLeftArm[j_idx]);
                if(command[j_idx] > 50)
                    command[j_idx] = 50;
                if(command[j_idx] < -50)
                    command[j_idx] = -50;
            }
        }
    }
    else
    {
        if(part_babbling == "arm")
        {
            if(part=="right_arm"){
                bool okEncArm = encsRightArm->getEncoders(encodersRightArm.data());
                if(!okEncArm) {
                    cerr << "Error receiving encoders";
                    for (unsigned int l=0; l<7; l++)
                        command[l] = 0;
                } else {
                    for (unsigned int l=0; l<7; l++)
                    {
                        //                    yWarning() << "error=" << ref_command[l] - encodersArm[l];
                        command[l] = 10 * (ref_command[l] - encodersRightArm[l]);
                        if(command[j_idx] > 20)
                            command[j_idx] = 20;
                        if(command[j_idx] < -20)
                            command[j_idx] = -20;
                    }
                }
            } else if (part=="left_arm"){

                bool okEncArm = encsLeftArm->getEncoders(encodersLeftArm.data());
                if(!okEncArm) {
                    cerr << "Error receiving encoders";
                    for (unsigned int l=0; l<7; l++)
                        command[l] = 0;
                } else {
                    for (unsigned int l=0; l<7; l++)
                    {
                        //                    yWarning() << "error=" << ref_command[l] - encodersArm[l];
                        command[l] = 10 * (ref_command[l] - encodersLeftArm[l]);
                        if(command[j_idx] > 20)
                            command[j_idx] = 20;
                        if(command[j_idx] < -20)
                            command[j_idx] = -20;
                    }
                }
            }


        }
        else if(part_babbling == "hand")
        {

            if(part=="right_arm")
            {
                bool okEncArm = encsRightArm->getEncoders(encodersRightArm.data());
                if(!okEncArm) {
                    cerr << "Error receiving encoders";
                    for (unsigned int l=7; l<command.size(); l++)
                        command[l] = 0;
                } else {
                    for (unsigned int l=7; l<command.size(); l++)
                    {
    //                    yWarning() << "error=" << ref_command[l] - encodersArm[l];
                        command[l] = 10 * (ref_command[l] - encodersRightArm[l]);
                        if(command[j_idx] > 20)
                            command[j_idx] = 20;
                        if(command[j_idx] < -20)
                            command[j_idx] = -20;
                    }
                }
            }
            else if(part=="left_arm")
            {
                bool okEncArm = encsLeftArm->getEncoders(encodersLeftArm.data());
                if(!okEncArm) {
                    cerr << "Error receiving encoders";
                    for (unsigned int l=7; l<command.size(); l++)
                        command[l] = 0;
                } else {
                    for (unsigned int l=7; l<command.size(); l++)
                    {
    //                    yWarning() << "error=" << ref_command[l] - encodersArm[l];
                        command[l] = 10 * (ref_command[l] - encodersLeftArm[l]);
                        if(command[j_idx] > 20)
                            command[j_idx] = 20;
                        if(command[j_idx] < -20)
                            command[j_idx] = -20;
                    }
                }
            }

        }
        else
        {
            yError("Can't babble the required body part.");
        }
    }

    Bottle& inDataB = portVelocityOut.prepare(); // Get the object
    inDataB.clear();
    for(unsigned int l=0; l<command.size(); l++)
    {
        inDataB.addDouble(command[l]);
    }
    portVelocityOut.write();

    if(part=="right_arm"){
        velRightArm->velocityMove(command.data());
    }
    else if(part=="left_arm"){
        velLeftArm->velocityMove(command.data());
    }
    else
        yError() << "Don't know which part to move to do babbling." ;


    // This delay is needed!!!
    yarp::os::Time::delay(0.05);

    return command;
}


int Babbling::babblingCommandsMatlab()
{

    if (!portToMatlab.open("/portToMatlab:o")) {
        cout << ": Unable to open port " << "/portToMatlab:o" << endl;
    }
    if (!portReadMatlab.open("/portReadMatlab:i")) {
        cout << ": Unable to open port " << "/portReadMatlab:i" << endl;
    }

    while(!Network::isConnected(portToMatlab.getName(), "/matlab/read")) {
        Network::connect(portToMatlab.getName(), "/matlab/read");
        cout << "Waiting for port " << portToMatlab.getName() << " to connect to " << "/matlab/read" << endl;
        Time::delay(1.0);
    }

    while(!Network::isConnected("/matlab/write", portReadMatlab.getName())) {
        Network::connect("/matlab/write", portReadMatlab.getName());
        cout << "Waiting for port " << "/matlab/write" << " to connect to " << portReadMatlab.getName() << endl;
        Time::delay(1.0);
    }

    yInfo() << "Connections ok..." ;

    for(int i=0; i<=6; i++)
    {
        ictrlLeftArm->setControlMode(i,VOCAB_CM_VELOCITY);
    }

    Bottle *endMatlab;
    Bottle *cmdMatlab;
    Bottle replyFromMatlab;
    Bottle bToMatlab;

    bool done = false;
    int nPr = 1;
    while(!done)
    {
    
        // Tell Matlab that motion is ready to be done
        bToMatlab.clear();replyFromMatlab.clear();
        bToMatlab.addInt(1);
        portToMatlab.write(bToMatlab,replyFromMatlab);
    
    
        for (unsigned int l=1; l<command.size(); l++)
            command[l]=0;

        // get the commands from Matlab
        cmdMatlab = portReadMatlab.read(true) ;
        for (int i=0; i<6; i++)
            command[i] = cmdMatlab->get(i).asDouble();
        
        // Move
        int j=1;
        while(j--)
        {
            if(part=="right_arm"){
                velRightArm->velocityMove(command.data());
            }
            else if(part=="left_arm"){
                velLeftArm->velocityMove(command.data());
            }
            else
                yError() << "Don't know which part to move to do babbling." ;

            Time::delay(0.00002);
        }
        

        // Tell Matlab that motion is done
        bToMatlab.clear();replyFromMatlab.clear();
        bToMatlab.addInt(1);
        portToMatlab.write(bToMatlab,replyFromMatlab);

        endMatlab = portReadMatlab.read(true) ;
        done = endMatlab->get(0).asInt();
        
        nPr = nPr +1;
    }

    yInfo() << "Going to tell Matlab that ports will be closed here.";
    // Tell Matlab that ports will be closed here
    bToMatlab.clear();replyFromMatlab.clear();
    bToMatlab.addInt(1);
    portToMatlab.write(bToMatlab,replyFromMatlab);

    portToMatlab.close();
    portReadMatlab.close();

    yInfo() << "Finished and ports to/from Matlab closed.";

    return 0;
}


bool Babbling::gotoStartPos()
{
    velHead->stop();
    velLeftArm->stop();
    velRightArm->stop();

    yarp::os::Time::delay(2.0);

    /* Move head to start position */
    commandHead = encodersHead;
    for (int i=0; i<=4; i++) {
        ictrlHead->setControlMode(i,VOCAB_CM_POSITION);
        commandHead[i] = start_commandHead[i];
    }
    if(part=="right_arm"){
    	commandHead[2] = -2*start_commandHead[2];
    	
    }
    posHead->positionMove(commandHead.data());

    /* Move arm to start position */

    if(part=="left_arm")
    {
        command = encodersLeftArm;
        bool successAll = false;
        while(!successAll) {
            successAll = true;

            for(int i=0; i<16; i++)
            {
                bool successIndividual = ictrlLeftArm->setControlMode(i, VOCAB_CM_POSITION);
                yDebug() << "Set joint " << i << " in position mode successful: " << successIndividual;
                command[i]=start_command[i];
                if(!successIndividual) {
                    successAll = false;
                }
            }
        }
        posLeftArm->positionMove(command.data());
    }
    else if(part=="right_arm")
    {
        command = encodersRightArm;
        bool successAll = false;
        while(!successAll) {
            successAll = true;

            for(int i=0; i<16; i++)
            {
                bool successIndividual = ictrlRightArm->setControlMode(i, VOCAB_CM_POSITION);
                yDebug() << "Set joint " << i << " in position mode successful: " << successIndividual;
                command[i]=start_command[i];
                if(!successIndividual) {
                    successAll = false;
                }
            }
        }
        posRightArm->positionMove(command.data());
    }
    else
        yError() << "Don't know which part to move to start position." ;


    bool done_head=false;
    bool done_arm=false;
    while (!done_head || !done_arm) {
        yInfo() << "Wait for position moves to finish" ;
        posHead->checkMotionDone(&done_head);
        posLeftArm->checkMotionDone(&done_arm);
//        yDebug() << "done_head: " << done_head << " done_arm: " << done_arm;
        Time::delay(0.04);
//        for(int i=0; i<16; i++)
//        {
//            int m;
//            ictrlLeftArm->getControlMode(i, &m);
//            if(m==VOCAB_CM_POSITION) {
//                yDebug() << i << " position";
//            } else if(m==VOCAB_CM_VELOCITY) {
//                yDebug() << i << " velocity";
//            } else if(m==VOCAB_CM_HW_FAULT) {
//                yDebug() << i << " HW fault";
//            } else {
//                yDebug() << i << " ???";
//            }
//        }
    }
    yInfo() << "Done." ;

    Time::delay(1.0);

    return true;
}

bool Babbling::init_iCub(string &part)
{
    /* Create PolyDriver for left arm */
    Property option;

    string portnameLeftArm = "left_arm";//part;
    cout << part << endl;
    option.put("robot", robot.c_str());
    option.put("device", "remote_controlboard");
    Value& robotnameLeftArm = option.find("robot");

    string sA("/babbling/");
    sA += robotnameLeftArm.asString();
    sA += "/";
    sA += portnameLeftArm.c_str();
    sA += "/control";
    option.put("local", sA.c_str());

    sA.clear();
    sA += "/";
    sA += robotnameLeftArm.asString();
    sA += "/";
    sA += portnameLeftArm.c_str();
    cout << sA << endl;
    option.put("remote", sA.c_str());

    yDebug() << option.toString().c_str() ;

    leftArmDev = new yarp::dev::PolyDriver(option);
    if (!leftArmDev->isValid()) {
        yError() << "Device not available.  Here are the known devices:";
        yError() << yarp::dev::Drivers::factory().toString();
        Network::fini();
        return false;
    }

    leftArmDev->view(posLeftArm);
    leftArmDev->view(velLeftArm);
    leftArmDev->view(encsLeftArm);
    leftArmDev->view(ictrlLeftArm);
    leftArmDev->view(ictrlLimLeftArm);

    double minLimArm[16];
    double maxLimArm[16];
    for (int l=0; l<16; l++)
        ictrlLimLeftArm->getLimits(l,&minLimArm[l],&maxLimArm[l]);
//    for (int l=7; l<16; l++)
//        start_command[l] = (maxLimArm[l]-minLimArm[l])/2;
    for (int l=0; l<16; l++)
        yInfo() << "Joint " << l << ": limits = [" << minLimArm[l] << "," << maxLimArm[l] << "]. start_commad = " << start_command[l];


    if (posLeftArm==NULL || encsLeftArm==NULL || velLeftArm==NULL || ictrlLeftArm==NULL ){
        cout << "Cannot get interface to robot device" << endl;
        leftArmDev->close();
    }

    if (encsLeftArm==NULL){
        yError() << "Cannot get interface to robot device";
        leftArmDev->close();
    }

    int nj = 0;
    posLeftArm->getAxes(&nj);
    velLeftArm->getAxes(&nj);
    encsLeftArm->getAxes(&nj);
    encodersLeftArm.resize(nj);

    yInfo() << "Wait for arm encoders";
    while (!encsLeftArm->getEncoders(encodersLeftArm.data()))
    {
        Time::delay(0.1);
        yInfo() << "Wait for arm encoders";
    }




    /* Create PolyDriver for right arm */
    string portnameRightArm = "right_arm";//part;
    cout << part << endl;
    option.put("robot", robot.c_str());
    option.put("device", "remote_controlboard");
    Value& robotnameRightArm = option.find("robot");

    string sAr("/babbling/");
    sAr += robotnameRightArm.asString();
    sAr += "/";
    sAr += portnameRightArm.c_str();
    sAr += "/control";
    option.put("local", sAr.c_str());

    sAr.clear();
    sAr += "/";
    sAr += robotnameRightArm.asString();
    sAr += "/";
    sAr += portnameRightArm.c_str();
    cout << sAr << endl;
    option.put("remote", sAr.c_str());

    yDebug() << option.toString().c_str() ;

    rightArmDev = new yarp::dev::PolyDriver(option);
    if (!rightArmDev->isValid()) {
        yError() << "Device not available.  Here are the known devices:";
        yError() << yarp::dev::Drivers::factory().toString();
        Network::fini();
        return false;
    }

    rightArmDev->view(posRightArm);
    rightArmDev->view(velRightArm);
    rightArmDev->view(encsRightArm);
    rightArmDev->view(ictrlRightArm);
    rightArmDev->view(ictrlLimRightArm);


    for (int l=0; l<16; l++)
        ictrlLimRightArm->getLimits(l,&minLimArm[l],&maxLimArm[l]);
//    for (int l=7; l<16; l++)
//        start_command[l] = (maxLimArm[l]-minLimArm[l])/2;
    for (int l=0; l<16; l++)
        yInfo() << "Joint " << l << ": limits = [" << minLimArm[l] << "," << maxLimArm[l] << "]. start_commad = " << start_command[l];


    if (posRightArm==NULL || encsRightArm==NULL || velRightArm==NULL || ictrlRightArm==NULL ){
        cout << "Cannot get interface to robot device" << endl;
        rightArmDev->close();
    }

    if (encsRightArm==NULL){
        yError() << "Cannot get interface to robot device";
        rightArmDev->close();
    }

    nj = 0;
    posRightArm->getAxes(&nj);
    velRightArm->getAxes(&nj);
    encsRightArm->getAxes(&nj);
    encodersRightArm.resize(nj);

    yInfo() << "Wait for arm encoders";
    while (!encsRightArm->getEncoders(encodersRightArm.data()))
    {
        Time::delay(0.1);
        yInfo() << "Wait for arm encoders";
    }



    yInfo() << "Arms initialized.";






    /* Init. head */
    string portnameHead = "head";
    option.put("robot", robot.c_str()); //"icub"); // typically from the command line.
    option.put("device", "remote_controlboard");
    Value& robotnameHead = option.find("robot");

    string sH("/babbling/");
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

    headDev = new yarp::dev::PolyDriver(option);
    if (!headDev->isValid()) {
        yError() << "Device not available.  Here are the known devices:";
        yError() << yarp::dev::Drivers::factory().toString();
        Network::fini();
        return false;
    }

    headDev->view(posHead);
    headDev->view(velHead);
    headDev->view(encsHead);
    headDev->view(ictrlHead);
    if (posHead==NULL || encsHead==NULL || velHead==NULL || ictrlHead==NULL ){
        cout << "Cannot get interface to robot head" << endl;
        headDev->close();
    }
    int jnts = 0;

    posHead->getAxes(&jnts);
    velHead->getAxes(&jnts);
    encodersHead.resize(jnts);
    commandHead.resize(jnts);

    yInfo() << "Wait for encoders (HEAD)" ;
    while (!encsHead->getEncoders(encodersHead.data()))
    {
        Time::delay(0.1);
        yInfo() << "Wait for head encoders";
    }

    yInfo() << "Head initialized." ;



    /* Set velocity control for arm */
    yInfo() << "Set velocity control mode";
    for(int i=0; i<16; i++)
    {
        ictrlLeftArm->setControlMode(i,VOCAB_CM_VELOCITY);
        ictrlRightArm->setControlMode(i,VOCAB_CM_VELOCITY);
    }

    yInfo() << "> Initialisation done.";

    return true;
}


Bottle Babbling::dealABM(const Bottle& command, int begin)
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
    bSubMain.addString(command.get(0).asString());
    bSubMain.addString("action");
    Bottle bSubArgument;
    bSubArgument.addString("arguments");
    Bottle bSubSubArgument;
    bSubSubArgument.addString(command.get(1).toString());
    bSubSubArgument.addString("limb");
    if(command.size()==3)
    {
        Bottle bSubSubArgument;
        bSubSubArgument.addInt(command.get(2).asInt());
        bSubSubArgument.addString("index");
    }
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
    yInfo() << "Bottle to ABM: " << bABM.toString();

    if(Network::connect(portToABM.getName(), "/autobiographicalMemory/rpc")) {
        portToABM.write(bABM,bABMreply);
    }

    return bABMreply;
}

/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors:  Maxime Petit
 * email:  m.petit@imperial.ac.uk
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

#include "jointsAwareness.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
//using namespace wysiwyd::wrdac;
using namespace std;

bool jointsAwareness::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("jointsAwareness")).asString().c_str();
    setName(moduleName.c_str());

    yInfo() << "findFileByName " << rf.findFileByName("jointsAwareness.ini") ;

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    //bool isRFVerbose = true;
    //iCub = new ICubClient(moduleName, "jointsAwareness", "client.ini", isRFVerbose);
    //iCub->opc->isVerbose &= true;

    arm   = rf.check("arm", Value("both")).asString().c_str();
    robot   = rf.check("robot", Value("icub")).asString().c_str();

    if(arm == "left_arm"){
        bEveryThingisGood = configCartesian(leftArmClientCartCtrl, armLeftPort, arm);
    } else if (arm == "right_arm") {
        bEveryThingisGood = configCartesian(rightArmClientCartCtrl, armRightPort, arm);
    } else if (arm == "both"){

        yDebug() << "Both arm are used!" ;
        bEveryThingisGood = configCartesian(leftArmClientCartCtrl, armLeftPort, "left_arm");
        bEveryThingisGood = configCartesian(rightArmClientCartCtrl, armRightPort, "right_arm");
    } else {
        yError() << "The arm used (" << arm << ") is NOT a valid one! Closing jointsAwareness!" ;
        return false ;
    }
    string baseTorsoPortName = "/" + moduleName + "/" + "torso" + "/";
    torsoPort.open((baseTorsoPortName + "jointsLoc:o").c_str());
    torso_2DProj_Port.open((baseTorsoPortName + "joints2DProj:o").c_str());

    Property optionGaze("(device gazecontrollerclient)");
    optionGaze.put("remote","/iKinGazeCtrl");
    optionGaze.put("local",("/"+moduleName+"/gaze").c_str());
    if (!gazeClientCtrl.open(optionGaze))
        yWarning("Gaze controller not available!");
    if(gazeClientCtrl.isValid()){
        gazeClientCtrl.view(iGaze);
    } else {
        yError() << "Invalid PolyDriver iGaze: exit!";
        return false;
    }



    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    if(!bEveryThingisGood){
        yError() << "Some dependencies or polydriver are not present/valid! Closing jointsAwareness!" ;
        return false ;
    }

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    return true;
}

bool jointsAwareness::configCartesian(PolyDriver& driver, BufferedPort<Bottle> &port, string part){

    string basePartPortName = "/" + moduleName + "/" + part + "/" ;
    port.open((basePartPortName + "jointsLoc:o").c_str());

    Property option;
    option.put("device","cartesiancontrollerclient");
    option.put("remote","/icub/cartesianController/" + part);
    option.put("local","/client/" + part);

    if (!driver.open(option)) {
        cout << "Device not available.  Here are the known devices:\n"<< endl;
        cout << Drivers::factory().toString().c_str() << endl;;
        return false;
    }

    yDebug() << "Check is driver is valid" ;
    if (driver.isValid()) {
        yDebug() << "driver IS valid" ;
        if(part == "left_arm"){
            driver.view(iLeftArm);
            armLeft_2DProj_Port.open((basePartPortName + "joints2DProj:o").c_str());
        } else if (part == "right_arm") {
            driver.view(iRightArm);
            armRight_2DProj_Port.open((basePartPortName + "joints2DProj:o").c_str());
        } else {
            yError() << "The arm used (" << part << ") is NOT a valid one! Closing jointsAwareness!" ;
            return false;
        }
    } else {
        yError() << "Invalid PolyDriver: exit!";
        return false;
    }

    return true;
}

bool jointsAwareness::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n" ;

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    } else {
        cout << helpMessage;
        reply.addString("ok");
    }

    rpcPort.reply(reply);

    return true;
}


/* Called periodically every getPeriod() seconds */
bool jointsAwareness::updateModule() {

    //yDebug() << "update Loop" ;
    isTorsoDone = false;

    isTorsoDone = streamCartesian(leftArmClientCartCtrl, armLeftPort, "left_arm");
    isTorsoDone = streamCartesian(rightArmClientCartCtrl, armRightPort, "right_arm");

    return true;
}

bool jointsAwareness::streamCartesian(PolyDriver& driver, BufferedPort<Bottle> &port, string part){

    if(driver.isValid()){

        Vector location, orientation;

        /**************************** Torso if not done with a previous driver ****************************/

        if(!isTorsoDone){ //do the torso only if not done before
            Bottle& bLocTorso = torsoPort.prepare();
            Bottle& bProjTorso = torso_2DProj_Port.prepare();
            bLocTorso.clear();
            bProjTorso.clear();
            for(unsigned int axis = 0; axis < torsoJointsNb; axis++){
                if(part == "left_arm"){
                    iLeftArm->getPose(axis, location, orientation);
                } else if (part == "right_arm"){
                    iRightArm->getPose(axis, location, orientation);
                } else {
                    yError() << "The arm used (" << part << ") is NOT a valid one! Closing jointsAwareness!" ;
                    return false ;
                }

                //project 3D coordinates into the gaze 2D
                Vector projection2D;
                iGaze->get2DPixel(0, location, projection2D); // 0 = left, 1 = right
                //yDebug() << "Projection in 2D" << projection2D.toString() ;

                Bottle bCurrentJointProj;
                for(unsigned int i = 0; i < projection2D.size(); i++){
                    bCurrentJointProj.addDouble(projection2D[i]);
                }
                bProjTorso.addList() = bCurrentJointProj;

                Bottle bCurrentJointLoc;
                for(unsigned int i = 0; i < location.size(); i++){
                    //bLocTorso.addDouble(location[i]);
                    bCurrentJointLoc.addDouble(location[i]);
                }
                bLocTorso.addList() = bCurrentJointLoc;

            }

            torsoPort.write();
            torso_2DProj_Port.write();
            //yDebug() << "bLoc for " << "torso" << " : (" << bLocTorso.toString() << ")" ;
        }

        /**************************** Arm ****************************/

        Bottle& bLoc = port.prepare();
        Bottle& bLeftArmProj = armLeft_2DProj_Port.prepare();
        Bottle& bRightArmProj = armRight_2DProj_Port.prepare();
        bLoc.clear();
        bLeftArmProj.clear();
        bRightArmProj.clear();

        for(unsigned int axis = 3; axis < armJointsNb+3; axis++){
            Vector projection2D;
            if(part == "left_arm"){
                iLeftArm->getPose(axis, location, orientation); //+3 because the first 3 are torso
                iGaze->get2DPixel(0, location, projection2D); // 0 = left, 1 = right
            } else if (part == "right_arm"){
                iRightArm->getPose(axis, location, orientation); //+3 because the first 3 are torso
                iGaze->get2DPixel(0, location, projection2D); // 0 = left, 1 = right
            } else {
                yError() << "The arm used (" << arm << ") is NOT a valid one! Closing jointsAwareness!" ;
                return false ;
            }

            Bottle bCurrentJointProj;
            for(unsigned int i = 0; i < projection2D.size(); i++){
                bCurrentJointProj.addDouble(projection2D[i]);
            }

            if(part == "left_arm"){
                bLeftArmProj.addList() = bCurrentJointProj;
            } else if (part == "right_arm"){
                bRightArmProj.addList() = bCurrentJointProj;
            }

            Bottle bCurrentJointLoc;
            for(unsigned int i = 0; i < location.size(); i++){
                bCurrentJointLoc.addDouble(location[i]);
            }
            bLoc.addList() = bCurrentJointLoc;

        }

        port.write();
        if(part == "left_arm"){
            yDebug() << "left_arm projection2D" ;
            armLeft_2DProj_Port.write();
        } else if (part == "right_arm"){
            armRight_2DProj_Port.write();
        }


        //yDebug() << "bLoc for " << part << " : (" << bLoc.toString() << ")" ;

    }

    return true;
}

bool jointsAwareness::configCartesian(string part){

    if(part != "left_arm" && part != "right_arm"){
        yError() << "The arm used (" << part << ") is NOT a valid one! Closing jointsAwareness!" ;
        return false;
    }

    string basePartPortName = "/" + moduleName + "/" + part + "/" ;
    locPorts_map.find(part)->second->open((basePartPortName + "jointsLoc:o").c_str());

    Property option;
    option.put("device","cartesiancontrollerclient");
    option.put("remote","/icub/cartesianController/" + part);
    option.put("local","/client/" + part);

    yarp::dev::PolyDriver *driver = polydriver_map.find(part)->second;

    if (!driver->open(option)) {
        cout << "Device not available.  Here are the known devices:\n"<< endl;
        cout << Drivers::factory().toString().c_str() << endl;;
        return false;
    }

    yDebug() << "Check is driver is valid" ;
    if (driver->isValid()) {
        yDebug() << "driver IS valid" ;
        driver->view(cartesian_map.find(part)->second);
        projPorts_map.find(part)->second->open((basePartPortName + "joints2DProj:o").c_str());
    } else {
        yError() << "Invalid PolyDriver: exit!";
        return false;
    }

    return true;
}

bool jointsAwareness::configMaps() {

    locPorts_map["left_arm"] = &armLeftPort;
    locPorts_map["right_arm"] = &armRightPort;
    locPorts_map["torso_arm"] = &torsoPort;

    projPorts_map["left_arm"] = &armLeft_2DProj_Port;
    projPorts_map["right_arm"] = &armRight_2DProj_Port;
    projPorts_map["torso_arm"] = &torso_2DProj_Port;

    polydriver_map["left_arm"] = &leftArmClientCartCtrl;
    polydriver_map["right_arm"] = &rightArmClientCartCtrl;

    cartesian_map["left_arm"] = iLeftArm;
    cartesian_map["right_arm"] = iRightArm;

    return true;
}


bool jointsAwareness::interruptModule() {
    rpcPort.interrupt();

    armLeftPort.interrupt();
    armRightPort.interrupt();
    torsoPort.interrupt();

    armLeft_2DProj_Port.interrupt();
    armRight_2DProj_Port.interrupt();
    torso_2DProj_Port.interrupt();

    return true;
}

bool jointsAwareness::close() {

    rpcPort.interrupt();
    rpcPort.close();

    armLeftPort.interrupt();
    armRightPort.interrupt();
    torsoPort.interrupt();

    armLeft_2DProj_Port.interrupt();
    armRight_2DProj_Port.interrupt();
    torso_2DProj_Port.interrupt();

    armLeftPort.close();
    armRightPort.close();
    torsoPort.close();

    armLeft_2DProj_Port.close();
    armRight_2DProj_Port.close();
    torso_2DProj_Port.close();

    return true;
}

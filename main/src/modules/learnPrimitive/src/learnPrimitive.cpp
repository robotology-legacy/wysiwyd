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

#include "learnPrimitive.h"
#include "wrdac/subsystems/subSystem_recog.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

bool learnPrimitive::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("learnPrimitive")).asString().c_str();
    setName(moduleName.c_str());

    GrammarYesNo           = rf.findFileByName(rf.check("GrammarYesNo", Value("nodeYesNo.xml")).toString());
    GrammarNameAction      = rf.findFileByName(rf.check("GrammarNameAction", Value("GrammarNameAction.xml")).toString());
    GrammarTypeAction      = rf.findFileByName(rf.check("GrammarTypeAction", Value("GrammarTypeAction.xml")).toString());

    yInfo() << "findFileByName " << rf.findFileByName("learnPrimitive.ini") ;
    pathToIniFile = rf.findFileByName("learnPrimitive.ini") ;

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    //bool    bEveryThingisGood = true;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = true;
    iCub = new ICubClient(moduleName, "learnPrimitive", "client.ini", isRFVerbose);
    iCub->opc->isVerbose &= true;

    string robot = rf.check("robot", Value("icubSim")).asString().c_str();
    string arm   = rf.check("arm", Value("left_arm")).asString().c_str();

    portToArm.open(("/" + moduleName + "/toArm:rpc").c_str());
    string portRobotArmName = "/" + robot + "/" + arm + "/rpc:i";

    yInfo() << "================> port controlling the arm : " << portRobotArmName;
    if (!Network::connect(portToArm.getName().c_str(),portRobotArmName))
    {
        yWarning() << "WARNING PORT TO CONTROL ARM (" << portRobotArmName << ") IS NOT CONNECTED";
    }


    if (!iCub->connect())
    {
        cout << "iCubClient : Some dependencies are not running..." << endl;
        Time::delay(1.0);
    }

    //rpc port
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    attach(rpcPort);

    if (!iCub->getRecogClient())
    {
        yWarning() << "WARNING SPEECH RECOGNIZER NOT CONNECTED";
    }
    if (!iCub->getABMClient())
    {
       yWarning() << "WARNING ABM NOT CONNECTED";
    }

    updateProtoAction(rf);
    updatePrimitive(rf);
    updateAction(rf);

    yInfo() << "\n \n" << "----------------------------------------------" << "\n \n" << moduleName << " ready ! \n \n ";

    iCub->say("learn Primitive is ready", false);

    return true;
}


bool learnPrimitive::interruptModule() {
    rpcPort.interrupt();

    return true;
}

bool learnPrimitive::close() {
    iCub->close();
    delete iCub;

    rpcPort.interrupt();
    rpcPort.close();

    return true;
}


bool learnPrimitive::respond(const Bottle& command, Bottle& reply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n" +
        "protoCommand actionName bodyPartName [true/false] \n" +
        "primitiveCommand actionName argument [true/false] \n" +
        "complexCommand actionName argument [true/false] \n" ;

    reply.clear();

    if (command.get(0).asString() == "quit") {
        reply.addString("quitting");

        rpcPort.reply(reply);
        return false;
    }
    //execute a protoCommand (e.g. fold/unfold fingerName)
    else if (command.get(0).asString() == "protoCommand"){  //describeAction : TODO -> protection and stuff

        if(command.size() < 2){
            yError() << " error in learnPrimitive::protoCommand | Too few arguments!";
            reply.addString("error");
            reply.addString("Too few arguments");

            rpcPort.reply(reply);
            return false;

        }
        string sActionName   = command.get(1).asString() ;
        string sBodypartName   = command.get(2).asString() ;
        int maxAngle = 10;
        if( command.size() >= 3 && command.get(3).isInt()) {
            maxAngle   = command.get(3).asInt() ;
        }
        reply = protoCommand(sActionName, sBodypartName, maxAngle);
    }  
    //execute a primitiveCommand (e.g. open/close hand)
    else if (command.get(0).asString() == "primitiveCommand"){  //describeAction : TODO -> protection and stuff

        if(command.size() < 2){
            yError() << " error in learnPrimitive::primitiveCommand | Too few arguments!";
            reply.addString("error");
            reply.addString("Too few arguments");

            rpcPort.reply(reply);
            return false;

        }
        string sPrimitiveName  = command.get(1).asString() ;
        string sPrimitiveArg   = command.get(2).asString() ;
        reply = primitiveCommand(sPrimitiveName, sPrimitiveArg);
    }
    //execute a complexCommand (e.g. show one)
    else if (command.get(0).asString() == "complexCommand"){  //describeAction : TODO -> protection and stuff

        if(command.size() < 2){
            yError() << " error in learnPrimitive::complexCommand | Too few arguments!";
            reply.addString("error");
            reply.addString("Too few arguments");

            rpcPort.reply(reply);
            return false;

        }
        string sActionName  = command.get(1).asString() ;
        string sActionArg   = command.get(2).asString() ;
        reply = actionCommand(sActionName, sActionArg);
    }
    else if (command.get(0).asString() == "proto"){
        //reply = protoDataToR(15366, 15370);
        reply = protoDataToR(15379, 15384);
        //reply = extractProtoProprio(15366, "left_arm", 15367, "fold", "thumb");
    }
    else if (command.get(0).asString() == "learn"){
        reply = learn();
    }
    else if (command.get(0).asString() == "execute"){
        reply = execute();
    }
    else if (command.get(0).asString() == "testRInside"){
        reply = testRInside();
    }
    else {
        cout << helpMessage;
        reply.addString("ok");
    }

    rpcPort.reply(reply);

    return true;
}

yarp::os::Bottle learnPrimitive::extractProtoSemantic(int babbling_begin, int babbling_end){

    ostringstream osRequest;
    Bottle bProtoWords;


    //the sentence might be checked as a SENTENCE_JOINT?
    osRequest.str("");
    osRequest << "CREATE EXTENSION IF NOT EXISTS tablefunc;" <<
                 "SELECT * " <<
                 "FROM crosstab( " <<
                 "  'SELECT instance, role, word " <<
                 "   FROM sentencedata " <<
                 "   WHERE (role = ''action_joint'' or role = ''finger'') and instance > " << babbling_begin << " and instance < " << babbling_end <<
                 "   ORDER by 1,2') " <<
                 "AS sentencedata(instance int, action text, finger text);";
    bProtoWords = iCub->getABMClient()->requestFromString(osRequest.str().c_str());
    yDebug() << "==> protoactions infos: " << bProtoWords.toString() ;

    return bProtoWords;
}

yarp::os::Bottle learnPrimitive::extractAllProtoProprio(int babbling_begin, Bottle bProtoWords, string babbling_arm){

    Bottle bBabblingProprio;
    for (int i = 0; i < bProtoWords.size(); i++){
        Bottle bProtoProprio;

        int currentProtoInstance  = atoi(bProtoWords.get(i).asList()->get(0).toString().c_str());

        //we need to indicate the previous protoInstance (or babbling begin) to not take into account the "noise" from other (pasar, etc.)
        int previousProtoInstance;
        if (i == 0) {
            previousProtoInstance = babbling_begin;
        } else {
            previousProtoInstance = atoi(bProtoWords.get(i-1).asList()->get(0).toString().c_str());
        }
        string currentProtoName   = bProtoWords.get(i).asList()->get(1).asString();
        string currentProtoFinger = bProtoWords.get(i).asList()->get(2).asString();

        yDebug() << "==> proto instance = " << currentProtoInstance << ", proto name = " << currentProtoName << " and proto finger = " << currentProtoFinger ;
        // --------------------------------(15366, 15367, "fold", "thumb")
        bProtoProprio = extractSingleProtoProprio(previousProtoInstance, currentProtoInstance, babbling_arm, currentProtoName, currentProtoFinger);

        bBabblingProprio.addList() = bProtoProprio ;
    }

    return bBabblingProprio;
}

yarp::os::Bottle learnPrimitive::extractSingleProtoProprio(int previousProtoInstance, int currentProtoInstance, string babbling_part, string proto_name, string proto_finger){

    ostringstream osRequest;
    Bottle bResult;

    osRequest.str("");
    osRequest << "SELECT main.time from main WHERE instance = " << previousProtoInstance;
    Bottle b_time_begin = iCub->getABMClient()->requestFromString(osRequest.str().c_str());

    osRequest.str("");
    osRequest << "SELECT main.time from main WHERE instance = " << currentProtoInstance;
    Bottle b_time_end = iCub->getABMClient()->requestFromString(osRequest.str().c_str());

    //********************* extract the joint number =========================================================> possibly spy on several?
    //iCub->opc->checkout();
    //should check at some point that the bodypart is there and loaded no?
    //Bodypart* bp = dynamic_cast<Bodypart*>(iCub->opc->getEntity(proto_finger));
    //int joint = bp->m_joint_number ;

    int joint = 9;

    yDebug() << "time begin = " << b_time_begin.toString() << " and time_end = " << b_time_end.toString();
    yDebug() << "proto_instance = " << currentProtoInstance << " and proto_name = " << proto_name << " and proto finger = " << proto_finger << "and " << babbling_part;

    //*********************************************************************************** protect if bodypart not found

    osRequest.str("");
    osRequest << "SELECT " << currentProtoInstance << " AS proto_instance, '" << proto_name <<"' AS proto_name, '" << proto_finger << "' AS proto_finger, proprioceptivedata.* " <<
                 "FROM   proprioceptivedata " <<
                 "WHERE proprioceptivedata.time > CAST('" << b_time_begin.toString() << "' AS TIMESTAMP) AND proprioceptivedata.time <  CAST('" << b_time_end.toString() << "' AS TIMESTAMP)" <<
                 "      AND subtype = '"<< joint << "' AND label_port = '/icub/" << babbling_part << "/state:o' " <<
                 "ORDER BY (frame_number);";
    bResult = iCub->getABMClient()->requestFromString(osRequest.str().c_str());

    //yDebug() << "extractProtoAction:\n" << bResult.toString();

    return bResult;
}

yarp::os::Bottle learnPrimitive::protoDataToR(int babbling_begin, int babbling_end){

    Bottle bOutput, bProtoWords, bProtoArm, bBabblingProprio;
    ostringstream osRequest;

    //********************* extract the arm used for the babbling
    osRequest << "SELECT argument FROM contentarg WHERE instance = " << babbling_begin << " AND role = 'side'";
    bProtoArm = iCub->getABMClient()->requestFromString(osRequest.str().c_str());
    yDebug() << "==> Arm used for babbling: " << bProtoArm.toString() ;
    string babbling_arm = bProtoArm.get(0).toString();


    //********************* extract action and finger for each proto-action
    bProtoWords = extractProtoSemantic(babbling_begin, babbling_end);

    //********************* extract proprioceptive data for each proto-action
    bBabblingProprio = extractAllProtoProprio(babbling_begin, bProtoWords, babbling_arm);


    //************************************************** Preparing R Session **************************************************//
    vector<int> v_instanceBabbling, v_instanceProto, v_frame_number, v_joint;
    vector<string> v_protoName, v_protoFinger;
    vector<double> v_value;


    yDebug() << "============================================== BEFORE THE LOOP ======================================" ;
    yDebug() << "We have " << bBabblingProprio.size() << " different protoactions!" ;
    for(int i = 0; i < bBabblingProprio.size(); i++){

        yDebug() << "-> current proto have " << bBabblingProprio.get(i).asList()->size() << " lines!" ;

        for(int j = 0; j < bBabblingProprio.get(i).asList()->size(); j++){
            Bottle* currentLine = bBabblingProprio.get(i).asList()->get(j).asList();
            //yDebug() << currentLine->toString();

            // proto_instance | proto_name | proto_finger | babbling_instance | time | port | joint_nb | value | frame
            //       0              1              2               3              4      5       6          7      8

            v_instanceProto.push_back(atoi(currentLine->get(0).toString().c_str()));
            v_protoName.push_back(currentLine->get(1).toString().c_str());
            v_protoFinger.push_back(currentLine->get(2).toString().c_str());
            v_instanceBabbling.push_back(atoi(currentLine->get(3).toString().c_str()));
            v_joint.push_back(atoi(currentLine->get(6).toString().c_str()));
            v_value.push_back(atof(currentLine->get(7).toString().c_str()));
            v_frame_number.push_back(atoi(currentLine->get(8).toString().c_str()));
        }
    }

    //2. data.frame or similar to be sent to R
    R["instanceProto"] = v_instanceProto;
    R["protoName"] = v_protoName;
    R["protoFinger"] = v_protoFinger;
    R["instanceBabbling"] = v_instanceBabbling;
    R["joint"] = v_joint;
    R["value"] = v_value;
    R["frame_number"] = v_frame_number;



    try {

        yDebug() << "============================================== R Session ======================================" ;

        std::string cmd =
            "myData <- data.frame(instanceProto, protoName, protoFinger, instanceBabbling, joint, value, frame_number); "
            "print(is.data.frame(myData)); "
            "print(head(myData)); "
            "cat('number of lines : ', nrow(myData))";
        R.parseEval(cmd);

        //3.2 Remove the first "flat" part: floating windows of 5? (i +/-2) and 'begin' when diff > threshold ~ 0.2. Assume that data are stored by ascending instance number
        // install.packages("zoo") for that!
        R["winSize"] = 5;
        R["winStep"] = 2;
        cmd =
            "library(zoo);"
            "sliding <- rollapply(myData$value, width = winSize, by = winStep, FUN = mean, align = 'left');"
            "cutFrom <- 1;"
            "for(i in 1:(length(sliding)-1)){"
            "    if( abs(sliding[i] - sliding[i+1]) > 0.4){"
            "        cat('i = ', i, '\n');"
            "        cutFrom <- i;"
            "        break;"
            "    }"
            "};"
            "cat('cutFrom: ', cutFrom, '\n');"
            "cutFrom <- cutFrom * winStep;"
            "cat('finalCutFrom: ', cutFrom, '\n');";

        R.parseEval(cmd);

        //3.3 extract the frameNumber to use for subset (depending on spying port, portnumber might not corresponding to line number)
        cmd =
            "frame2cut <- myData$frame_number[cutFrom];"
            "cat('frame below which to cut: ', frame2cut, '\n')";
        R.parseEval(cmd);

        //3.4 do the subset
        cmd =
            "myCleanedData <- subset(myData, (myData$frame_number > frame2cut));"
            "cat('myCleanedData has now ', nrow(myCleanedData), ' lines\n');"
            "print(head(myCleanedData));";
        R.parseEval(cmd);

        //look at what we cut -------------> just to check, to be removed <---------------------------------------------
        cmd =
            "dataToRemove <- subset(myData, (myData$frame_number < frame2cut));"
            "cat('dataToRemove has ', nrow(dataToRemove), ' lines\n');"
            "print(dataToRemove[c('instanceProto', 'value','frame_number')])";
        R.parseEval(cmd);

        //3.4 print to check
        cmd =
            "print(head(myCleanedData[c('instanceProto', 'value','frame_number')]))";
        R.parseEval(cmd);

        /******************************************** /!\ value transformed in percentage TODO: Use a pre-defined dictionary for all joints or ini file /!\ ********************************************/
        double max_angle;
        int bp_joint = 9;
        if (bp_joint != 15){
            max_angle = 90.0;
        } else{
            max_angle = 250.0;
        }

        cmd = "myCleanedData$value <- (myCleanedData$value*100.0)/" + to_string(max_angle);
        R.parseEval(cmd);

        cmd =
            "print(head(myCleanedData[c('instanceProto', 'value','frame_number')]))";
        R.parseEval(cmd);

        //check that 15381 is not taken because of wrong recog
        //cmd =
        //    "cat('============> Check the 15381 is NOT present as it is some noice/false recog \n');"
        //    "mySubset <- subset(myCleanedData, myCleanedData$instanceProto == 15382);"
        //    "cat('We have mySubset with ', nrow(mySubset), 'lines\n');"
        //    "print(head(mySubset[c('instanceProto', 'value','frame_number')]));";
        //R.parseEval(cmd);


        cmd = "library(ggplot2);"
              "x11();"
              "myQplot <- qplot(myCleanedData$frame_number, myCleanedData$value, col=as.factor(myCleanedData$instanceProto), pch = myCleanedData$protoName);"
              "plot(myQplot);"
              "Sys.sleep(5);cat('\nThat was an amazing plot!\n');dev.off();"
              "filepath <- '/home/maxime/CloudStation/R/fold-unfold/plot/myplot.pdf';"
              "x11();pdf(filepath);"
              "print(myQplot);"
              "cat('\nPlot saved in !\n', filepath);"
              "dev.off();dev.off();"
              "cat('\n');";

        //parseEvalQ evluates without assignment
        R.parseEval(cmd);






    } catch(std::exception& ex) {
        yError() << "RInside: Exception caught: " << ex.what() ;
        bOutput.addInt(0);
        bOutput.addString(ex.what());
    } catch(...) {
        yError() << "RInside: Unknown exception caught" ;
        bOutput.addInt(0);
    }

    bOutput.addInt(1);
    return bOutput;
}

//execute action, stop to go out
Bottle learnPrimitive::testRInside(){

    Bottle bOutput;

    try {

        /* ************************************* sample 0: just test *************************************
        R.setVerbose(true);;
        std::string txt = "suppressMessages(require(stats));" "swisssum <- summary(lm(Fertility ~ . , data = swiss));" "print(swisssum)";
        R.parseEvalQ(txt);           // eval the init string, ignoring any returns
        **************************************************************************************/

        /* ************************************* sample 1: call an rHelper function ************************************/
        /* ************************************* sample 3: run lm + extract variables from R to C++, also in sample1*************************************/
        /* ************************************* sample 6: assign values *************************************
        double d1 = 1.234;		// scalar double
        R["d1"] = d1;			// or R.assign(d1, "d1")

        std::vector<double> d2;		// vector of doubles
        d2.push_back(1.23);
        d2.push_back(4.56);
        R["d2"] = d2;			// or R.assign(d2, "d2");

        std::map< std::string, double > d3; // map of doubles
        d3["a"] = 7.89;
        d3["b"] = 7.07;
        R["d3"] = d3;			// or R.assign(d3, "d3");

        std::list< double > d4; 	// list of doubles
        d4.push_back(1.11);
        d4.push_back(4.44);
        R["d4"] = d4;			// or R.assign(d4, "d4");

        std::string txt = 		// now access in R
            "cat('\nd1=', d1, '\n'); print(class(d1));"
            "cat('\nd2=\n'); print(d2); print(class(d2));"
            "cat('\nd3=\n'); print(d3); print(class(d3));"
            "cat('\nd4=\n'); print(d4); print(class(d4));";
            R.parseEvalQ(txt);
        **************************************************************************************/

        /* ************************************* sample 11: plot a graph ************************************/
        /* ************************************* sample 13: quiet errors ************************************/

        //create some dummy data.frame
        std::string cmd =
            "L3 <- LETTERS[1:3]; "
            "fac <- sample(L3, 10, replace = TRUE); "
            "(d <- data.frame(x = 1, y = 1:10, fac = fac)); "
            "print(is.data.frame(d)); "
            "print(head(d)); ";
        R.parseEval(cmd);

        //extract some info from it
        double myMean = R.parseEval("mean(d$y)");
        yInfo() << "myMean = " << myMean ;

        Rcpp::NumericVector colY( (SEXP) R.parseEval("d$y"));
        for (int i=0; i<colY.size(); i++) {
            yInfo() << colY[i] << "\t";
        }

        bOutput.addInt(1);

    } catch(std::exception& ex) {
        yError() << "RInside: Exception caught: " << ex.what() ;
        bOutput.addInt(0);
    } catch(...) {
        yError() << "RInside: Unknown exception caught" ;
        bOutput.addInt(0);
    }

    yInfo() << "RInside end!";
    return bOutput;
}

//execute action, stop to go out
Bottle learnPrimitive::execute(){
    Bottle bOutput ;
    Bottle bRecognized, //received FROM speech recog with transfer information (1/0 (bAnswer))
    bAnswer, //response from speech recog without transfer information, including raw sentence
    bSemantic; // semantic information of the content of the recognition

    string sSay = " What do you want me to do?";
    yInfo() << sSay;
    iCub->say(sSay);

    Bottle bCurrentOrder = nodeNameAction("any");

    string orderType = bCurrentOrder.get(0).asString();
    if(orderType == "stop"){
        string sSay = " Allright, thank you for the exercice";
        yInfo() << sSay;
        iCub->say(sSay);

        return bCurrentOrder ;
    }

    string orderVerb = bCurrentOrder.get(1).asString();
    string orderArg  = bCurrentOrder.get(2).asString();

    sSay = orderVerb + "ing my " + orderArg;
    if(orderArg == "one" || orderArg == "two" || orderArg == "three" || orderArg == "four" || orderArg == "five"){
        sSay = orderVerb + "ing " + orderArg;
    }
    sSay = sSay ;
    yInfo() << sSay;
    iCub->say(sSay);

    if(orderType == "proto-action") {
        protoCommand(orderVerb, orderArg);
    } else if (orderType == "primitive") {
        primitiveCommand(orderVerb, orderArg);
    } else {
        actionCommand(orderVerb, orderArg);
    }

    yarp::os::Time::delay(6);
    return execute();
}

/* Called periodically every getPeriod() seconds */
bool learnPrimitive::updateModule() {
    return true;
}


//action type : proto-action, primitive, action and stop
Bottle learnPrimitive::nodeNameAction(string actionTypeNeeded){
    Bottle bOutput ;
    Bottle bRecognized, //received FROM speech recog with transfer information (1/0 (bAnswer))
    bAnswer, //response from speech recog without transfer information, including raw sentence
    bSemantic; // semantic information of the content of the recognition

    //Load the Speech Recognition with grammar according to entityType
    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarNameAction), 20);

    yInfo() << "bRecognized = " << bRecognized.toString();

    if (bRecognized.get(0).asInt() == 0)
    {
        yError() << " error in proactiveTagging::nodeNameAction | Error in speechRecog (nodeNamePrimitive)";
        bOutput.addString("error");
        bOutput.addString("error in speechRecog (nodeNameAction)");
        return bOutput;
    }

    bAnswer = *bRecognized.get(1).asList();
    bSemantic = *bAnswer.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

    string actionType = bSemantic.get(0).asString() ;
    yInfo() << "bSemantic = " << bSemantic.toString() ;
    yInfo() << "actionTypeNeeded = " << actionTypeNeeded << " and actionType = " << actionType ;


    //If not "any" : should match (e.g. when "I will teach you how to ...")
    if (actionTypeNeeded != "any"){
        if(actionType != actionTypeNeeded && actionType != "stop"){
            yError() << " error in proactiveTagging::nodeNameAction | Error in speechRecog (nodeNameAction) : actionType mismatch" ;
            bOutput.addString("error");
            bOutput.addString("Error in speechRecog (nodeNameAction) : actionType mismatch");
            return bOutput;
        }
    }

    string sName, sArg;

    //get(1).asList() because there are several sub part in the semantic
    if(actionType == "proto-action") {
        sName = bSemantic.get(1).asList()->check("proto-action_name", Value("unknown")).asString();
        sArg = bSemantic.get(1).asList()->check("proto-action_arg", Value("unknown")).asString();
    } else if(actionType == "primitive") {
        sName = bSemantic.get(1).asList()->check("primitive_name", Value("unknown")).asString();
        sArg = bSemantic.get(1).asList()->check("primitive_arg", Value("unknown")).asString();
    } else if(actionType == "action") {
        sName = bSemantic.get(1).asList()->check("action_name", Value("unknown")).asString();
        sArg = bSemantic.get(1).asList()->check("action_arg", Value("unknown")).asString();
    } else if (actionType == "stop") {
        bOutput.addString("stop") ;
        return bOutput ;
    } else {
        yError() << " error in proactiveTagging::nodeNameAction | ACtion type " << actionType << "is not known" ;
        bOutput.addString("error");
        bOutput.addString("Entity type is not known");
        return bOutput;
    }

    bOutput.addString(actionType);
    bOutput.addString(sName);
    bOutput.addString(sArg);

    yInfo() << "Bottle sent by nodeNameAction =  " << bOutput.toString();

    return bOutput ;
}

//For now only control in position. Careful, angle is in percentage of maxAngle
Bottle learnPrimitive::learn(){
    Bottle bOutput;

    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
    bAnswer, //response from speech recog without transfer information, including raw sentence
    bSemantic; // semantic information of the content of the recognition
    
    string sSay = " What do you want to teach me?";
    yInfo() << sSay;
    iCub->say(sSay);

    //Load the Speech Recognition with grammar according to entityType
    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(GrammarTypeAction), 20);

    if (bRecognized.get(0).asInt() == 0)
    {
        yError() << " error in proactiveTagging::askName | Error in speechRecog";
        bOutput.addString("error");
        bOutput.addString("error in speechRecog");
        return bOutput;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

    if (bAnswer.get(0).asString() == "stop")
    {
        yError() << " in proactiveTagging::askName | stop called";
        bOutput.addString("error");
        bOutput.addString("stop called");
        return bOutput;
    }

    bSemantic = *bAnswer.get(1).asList();
    yInfo() << " bAnswer = " << bAnswer.toString() ;
    yInfo() << " bSemantic = " << bSemantic.toString() ;
    string sType = bSemantic.check("actionType", Value("unknown")).asString();

    if(sType == "proto-action"){
        sSay = " Oh I cannot learn a proto-action right now";
        yInfo() << sSay;
        iCub->say(sSay);
        bOutput.addString("proto-action");
        return bOutput;
    }

    sSay = " Let's learn some " + sType;
    yInfo() << sSay;
    iCub->say(sSay);

    sSay = " What is the exact "  + sType + " that you will teach me?";
    yInfo() << sSay;
    iCub->say(sSay);

    //2. recog for name : I will teach you how to <close> your <hand>
    Bottle bReply = nodeNameAction(sType);  //provide bottle : <type> <name> <arg>. type == error if not good
    while(bReply.get(0).asString() == "error"){
        sSay = " This action is not corresponding to a " + sType + ". Can you repeat please?";
        yInfo() << sSay;
        iCub->say(sSay);
        bReply = nodeNameAction(sType);
    } //TODO : do/while and check if error?

    yInfo() << "Bottle from nodeNameAction = " << bReply.toString() ;

    string sName = bReply.get(1).asString();
    string sArg  = bReply.get(2).asString();

    //2. Check that action is not already learned (in vPrimitiveActionBottle; or vActionBottle)
    std::vector<yarp::os::Bottle> vAction ;
    if(sType == "primitive"){
        vAction =  vPrimitiveActionBottle ;
    } else {
        vAction = vActionBottle ; 
    }

    Bottle bAction;
    for(std::vector<yarp::os::Bottle>::iterator it = vAction.begin(); it < vAction.end(); it++){
        string currentName = it->get(0).toString();
        if(currentName == sName){
            yInfo() << "found " << currentName << "as a known primitive";
            string currentArg = it->get(1).toString();
            if(currentArg == sArg){
                yInfo() << "and we have a corresponding argument " << currentArg ;
                for(int i = 0; i < it->get(2).asList()->size(); i++){
                    bAction.addList() = *it->get(2).asList()->get(i).asList() ;
                }
                break;
            } else {
                yInfo() << " BUT argument " << currentArg << " does NOT match" ;
            }

        }
    }

    if (bAction.size() != 0){
        yError() << " error in proactiveTagging::learn | action '" << sName << " " << sArg << "' is known, then cannot be learned";
        bOutput.addString("error");
        bOutput.addString("action is known so cannot be learned");
        return bOutput ;
    }

    //3. Loop in recog to build with protoAction
    sSay = " Allright, can you describe how I can " + sName + " my " + sArg + ", please?";
   if(sArg == "one" || sArg == "two" || sArg == "three" || sArg == "four" || sArg == "five"){
        sSay = " Allright, can you describe how I can " + sName + sArg + ", please?";
    }
    yInfo() << sSay;
    iCub->say(sSay);
    string sTypeNeeded = "any" ;
    if(sType == "primitive"){
        sTypeNeeded = "proto-action" ;
    } //should be action cause proto-action quit before : any is fine then


    //Bottle bReplyFromNameAction = nodeNameAction(sTypeNeeded);  //provide bottle : <type> <name> <arg>. type == error if not good

    Bottle bDescriptionAction, bReplyFromNameAction ;
    do {

        bReplyFromNameAction = nodeNameAction(sTypeNeeded) ;
        yInfo() << "bReplyFromNameAction.get(0).asString() = " << bReplyFromNameAction.get(0).asString() ;
        if(bReplyFromNameAction.get(0).asString() != "error" && bReplyFromNameAction.get(0).asString() != "stop"){
            
            string sCurrentType = bReplyFromNameAction.get(0).asString();
            string sCurrentName = bReplyFromNameAction.get(1).asString();
            string sCurrentArg  = bReplyFromNameAction.get(2).asString();

            sSay = " I " + sCurrentName + " my " + sCurrentArg;
            yInfo() << sSay;
            iCub->say(sSay);

            //a. do the action
            Bottle bReplyAction ;
            if(sCurrentType == "proto-action"){
                bReplyAction = protoCommand(sCurrentName, sCurrentArg) ;
            } else if (sCurrentType == "primitive"){
                bReplyAction = primitiveCommand(sCurrentName, sCurrentArg) ;
            } else {
                //TODO : actionCommand
            }

            //b. write the action in the Bottle
            Bottle bCurrentAction ;
            Bottle bCurrentListArg ;
            bCurrentAction.addString(sCurrentName);
            bCurrentListArg.addString(sCurrentArg);
            bCurrentAction.addList() = bCurrentListArg;
            //it is because I need a list of arg to write the sequence but right now I only receive one string as arg by nodeNameAction. TODO : change to be generic in argument number
            bDescriptionAction.addList() = bCurrentAction ;

            yInfo() << "bDescriptionAction in lproto_oop : " << bDescriptionAction.toString() ;
            //bReplyFromNameAction = nodeNameAction(sTypeNeeded);
            //yInfo() << "INSIDE THE LOOP : bReplyFromNameAction.get(0).asString() = " << bReplyFromNameAction.get(0).asString() ;

        } else if (bReplyFromNameAction.get(0).asString() == "stop"){
            sSay = " Thank you for your instructions";
            yInfo() << sSay;
            iCub->say(sSay);            
        } else {
            sSay = " This action is not available to learn a " + sType + ". Can you use another one?";
            yInfo() << sSay;
            iCub->say(sSay);

            bReplyFromNameAction = nodeNameAction(sTypeNeeded);
        }
    } while (bReplyFromNameAction.get(0).asString() != "stop") ;



    yInfo() << "bDescriptionAction : " << bDescriptionAction.toString();
    //4. write it in memory
    if(sType == "primitive"){
        Bottle bPrimitive ;
        bPrimitive.addString(sName);
        bPrimitive.addString(sArg);
        bPrimitive.addList() = bDescriptionAction;

        yInfo() << "New Primitive added = " << bPrimitive.toString();
        vPrimitiveActionBottle.push_back(bPrimitive) ;

    } else {
        Bottle bAction ;
        bAction.addString(sName);
        bAction.addString(sArg);
        bAction.addList() = bDescriptionAction;

        yInfo() << "New Primitive added = " << bAction.toString();
        vActionBottle.push_back(bAction) ;
    }

    //5. write it in config file
    //write in ini file the stuff
    if(saveToIniFile(sType, sName, sArg, bDescriptionAction) == false){
        yError() << " error in proactiveTagging::learn | CANNOT save the action in " << pathToIniFile;
        bOutput.addString("error");
        bOutput.addString("CANNOT save the action in the ini file");
        return bOutput ;
    }

    bOutput.addString("ack");
    bOutput.addString(sName);
    bOutput.addString(sArg);

    return learn(); //loop the learning until "stop"
    //return bOutput; learn just once
}

//For now only control in position. Careful, angle is in percentage of maxAngle
bool learnPrimitive::saveToIniFile(string sType, string sName, string sArg, Bottle bDescriptionAction){

    if( sType != "primitive" && sType != "action") {
        yError() << "Wrong Type! => " << sType ;
        return false ;
    } 

    ifstream iniFile;
    iniFile.open(pathToIniFile.c_str(), ios::in | ios::out | ios::app);

    ofstream tempFile;
    string tempPath = pathToIniFile + ".tmp.xml" ;
    tempFile.open(tempPath.c_str(), ios::out);

    if(!tempFile.is_open()){
        yError() << "Cannot open the temp file in " << tempPath ;
    } else {
        yInfo() << " Temp file created in " << tempPath ;
    }

    string sLine;

    int lineNumber = 1;

    string sActionName, sActionArg ;
    if(sType == "primitive"){
        sActionName = "primitiveActionName";
        sActionArg = "primitiveActionArg";
     } else { //action
        sActionName = "ActionName";
        sActionArg = "ActionArg";
     }

    if(iniFile.is_open()){
        while(!iniFile.eof()){
            getline(iniFile, sLine);
            int beginActionName = -1;
            int beginActionArg = -1 ;
            int parenthesis = -1;

            beginActionName = sLine.find(sActionName);
            beginActionArg= sLine.find(sActionArg);

            
            if(beginActionName != -1){
                parenthesis = sLine.find(")");
                yInfo() << " primitive Action Name found in Line number " << lineNumber << " parenthesis position " << parenthesis ;

                string currentSubstrLine = sLine.substr(0, parenthesis);
                yInfo() << " Line without the parenthesis : " << currentSubstrLine ;
                currentSubstrLine = currentSubstrLine + " " + sName + ")" ;

                tempFile << currentSubstrLine << endl;
            } else  if (beginActionArg != -1) {
                parenthesis = sLine.find(")");
                yInfo() << " primitive Arg Name found in Line number " << lineNumber << " parenthesis position " << parenthesis ;

                string currentSubstrLine = sLine.substr(0, parenthesis);
                yInfo() << " Line without the parenthesis : " << currentSubstrLine ;
                currentSubstrLine = currentSubstrLine + " " + sArg + ")" ;

                tempFile << currentSubstrLine << endl;
            } else {
                tempFile << sLine <<endl;
            }

            lineNumber += 1;
        }
    }

    tempFile.close();
    iniFile.close();

    string saveIni = pathToIniFile + ".save";
    std::rename(pathToIniFile.c_str(), saveIni.c_str());
    std::rename(tempPath.c_str(), pathToIniFile.c_str()) ;


    ofstream iniFileOut;
    iniFileOut.open(pathToIniFile.c_str(), ios::out | ios::app);
    //add at the end
    if(iniFileOut.is_open()){
        iniFileOut << endl;
        string line = "[" + sName + "_" + sArg + "]";
        iniFileOut << line << endl ;
        line = "actionList    (" + bDescriptionAction.toString() + ")";
        iniFileOut << line <<endl ;

        iniFileOut.close();
    }

    return true;
}

//For now only control in position. Careful, angle is in percentage of maxAngle
Bottle learnPrimitive::learnPrim(){
    Bottle bOutput;

    //1. recog for name primitive : I will teach you how to <close> your <hand>
    bOutput = nodeNameAction("primitive");

    //2. Check that action is not already there (need to update list like in updateProtoAction)

    //3. Loop in recog to build with protoAction

    //4. write it

    return bOutput;
}

//For now only control in position. Careful, angle is in percentage of maxAngle
Bottle learnPrimitive::learnAction(){
    Bottle bOutput;

    //1. recog for name primitive : I will teach you how to <close> your <hand>
    bOutput = nodeNameAction("action");

    //2. Check that action is not already there (need to update list like in updateProtoAction)

    //3. Loop in recog to build with protoAction

    //4. write it

    return bOutput;
}

//For now only control in position. Careful, angle is in percentage of maxAngle
Bottle learnPrimitive::protoCommand(string sActionName, string sBodyPartName, int maxAngle){
    Bottle bOutput;

    if(sBodyPartName == "ring" || sBodyPartName == "little"){
        maxAngle = 250 ;
    } else if (sBodyPartName == "thumb" || sBodyPartName == "index" || sBodyPartName == "middle") {
        maxAngle = 90 ;
    }

    int targetAngle = 0;
    //1. check if proto is known
    if ( mProtoActionEnd.find(sActionName) == mProtoActionEnd.end() ) {
        yError() << " error in learnPrimitive::protoCommand | for " << sActionName << " | sActionName is unknown";
        bOutput.addString("error");
        bOutput.addString("sActionName is unknown");
        return bOutput;
    } else {
        targetAngle = mProtoActionEnd.at(sActionName);
    }

    //yInfo() << " Basic angle position for action " << sActionName << " is " << targetAngle ;

    //2. if yes, check if bodypart is known. Warning if not found for generalization
    int effectAngleBodyPart = 0 ;
    if ( mBodyPartEnd.find(sBodyPartName) == mBodyPartEnd.end() ) {
        yWarning() << " warning in learnPrimitive::protoCommand | for " << sBodyPartName << " | not protoaction effect define, GENERALIZATION MODE then";
    } else {
        effectAngleBodyPart = mBodyPartEnd.at(sBodyPartName.c_str());
       // yInfo() << " effect of the bodypart : " <<  effectAngleBodyPart ;
    }

    //3. add protoaction with bodypart effect. Careful, Angle is in percentage of max angle
    yInfo() << " Target Angle Final (percentage) : " << (targetAngle + effectAngleBodyPart) << ", maxAngle = " << maxAngle ;
    int finalTargetAngle = (targetAngle + effectAngleBodyPart)*maxAngle/100;

    //angle should be at least 0? check with min angle otherwise
    if(finalTargetAngle < 0){
        yInfo() << "Angle cannot be negative value : from " << finalTargetAngle << " to 0" ;
        finalTargetAngle = 0;
    }
    yInfo() << " Target Angle Final : " << finalTargetAngle ;
    //TODOOOOOOOOOOOOOOOOOOOOOOOOOOO : For now use maxangle provided but should extract it or provided in the OPC bodypart!

    iCub->opc->checkout();
    //should check at some point that the bodypart is there and loaded no?
    Bodypart* bp = dynamic_cast<Bodypart*>(iCub->opc->getEntity(sBodyPartName));
    int joint = bp->m_joint_number ;

    //4. execute action.
    Bottle bToArm;
    bToArm.addString("set");
    bToArm.addString("pos");
    bToArm.addInt(joint);
    bToArm.addInt(finalTargetAngle);

    yInfo() << " cmd sent : " << bToArm.toString();

    portToArm.write(bToArm, bOutput);

    return bOutput;
}


Bottle learnPrimitive::actionCommand(string sActionName, string sArg){

    Bottle bOutput;

    //1. check if primitive is known
    //   vPrimitiveActionBottle =
    //   open    (hand)     ( (unfold thumb) (unfold index) (unfold middle) (unfold ring) )
    //   close   (hand)     ( (fold thumb) (fold index) (fold middle) (fold ring) )
    //   b.get(1) b.get(2)  b.get(3)
    //   name     arg        list of proto-action
    Bottle bSubActionList;
    for(std::vector<yarp::os::Bottle>::iterator it = vActionBottle.begin(); it < vActionBottle.end(); it++){
        string currentName = it->get(0).toString();
        //yInfo() << "Current name of the knwon actions : " << currentName ;
        if(currentName == sActionName){
            yInfo() << "found " << currentName << "as a known complex action";
            string currentArg = it->get(1).toString();
            if(currentArg == sArg){
                yInfo() << "and we have a corresponding argument " << currentArg ;
                for(int i = 0; i < it->get(2).asList()->size(); i++){
                    bSubActionList.addList() = *it->get(2).asList()->get(i).asList() ;
                }
                break;
            } else {
                yInfo() << " BUT argument " << currentArg << " does NOT match" ;
            }

        }
    }

    if (bSubActionList.size() == 0){
        yError() << " error in learnPrimitive::actionCommand | action '" << sActionName << " " << sArg << "' is NOT known";
        bOutput.addString("error");
        bOutput.addString("action is NOT known");
        return bOutput ;
    }

    yInfo() << "Actions to do : " << bSubActionList.toString() ;

    for(int i = 0; i < bSubActionList.size(); i++){
        yInfo() << "action #" << i << " : "<< bSubActionList.get(i).asList()->get(0).toString() << " the " << bSubActionList.get(i).asList()->get(1).asList()->get(0).toString() ;

        //1. check if subaction is a proto 
        if ( mProtoActionEnd.find(bSubActionList.get(i).asList()->get(0).toString()) != mProtoActionEnd.end() ) {  //proto-action
            bOutput.addList() = protoCommand(bSubActionList.get(i).asList()->get(0).toString(), bSubActionList.get(i).asList()->get(1).asList()->get(0).toString());
        } else { //primitive
            bOutput.addList() = primitiveCommand(bSubActionList.get(i).asList()->get(0).toString(), bSubActionList.get(i).asList()->get(1).asList()->get(0).toString());
            yarp::os::Time::delay(2);
        } //else { //another action

        //}
    }

    bOutput.addString("ack");

    return bOutput;
}

Bottle learnPrimitive::primitiveCommand(string sActionName, string sArg){

    Bottle bOutput;

    //1. check if primitive is known
    //   vPrimitiveActionBottle =
    //   open    (hand)     ( (unfold thumb) (unfold index) (unfold middle) (unfold ring) )
    //   close   (hand)     ( (fold thumb) (fold index) (fold middle) (fold ring) )
    //   b.get(1) b.get(2)  b.get(3)
    //   name     arg        list of proto-action
    Bottle bProtoActionList; //primitive only composed by proto-action
    for(std::vector<yarp::os::Bottle>::iterator it = vPrimitiveActionBottle.begin(); it < vPrimitiveActionBottle.end(); it++){
        string currentName = it->get(0).toString();
        if(currentName == sActionName){
            yInfo() << "found " << currentName << "as a known primitive";
            string currentArg = it->get(1).toString();
            if(currentArg == sArg){
                yInfo() << "and we have a corresponding argument " << currentArg ;
                for(int i = 0; i < it->get(2).asList()->size(); i++){
                    bProtoActionList.addList() = *it->get(2).asList()->get(i).asList() ;
                }
                break;
            } else {
                yInfo() << " BUT argument " << currentArg << " does NOT match" ;
            }

        }
    }

    if (bProtoActionList.size() == 0){
        yError() << " error in learnPrimitive::primitiveCommand | action '" << sActionName << " " << sArg << "' is NOT known";
        bOutput.addString("error");
        bOutput.addString("action is NOT known");
        return bOutput ;
    }

    yInfo() << "Actions to do : " << bProtoActionList.toString() ;

    for(int i = 0; i < bProtoActionList.size(); i++){
        yInfo() << "action #" << i << " : "<< bProtoActionList.get(i).asList()->get(0).toString() << " the " << bProtoActionList.get(i).asList()->get(1).asList()->get(0).toString() ;
        bOutput.addList() = protoCommand(bProtoActionList.get(i).asList()->get(0).toString(), bProtoActionList.get(i).asList()->get(1).asList()->get(0).toString());
    }

    bOutput.addString("ack");

    return bOutput;
}

bool learnPrimitive::updateProtoAction(ResourceFinder &rf){
    Bottle bOutput;

    //1. Protoaction
    Bottle bProtoAction = rf.findGroup("Proto_Action");

    if (!bProtoAction.isNull())
    {
        Bottle * bProtoActionName = bProtoAction.find("protoActionName").asList();
        Bottle * bProtoActionEnd = bProtoAction.find("protoActionEnd").asList();
        Bottle * bProtoActionSpeed = bProtoAction.find("protoActionSpeed").asList();

       if(bProtoActionName->isNull() || bProtoActionEnd->isNull() || bProtoActionSpeed->isNull()){
           yError() << " [updateProtoAction] : one of the protoAction conf is null : protoActionName, protoActionEnd, protoActionSpeed" ;
            return false ;
        }


        int protoActionSize = -1 ;
        if(bProtoActionName->size() == bProtoActionEnd->size() && bProtoActionEnd->size() == bProtoActionSpeed->size()){
            protoActionSize =  bProtoActionName->size() ;
        } else {
            yError() << " [updateProtoAction] : one of the protoAction conf has different size!" ;
            return false ;
        }

        if(protoActionSize == 0) {
            yWarning() << " [updateProtoAction] : there is no protoaction defined at startup!" ;
        } else {

            for(int i = 0; i < protoActionSize ; i++) {

                //insert protoaction even if already there
                yInfo() << "ProtoAction added : (" << bProtoActionName->get(i).asString() << ", " << bProtoActionEnd->get(i).asInt() << ", " << bProtoActionSpeed->get(i).asDouble() << ")" ;
                mProtoActionEnd[bProtoActionName->get(i).asString()] = bProtoActionEnd->get(i).asInt();
                mProtoActionSpeed[bProtoActionName->get(i).asString()] = bProtoActionSpeed->get(i).asDouble();
            }
        }
    } else {
        yError() << " error in learnPrimitive::updateProtoAction | Proto_Action is NOT defined in the learnPrimitive.ini";
        return false;
    }

    //2. Effect of bodypart
    Bottle bBodyPart = rf.findGroup("BodyPart");

    if (!bBodyPart.isNull())
    {
        Bottle * bBodyPartName = bBodyPart.find("bodyPartName").asList();
        Bottle * bBodyPartEnd = bBodyPart.find("bodyPartProtoEnd").asList();
        Bottle * bBodyPartSpeed = bBodyPart.find("bodyPartProtoSpeed").asList();
        
        //Crash if no match : isnull is not good for that

        if(bBodyPartName->isNull() || bBodyPartEnd->isNull() || bBodyPartSpeed->isNull()){
            yError() << "[updateProtoAction] : one of the bodyPartProto conf is null : bodyPartName, bodyPartProtoEnd, bodyPartProtoSpeed" ;
            return false ;
        }


        int bodyPartSize = -1 ;
        if(bBodyPartName->size() == bBodyPartEnd->size() && bBodyPartEnd->size() == bBodyPartSpeed->size()){
            bodyPartSize =  bBodyPartName->size() ;
        } else {
            yError() << "[updateProtoAction] : one of the bodyPartProto conf has different size!" ;
            return false ;
        }

        if(bodyPartSize == 0) {
            yWarning() << "[updateProtoAction] : there is no bodyPartProto defined at startup!" ;
        } else {
            for(int i = 0; i < bodyPartSize ; i++) {

                //insert protoaction even if already there
                yInfo() << "bodyPartProto added : (" << bBodyPartName->get(i).asString() << ", " << bBodyPartEnd->get(i).asInt() << ", " << bBodyPartSpeed->get(i).asDouble() << ")" ;
                mBodyPartEnd[bBodyPartName->get(i).asString()] = bBodyPartEnd->get(i).asInt();
                mBodyPartSpeed[bBodyPartName->get(i).asString()] = bBodyPartSpeed->get(i).asDouble();
            }
        }
    } else {
        yError() << " error in learnPrimitive::updateProtoAction | BodyPart is NOT defined in the learnPrimitive.ini";
        return false;
    }

    return true;
}

bool learnPrimitive::updatePrimitive(ResourceFinder &rf){
    Bottle bOutput;

    //1. Primitive
    Bottle bPrimitiveAction = rf.findGroup("Primitive_Action");

    if (!bPrimitiveAction.isNull())
    {
        Bottle * bPrimitiveActionName = bPrimitiveAction.find("primitiveActionName").asList();
        Bottle * bPrimitiveActionArg  = bPrimitiveAction.find("primitiveActionArg").asList();

       if(bPrimitiveActionName->isNull() || bPrimitiveActionArg->isNull()){
           yError() << " [updatePrimitiveAction] : one of the primitiveAction conf is null : primitiveActionName, primitiveActionArg" ;
            return false ;
        }

        int primitiveActionSize = -1 ;
        if(bPrimitiveActionName->size() == bPrimitiveActionArg->size()){
            primitiveActionSize =  bPrimitiveActionName->size() ;
        } else {
            yError() << " [updatePrimitiveAction] : one of the primitiveAction conf has different size!" ;
            return false ;
        }

        if(primitiveActionSize == 0) {
            yWarning() << " [updatePrimitiveAction] : there is no primitiveAction defined at startup!" ;
        } else {

            for(int i = 0; i < primitiveActionSize ; i++) {
                //insert protoaction even if already there
                Bottle bPrimitive;
                string currentPrimName = bPrimitiveActionName->get(i).asString();
                string currentPrimArg  = bPrimitiveActionArg->get(i).asString();

                bPrimitive.addString(currentPrimName);
                bPrimitive.addString(currentPrimArg);

                string concat = currentPrimName + "_" + currentPrimArg;
                Bottle bCurrentPrim = rf.findGroup(concat);
                if(bCurrentPrim.isNull()){
                    yError() << " [updatePrimitiveAction] : " << concat << "is NOT defined" ;
                    return false ;
                }
                Bottle * bListProtoAction = bCurrentPrim.find("actionList").asList();
                if(bListProtoAction->isNull()){
                    yError() << " [updatePrimitiveAction] : " << concat << "is there but is not defined (actionList)" ;
                    return false ;
                }
                bPrimitive.addList() = *bListProtoAction ;
                yInfo() << "Primitive Action added : (" <<bPrimitive.get(0).asString() << ", " << bPrimitive.get(1).asString() << ", ( " << bPrimitive.get(2).asList()->toString()<< "))" ;
                vPrimitiveActionBottle.push_back(bPrimitive);
            }
        }
    } else {
        yError() << " error in learnPrimitive::updatePrimitive | Primitive_Action is NOT defined in the learnPrimitive.ini";
        return false;
    }


    return true;
}

bool learnPrimitive::updateAction(ResourceFinder &rf){
    Bottle bOutput;

    //1. Primitive
    Bottle bAction = rf.findGroup("Action");

    if (!bAction.isNull())
    {
        Bottle * bActionName = bAction.find("ActionName").asList();
        Bottle * bActionArg  = bAction.find("ActionArg").asList();

       if(bActionName->isNull() || bActionArg->isNull()){
           yError() << " [updateAction] : one of the primitiveAction conf is null : ActionName, ActionArg" ;
            return false ;
        }

        int actionSize = -1 ;
        if(bActionName->size() == bActionArg->size()){
            actionSize =  bActionName->size() ;
        } else {
            yError() << " [updateAction] : one of the Action conf has different size!" ;
            return false ;
        }

        if(actionSize == 0) {
            yWarning() << " [updateAction] : there is no Action defined at startup!" ;
        } else {

            for(int i = 0; i < actionSize ; i++) {
                //insert protoaction even if already there
                Bottle bSubAction;
                string currentActionName = bActionName->get(i).asString();
                string currentActionArg  = bActionArg->get(i).asString();

                bSubAction.addString(currentActionName);
                bSubAction.addString(currentActionArg);

                string concat = currentActionName + "_" + currentActionArg;

               // yInfo() << " Looking for " << concat ;

                Bottle bCurrent = rf.findGroup(concat);
                if(bCurrent.isNull()){
                    yError() << " [updateAction] : " << concat << "is NOT defined" ;
                    return false ;
                }

                Bottle * bListSubAction = bCurrent.find("actionList").asList();

                //yInfo() << "Subaction found : " << bListSubAction->toString() ;
                if(bListSubAction->isNull()){
                    yError() << " [updateAction] : " << concat << "is there but is not defined (actionList)" ;
                    return false ;
                }

                bSubAction.addList() = *bListSubAction ;
                yInfo() << "Complex Action added : (" <<bSubAction.get(0).asString() << ", " << bSubAction.get(1).asString() << ", ( " << bSubAction.get(2).asList()->toString()<< "))" ;
                vActionBottle.push_back(bSubAction);
            }
        }
    } else {
        yError() << " error in learnPrimitive::updatePrimitive | Primitive_Action is NOT defined in the learnPrimitive.ini";
        return false;
    }


    return true;
}

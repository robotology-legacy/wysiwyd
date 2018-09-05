#include "abmInteraction.h"
#include "wrdac/subsystems/subSystem_ABM.h"
#include "wrdac/subsystems/subSystem_recog.h"

/*
* Configure method, connect to different subsystem
*/
bool abmInteraction::configure(yarp::os::ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("abmInteraction")).asString().c_str();
    setName(moduleName.c_str());

    nameGrammarHumanFeedback = rf.findFileByName(rf.check("nameGrammarHumanFeedback", Value("hFeedback.xml")).toString());
    //nameGrammarHumanFeedback = rf.findFileByName(rf.check("nameGrammarHumanFeedback", Value("hFeedbackDescription")).toString());
    
    nameGrammarYesNo = rf.findFileByName(rf.check("nameGrammarYesNo", Value("nodeYesNo.xml")).toString());

    cout << moduleName << ": finding configuration files..." << endl;
    period = rf.check("period", Value(0.1)).asDouble();

    //tryAgain = false ; //at start, iCub will explain and speaks
    //bestRank = 0 ;

    //Create an iCub Client and check that all dependencies are here before starting
    bool isRFVerbose = false;
    iCub = new ICubClient(moduleName, "abmInteraction", "client.ini", isRFVerbose);
    iCub->opc->isVerbose = false;
    if (!iCub->connect())
    {
        cout << "iCubClient : Some dpeendencies are not running..." << endl;
        Time::delay(1.0);
    }

    rpc.open(("/" + moduleName + "/rpc").c_str());
    attach(rpc);

    if (!iCub->getRecogClient())
    {
        cout << "WARNING SPEECH RECOGNIZER NOT CONNECTED" << endl;
    }
    if (!iCub->getABMClient())
    {
        cout << "WARNING ABM NOT CONNECTED" << endl;
    }

    rememberedInstance = rf.check("rememberedInstance", Value(1333)).asInt();
    agentName = rf.check("agentName", Value("Bob")).asString().c_str();
    img_provider_port = rf.check("img_provider_port", Value("/icub/camcalib/left/out/kinematic_structure")).asString().c_str();
    
    //resume : no (take all the augmented_time from the instance)
    //resume : yes (take only the augmented_time without feedback from the instance)
    //resume : agent (take only the augmented_time with no feedback from the agent, from the instance)
    resume = rf.check("resume", Value("agent")).asString().c_str();

    it_augmentedTime = vAugmentedTime.begin();
    /*pair<string, int> bestTimeAndRank("", -1);

    if (createAugmentedTimeVector(bestTimeAndRank) == -1){
        yError() << " Something is wrong with the augmented memories! quit";
        return false;
    }*/

    //testing
    /*Bottle bRecognized;
    bool blop = true;
    while (blop){

    iCub->say("Check for sending a grammar");

    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarHumanFeedback));
    yInfo() << "bRecognized " << bRecognized.toString();
    cout << bRecognized.get(1).toString() << endl;
    bRecognized.clear();
    }   */
    //end testing

    return true;
}


bool abmInteraction::close() {
    iCub->close();
    delete iCub;

    return true;
}


bool abmInteraction::respond(const Bottle& bCommand, Bottle& bReply) {
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit \n";

    Bottle bError;
    bReply.clear();

    if (bCommand.get(0).asString() == "set")
    {
        bool changeSomething = false;

        if (bCommand.size() > 1)
        {
            Value vRememberedInstance = bCommand.find("rememberedInstance");
            if (!vRememberedInstance.isNull() && vRememberedInstance.isInt()) {
                rememberedInstance = vRememberedInstance.asInt();
                changeSomething = true;
            }

            Value vImgProviderPort = bCommand.find("img_provider_port");
            if (!vImgProviderPort.isNull() && vImgProviderPort.isString()) {
                img_provider_port = vImgProviderPort.asString();
                changeSomething = true;
            }

            Value vAgentName = bCommand.find("agentName");
            if (!vAgentName.isNull() && vAgentName.isString()) {
                agentName = vAgentName.asString();
                changeSomething = true;
            }

            //resum : check for augmented_time with empty feedback from the agentName. Another one needed to check empty feedback, no matter the agent?
            Value vResume = bCommand.find("resume");
            if (!vResume.isNull() && vResume.isString()) {

                if(vResume.asString() != "agent" && vResume.asString() != "yes" && vResume.asString() != "no"){
                    string sError = "[set]: Wrong resume keyWord : should be 1) yes 2) no or 3) agent (for agent specific resume)";
                    yError() << sError;
                    bError.addString(sError);
                    bReply = bError;
                } else {
                    resume = vResume.asString() ;
                    changeSomething = true;
                }
            }

            yDebug() << "rememberedInstance: " << rememberedInstance;
            yDebug() << "img_provider_port: " << img_provider_port;
            yDebug() << "agentName: " << agentName;
            yDebug() << "resume: " << resume;

            bReply.addString("ack");
        }
        else
        {
            string sError = "[set]: Wrong Bottle  => set (rememberedInstance int) (img_provider_port string) (agentName string)";
            yError() << sError;
            bError.addString(sError);
            bReply = bError;
        }

        if (!changeSomething) {
            string sError = "Nothing has been changed, check the Bottle";
            yError() << sError;
            bError.addString(sError);
            bReply = bError;
        }
    }

    if (bCommand.get(0).asString() == "runFeedback") {
        bReply.addString("runFeedback");

        pair <string, int> bestTimeAndRank;

        showBestOfAllForAgent();

        createAugmentedTimeVector(bestTimeAndRank) ;
        if (bestTimeAndRank.second == -1){
            yError() << " Something is wrong with the augmented memories! quit";
            return false;
        }

        nodeFeedback(false, bestTimeAndRank);

        return false;
    }

    if (bCommand.get(0).asString() == "quit") {
        bReply.addString("quitting");
        return false;
    }
    else if (bCommand.get(0).asString() == "help") {
        cout << helpMessage;
        bReply.addString("ok");
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool abmInteraction::updateModule() {
    //  mainLoop();
    return true;
}

/*
* Node to ask Human to give feedback on quality of augmented kinematic structure image
*/
void    abmInteraction::nodeFeedback(bool tryAgain, pair<string,int> & bestTimeAndRank)
{
    ostringstream osError;          // Error message
    Bottle bOutput;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bMessenger; //to be send TO speech recog
    ostringstream osResponse;

    /************************************ Usual feedback : show an image and ask for feedback ************************************/
    if (tryAgain == false){

        yInfo() << " Current time = " << *it_augmentedTime;
        iCub->say("Note this kinematic structure between 1 and 10 please", false);
        yInfo() << " iCub says : Note this kinematic structure between 1 and 10 please";

        //feedback likert 1-5
        /*iCub->say("Note this kinematic structure, Likert 1-5 quality");
        yInfo() << " iCub says : Note this kinematic structure, Likert 1-5 quality" ;*/

        //Preparing bottle to trigger the augmenting remembering
        Bottle bRpc, bSubRealtime, bSubAugmentedTimes;

        bRpc.addString("triggerStreaming");
        bRpc.addInt(rememberedInstance);

        bSubRealtime.addString("realtime");
        bSubRealtime.addInt(1);

        bSubAugmentedTimes.addString("augmentedTimes");
        bSubAugmentedTimes.addString(*it_augmentedTime);

        //If we have a previously best rank
        if (bestTimeAndRank.second != 0) {
            osResponse.str("");
            osResponse << "The current best structure is shown at left. The rank is " << bestTimeAndRank.second; //<< " for time = " << bestAugmentedTime ;
            iCub->say(osResponse.str().c_str(), false);
            yInfo() << "iCub says : " << osResponse.str();

            bSubAugmentedTimes.addString(bestTimeAndRank.first);
        }

        //Ask for showing the current testing augmented + the best one if relevant
        bRpc.addList() = bSubRealtime;
        bRpc.addList() = bSubAugmentedTimes;

        iCub->getABMClient()->rpcCommand(bRpc);

    }
    else {
        iCub->say("Can you repeat your feedback please?", false);
        yInfo() << "iCub says : Can you repeat your feedback please?";
    }



    /************************************ Retrieve Human spoken feedback ************************************/


    bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarHumanFeedback));

    if (bRecognized.get(0).asInt() == 0)
    {
        return;
    }

    bAnswer = *bRecognized.get(1).asList();
    // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)


    if (bAnswer.get(0).asString() == "stop")
    {
        osError.str("");
        osError << " | STOP called";
        bOutput.addString(osError.str());
        cout << osError.str() << endl;
    }

    yInfo() << "bRecognized " << bRecognized.toString();
    cout << bRecognized.get(1).toString() << endl;

    string sQuestionKind = bAnswer.get(1).asList()->get(0).toString();

    //feedback 1-10
    if (sQuestionKind == "FEEDBACK") {
        yInfo() << "FEEDBACK received from Human!";
    }
    else {
        yError() << " The sentence type is not recognized, waiting for FEEDBACK";
        tryAgain = true;
        nodeFeedback(tryAgain, bestTimeAndRank);
        return;
    }


    // semantic is the list of the semantic elements of the sentence except the type ef sentence
    bSemantic = *bAnswer.get(1).asList()->get(1).asList();

    //feedback number 1-10
    string sFeedback10 = bSemantic.check("feedback10", Value("0")).asString();
    int iFeedback10 = atoi(sFeedback10.c_str());

    //feedback likert 1-5 quality
    /*string sFeedback10 = bAnswer.get(1).asList()->get(0).asString() ;
    int iFeedback10    = atoi(sFeedback10.c_str())*2 ;*/


    /************************************ Ask confirmation for the feedback (i.e. if misrecognition) ************************************/
    osResponse.str("");
    osResponse << "So for you, this kinematic structure has a score of " << iFeedback10 << ", right?";
    iCub->say(osResponse.str().c_str(), false);
    yInfo() << "iCub says : " << osResponse.str();

    if (!nodeYesNo())
    {
        osResponse.str("");
        osResponse << "Oups, I am sorry";
        tryAgain = true;
        iCub->say(osResponse.str().c_str(), false);
        yInfo() << "iCub says : " << osResponse.str();
        nodeFeedback(tryAgain, bestTimeAndRank);

        return;
    }
    else {
        tryAgain = false;
    }


    /************************************ put the feedback into ABM, feedback table ************************************/
    list<pair<string, string> > lArgument;
    lArgument.push_back(pair<string, string>("Bob", "agent"));
    lArgument.push_back(pair<string, string>("kinematic structure", "about"));
    iCub->getABMClient()->sendActivity("action", "sentence", "feedback", lArgument, true);

    Bottle bResult;
    ostringstream osRequest;
    //only augmented_time is needed but better clarity for the print
    osRequest << "SELECT instance FROM main WHERE activitytype = 'feedback' ORDER BY \"time\" DESC LIMIT 1 ;";
    bResult = iCub->getABMClient()->requestFromString(osRequest.str().c_str());
    int feedbackInstance = -1;
    feedbackInstance = atoi(bResult.get(0).asList()->get(0).toString().c_str());

    yInfo() << "Feedback instance stored in main (from Bottle) : " << bResult.get(0).asList()->get(0).toString().c_str();
    yInfo() << "Feedback instance stored in main (from feedbackInstance) : " << feedbackInstance;

    //insert the feedback to the SQL database
    insertFeedback(iFeedback10, feedbackInstance);


    /************************************ Update best feedback ************************************/
    if (iFeedback10 > bestTimeAndRank.second) {
        bestTimeAndRank.first = *it_augmentedTime;
        bestTimeAndRank.second = iFeedback10;

        osResponse.str("");
        osResponse << "Yes, I have improved my skills : best rank is now " << bestTimeAndRank.second;
        iCub->say(osResponse.str().c_str(), false);
        yInfo() << "iCub says : " << osResponse.str();

    }
    else {

        osResponse.str("");
        osResponse << "Erf, too bad";
        iCub->say(osResponse.str().c_str(), false);
        yInfo() << "iCub says : " << osResponse.str();

    }



    //Check that we still have augmented feedback to do
    if (++it_augmentedTime == vAugmentedTime.end()){
        osResponse.str("");
        osResponse << "I have no more augmented to check, thank you for your feedback";
        iCub->say(osResponse.str().c_str(), false);
        yInfo() << "iCub says : " << osResponse.str();

        return;
    }

    osResponse.str("");
    osResponse << "Another one?";
    iCub->say(osResponse.str().c_str(), false);
    yInfo() << "iCub says : " << osResponse.str();

    if (nodeYesNo())
    {
        nodeFeedback(tryAgain, bestTimeAndRank);
        return;
    }
    else
    {

        osResponse.str("");
        osResponse << "Ok, thanks anyway, bye";
        iCub->say(osResponse.str().c_str(), false);
        yInfo() << "iCub says : " << osResponse.str();

        //set back default value
        bestTimeAndRank.first = "";
        bestTimeAndRank.second = 0;


        return;
    }
}


/*
* Yes/No asking for confirmation, another trial, ...
*/
bool abmInteraction::nodeYesNo()
{
    //bool fGetaReply = false;
    Bottle bRecognized, //recceived FROM speech recog with transfer information (1/0 (bAnswer))
        bAnswer, //response from speech recog without transfer information, including raw sentence
        bSemantic, // semantic information of the content of the recognition
        bSendReasoning, // send the information of recall to the abmReasoning
        bMessenger; //to be send TO speech recog

    string sYesNo = "unknown";

    while (!((sYesNo == "yes") || (sYesNo == "no"))){

        bRecognized = iCub->getRecogClient()->recogFromGrammarLoop(grammarToString(nameGrammarYesNo));

        if (bRecognized.get(0).asInt() == 0)
        {
            cout << bRecognized.get(1).toString() << endl;
            return false;
        }

        bAnswer = *bRecognized.get(1).asList();
        // bAnswer is the result of the regognition system (first element is the raw sentence, 2nd is the list of semantic element)

        yInfo() << " bAnswer : " << bRecognized.get(1).toString();
        yInfo() << " Yes/No : " << bAnswer.get(1).asList()->get(0).asString();

        sYesNo = bAnswer.get(1).asList()->get(0).asString();

        if (sYesNo == "yes")        return true;
        else if (sYesNo == "no")        return false;

        //if here, recognition of the default background grammar : should repeat (with sending again grammar?) TOCHECK
        iCub->say("I am sorry, can you repeat please?", false);

    }



    return false;
}

/*
* create the vAugmentedTime for the current used Instance : sql to obtain the foreign key for the feedback table
*
*/
bool abmInteraction::createAugmentedTimeVector(pair<string,int> & bestTimeAndRank)
{
    Bottle bResult;
    ostringstream osRequest;

    //only augmented_time is needed but better clarity for the print
    osRequest << "SELECT DISTINCT instance, augmented_time, img_provider_port FROM visualdata WHERE instance = " << rememberedInstance << " AND augmented IS NOT NULL AND img_provider_port = '" << img_provider_port << "' " ;
    if(resume == "agent"){
        // not proposing instance with feedback from the agent already. 
        osRequest << "AND augmented_time NOT IN (SELECT DISTINCT augmented_time FROM feedback WHERE instance = '" << rememberedInstance << "' AND agent = '" << agentName << "') " ;
        createBestAugmentedTime(bestTimeAndRank) ;
    } else if (resume == "yes") {
        osRequest << "AND augmented_time NOT IN (SELECT DISTINCT augmented_time FROM feedback WHERE instance = '" << rememberedInstance << "') " ;
        createBestAugmentedTime(bestTimeAndRank);
    } else if (resume == "no") {
        //come back to default value for best rank/time
        bestTimeAndRank.first = "";
        bestTimeAndRank.second = 0;
    }

    osRequest << " ORDER BY augmented_time ASC ;";

    osRequest << " ;";
    bResult = iCub->getABMClient()->requestFromString(osRequest.str());
    yInfo() << "[createAugmentedTimeVector] SQL request bReply : " << bResult.toString();
    vAugmentedTime.clear();

    if (bResult.toString() != "NULL") {
        for (int i = 0; i < bResult.size(); i++){
            //get(1) because augmented is in second column of the result
            vAugmentedTime.push_back(bResult.get(i).asList()->get(1).toString());
        }
    }
    else {
        yError() << "Request to obtain augmented from instance " << rememberedInstance << " is NULL : are you sure there are some augmented images?";
        bestTimeAndRank.first = "";
        bestTimeAndRank.second = -1;
        return false;
    }

    ostringstream osAugmentedTime;
    const char* const delim = ", ";

    copy(vAugmentedTime.begin(), vAugmentedTime.end(), std::ostream_iterator<std::string>(osAugmentedTime, delim));
    yInfo() << " vAugmentedTime = " << osAugmentedTime.str();

    it_augmentedTime = vAugmentedTime.begin();

    return true;
}

/*
* update bestRank and bestAugmentedTime, depending on the resume
*
*/
bool abmInteraction::createBestAugmentedTime(pair<string,int> & bestTimeAndRank)
{
    Bottle bResult;
    ostringstream osRequest;

    //main select querry
    osRequest << "SELECT feedback.augmented_time, feedback.value FROM visualdata, feedback WHERE instance = " << rememberedInstance << " AND augmented IS NOT NULL AND img_provider_port = '" << img_provider_port << "' AND ";
    //foreign key to join feedback/visualdata
    osRequest << "augmented_port = img_provider_port AND original_time = \"time\" AND feedback.augmented_time = visualdata.augmented_time " ;
    if(resume == "agent"){
        // not proposing instance with feedback from the agent already. 
        osRequest << "AND agent = '" << agentName << "' " ;
    } 
    //just the best feedback value
    osRequest << " ORDER BY value DESC LIMIT 1;";

    bResult = iCub->getABMClient()->requestFromString(osRequest.str());
    //yInfo() << "[createBestAugmentedTime] SQL request bRequest : " << osRequest.str();
    //yInfo() << "[createBestAugmentedTime] SQL request bReply : " << bResult.toString();

    if (bResult.toString() != "NULL" && bResult.size() > 0) {
            bestTimeAndRank.first = bResult.get(0).asList()->get(0).toString();
            bestTimeAndRank.second = atoi(bResult.get(0).asList()->get(1).toString().c_str());

            yInfo() << "[createBestAugmentedTime] bestAugmentedTime : " << bestTimeAndRank.first << " with rank = " << bestTimeAndRank.second;
    }
    else {
        yInfo() << "No feedback yet according to resume setup for instance " << rememberedInstance ;
        bestTimeAndRank.first = "";
        bestTimeAndRank.second = 0;
        
        return false;
    }

    return true;
}


/*
* Show best augmented so far for the current agent, no matter the instance
*
*/
bool abmInteraction::showBestOfAllForAgent()
{
    Bottle bResult;
    ostringstream osRequest;

    //main select querry
    osRequest << "SELECT feedback.augmented_port, feedback.original_time, feedback.augmented_time, feedback.value FROM visualdata, feedback WHERE augmented IS NOT NULL AND img_provider_port = '" << img_provider_port << "' AND ";
    //foreign key to join feedback/visualdata
    osRequest << "augmented_port = img_provider_port AND original_time = \"time\" AND feedback.augmented_time = visualdata.augmented_time ";
    if (resume == "agent"){
        // not proposing instance with feedback from the agent already.
        osRequest << "AND agent = '" << agentName << "' ";
    }
    //just the best feedback value
    osRequest << " ORDER BY value DESC LIMIT 1;";



    bResult = iCub->getABMClient()->requestFromString(osRequest.str());

    if (bResult.toString() != "NULL" && bResult.size() > 0) {

        //take the data from best
        string augmented_port = bResult.get(0).asList()->get(0).toString();
        string original_time = bResult.get(0).asList()->get(1).toString();
        string augmented_time = bResult.get(0).asList()->get(2).toString();
        int value = atoi(bResult.get(0).asList()->get(3).toString().c_str());

        bResult.clear();

        //find the corresponding instance number
        osRequest.str("");
        osRequest << "SELECT instance FROM visualdata WHERE visualdata.time = '" << original_time << "' AND img_provider_port = '" << augmented_port << "' AND augmented_time = '" << augmented_time << "';";

        bResult = iCub->getABMClient()->requestFromString(osRequest.str());

        if (bResult.toString() != "NULL" && bResult.size() > 0){
            int instance = atoi(bResult.get(0).asList()->get(0).toString().c_str());
            yInfo() << "[showBestOfAll] best feedback : " << " instance = " << instance << " augmented_time = " << augmented_time << " with rank = " << value;

            //show the best with triggerstreaming
            Bottle bRpc, bSubRealtime, bSubAugmentedTimes;

            bRpc.addString("triggerStreaming");
            bRpc.addInt(instance);

            bSubRealtime.addString("realtime");
            bSubRealtime.addInt(1);

            bSubAugmentedTimes.addString("augmentedTimes");
            bSubAugmentedTimes.addString(augmented_time);

            //Ask for showing the current testing augmented + the best one if relevant
            bRpc.addList() = bSubRealtime;
            bRpc.addList() = bSubAugmentedTimes;

            ostringstream osResponse;
            osResponse.str("");
            osResponse << "The current best structure you have ever ranked is shown. The rank is " << value; //<< " for time = " << bestAugmentedTime ;
            iCub->say(osResponse.str().c_str(), false);
            yInfo() << "iCub says : " << osResponse.str();

            iCub->getABMClient()->rpcCommand(bRpc);
        }
        else {
            yError() << " [showBestOfAll] : could not find the instance from the best augmented : something is wrong!!!";
        }


    }
    else {
        yInfo() << "This is the first time " << agentName << " give feedback!";
    }

    //check that the streaming is finished before going next
    bResult.clear();
    Bottle bRpc;
    bRpc.addString("getStreamStatus");

    do{
        bResult = iCub->getABMClient()->rpcCommand(bRpc);
        yarp::os::Time::delay(1);
    } while (bResult.get(0).asString() != "none");

    return true;
}

/*
* create the vAugmentedTime for the current used Instance : sql to obtain the foreign key for the feedback table
*
*/
bool abmInteraction::insertFeedback(int feedback, int feedbackInstance)
{
    Bottle bResult;
    ostringstream osRequest;
    //take info from visualdata to put in feedback. In particular  the time of original memories is the smallest one from augmented
    osRequest << "SELECT DISTINCT instance, augmented_time, img_provider_port, min(\"time\") FROM visualdata WHERE instance = " << rememberedInstance << " AND augmented IS NOT NULL AND augmented_time = '" << *it_augmentedTime << "' AND img_provider_port = '" << img_provider_port << "' GROUP BY instance, augmented_time, img_provider_port;";
    bResult = iCub->getABMClient()->requestFromString(osRequest.str());
    yInfo() << bResult.toString();

    if (bResult.toString() != "NULL") {
        //we should only have one line, because instance,augmented_time,img_provider_port is primary key
        Bottle bResInsert;
        ostringstream osInsertFeedback;



        osInsertFeedback << "INSERT INTO feedback VALUES (" << feedbackInstance << ", '" << bResult.get(0).asList()->get(1).toString() << "', '" << bResult.get(0).asList()->get(2).toString() << "', '" << bResult.get(0).asList()->get(3).toString() << "', '" << agentName << "', " << feedback << ", 'none');";
        bResInsert = iCub->getABMClient()->requestFromString(osInsertFeedback.str().c_str());

        yInfo() << " Request sent : " << osInsertFeedback.str();
    }
    else {
        yError() << "Request to obtain augmented from instance " << rememberedInstance << " with augmented_time = " << *it_augmentedTime << " is NULL";
        return false;
    }

    return true;
}

#include "autobiographicalMemory.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

Bottle autobiographicalMemory::snapshot(const Bottle &bInput)
{
    /*
    format of input bottle :
    snapshot (action name_action type_action) (time t_time) (arguments (arg1) (arg2, role) ... argN) (begin 0/1) (mental)

    Arguments are from 2 types:
    - something from the OPC : just argN is enough
    - something external : argN + role is mandatory
    */

    // run only one snapshot at a time
    LockGuard lg(mutexSnapshot);

    while(!mutexStreamRecord.tryLock()) {
        yDebug() << "[mutexStreamRecord] tryLock";
        yarp::os::Time::delay(0.3);
    }
    yDebug() << "[mutexStreamRecord] unlock";
    mutexStreamRecord.unlock();

    //get Instance of the next opc
    string sRequest_instance;

    // get the OPC to check en bois
    bool bMental = bInput.check("mental");

    if (bMental)
    {
        // change the providers !!
    }

    if (isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("updateObjectLocation");
        b2reasoning.addString(s_real_OPC.c_str());
        abm2reasoning.write(b2reasoning);
    }

    sRequest_instance = "SELECT instance FROM main ORDER BY instance DESC LIMIT 1;";
    Bottle bRequest, bTemp;
    bRequest.addString("request");
    bRequest.addString(sRequest_instance.c_str());
    bRequest = request(bRequest);
    int instance;
    if (bRequest.toString() != "NULL") {
        instance = atoi(bRequest.get(0).asList()->get(0).toString().c_str()) + 1;
    }
    else {
        instance = 0;
    }

    OPCEARS.setInstance(instance);
    currentInstance = instance;

    // Filling table main:
    Bottle bMain;
    bMain.addString("request");
    ostringstream osMain;

    osMain << "INSERT INTO main(activityname, activitytype, time, instance, begin, opcname) VALUES ('";
    string sName;

    //for streaming image
    bool isStreamActivity = false;
    string fullSentence = "defaultLabel";

    string activityType;
    //Action
    bool done = false;
    for (int i = 1; i < bInput.size(); i++)
    {
        bTemp = *(bInput.get(i).asList());
        if (bTemp.get(0) == "action" && !done)
        {
            osMain << bTemp.get(1).asString() << "' , '"; // activityname
            sName = bTemp.get(1).asString();
            imgLabel = bTemp.get(1).asString();

            //used to name the single image
            ostringstream labelImg;
            labelImg << imgLabel << "_" << instance;
            fullSentence = labelImg.str();

            activityType = bTemp.get(2).asString();
            //if activity is an action -> stream
            if (activityType == "action") {
                isStreamActivity = true;
            }

            osMain << bTemp.get(2).asString() << "' , '"; // activitytype
            done = true;
        }
    }
    if (!done) {
        osMain << "unknown' , 'unknown', '";
    }
    // Time
    string sTime = getCurrentTime();
    osMain << sTime << "' , " << instance << " , "; // time + instance

    //Begin
    done = false;
    bool bBegin = false;
    //yDebug() << "bInput has a size of " << bInput.size() << " and is : " << bInput.toString().c_str();
    for (int i = 1; i < bInput.size(); i++)
    {
        bTemp = *(bInput.get(i).asList());
        if (bTemp.get(0) == "begin" && !done)
        {
            if (bTemp.get(1).asInt() == 1)
            {
                osMain << "TRUE  ";
                bBegin = true;
            }
            else
            {
                osMain << "FALSE  ";
                bBegin = false;
            }
            done = true;
        }
    }
    if (!done) {
        osMain << "FALSE";
    }

    if (isStreamActivity == true && !bBegin) { //just stop stream images stores when relevant activity
        yDebug() << "[mutexChangeover] try locking in snapshot end";
        mutexChangeover.lock();
        yDebug() << "[mutexChangeover] locked in snapshot end";

        streamStatus = "end"; //is done here (before the OPC snapshot), because the snapshot is slowing everything down
    }

    if(bMental) {
        osMain << " , '" << s_mental_OPC << "' ) ; ";
    } else {
        osMain << " , '" << s_real_OPC << "' ) ; ";
    }
    bMain.addString(string(osMain.str()).c_str());
    request(bMain);

    OPCClient *opcWorld;
    //Connection to the OPC
    (bMental) ? opcWorld = opcWorldMental : opcWorld = opcWorldReal;

    Bottle bOPCEearsResponse = OPCEARS.snapshot(bInput, opcWorld);
    if(bOPCEearsResponse.get(0).asString() == "nack") {
        return bOPCEearsResponse;
    }

    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();                         //I dont understand this Gregoire : you concatenate the name with nameInstance with itself, producing namenameinstance
    //yDebug() << "OPCEARS: " << sName;
    Bottle bSnapShot = OPCEARS.insertOPC(sName);
    if(bSnapShot.get(0).asString() == "nack") {
        return bSnapShot;
    }

    ostringstream osAllArg;
    Bottle bRecogSemantic;
    bool bShouldSend = true;
    // Filling contentArg
    if (opcWorld->isConnected()) {
        for (int i = 1; i < bInput.size(); i++)
        {
            bTemp = *(bInput.get(i).asList());
            if (bTemp.get(0) == "arguments" && bTemp.size() > 1)
            {
                for (int j = 1; j < bTemp.size(); j++)
                {
                    ostringstream osArg;
                    string cArgArgument, cArgType, cArgSubtype, cArgRole;

                    //check if the argument is an entity in OPC
                    Entity* currentEntity = opcWorld->getEntity(bTemp.get(j).asList()->get(0).toString().c_str());

                    if (currentEntity == NULL) {
                        cArgArgument = bTemp.get(j).asList()->get(0).asString();
                        cArgType = "external";
                        cArgSubtype = "default";
                    }
                    else {
                        cArgArgument = currentEntity->name();
                        cArgType = "entity";
                        cArgSubtype = currentEntity->entity_type();
                    }

                    if (bTemp.get(j).asList()->size() > 1) {
                        cArgRole = bTemp.get(j).asList()->get(1).asString();

                        //add sentence for single img label
                        if (cArgRole == "sentence"){
                            fullSentence = cArgArgument;
                        }
                        if (cArgRole == "semantic"){

                            yInfo() << " " << bTemp.get(j).asList()->get(0).toString();
                            bRecogSemantic.fromString(bTemp.get(j).asList()->get(0).toString());
                            yInfo() << " " << bRecogSemantic.toString();
                            yInfo() << "  bRecog size: " << bRecogSemantic.size();
                            bShouldSend = false;
                        }
                    }
                    else {
                        cArgRole = "unknown";
                    }

                    std::replace( cArgArgument.begin(), cArgArgument.end(), '\'', ' ');

                    if (bShouldSend){
                        osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '" << cArgArgument << "', " << "'" << cArgType << "', '" << cArgSubtype << "', '" << cArgRole << "') ; ";
                    }
                    else{
                        bShouldSend = true;
                    }
                    // one stringstream with all argments
                    osAllArg << osArg.str().c_str();
                }
            }
        }


        // add the snapshot of the OPC
        osAllArg << bSnapShot.get(0).asString();

        bRequest.clear();
        if(osAllArg.str() != "") {
            bRequest.addString("request");
            bRequest.addString(osAllArg.str().c_str());
            request(bRequest);
        }
    }


    if ((!bBegin) && isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("addLastActivity");
        b2reasoning.addString("action");

        abm2reasoning.write(b2reasoning);
    }

    if (isStreamActivity == true) { //just launch stream images stores when relevant activity
        if (bBegin) {
            yDebug() << "[mutexChangeover] try locking in snapshot begin";
            mutexChangeover.lock();
            yDebug() << "[mutexChangeover] locked in snapshot begin";

            streamStatus = "begin"; //streamStatus = "end" is done before the OPC snapshot, because the snapshot is slowing everything down
        }
    }
    else
    {   //just one image (sentence?)
        imgInstance = currentInstance;
        string synchroTime = getCurrentTime();
        frameNb = 0;

        storeImagesAndData(synchroTime, true, fullSentence);

        //if activity = say, we have to take one image/data + the sound that is coming from another port and catch in the update method of ABM (store in /tmp/sound/default.wav
        if (activityType == "recog") {
            string sndName;

            //take the full sentence, replace space by _ to have the sound name
            replace(fullSentence.begin(), fullSentence.end(), ' ', '_');
            sndName = fullSentence + ".wav";

            //read sound from file and put data in yarp::sig::Sound to store properly with semantic/instance that we are now aware of
            yarp::sig::Sound s;
            string defaultSoundFullPath = storingPath + "/" + storingTmpSuffix + "/sound/" + "default.wav";
            printf("opening file %s\n", defaultSoundFullPath.c_str());
            if (yarp::sig::file::read(s, defaultSoundFullPath.c_str()) == false) {
                yError() << "Cannot open the default sound file : check " << defaultSoundFullPath;
            }
            else {
                yInfo() << "Default sound file loaded from " << defaultSoundFullPath;
                stringstream sInstance;
                sInstance << currentInstance;

                //build the path and the name of the sound according to the instance and sentence said
                string relativePath = sInstance.str() + "/" + sndName;
                string fullPath = storingPath + "/" + relativePath;

                if (yarp::sig::file::write(s, fullPath.c_str()) == false) {
                    yError() << "Cannot save the default sound file to " << fullPath;
                }
                else {
                    yInfo() << "Default sound file renamed and moved to " << fullPath;

                    database_mutex.lock();
                    //add the sound into the  large_objects table of ABM
                    unsigned int snd_oid = ABMDataBase->lo_import(fullPath.c_str());
                    database_mutex.unlock();

                    Bottle bRequest;
                    ostringstream osArg;

                    //Populate the sounddata table with the infos
                    bRequest.addString("request");
                    osArg << "INSERT INTO sounddata(instance, relative_path, time, snd_provider_port, snd_oid) VALUES ('" << currentInstance << "', '" << relativePath << "', '" << synchroTime << "', '" << portSoundStreamInput.getName() << "', '" << snd_oid << "');";
                    bRequest.addString(osArg.str());
                    request(bRequest);
                }
            }

            osInsertTemp.str("");
            recogFromGrammarSemantic(bRecogSemantic, "", 1, currentInstance);
            requestFromString(osInsertTemp.str());
        }
    }

    bSnapShot.addInt(instance);

    return bSnapShot;
}


Bottle autobiographicalMemory::snapshotSP(const Bottle &bInput)
{
    /*
    format of input bottle :
    snapshot (action name_action type_action) (arg1 arg2 argn) (role1 role 2 rolen) (begin 0/1)

    Arguments are from 2 types:
    - something from the OPC : just argN is enough
    - something external : argN + role is mandatory
    */

    LockGuard lg(mutexSnapshot);

    Bottle bOutput;

    // check input
    if (bInput.size() != 5)
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong number of input (!= 5)";
        yError() << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong format of input";
        yError() << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | number of argument different of number of role";
        yError() << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    int iNbArg = bInput.get(2).asList()->size();

    //get Instance of the next opc
    Bottle bRequest, bTemp, bArg, bArguments, bRoles;
    bRequest = requestFromString("SELECT instance FROM main ORDER BY instance DESC LIMIT 1;");
    int instance;
    if (bRequest.toString() != "NULL") {
        instance = atoi(bRequest.get(0).asList()->get(0).toString().c_str()) + 1;
    }
    else {
        instance = 0;
    }
    OPCEARS.setInstance(instance);

    // Filling table main :
    Bottle bMain;
    ostringstream osMain;

    osMain << "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES('";
    string sName;

    //Action
    bTemp = *(bInput.get(1).asList());
    if (bTemp.get(0) == "action") {
        sName = bTemp.get(1).asString();
        osMain << sName << "' , '";
        osMain << bTemp.get(2).asString() << "' , '";
    }
    else {
        osMain << "unknown' , 'unknown', '";
    }

    // Time
    osMain << getCurrentTime() << "' , " << instance << " , ";

    //Begin
    bool inSharedPlan = false;
    bTemp = *(bInput.get(4).asList());
    if (bTemp.get(0) == "begin") {
        if (bTemp.get(1).asInt() == 1)
        {
            osMain << "TRUE ); ";
            inSharedPlan = true;
        }
        else
        {
            osMain << "FALSE ); ";
            inSharedPlan = false;
        }
    }
    else {
        osMain << "FALSE);";
    }

    // Fill contentArg
    vector<string> vAgent, vObject, vSpatial;

    string sManner = "none";

    // catch the arguments and the role associate
    bArguments = *bInput.get(2).asList();
    bRoles = *bInput.get(3).asList();

    for (int i = 0; i < iNbArg; i++)
    {
        if (bRoles.get(i).toString() == "agent")
            vAgent.push_back(bArguments.get(i).toString().c_str());
        if (bRoles.get(i).toString() == "object")
            vObject.push_back(bArguments.get(i).toString().c_str());
        if (bRoles.get(i).toString() == "spatial")
            vSpatial.push_back(bArguments.get(i).toString().c_str());
        if (bRoles.get(i).toString() == "manner")
        {
            sManner = bArguments.get(i).toString().c_str();
        }
    }

    if (sManner == "none")
    {
        yWarning() << "manner not found. Auto set to : none";
    }

    //Connection to the OPC and snapshot
    OPCEARS.snapshot(bInput, opcWorldReal);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    ostringstream osArg;
    osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << " , '" << sManner << "' , 'manner' , 'manner' , 'manner' ) ";

    if (opcWorldReal->isConnected()) {
        // Fill agents:
        for (unsigned int i = 0; i < vAgent.size(); i++)
        {
            Entity* currentEntity = opcWorldReal->getEntity(vAgent[i]);

            if (currentEntity == NULL)
                osArg << ", ( " << instance << ", '" << vAgent[i] << "', " << "'external', 'default', 'agent" << i + 1 << "') ";
            else
                osArg << ", ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'agent" << i + 1 << "') ";
        }

        // Fill objects:
        for (unsigned int i = 0; i < vObject.size(); i++)
        {
            Entity* currentEntity = opcWorldReal->getEntity(vObject[i]);

            if (currentEntity == NULL)
                osArg << ", ( " << instance << ", '" << vObject[i] << "', " << "'external', 'default', 'object" << i + 1 << "') ";
            else
                osArg << ", ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'object" << i + 1 << "') ";
        }

        // Fill spatials:
        for (unsigned int i = 0; i < vSpatial.size(); i++)
        {
            Entity* currentEntity = opcWorldReal->getEntity(vSpatial[i]);

            if (currentEntity == NULL)
                osArg << ", ( " << instance << ", '" << vSpatial[i] << "', " << "'external', 'default', 'spatial" << i + 1 << "') ";
            else
                osArg << ", ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'spatial" << i + 1 << "') ";
        }
    }

    // Insert the main request.
    bMain = requestFromString(osMain.str().c_str());

    // send filling contentarg
    bArg = requestFromString(osArg.str().c_str());

    for (int i = 0; i < bSnapShot.size(); i++)
    {
        bTemp = requestFromString(bSnapShot.get(i).toString().c_str());
    }

    if (!inSharedPlan && isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("addLastActivity");
        b2reasoning.addString("sharedplan");

        abm2reasoning.write(b2reasoning);
    }

    return bSnapShot;
}

Bottle autobiographicalMemory::snapshotBehavior(const Bottle &bInput)
{
    /*
    format of input bottle :
    snapshot (action name_action type_action) (arg1 arg2 argn) (role1 role 2 rolen) (begin 0/1)

    Arguments are from 2 types:
    - something from the OPC : just argN is enough
    - something external : argN + role is mandatory
    */

    LockGuard lg(mutexSnapshot);

    Bottle bOutput;

    // check input
    if (bInput.size() != 5)
    {
        string sError = "Error in autobiographicalMemory::snapshotBE | Wrong number of input (!= 5)";
        yError() << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError = "Error in autobiographicalMemory::snapshotBE | Wrong format of input";
        yError() << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError = "Error in autobiographicalMemory::snapshotBE | number of argument different of number of role";
        yError() << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    //get Instance of the next opc
    Bottle bRequest, bTemp, bArg, bArguments, bRoles;
    bRequest = requestFromString("SELECT instance FROM main ORDER BY instance DESC LIMIT 1;");
    int instance;
    if (bRequest.toString() != "NULL") {
        instance = atoi(bRequest.get(0).asList()->get(0).toString().c_str()) + 1;
    }
    else {
        instance = 0;
    }
    OPCEARS.setInstance(instance);

    // Filling table main :
    Bottle bMain;
    ostringstream osMain;

    bool bBegin = false;
    osMain << "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES('";
    string sName;

    //Action
    bTemp = *(bInput.get(1).asList());
    if (bTemp.get(0) == "action") {
        sName = bTemp.get(1).asString();
        osMain << sName << "' , '";
        osMain << bTemp.get(2).asString() << "' , '";
    }
    else {
        osMain << "unknown' , 'unknown', '";
    }

    // Time
    osMain << getCurrentTime() << "' , " << instance << " , ";

    //Begin
    bTemp = *(bInput.get(4).asList());
    if (bTemp.get(0) == "begin") {
        if (bTemp.get(1).asInt() == 1)
        {
            osMain << "TRUE ); ";
            bBegin = true;
        }
        else
        {
            osMain << "FALSE ); ";
            bBegin = false;
        }
    }
    else {
        osMain << "FALSE);";
    }

    // catch the arguments and the role associate
    string sArguments = (*bInput.get(2).asList()).get(0).toString();
    string sRole = (*bInput.get(3).asList()).get(0).toString(); // TODO: This is unused, can it be deleted?

    //Connection to the OPC and snapshot
    OPCEARS.snapshot(bInput, opcWorldReal);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    ostringstream osArg;
    osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << " , '" << sArguments << "' , 'none' , 'none' , 'argument' ) ";

    // Insert the main request.
    bMain = requestFromString(osMain.str().c_str());

    // send filling contentarg
    bArg = requestFromString(osArg.str().c_str());

    if (opcWorldReal->isConnected()) {
        for (int i = 0; i < bSnapShot.size(); i++)
        {
            bTemp = requestFromString(bSnapShot.get(i).toString().c_str());
        }
    }

    if (!bBegin && isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("addLastActivity");
        b2reasoning.addString("behavior");

        abm2reasoning.write(b2reasoning);
    }

    return bSnapShot;
}


/**
* Recursive method to extract the semantic cues of each word recognized
* the bRecogBottle is the sentence bottle sent by speechRecog (so without the sentence at first)
* e.g.  from Recgo : sentence ((temporal "before you") (actionX (action1 ((verb1 point) (object "the circle")))) (actionX (action2 ((verb2 push) (object "the ball")))))
* bRecogBottle     : (temporal "before you") (actionX (action1 ((verb1 point) (object "the circle")))) (actionX (action2 ((verb2 push) (object "the ball"))))
* Modify the ostringstream osInsertTemp
*/
void autobiographicalMemory::recogFromGrammarSemantic(const Bottle &bRecogBottle, string s_deep, int i_deep, const int iInstance)
{
    //TODO: list of string for the deepness, no need for the int in that case
    //TODO: careful, may have to copy each time because of recursive

    string currentWord = "";
    string currentRole = "";

    yInfo() << "bRecogBottle = " << bRecogBottle.toString();

    //case 1 : string string -> end of the recursive
    if (bRecogBottle.get(0).isString() && bRecogBottle.get(1).isString()){

        //yInfo() << "===== case 1 : string/string =====" ;

        currentRole = bRecogBottle.get(0).asString();
        currentWord = bRecogBottle.get(1).asString();

        //SQL insert
        //yInfo() << " === s_deep = " << s_deep << " and i_deep = " << i_deep << "===" ;
        //yInfo() << " C1 : -------> role = " << currentRole << " and word = " << currentWord << " and level " << i_deep ;
        osInsertTemp << "INSERT INTO sentencedata(instance, word, role, \"level\") VALUES (" << iInstance << ", '" << currentWord << "' , '" << currentRole << "', " << i_deep << " ) ; ";

    }

    //case 2 : string list -> sub-sentence, sub-part
    else if (bRecogBottle.get(0).isString() && bRecogBottle.get(1).isList()){

        //  yInfo() << "===== case 2 : string/List =====" ;

        s_deep = bRecogBottle.get(0).asString(); //TODO : increase the list
        //yInfo() << " C2 : -------> role = " << "semantic" << " and word = " << s_deep << " and level = " << i_deep ;
        osInsertTemp << "INSERT INTO sentencedata(instance, word, role, \"level\") VALUES (" << iInstance << ", '" << s_deep << "' , '" << "semantic" << "', " << i_deep << " ) ; ";


        i_deep = i_deep * 10 + 1;
        int i_deep_cp = i_deep;
        recogFromGrammarSemantic(*bRecogBottle.get(1).asList(), s_deep, i_deep_cp, iInstance);
    }

    //case 3 : it is not case 1 or 2, so we should have reach the "end" of a semantic, and having group of pairs (role1 arg1) (role2 arg2) (role3 arg3) 
    //list -> list of word
    else if (bRecogBottle.size() > 1){
        //       yInfo() << "===== case 3 : List =====" ;

        for (int i = 0; i < bRecogBottle.size(); i++) {

            //   yInfo() << " --> i = " << i ;

            int i_deep_cp = i_deep;
            if (i != 0)
            {
                i_deep += 1;
                i_deep_cp = i_deep;
                //yInfo() << " C3 : -------> role = " << "semantic" << " and word = " << bRecogBottle.get(i).toString() << " and level = " << i_deep << std::endl;
            }
            recogFromGrammarSemantic(*bRecogBottle.get(i).asList(), s_deep, i_deep_cp, iInstance);
        }
    }
    else {
        yError() << "None possible case in recogFronGrammarSemantic : something is wrong (Bottle from SpeechRecog?) : " << bRecogBottle.toString();
    }

}

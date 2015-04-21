#include "autobiographicalMemory.h"

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

Bottle autobiographicalMemory::snapshot(const Bottle &bInput)
{
    /*
    format of input bottle :
    snapshot (action name_action type_action) (time t_time) (arguments (arg1) (arg2, role) ... argN) (begin 0/1)

    Arguments are from 2 types:
    - something from the OPC : just argN is enough
    - something external : argN + role is mandatory
    */

    //get Instance of the next opc
    string sRequest_instance;
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
    if(bRequest.toString()!="NULL") {
        instance = atoi(bRequest.get(0).asList()->get(0).toString().c_str()) + 1;
    } else {
        instance = 0;
    }
    OPCEARS.setInstance(instance);
    currentInstance = instance;

    // Filling table main:
    Bottle bMain;
    bMain.addString("request");
    ostringstream osMain;

    osMain << "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES ('";
    string sName;

    //for streaming image
    string activityName;
    bool isStreamActivity = false;
    string fullSentence;

    //Action
    bool done = false;
    for (int i = 1; i < bInput.size(); i++)
    {
        bTemp = *(bInput.get(i).asList());
        if (bTemp.get(0) == "action" && !done)
        {
            osMain << bTemp.get(1).asString() << "' , '";
            sName = bTemp.get(1).asString();
            activityName = bTemp.get(1).asString(); //sName is concatenated after...need to save label
            imgLabel = activityName;

            //if it is not a sentence -> stream
            if (activityName != "sentence") {
                isStreamActivity = true;
            }

            osMain << bTemp.get(2).asString() << "' , '";
            done = true;
        }
    }
    if (!done) {
        osMain << "unknown' , '";
    }
    // Time
    string sTime = getCurrentTime();
    osMain << sTime << "' , " << instance << " , ";

    //Begin
    done = false;
    bool bBegin = false;
    //cout << "bInput has a size of " << bInput.size() << " and is : " << bInput.toString().c_str() << endl;
    for (int i = 1; i < bInput.size(); i++)
    {
        bTemp = *(bInput.get(i).asList());
        if (bTemp.get(0) == "begin" && !done)
        {
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
            done = true;
        }
    }
    if (!done) {
        osMain << "FALSE);";
    }

    if (isStreamActivity == true && !bBegin) { //just stop stream images stores when relevant activity
        streamStatus = "end"; //is done here (before the OPC snapshot), because the snapshot is slowing everything down
    }

    bMain.addString(string(osMain.str()).c_str());
    bMain = request(bMain);

    //Connection to the OPC
    OPCEARS.snapshot(bInput, opcWorld);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();                         //I dont understand this Gregoire : you concatenate the name with nameInstance with itself, producing namenameinstance
    //cout << "OPCEARS: " << sName << endl;
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    ostringstream osAllArg;

    // Filling contentArg
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
                }
                else {
                    cArgRole = "unknown";
                }

                osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '" << cArgArgument << "', " << "'" << cArgType << "', '" << cArgSubtype << "', '" << cArgRole << "') ; ";

                // one stringstream with all argments
                osAllArg << osArg.str().c_str() ;
            }
        }
    }


    // add the snapshot of the OPC
    osAllArg << bSnapShot.get(0).asString() ;


    bRequest.clear();
    bRequest.addString("request");
    bRequest.addString(osAllArg.str().c_str());
    request(bRequest);


    if ((!bBegin) && isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("addLastActivity");
        b2reasoning.addString("action");

        abm2reasoning.write(b2reasoning);
    }

    string isConnectedToImgProviders = connectToImgStreamProviders().toString().c_str();
    string isConnectedToContDataProviders = connectDataStreamProviders().toString().c_str();

    if (isConnectedToImgProviders != "ack" && isConnectedToContDataProviders != "ack"){
        cout << "ABM failed to connect to imgProviders / contDataProviders" << endl;
        cout << "Reason image providers: " << isConnectedToImgProviders << endl;
        cout << "Reason cont data providers: " << isConnectedToContDataProviders << endl;
    }
    if (isStreamActivity == true) { //just launch stream images stores when relevant activity
        if(bBegin) {
            streamStatus = "begin"; //streamStatus = "end" is done before the OPC snapshot, because the snapshot is slowing everything down
        }
    }
    else
    {   //just one image (sentence?)
        imgInstance = currentInstance;
        string synchroTime = getCurrentTime();
        storeImagesAndData(synchroTime, true, fullSentence);

        //Network::disconnect(imgProviderPort, imagePortIn.getName().c_str()) ;
        string reply = disconnectFromImgStreamProviders().toString().c_str();
        if (reply != "ack"){
            cout << "ABM failed to disconnect to one imgProvider" << endl;
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
    Bottle bOutput;

    // check input
    if (bInput.size() != 5)
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong number of input (!= 5)";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong format of input";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | number of argument different of number of role";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    int iNbArg = bInput.get(2).asList()->size();

    //get Instance of the next opc
    Bottle bRequest, bTemp, bArg, bArguments, bRoles;
    bRequest = requestFromString("SELECT instance FROM main ORDER BY instance DESC LIMIT 1;");
    int instance;
    if(bRequest.toString()!="NULL") {
        instance = atoi(bRequest.get(0).asList()->get(0).toString().c_str()) + 1;
    } else {
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
    } else {
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
    } else {
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
        cout << "manner not found. Auto set to : none" << endl;
    }

    //Connection to the OPC and snapshot
    OPCEARS.snapshot(bInput, opcWorld);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    ostringstream osArg;
    osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << " , '" << sManner << "' , 'manner' , 'manner' , 'manner' ) ";

    // Fill agents:
    for (unsigned int i = 0; i < vAgent.size(); i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vAgent[i]);

        if (currentEntity == NULL)
            osArg << ", ( " << instance << ", '" << vAgent[i] << "', " << "'external', 'default', 'agent" << i + 1 << "') ";
        else
            osArg << ", ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'agent" << i + 1 << "') ";
    }

    // Fill objects:
    for (unsigned int i = 0; i < vObject.size(); i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vObject[i]);

        if (currentEntity == NULL)
            osArg << ", ( " << instance << ", '" << vObject[i] << "', " << "'external', 'default', 'object" << i + 1 << "') ";
        else
            osArg << ", ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'object" << i + 1 << "') ";
    }

    // Fill spatials:
    for (unsigned int i = 0; i < vSpatial.size(); i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vSpatial[i]);

        if (currentEntity == NULL)
            osArg << ", ( " << instance << ", '" << vSpatial[i] << "', " << "'external', 'default', 'spatial" << i + 1 << "') ";
        else
            osArg << ", ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'spatial" << i + 1 << "') ";
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

    Bottle bOutput;

    // check input
    if (bInput.size() != 5)
    {
        string sError = "Error in autobiographicalMemory::snapshotBE | Wrong number of input (!= 5)";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError = "Error in autobiographicalMemory::snapshotBE | Wrong format of input";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError = "Error in autobiographicalMemory::snapshotBE | number of argument different of number of role";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    //get Instance of the next opc
    Bottle bRequest, bTemp, bArg, bArguments, bRoles;
    bRequest = requestFromString("SELECT instance FROM main ORDER BY instance DESC LIMIT 1;");
    int instance;
    if(bRequest.toString()!="NULL") {
        instance = atoi(bRequest.get(0).asList()->get(0).toString().c_str()) + 1;
    } else {
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
    } else {
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
    OPCEARS.snapshot(bInput, opcWorld);
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

    for (int i = 0; i < bSnapShot.size(); i++)
    {
        bTemp = requestFromString(bSnapShot.get(i).toString().c_str());
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

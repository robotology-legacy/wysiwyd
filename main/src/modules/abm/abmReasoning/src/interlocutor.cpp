#include <interlocutor.h>


using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


void interlocutor::initialize()
{
    senderPort.open("/abmReasoning/interlocutor/toAbm/request:o");
    port_to_OPCManager.open("/abmReasoning/interlocutor/toOPCManager");

    Network::connect(senderPort.getName(), "/autobiographicalMemory/rpc");
    Network::connect(port_to_OPCManager.getName(), "/opcManager/rpc");
    //  iCub  = new ICubClient("interlocutor", false);
    //  iCub->opc->isVerbose = false;

    //  iCub->connect();
    connectOPC();
}


void interlocutor::close()
{
    senderPort.close();
    port_to_OPCManager.close();

    mentalOPC->interrupt();
    mentalOPC->close();
    realOPC->interrupt();
    realOPC->close();

    delete realOPC;
    delete mentalOPC;

}

Bottle interlocutor::connectOPC()
{
    Bottle bOutput;

    realOPC = new OPCClient("abmReasoning/interlocutor/torealOPC");
    int iTry = 0;
    while (!realOPC->isConnected())
    {
        yInfo() << "\t" << "interlocutor Connecting to " << abmReasoningFunction::s_realOPC << "..." << realOPC->connect(abmReasoningFunction::s_realOPC)  ;
        if (!realOPC->isConnected())
            Time::delay(0.5);
        iTry++;
        if (iTry > 1)
        {
            yInfo() << "\t" << "Interlocutor failed to connect to " << abmReasoningFunction::s_realOPC  ;
            bOutput.addString("Connection to Real OPC failed, please check your port");
            break;
        }
    }

    if (realOPC->isConnected())
    {
        realOPC->checkout();
        realOPC->update();
    }

    mentalOPC = new OPCClient("abmReasoning/interlocutor/toMentalOPC");
    iTry = 0;
    while (!mentalOPC->isConnected())
    {
        yInfo() << "\t" << "interlocutor Connecting to " << abmReasoningFunction::s_mentalOPC << "..." << mentalOPC->connect(abmReasoningFunction::s_mentalOPC)  ;
        if (!mentalOPC->isConnected())
            Time::delay(0.5);
        iTry++;
        if (iTry > 1)
        {
            yInfo() << "\t" << "Interlocutor failed to connect to " << abmReasoningFunction::s_mentalOPC  ;
            bOutput.addString("Connection failed, please check your port");
            return bOutput;
        }
    }

    mentalOPC->checkout();
    mentalOPC->update();

    bOutput.addString("Connection done");
    return bOutput;
}



/*   -------------------------   METHODS CALLED VIA HANDLERPORT  ------------------------------   */



// Send a SQL query (within a bottle) to AutobiographicalMemory. bRequest must be the complete request
Bottle interlocutor::request(Bottle bRequest)
{
    Bottle bReplyRequest;
    //send the SQL query within a bottle to autobiographicalMemory
    senderPort.write(bRequest, bReplyRequest);
    return bReplyRequest;
}


Bottle interlocutor::requestFromStream(string sInput)
{
    Bottle bReplyRequest;
    //send the SQL query within a bottle to autobiographicalMemory
    Bottle bQuery;
    bQuery.addString("request");
    bQuery.addString(sInput.c_str());
    senderPort.write(bQuery, bReplyRequest);
    return bReplyRequest;
}

/*   -------------------------   ASKING FUNCTIONS  ------------------------------   */


/**
* Return the last action stored in the ABM
* @b output format : ("action" action_name) ("argument" arg1 arg2 ... argn) ("object1" XtYtn Xt+1Yt+1) facultative : ("object2" XtYtn Xt+1Yt+1)
*/
Bottle interlocutor::askLastAction()
{
    //extract the instances of the OPC
    Bottle  bOpcIdBegin = requestFromStream("SELECT instance FROM main WHERE activitytype = 'action' AND begin = TRUE ORDER BY instance DESC LIMIT 1");

    int opcIdBegin = atoi(bOpcIdBegin.get(0).asList()->get(0).toString().c_str());

    return askActionFromId(opcIdBegin);
}

/*
* Return the consequence of an action according to its ID
*
* bOutput :
*   if only one spatial argument :  (action) (arg) "{x,y,z, before arg1}" "{x,y,z, after arg1}" presence_before_arg1 presence_after_arg1
*   if 2 spatial arguments : (action) (arg) "{x,y,z, before arg1}" "{x,y,z, after arg1}" presence_before_arg1 presence_after_arg1 "{x,y,z, before arg2}" "{x,y,z, after arg2}"
*/
Bottle interlocutor::askActionFromId(int Id)
{
    Bottle  bOutput,    // main output
        bQuery,
        bAction,
        bArguments,
        bOject1,
        bIdArgBegin,
        bSubTypeArgBegin,
        bPosArgBegin,
        bPosArgEnd,
        bContent,
        bTime,
        bName;

    string sTimeBegin, sTimeEnd;

    ostringstream osName;
    osName << "SELECT main.activityname, contentarg.argument FROM main, contentarg WHERE main.instance = contentarg.instance AND contentarg.role = 'spatial1' AND main.instance = " << Id;
    bContent = requestFromStream(osName.str());
    bName.addString((*bContent.get(0).asList()).get(0).toString().c_str());

    bOutput.addList() = bName;
    //yInfo() << "\t" << "bArguments : " << bArguments.toString()  ;
    for (int i = 0; i < bContent.size(); i++)
    {
        bArguments.addString((*bContent.get(i).asList()).get(1).toString().c_str());
    }

    bOutput.addList() = bArguments;

    //-- 1. extract the id of the argument, assuming it is an entity
    ostringstream osEntity;
    osEntity << "SELECT entity.opcid FROM entity WHERE entity.instance = " << Id << " AND entity.name IN (SELECT DISTINCT contentarg.argument FROM entity, contentarg WHERE contentarg.instance = " << Id << " AND contentarg.role = 'object')";
    bIdArgBegin = requestFromStream(osEntity.str().c_str());

    //clear things
    osEntity.str("");

    int idArg = atoi(bIdArgBegin.get(0).asList()->get(0).toString().c_str());
    //yInfo() << "\t" << "Argument Id Begin id : " << idArg  ;


    //-- 2. select the subtype of the argument in order to extract it accordingly
    osEntity << "SELECT contentopc.subtype from contentopc WHERE contentopc.instance = " << Id << " AND contentopc.opcid = " << idArg;
    bSubTypeArgBegin = requestFromStream(osEntity.str().c_str());
    string subtypeArg = bSubTypeArgBegin.get(0).asList()->get(0).toString().c_str();



    //  yInfo() << "\t" << "Subtype Argument Begin : " << subtypeArg  ;
    int ObjectPresentBefore, ObjectPresentAfter;
    string test = "t";


    //-- 3. extract the x, y of the object at the beginning of the activity and the presence and absence
    osEntity.str("");
    osEntity << "SELECT " << subtypeArg << ".position, " << subtypeArg << ".presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << Id << " AND " << subtypeArg << ".opcid = " << idArg;
    bPosArgBegin = requestFromStream(osEntity.str().c_str());
    string posArgBegin = bPosArgBegin.get(0).asList()->get(0).toString().c_str();
    if (bPosArgBegin.get(0).asList()->get(1).toString().c_str() == test)
        ObjectPresentBefore = 1;
    else
        ObjectPresentBefore = 0;



    //clear things
    osEntity.str("");


    //-- 4. extract the x, y of the object at the end of the activity and the presence and absence
    osEntity << "SELECT " << subtypeArg << ".position, " << subtypeArg << ".presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << Id + 1 << " AND " << subtypeArg << ".opcid = " << idArg;
    bPosArgEnd = requestFromStream(osEntity.str().c_str());
    string posArgEnd = bPosArgEnd.get(0).asList()->get(0).toString().c_str();
    if (bPosArgEnd.get(0).asList()->get(1).toString().c_str() == test)
        ObjectPresentAfter = 1;
    else
        ObjectPresentAfter = 0;
    //clear things
    osEntity.str("");

    bOutput.addString(posArgBegin.c_str());
    bOutput.addString(posArgEnd.c_str());
    bOutput.addInt(ObjectPresentBefore);
    bOutput.addInt(ObjectPresentAfter);


    ostringstream osTime;
    osTime << "SELECT time FROM main WHERE instance = " << Id;
    bContent = requestFromStream(osTime.str());
    sTimeBegin = (*bContent.get(0).asList()).get(0).toString();

    osTime << "SELECT time FROM main WHERE instance = " << Id + 1;
    bContent = requestFromStream(osTime.str());
    sTimeEnd = (*bContent.get(0).asList()).get(0).toString();

    double dTiming = abmReasoningFunction::timeDiffSecondFromString(sTimeBegin, sTimeEnd);

    bOutput.addDouble(dTiming);

    ostringstream osSubject;
    osSubject << "SELECT argument FROM contentarg WHERE role = 'agent' AND instance = " << Id;
    bContent = requestFromStream(osSubject.str());
    bOutput.addString(bContent.get(0).asList()->get(0).toString());

    ostringstream osSpatial2;
    osSpatial2 << "SELECT argument FROM contentarg WHERE (role = 'adv1' OR role = 'adv2') AND instance = " << Id;
    bContent = requestFromStream(osSpatial2.str());

    string sNull = "NULL";

    if (bName.get(0).toString() == "hanoi")
    {
        bOutput.clear();
        bName.clear();
        bArguments.clear();

        ostringstream osObject1;
        osObject1 << "SELECT main.activityname, contentarg.argument FROM main, contentarg WHERE main.instance = contentarg.instance AND contentarg.role = 'object1' AND main.instance = " << Id;
        Bottle bObject1 = requestFromStream(osObject1.str());

        bName.addString((*bObject1.get(0).asList()).get(0).toString().c_str());

        bOutput.addList() = bName;
        //yInfo() << "\t" << "bArguments : " << bArguments.toString()  ;
        for (int i = 0; i < bObject1.size(); i++)
        {
            bArguments.addString((*bObject1.get(i).asList()).get(1).toString().c_str());
        }

        bOutput.addList() = bArguments;

        bOutput.addString(posArgBegin.c_str());
        bOutput.addString(posArgEnd.c_str());
        bOutput.addInt(ObjectPresentBefore);
        bOutput.addInt(ObjectPresentAfter);

        ostringstream osRTOpresent;
        osRTOpresent << "SELECT position, name FROM rtobject WHERE instance = " << Id << " AND presence = true";
        Bottle bRTOpresent = requestFromStream(osRTOpresent.str());


        if (bRTOpresent.toString().c_str() != sNull)
        {
            bOutput.addList() = bRTOpresent;
        }
        return bOutput;
    }

    //--5. get the other rtobject present


    if (bContent.toString().c_str() == sNull)
    {
        return bOutput;
    }

    if (!realOPC->isConnected())
    {
        bOutput.clear();
        bOutput.addString("opc not connected");
        return bOutput;
    }

    string sArgSpatial2 = bContent.get(0).toString();

    if (sArgSpatial2 == sNull){

        return bOutput;
    }

    //-- 1. extract the id of the argument, assuming it is an entity
    osEntity << "SELECT entity.opcid FROM entity WHERE entity.instance = " << Id << " AND entity.name = '" << sArgSpatial2 << "'";
    bIdArgBegin = requestFromStream(osEntity.str().c_str());

    //clear things
    osEntity.str("");

    idArg = atoi(bIdArgBegin.get(0).asList()->get(0).toString().c_str());
    //yInfo() << "\t" << "Argument Id Begin id : " << idArg  ;


    //-- 2. select the subtype of the argument in order to extract it accordingly
    osEntity << "SELECT contentopc.subtype from contentopc WHERE contentopc.instance = " << Id << " AND contentopc.opcid = " << idArg;
    bSubTypeArgBegin = requestFromStream(osEntity.str().c_str());
    subtypeArg = bSubTypeArgBegin.get(0).asList()->get(0).toString().c_str();


    //-- 3. extract the x, y of the object at begin and end of the activity and the presence and absence
    osEntity.str("");
    osEntity << "SELECT " << subtypeArg << ".position, " << subtypeArg << ".presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << Id << " AND " << subtypeArg << ".opcid = " << idArg;
    bPosArgBegin = requestFromStream(osEntity.str().c_str());
    posArgBegin = bPosArgBegin.get(0).asList()->get(0).toString().c_str();

    //clear things
    osEntity.str("");


    osEntity << "SELECT " << subtypeArg << ".position, " << subtypeArg << ".presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << Id + 1 << " AND " << subtypeArg << ".opcid = " << idArg;
    bPosArgEnd = requestFromStream(osEntity.str().c_str());
    posArgEnd = bPosArgEnd.get(0).asList()->get(0).toString().c_str();

    bOutput.addString(posArgBegin.c_str());
    bOutput.addString(posArgEnd.c_str());

    bOutput.addString(subtypeArg.c_str());

    return bOutput;
}

/*
* Return the consequence of an action according to its ID
*
* bOutput :
*   action_name ( (role1 arg1)  (role2 arg2) ... (rolen argn) )  "x, y, z (t)"  "x, y, z (t+1)" "presence(t)" "presence(t+1)"  timing
*/
Bottle interlocutor::askActionFromIdV2(int Id)
{
    Bottle  bOutput,    // main output
        bQuery,
        bAction,
        bArguments,
        bIdArgBegin,
        bSubTypeArgBegin,
        bPosArgBegin,
        bPosArgEnd,
        bContent,
        bError,
        bTime;

    string sTimeBegin,	//timing beginning of action
        sTimeEnd,		//timing end of action
        sObject,		// name of the object of focus
        sName,			// name of the action (verb)
        sAgent;			// agent of the action

    list<string>	lAdjectives;

    ostringstream osError;
    osError << "Error in instance: " << Id << ". wrong data in ELM.";
    bError.addString("error");
    bError.addString(osError.str().c_str());


    ostringstream osTime;
    osTime << "SELECT time FROM main WHERE instance = " << Id;
    bContent = requestFromStream(osTime.str());
    if (bContent.toString() == "NULL")	return bError;
    sTimeBegin = (*bContent.get(0).asList()).get(0).toString();

    osTime.str("");
    osTime << "SELECT time FROM main WHERE instance = " << Id + 1;
    bContent = requestFromStream(osTime.str());
    if (bContent.toString() == "NULL")	return bError;
    sTimeEnd = (*bContent.get(0).asList()).get(0).toString();

    double dTiming = abmReasoningFunction::timeDiffSecondFromString(sTimeBegin, sTimeEnd);

    ostringstream osName;
    osName << "SELECT role, argument FROM contentarg WHERE instance = " << Id;
    bArguments = requestFromStream(osName.str());

    sAgent = bArguments.check("agent", Value("none")).asString();
    if (sAgent == "none")
        sAgent = bArguments.check("agent1", Value("none")).asString();

    sObject = bArguments.check("object", Value("none")).asString();
    if (sObject == "none")
        sObject = bArguments.check("object1", Value("none")).asString();

    lAdjectives.push_back(bArguments.check("adv1", Value("none")).asString());
    lAdjectives.push_back(bArguments.check("adv2", Value("none")).asString());
    lAdjectives.push_back(bArguments.check("spatial1", Value("none")).asString());

    sName = bArguments.check("action", Value("none")).asString();
    if (sName == "none")
        sName = bArguments.check("action1", Value("none")).asString();
    if (sName == "none")
    {
        ostringstream osGetNameAction;
        osGetNameAction << "SELECT (activityname) FROM main WHERE instance = " << Id;
        Bottle bGetNameAction = requestFromStream(osGetNameAction.str().c_str());
        if (bGetNameAction.toString() == "NULL")	return bError;
        sName = (*bGetNameAction.get(0).asList()).get(0).toString();
    }

    Bottle bNewArgument;
    Bottle bAgent,
        bObject,
        bName;

    bAgent.addString("agent");
    bAgent.addString(sAgent);
    bNewArgument.addList() = bAgent;

    bObject.addString("object");
    bObject.addString(sObject);
    bNewArgument.addList() = bObject;

    bName.addString("action");
    bName.addString(sName);
    bNewArgument.addList() = bName;


    int ii = 1;
    for (list<string>::iterator itSt = lAdjectives.begin(); itSt != lAdjectives.end(); itSt++)
    {
        if (*itSt != "none")
        {
            Bottle bArgTemp;
            ostringstream sArgTem;
            sArgTem << "adv" << ii;
            bArgTemp.addString(sArgTem.str().c_str());
            bArgTemp.addString(*itSt);
            bNewArgument.addList() = bArgTemp;
            ii++;
        }
    }


    //-- 1. extract the id of the argument, assuming it is an entity
    ostringstream osEntity;
    osEntity << "SELECT opcid FROM entity WHERE instance = " << Id << " AND name = '" << sObject << "'";
    bIdArgBegin = requestFromStream(osEntity.str().c_str());
    if (bIdArgBegin.toString() == "NULL")	return bError;
    int idArg = atoi(bIdArgBegin.get(0).asList()->get(0).toString().c_str());
    //yInfo() << "\t" << "Argument Id Begin id : " << idArg  ;


    //clear things
    osEntity.str("");


    //-- 2. select the subtype of the argument in order to extract it accordingly
    osEntity << "SELECT subtype FROM contentopc WHERE instance = " << Id << " AND opcid = " << idArg;
    bSubTypeArgBegin = requestFromStream(osEntity.str().c_str());
    if (bSubTypeArgBegin.toString() == "NULL")	return bError;
    string subtypeArg = bSubTypeArgBegin.get(0).asList()->get(0).toString().c_str();


    //  yInfo() << "\t" << "Subtype Argument Begin : " << subtypeArg  ;
    int ObjectPresentBefore, ObjectPresentAfter;
    string test = "t";


    //-- 3. extract the x, y of the object at the beginning of the activity and the presence and absence
    osEntity.str("");
    osEntity << "SELECT " << subtypeArg << ".position, " << subtypeArg << ".presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << Id << " AND " << subtypeArg << ".opcid = " << idArg;
    bPosArgBegin = requestFromStream(osEntity.str().c_str());
    if (bPosArgBegin.toString() == "NULL")	return bError;
    string posArgBegin = bPosArgBegin.get(0).asList()->get(0).toString().c_str();

    (bPosArgBegin.get(0).asList()->get(1).toString().c_str() == test) ? ObjectPresentBefore = 1 : ObjectPresentBefore = 0;

    //clear things
    osEntity.str("");


    //-- 4. extract the x, y of the object at the end of the activity and the presence and absence
    osEntity << "SELECT " << subtypeArg << ".position, " << subtypeArg << ".presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << Id + 1 << " AND " << subtypeArg << ".opcid = " << idArg;
    bPosArgEnd = requestFromStream(osEntity.str().c_str());
    if (bPosArgEnd.toString() == "NULL")	return bError;
    string posArgEnd = bPosArgEnd.get(0).asList()->get(0).toString().c_str();

    (bPosArgEnd.get(0).asList()->get(1).toString().c_str() == test) ? ObjectPresentAfter = 1 : ObjectPresentAfter = 0;


    bOutput.addString(sName);
    bOutput.addList() = bNewArgument;

    bOutput.addString(posArgBegin.c_str());
    bOutput.addString(posArgEnd.c_str());
    bOutput.addInt(ObjectPresentBefore);
    bOutput.addInt(ObjectPresentAfter);

    bOutput.addDouble(dTiming);


    return bOutput;
}


/*
*   Return the (Speaker, Addressee, Subject (pronom) and Agent) of a sentence and of the action related.
*
*   1- Get the information about the sentence
*   2- Get previous action + time
*   3- Get next action + time
*   4- Keep the closest action
*   5- Return Speaker - Addressee - Agent - Subject (pronom)
*/
Bottle interlocutor::askSentenceFromId(int Id)
{
    Bottle  bOutput,    // main output
        bContent,       // content of the sentence
        bPrevious,      // previous action
        bNext,          // next action
        bTemp;


    yInfo() << "\t"   << "Treatment of sentence instance : " << Id  ;

    // 1-
    ostringstream osName;
    osName << "SELECT main.time, contentarg.argument, contentarg.role FROM main, contentarg WHERE main.instance = contentarg.instance AND main.instance =" << Id;
    bContent = requestFromStream(osName.str());

    yInfo() << "\t" << "bContent : "   << bContent.toString()    ;

    string sSpeaker,
        sAddressee,
        sSubject,
        sTimeSentence;

    bool fSpeaker = false,
        fAddressee = false,
        fSubject = false;

    for (int iElement = 0; iElement < bContent.size(); iElement++)
    {
        Bottle bDecomposed = *bContent.get(iElement).asList();
        sTimeSentence = bDecomposed.get(0).toString().c_str();

        if (bDecomposed.get(2).toString().c_str() == abmReasoningFunction::TAG_SPEAKER && !fSpeaker)
        {
            fSpeaker = true;
            sSpeaker = bDecomposed.get(1).toString().c_str();
        }

        if (bDecomposed.get(2).toString().c_str() == abmReasoningFunction::TAG_ADRESSEE && !fAddressee)
        {
            fAddressee = true;
            sAddressee = bDecomposed.get(1).toString().c_str();
        }

        if (bDecomposed.get(2).toString().c_str() == abmReasoningFunction::TAG_SUBJECT && !fSubject)
        {
            fSubject = true;
            sSubject = bDecomposed.get(1).toString().c_str();
        }

    }

    if (!fAddressee || !fSpeaker || !fSubject)
    {
        yInfo() << "\t" << "Error in abmReasoning::FindAllSentence::FindSentenceFromId -  Id = " << Id << ". Lack of information in the sentence."  ;
        return bOutput;
    }

    // 2-
    int iInstancePrevious, iInstanceNext;
    string sTimePrevious, sTimeNext,
        sAgentPrevious, sAgentNext;

    osName.str("");
    osName << "select main.time,main.instance, contentarg.argument  from main, contentarg where main.instance = contentarg.instance AND contentarg.role = 'agent1' AND activitytype = 'action' and main.instance < " << Id << " and begin = 'FALSE' order by instance DESC limit 1";
    bTemp = *(requestFromStream(osName.str()).get(0).asList());

    sTimePrevious = bTemp.get(0).toString().c_str();
    iInstancePrevious = atoi(bTemp.get(1).toString().c_str());
    sAgentPrevious = bTemp.get(2).toString().c_str();


    // 3-
    osName.str("");
    osName << "select main.time,main.instance, contentarg.argument  from main, contentarg where main.instance = contentarg.instance AND contentarg.role = 'agent1' AND activitytype = 'action' and main.instance > " << Id << " and begin = 'TRUE' order by instance limit 1";
    Bottle bRequest = requestFromStream(osName.str());
    yInfo() << "\t" << "brequest : " << bRequest.toString()  ;
    int iDiffTimefromNext = 10000;

    if (!(bRequest.toString() == "NULL"))
    {
        bTemp = *(bRequest.get(0).asList());
        yInfo() << "\t" << "bTemp : " << bTemp.toString();

        sTimeNext = bTemp.get(0).toString().c_str();
        iInstanceNext = atoi(bTemp.get(1).toString().c_str());
        sAgentNext = bTemp.get(2).toString().c_str();
        iDiffTimefromNext = abs(abmReasoningFunction::timeDiffSecondFromString(sTimeSentence, sTimeNext));
    }

    // 4-
    int iDiffTimefromPrev = abs(abmReasoningFunction::timeDiffSecondFromString(sTimeSentence, sTimePrevious));

    bOutput.addString(sSpeaker.c_str());
    bOutput.addString(sAddressee.c_str());
    bOutput.addString(sSubject.c_str());

    // if next is closer in time
    if (iDiffTimefromNext < iDiffTimefromPrev)
        bOutput.addString(sAgentNext.c_str());
    else
        bOutput.addString(sAgentPrevious.c_str());

    return bOutput;
}


/*
* Return the consequence of an action object related
*
* bOutput :
*   Bottle 1 : Name and Object
*   Bottle 2 : location of object before
*   Bottle 3 : location of object after
*/
Bottle interlocutor::askActionForLevel3Reasoning(int Id)
{
    Bottle bOutput,
        bName,
        bRelationsBefore,
        bRelationsAfter;


    // Get name and argument.
    ostringstream osName;
    osName << "SELECT main.activityname, contentarg.argument FROM main, contentarg WHERE main.instance = contentarg.instance AND contentarg.role = 'object1' AND main.instance = " << Id;
    bName = requestFromStream(osName.str());

    string sName = (*bName.get(0).asList()).get(0).toString().c_str(),
        sArgument = (*bName.get(0).asList()).get(1).toString().c_str();


    ostringstream osRelation;

    //get location of objects before
    osRelation << "SELECT subject, object FROM relation WHERE instance = " << Id << " AND verb = 'isAtLoc'";
    bRelationsBefore = requestFromStream(osRelation.str().c_str());

    //clear things
    osRelation.str("");


    //get location of objects after
    osRelation << "SELECT subject, object FROM relation WHERE instance = " << Id + 1 << " AND verb = 'isAtLoc'";
    bRelationsAfter = requestFromStream(osRelation.str().c_str());

    bName.clear();
    bName.addString(sName.c_str());
    bName.addString(sArgument.c_str());

    bOutput.addList() = bName;
    bOutput.addList() = bRelationsBefore;
    bOutput.addList() = bRelationsAfter;

    if (sName == "hanoi")
    {
        ostringstream osSpatial;
        osSpatial << "SELECT argument,role FROM contentarg WHERE role IN ('spatial1', 'spatial2') AND instance = " << Id;
        Bottle bSpatial = requestFromStream(osSpatial.str());
        bOutput.addList() = bSpatial;
    }


    return bOutput;
}

/**
* Return the last commplex stored in the ABM
*
*/
Bottle interlocutor::askLastComplex()
{
    //bottle bQuery to ask autobiographicalMemory for ABM data the instance of OPC
    Bottle bOpcIdBegin = requestFromStream("SELECT instance FROM main WHERE activitytype = 'complex' AND begin = TRUE ORDER BY instance DESC LIMIT 1");

    int opcIdBegin = atoi(bOpcIdBegin.get(0).asList()->get(0).toString().c_str());

    return askComplexFromId(opcIdBegin);
}

/**
* Return the last commplex stored in the ABM
*
*/
Bottle interlocutor::askComplexFromId(int Id)
{
    Bottle  bOutput,    // main output
        bAction,
        bArguments,
        bOject1,
        bName,
        bQuery,
        bTemp,
        bTemporal;

    int opcIdBegin = Id;
    ostringstream osOpcEnd, osArg, osTemp;
    osOpcEnd << "SELECT instance FROM main WHERE instance > " << Id << " AND begin = false AND activitytype = 'complex' LIMIT 1 ";
    bQuery = requestFromStream(osOpcEnd.str().c_str());

    //yInfo() << "\t"   << bQuery.toString()  ;

    int opcIdEnd = atoi(bQuery.get(0).asList()->get(0).toString().c_str());

    //  -- 0. extract all the arguments
    osArg << "SELECT argument FROM contentarg WHERE instance = " << opcIdBegin;
    bArguments = requestFromStream(osArg.str());

    //  yInfo() << "\t" << "bArguments : " << bArguments.toString()  ;
    string  sObject1, sObject2,
        sAgent1, sAgent2,
        sAction1, sAction2,
        sRTO1, sRTO2,
        sTime1, sTime2,
        sTemporal;

    if (bArguments.size() != 9)
    {
        yInfo() << "\t" << "in askLastComplex : wrong number of argument for the last complex ( != 9)"  ;
        bOutput.clear();
        bOutput.addString("in askLastComplex : wrong number of argument for the last complex ( != 9)");
        return bOutput;
    }
    else
    {
        sRTO1 = bArguments.get(0).toString();
        sObject1 = bArguments.get(1).toString();
        sAgent1 = bArguments.get(2).toString();
        sAction1 = bArguments.get(3).toString();
        sTemporal = bArguments.get(4).toString();
        sRTO2 = bArguments.get(5).toString();
        sObject2 = bArguments.get(6).toString();
        sAgent2 = bArguments.get(7).toString();
        sAction2 = bArguments.get(8).toString();
    }

    // Verification of the data inside the complex

    //  osTemp << "SELECT main.activityname, contentarg.argument FROM main, contentarg WHERE main.instance = contentarg.instance AND main.instance = " << opcIdBegin +1 ;
    osTemp.str();
    osTemp << "SELECT main.activityname FROM main WHERE main.instance = " << opcIdBegin + 1;
    bTemp = requestFromStream(osTemp.str().c_str());

    if (bTemp.get(0).toString().c_str() != sAction1 || bTemp.get(0).toString().c_str() != sAction2)
    {
        yInfo() << "\t" << "in askLastComplex : Action in complex different of the ones discribed by the arguments"  ;
        bOutput.clear();
        bOutput.addString("in askLastComplex : Action in complex different of the ones discribed by the arguments");
        return bOutput;
    }

    osTemp.str("");
    osTemp << "SELECT main.activityname FROM main WHERE main.instance = " << opcIdBegin + 3;
    bTemp.clear();
    bTemp.addString("request");
    bTemp.addString(osTemp.str().c_str());
    bTemp = request(bTemp);
    if (bTemp.get(0).toString().c_str() != sAction1 || bTemp.get(0).toString().c_str() != sAction2)
    {
        yInfo() << "\t" << "in askLastComplex : Action in complex different of the ones discribed by the arguments"  ;
        bOutput.clear();
        bOutput.addString("in askLastComplex : Action in complex different of the ones discribed by the arguments");
        return bOutput;
    }

    string sDiff1, sDiff2;
    if (sAgent1 == sAgent2)
    {
        if (sRTO1 == sRTO2)
        {
            sDiff1 = sObject1;
            sDiff2 = sObject2;
        }
        else
        {
            sDiff1 = sRTO1;
            sDiff2 = sRTO2;
        }
    }
    else
    {
        sDiff1 = sAgent1;
        sDiff2 = sAgent2;
    }

    int instance1, instance2;

    ostringstream osAction1, osAction2;
    osAction1 << "SELECT main.instance FROM main, contentarg WHERE main.instance > " << opcIdBegin << " AND main.instance < " << opcIdEnd << " AND main.begin = true AND main.instance = contentarg.instance AND contentarg.argument = '" << sDiff1 << "'";
    bQuery = requestFromStream(osAction1.str());
    instance1 = atoi(bQuery.get(0).asList()->get(0).toString().c_str());
    osAction2 << "SELECT main.instance FROM main, contentarg WHERE main.instance > " << opcIdBegin << " AND main.instance < " << opcIdEnd << " AND main.begin = true AND main.instance = contentarg.instance AND contentarg.argument = '" << sDiff2 << "'";
    bQuery = requestFromStream(osAction2.str());
    instance2 = atoi(bQuery.get(0).asList()->get(0).toString().c_str());

    osTemp.str("");
    osTemp << "SELECT main.time FROM main WHERE main.instance = " << instance1;
    bTemp.clear();
    bTemp.addString("request");
    bTemp.addString(osTemp.str().c_str());
    bTemp = request(bTemp);
    sTime1 = bTemp.get(0).toString();

    osTemp.str("");
    osTemp << "SELECT main.time FROM main WHERE main.instance = " << instance2;
    bTemp.clear();
    bTemp.addString("request");
    bTemp.addString(osTemp.str().c_str());
    bTemp = request(bTemp);
    sTime2 = bTemp.get(0).toString();


    timeKnowledge tkTemporal;

    bTemporal.addString(sTemporal.c_str());
    bTemporal.addString(sTime1.c_str());
    bTemporal.addString(sTime2.c_str());

    //yInfo() << "\t" << "askLastComplex Output : " << bTemporal.toString()  ;
    return bTemporal;
}


/*
* imagine the state of the OPC according to its ID in the mentalOPC
*
*/
Bottle interlocutor::imagineOPC(int Id)
{
    Bottle bOutput,
        bMessenger;

    yInfo() << "\t" << "in imagination"  ;

    if (!mentalOPC->isConnected())
    {
        yInfo() << "\t" << "Problem in interlocutor::imagineOPC | mentalOPC not connected"  ;
        bOutput.addString("Problem in interlocutor::imagineOPC | mentalOPC not connected");
        return bOutput;
    }


    mentalOPC->checkout();

    //clean GUI :
    list<Entity*> lMental = mentalOPC->EntitiesCacheCopy();
    for (list<Entity*>::iterator it_E = lMental.begin(); it_E != lMental.end(); it_E++)
    {
        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_OBJECT)   {
            Object *Ob = mentalOPC->addObject((*it_E)->name());
            Ob->m_present = 0;
        }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_AGENT)    {
            Agent *Ag = mentalOPC->addAgent((*it_E)->name());
            Ag->m_present = 0;
        }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_RTOBJECT) {
            RTObject *Rt = mentalOPC->addRTObject((*it_E)->name());
            Rt->m_present = 0;
        }
    }

    mentalOPC->commit();
    Time::delay(0.1);
    mentalOPC->clear();
    mentalOPC->checkout();

    // ADD Agent iCub

    Agent *icub = mentalOPC->addAgent("icub");
    icub->m_present = true;
    mentalOPC->commit(icub);

    // Get the id of the RTO present
    ostringstream osIdRTO;
    osIdRTO << "SELECT position,presence,name,color FROM rtobject WHERE instance = " << Id;
    bMessenger = requestFromStream(osIdRTO.str().c_str());

    string test = "t";
    if (bMessenger.toString() == "NULL")
    {
        bOutput.addString("No RTObject.");
        return bOutput;
    }
    for (int iRTO = 0; iRTO < bMessenger.size(); iRTO++)
    {
        Bottle bRTO = *(bMessenger.get(iRTO).asList());
        string sCoordinate = bRTO.get(0).toString(),
            sPresence = bRTO.get(1).toString().c_str(),
            sName = bRTO.get(2).toString(),
            sColor = bRTO.get(3).toString();

        pair<double, double> pCoordinate = abmReasoningFunction::coordFromString(sCoordinate);
        bool bPresence = test == sPresence;
        tuple<int, int, int> tColor = abmReasoningFunction::tupleIntFromString(sColor);

        RTObject *RTOtemp = mentalOPC->addRTObject(sName);
        RTOtemp->m_ego_position[0] = pCoordinate.first;
        RTOtemp->m_ego_position[1] = pCoordinate.second;
        RTOtemp->m_present = bPresence;
        RTOtemp->m_color[0] = get<0>(tColor);
        RTOtemp->m_color[1] = get<1>(tColor);
        RTOtemp->m_color[2] = get<2>(tColor);

        mentalOPC->commit(RTOtemp);
    }

    mentalOPC->update();

    return bOutput;
}


/**
* Return the last commplex stored in the ABM
*
*/
plan interlocutor::askLastSharedPlan()
{
    //bottle bQuery to ask autobiographicalMemory for ABM data the instance of OPC
    Bottle bOpcIdBegin = requestFromStream("SELECT instance FROM main WHERE activitytype = 'sharedplan' AND begin = TRUE ORDER BY instance DESC LIMIT 1");

    int opcIdBegin = atoi(bOpcIdBegin.get(0).asList()->get(0).toString().c_str());

    return askSharedPlanFromId(opcIdBegin);
}

/**
* Return a plan according to the instance of the begining
*
*/
plan interlocutor::askSharedPlanFromId(int opcIdBegin)
{
    Bottle  bOutput;

    ostringstream osOpcEnd, osArg, osName;
    osOpcEnd << "SELECT instance FROM main WHERE instance > " << opcIdBegin << " AND begin = false AND activitytype = 'sharedplan' ORDER BY instance LIMIT 1 ";
    Bottle bQuery = requestFromStream(osOpcEnd.str().c_str());

    //yInfo() << "\t"   << bQuery.toString()  ;

    int opcIdEnd = atoi(bQuery.get(0).asList()->get(0).toString().c_str());
    osName << "SELECT activityname FROM main WHERE instance = " << opcIdBegin;
    bQuery = requestFromStream(osName.str().c_str());
    string sName = bQuery.get(0).asList()->toString().c_str();
    string sManner;
    //  -- 0. extract all the arguments
    osArg << "SELECT argument,role FROM contentarg WHERE instance = " << opcIdBegin;
    Bottle bArguments = requestFromStream(osArg.str());

    plan newPlan;
    bool fManner = false;
    // extracting argument of the plan
    for (int arg = 0; arg < bArguments.size(); arg++)
    {
        pair<string, string>    pArg;
        Bottle bTemp = *bArguments.get(arg).asList();
        pArg.first = bTemp.get(0).toString();
        pArg.second = bTemp.get(1).toString();

        if (pArg.second == abmReasoningFunction::TAG_DB_MANNER)
        {
            fManner = true;
            sManner = pArg.first;
        }
        newPlan.vArguments.push_back(pArg);
    }

    if (!fManner)
    {
        sManner = abmReasoningFunction::TAG_DB_NONE;
    }

    int NbActivity = (opcIdEnd - opcIdBegin) / 2;

    // extracting activity of the plan
    for (int acti = 0; acti < NbActivity; acti++)
    {
        // get type and name of activity
        ostringstream osActivity;
        osActivity << "SELECT activitytype, activityname FROM main WHERE instance = " << opcIdBegin + 1 + 2 * acti;
        Bottle bActivity = *(requestFromStream(osActivity.str().c_str()).get(0).asList());

        // fill newPlan 
        newPlan.vActivitytype.push_back(bActivity.get(0).toString().c_str());
        newPlan.vActivityname.push_back(bActivity.get(1).toString().c_str());

        // get argument of activity
        osActivity.str("");
        osActivity << "SELECT argument, role FROM contentarg WHERE instance = " << opcIdBegin + 1 + 2 * acti;

        bActivity = requestFromStream(osActivity.str().c_str());
        list<pair<string, string> > lArgument;
        for (int arg = 0; arg < bActivity.size(); arg++)
        {
            Bottle bRole = *bActivity.get(arg).asList();
            string sArgument = bRole.get(0).toString().c_str(),
                sRole = bRole.get(1).toString().c_str();

            for (vector< pair <string, string > >::iterator it_p = newPlan.vArguments.begin(); it_p != newPlan.vArguments.end(); it_p++)
            {
                if (it_p->first == sArgument)
                {
                    sRole = it_p->second;
                }
            }

            pair <string, string> pRole;
            pRole.first = sArgument;
            pRole.second = sRole;

            lArgument.push_back(pRole);
        }
        newPlan.vActivityArguments.push_back(lArgument);
    }

    newPlan.sManner = sManner;
    newPlan.sName = sName;

    return newPlan;
}

/**
* Return the last Behavior stored in the ABM
*
*/
behavior interlocutor::askLastBehavior()
{
    //bottle bQuery to ask autobiographicalMemory for ABM data the instance of OPC
    Bottle bOpcIdBegin = requestFromStream("SELECT instance FROM main WHERE activitytype = 'behavior' AND begin = TRUE ORDER BY instance DESC LIMIT 1");

    int opcIdBegin = atoi(bOpcIdBegin.get(0).asList()->get(0).toString().c_str());

    return askBehaviorFromId(opcIdBegin);
}

/**
* Return the last commplex stored in the ABM
*
*/
behavior interlocutor::askBehaviorFromId(int opcIdBegin)
{
    Bottle  bOutput, bBehaviorBegin, bBehaviorEnd, bTemp, bName, bArgument;
    behavior  beReturn;
    beReturn.sName = abmReasoningFunction::TAG_DB_NONE;

    // get the end of the behavior
    ostringstream osOpcEnd, osArg, osName, osBehaviorBegin, osBehaviorEnd;
    osOpcEnd << "SELECT instance FROM main WHERE instance > " << opcIdBegin << " AND begin = false AND activitytype = 'behavior' LIMIT 1 ";
    Bottle bQuery = requestFromStream(osOpcEnd.str().c_str());

    int opcIdEnd = atoi(bQuery.get(0).asList()->get(0).toString().c_str());


    // get the drives before and after the behavior
    osBehaviorBegin << "SELECT name, value FROM drives WHERE instance = " << opcIdBegin;
    osBehaviorEnd << "SELECT name, value FROM drives WHERE instance = " << opcIdEnd;

    bBehaviorBegin = requestFromStream(osBehaviorBegin.str().c_str());
    bBehaviorEnd = requestFromStream(osBehaviorEnd.str().c_str());

    vector <pair <string, double> >     vDrivesBegin, vDrivesEnd, vBehaviorEffect;

    for (int d = 0; d < bBehaviorBegin.size(); d++)
    {
        bTemp = *bBehaviorBegin.get(d).asList();
        pair <string, double>   pDrive;
        pDrive.first = bTemp.get(0).toString();
        pDrive.second = atof(bTemp.get(1).toString().c_str());
        vDrivesBegin.push_back(pDrive);
    }

    for (int d = 0; d < bBehaviorEnd.size(); d++)
    {
        bTemp = *bBehaviorEnd.get(d).asList();
        pair <string, double>   pDrive;
        pDrive.first = bTemp.get(0).toString();
        pDrive.second = atof(bTemp.get(1).toString().c_str());
        vDrivesEnd.push_back(pDrive);
    }


    // if the drives are not the same
    if (vDrivesBegin.size() != vDrivesEnd.size())
    {
        yInfo() << "\t" << "Error in abmReasoning::askBehaviorFromId | not the same drives at begining and end of a behavior"  ;
    }

    // calculate the effect on each drive
    for (vector <pair <string, double> >::iterator it_begin = vDrivesBegin.begin(); it_begin != vDrivesBegin.end(); it_begin++)
    {
        for (vector <pair <string, double> >::iterator it_end = vDrivesEnd.begin(); it_end != vDrivesEnd.end(); it_end++)
        {
            if (it_begin->first == it_end->first)
            {
                pair<string, double>        pDrive;
                pDrive.first = it_begin->first;
                pDrive.second = it_end->second - it_begin->second;

                vBehaviorEffect.push_back(pDrive);
            }
        }
    }

    // get the name of the behavior
    osName << "SELECT activityname FROM main WHERE instance = " << opcIdBegin;
    bName = requestFromStream(osName.str().c_str());
    string sName = bName.get(0).asList()->toString().c_str();

    // get the argument of the behavior
    osArg << "SELECT argument FROM contentarg WHERE instance = " << opcIdBegin << " AND role = 'argument'";
    bArgument = requestFromStream(osArg.str().c_str());
    string sArgument = bArgument.get(0).asList()->toString().c_str();


    // Fill behavior return
    beReturn.sName = sName;
    beReturn.sArgument = sArgument;
    beReturn.vEffect.push_back(vBehaviorEffect);

    return beReturn;
}


///                 SENDING KNOWLEDGE FUNCTIONS


int interlocutor::sendAdjectiveKnowledge(vector<adjKnowledge> listADK)
{
    Bottle bRequest;
    int serialSpatial = 0;
    bool bFirst = true;
    //  Spatial Knowledge
    ostringstream   osInsertKnowledge;
    osInsertKnowledge << "INSERT INTO adjectivetemporal (name, argument, timing) VALUES ";

    for (vector<adjKnowledge>::iterator it = listADK.begin(); it != listADK.end(); it++)
    {

        // add temporal timing
        if (it->vdGnlTiming.size() >= 1)
        {
            for (map<string, vector<double> >::iterator itActTim = it->mActionTiming.begin(); itActTim != it->mActionTiming.end(); itActTim++)
            {
                for (vector<double>::iterator itTiming = itActTim->second.begin(); itTiming != itActTim->second.end(); itTiming++)
                {
                    (bFirst) ? bFirst = false : osInsertKnowledge << " , ";
                    osInsertKnowledge << "( '" << it->sLabel << "' , '" << itActTim->first << "' , " << *itTiming << ") ";

                }
            }
        }
    }
    bRequest = requestFromStream(osInsertKnowledge.str().c_str());

    osInsertKnowledge.str("");
    bFirst = true;
    osInsertKnowledge << "INSERT INTO adjectivespatial (name, argument, x, y) VALUES ";

    for (vector<adjKnowledge>::iterator it = listADK.begin(); it != listADK.end(); it++)
    {

        for (map<string, vector< pair<double, double > > >::iterator itActXY = it->mActionAbsolut.begin(); itActXY != it->mActionAbsolut.end(); itActXY++)
        {

            for (vector< pair<double, double > >::iterator itXY = itActXY->second.begin(); itXY != itActXY->second.end(); itXY++)
            {
                (bFirst) ? bFirst = false : osInsertKnowledge << " , ";
                osInsertKnowledge << "( '" << it->sLabel << "' , '" << itActXY->first << "' , " << itXY->first << " , " << itXY->second << ") ";
            }
        }
    }

    bRequest = requestFromStream(osInsertKnowledge.str().c_str());

    osInsertKnowledge.str("");
    bFirst = true;
    osInsertKnowledge << "INSERT INTO adjectivespatial (name, argument, dx, dy) VALUES ";
    for (vector<adjKnowledge>::iterator it = listADK.begin(); it != listADK.end(); it++)
    {
        for (map<string, vector< pair<double, double > > >::iterator itActXY = it->mActionDelta.begin(); itActXY != it->mActionDelta.end(); itActXY++)
        {

            for (vector< pair<double, double > >::iterator itXY = itActXY->second.begin(); itXY != itActXY->second.end(); itXY++)
            {
                (bFirst) ? bFirst = false : osInsertKnowledge << " , ";
                osInsertKnowledge << "( '" << it->sLabel << "' , '" << itActXY->first << "' , " << itXY->first << " , " << itXY->second << ") ";
            }
        }

        serialSpatial++;
    }
    bRequest = requestFromStream(osInsertKnowledge.str().c_str());

    return serialSpatial;
}


/*
* Send all the spatialKnowledge to ABM
* return the number of knowledge sent
*/
int interlocutor::sendSpatialKnowledge(vector<spatialKnowledge> listSpatialKnowledge)
{
    Bottle bRequest;
    int serialSpatial = 0;
    //  Spatial Knowledge
    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
    {
        if (it->vX.size() >= 1)
        {
            ostringstream   osInsertKnowledge,
                osInsertData;

            osInsertKnowledge << "INSERT INTO spatialknowledge (name, argument, dependance, instance) VALUES ( '" << it->sName << "' , '" << it->sArgument << "', '" << it->sDependance << "', " << serialSpatial << ") ";
            bRequest = requestFromStream(osInsertKnowledge.str().c_str());
            osInsertData.str("");
            osInsertData << "INSERT INTO spatialdata (vx, vy, vdx, vdy, instance) VALUES ";
            for (unsigned int i = 0; i < it->vX.size(); i++)
            {
                osInsertData << " ( " << it->vX[i] << " , " << it->vY[i] << " , " << it->vDX[i] << " , " << it->vDY[i] << " , " << serialSpatial << ") ";
                if (i != it->vX.size() - 1)
                    osInsertData << " , ";
            }
            bRequest = requestFromStream(osInsertData.str().c_str());
            serialSpatial++;
        }
    }

    return serialSpatial;
}

/*
* Send all the temporalKnowledge to ABM
* return the number of knowledge sent
*/
int interlocutor::sendTemporalKnowledge(vector<timeKnowledge> listTimeKnowledge)
{
    int serialTime = 0;
    Bottle bRequest;
    for (vector<timeKnowledge>::iterator it = listTimeKnowledge.begin(); it != listTimeKnowledge.end(); it++)
    {
        if (it->timeArg1.size() >= 1)
        {
            ostringstream   osInsertKnowledge,
                osInsertData;

            osInsertKnowledge << "INSERT INTO timeknowledge (temporal) VALUES ('" << it->sTemporal << "' )";
            bRequest = requestFromStream(osInsertKnowledge.str().c_str());

            osInsertData.str("");
            osInsertData << "INSERT INTO timedata (temporal, timearg1, timearg2) VALUES ";

            for (unsigned int i = 0; i < it->timeArg2.size(); i++)
            {
                osInsertData << " ( '" << it->sTemporal << "' , '" << abmReasoningFunction::time2string(it->timeArg1[i]) << "' , '" << abmReasoningFunction::time2string(it->timeArg2[i]) << "') ";
                if (i != it->timeArg2.size() - 1)
                    osInsertData << " , ";
            }
            bRequest = requestFromStream(osInsertData.str().c_str());
            serialTime++;
        }
    }

    return serialTime;
}

/*
* Send all the Behaviors to ABM
* return the number of behaviors sent
*/
int interlocutor::sendBehaviors(vector<behavior> listBehaviors)
{
    int serialBehavior = 0;
    Bottle bRequest;
    ostringstream osInsertBehavior;

    // for each behavior
    for (vector<behavior>::iterator it_behavior = listBehaviors.begin(); it_behavior != listBehaviors.end(); it_behavior++)
    {
        int occurence = 0;
        osInsertBehavior.str("");
        osInsertBehavior << "INSERT INTO behavior (name, argument, instance) VALUES ( '" << it_behavior->sName << "' , '" << it_behavior->sArgument << "', " << serialBehavior << ") ";
        bRequest = requestFromStream(osInsertBehavior.str().c_str());

        // for each occurence
        for (vector< vector <pair <string, double> > >::iterator it_occurence = it_behavior->vEffect.begin(); it_occurence != it_behavior->vEffect.end(); it_occurence++)
        {

            osInsertBehavior.str("");
            osInsertBehavior << "INSERT INTO behaviordata (drive, effect, instance, occurence) VALUES ";

            //for each drive
            unsigned int iDrive = 0;
            for (vector<pair <string, double> >::iterator it_drive = it_occurence->begin(); it_drive != it_occurence->end(); it_drive++)
            {
                osInsertBehavior << "( '" << it_drive->first << "' , " << it_drive->second << " , " << serialBehavior << " , " << occurence << ") ";
                if (iDrive != it_occurence->size() - 1)
                    osInsertBehavior << " , ";
                iDrive++;
            }


            bRequest = requestFromStream(osInsertBehavior.str().c_str());
            occurence++;
        }
        serialBehavior++;
    }

    return serialBehavior;
}

/*
* Send all the plan to the ABM
* return the nomber of plan sent
*/
int interlocutor::sendPlan(vector<plan> listPlan)
{
    int serialPlan = 0;
    Bottle bRequest;
    ostringstream osInsertPlan,
        osInsertAction,
        osInsertActArg;

    // for each plan
    for (vector<plan>::iterator it_plan = listPlan.begin(); it_plan != listPlan.end(); it_plan++)
    {

        // insert the shared plan
        osInsertPlan.str("");
        osInsertPlan << "INSERT INTO sharedplan (name, manner, instance) VALUES ( '" << it_plan->sName << "' , '" << it_plan->sManner << "', " << serialPlan << ") ";
        bRequest = requestFromStream(osInsertPlan.str().c_str());

        // insert the arguments of the shared plan
        osInsertPlan.str("");
        osInsertPlan << "INSERT INTO sharedplanarg (instance, argument, role) VALUES ";
        bool bFirst = true;
        for (vector< pair <string, string> >::iterator it_spArg = it_plan->vArguments.begin(); it_spArg != it_plan->vArguments.end(); it_spArg++)
        {
            if (!bFirst)
                osInsertPlan << " , ";
            osInsertPlan << " (  " << serialPlan << " , '" << it_spArg->first << "' , '" << it_spArg->second << "' ) ";
            bFirst = false;
        }
        bRequest = requestFromStream(osInsertPlan.str().c_str());

        // insert the activities
        osInsertPlan.str("");
        osInsertAction.str("");
        osInsertAction << "INSERT INTO sharedplandata (activitytype, activityname, instance, id) VALUES ";
        bFirst = true;
        // for each activity of the plan
        for (unsigned int iAction = 0; iAction < it_plan->vActivityname.size(); iAction++)
        {
            if (!bFirst)
                osInsertAction << " , ";
            osInsertAction << " ( '" << it_plan->vActivitytype[iAction] << "' , '" << it_plan->vActivityname[iAction] << "' , " << serialPlan << " , " << iAction << " ) ";
            bFirst = false;
        }
        bRequest = requestFromStream(osInsertAction.str().c_str());

        int serialAction = 0;
        osInsertActArg.str("");
        osInsertActArg << "INSERT INTO spdataarg (id, instance, argument, role) VALUES ";
        bFirst = true;
        //for each activity of the plan
        for (vector< list < pair < string, string > > >::iterator it_actArg = it_plan->vActivityArguments.begin(); it_actArg != it_plan->vActivityArguments.end(); it_actArg++)
        {
            // for each argument
            for (list < pair < string, string > >::iterator it_ArgRole = it_actArg->begin(); it_ArgRole != it_actArg->end(); it_ArgRole++)
            {
                if (!bFirst)
                    osInsertActArg << " , ";
                bFirst = false;
                osInsertActArg << " ( " << serialAction << " , " << serialPlan << " , '" << it_ArgRole->first << "' , '" << it_ArgRole->second << "' ) ";
            }
            serialAction++;
        }

        bRequest = requestFromStream(osInsertActArg.str().c_str());

        serialPlan++;


    }

    return serialPlan;
}

/*
* Send all the temporalKnowledge to ABM
* return the number of knowledge sent
*/
int interlocutor::sendInteractionKnowledge(vector<knownInteraction> listIN)
{
    int serialInteraction = 0;
    Bottle bRequest;
    // for each interaction
    for (vector<knownInteraction>::iterator itInterac = listIN.begin(); itInterac != listIN.end(); itInterac++)
    {


        ostringstream osInsert;
        bool bFirst = true;
        osInsert << "INSERT INTO interactionknowledge (subject, argument, number, type, role) VALUES ";
        for (vector<tuple<string, int, string, string>>::iterator itTuple = itInterac->listInteraction.begin(); itTuple != itInterac->listInteraction.end(); itTuple++)
        {
            if (!bFirst)
                osInsert << " , ";
            bFirst = false;
            osInsert << " ( '" << itInterac->sSubject << "' , '" << get<0>(*itTuple) << "' , " << get<1>(*itTuple) << " , '" << get<2>(*itTuple) << "' , '" << get<3>(*itTuple) << "' ) ";
        }

        bRequest = requestFromStream(osInsert.str().c_str());
        serialInteraction++;
    }

    return serialInteraction;
}

/*
* Send all the temporalKnowledge to ABM
* return the number of knowledge sent
*/
int interlocutor::sendContextual(vector<contextualKnowledge> listContextualKnowledge)
{
    int serialContext = 0;
    Bottle bRequest;
    for (vector<contextualKnowledge>::iterator it = listContextualKnowledge.begin(); it != listContextualKnowledge.end(); it++)
    {
        if (it->vObjectPresent.size() >= 1)
        {
            ostringstream   osInsertKnowledge,
                osInsertData;

            osInsertKnowledge << "INSERT INTO contextknowledge (name, argument, dependance, instance) VALUES ( '" << it->sName << "' , '" << it->sArgument << "' , '" << it->sDependance << "', " << serialContext << ") ";
            bRequest = requestFromStream(osInsertKnowledge.str().c_str());
            osInsertData.str("");
            osInsertData << "INSERT INTO contextdata (presencebegin, presenceend, instance) VALUES ";
            for (unsigned int i = 0; i < it->vObjectPresent.size(); i++)
            {
                if (it->vObjectPresent[i].first)
                    osInsertData << " (  TRUE  , ";
                else
                    osInsertData << " ( FALSE , ";

                if (it->vObjectPresent[i].second)
                    osInsertData << "  TRUE  , " << serialContext << ") ";
                else
                    osInsertData << " FALSE , " << serialContext << ") ";

                if (i != it->vObjectPresent.size() - 1)
                    osInsertData << " , ";
            }
            bRequest = requestFromStream(osInsertData.str().c_str());

            osInsertData.str("");
            osInsertData << "INSERT INTO contextagent (instance, agent, number) VALUES  ";
            unsigned int m = 0;
            for (map<string, int>::iterator itMap = it->mAgentRelated.begin(); itMap != it->mAgentRelated.end(); itMap++)
            {
                osInsertData << "( " << serialContext << " , '" << itMap->first.c_str() << "' , " << itMap->second << ") ";
                if (m != it->mAgentRelated.size() - 1)
                    osInsertData << " , ";
                m++;
            }

            osInsertData << ";";
            bRequest = requestFromStream(osInsertData.str().c_str());

            serialContext++;
        }
    }

    return serialContext;
}


/*
*   Send the relation in the mentalOPC to the ABM at a given instance
*   input : instance
*/
Bottle interlocutor::sendRelation(int instance)
{
    Bottle bOutput;
    list<Relation>  lRelations = mentalOPC->getRelations();
    int iNbRel = getNumberRelation(instance);
    for (list<Relation>::iterator it_R = lRelations.begin(); it_R != lRelations.end(); it_R++)
    {
        Bottle bTemp;
        ostringstream osRelation;
        osRelation << "INSERT INTO relation( opcid, instance, subject, verb, object, time, manner, place ) VALUES ";
        osRelation << " ( " << it_R->ID() + iNbRel + 1 << " , " << instance << " , '" << it_R->subject().c_str() << "' , '" << it_R->verb().c_str() << "' , '" << it_R->object().c_str() << "' , '" << it_R->complement_time().c_str() << "' , '" << it_R->complement_manner().c_str() << "' , '" << it_R->complement_place().c_str() << "' ) ";
        bTemp = requestFromStream(osRelation.str());
        bOutput.addList() = bTemp;
    }

    return bOutput;
}




/**
*   Get the number of relation for a given instance
*
*/
int interlocutor::getNumberRelation(int instance)
{
    ostringstream osRequest;
    osRequest << "SELECT opcid FROM relation WHERE instance = " << instance << " ORDER BY opcid DESC LIMIT 1";

    Bottle bRequest = requestFromStream(osRequest.str());

    int iOutput = 0;

    string sNull = "NULL";
    if (bRequest.toString().c_str() != sNull)
    {
        iOutput += atoi(bRequest.get(0).asList()->get(0).toString().c_str());
    }

    return iOutput;
}


/*
*   Save the knowledge in the semantical memory
*/
Bottle interlocutor::saveKnowledge(vector<spatialKnowledge> listSK, vector<timeKnowledge> listTK, vector<behavior> listBehavior, vector<plan> listPlan, vector<contextualKnowledge> listCK, vector<knownInteraction> listInc)
{
    Bottle  bOutput,
        bRequest,
        bMessenger;

    yInfo() << "\t"   << "starting to save knowledge ... "  ;

    int serialSpatial = 0,
        serialTime = 0,
        serialBehavior = 0,
        serialPlan = 0,
        serialContext = 0,
        serialInteraction = 0;

    bMessenger.addString("resetKnowledge");
    bMessenger = request(bMessenger);

    serialSpatial = sendSpatialKnowledge(listSK);
    serialTime = sendTemporalKnowledge(listTK);
    serialBehavior = sendBehaviors(listBehavior);
    serialPlan = sendPlan(listPlan);
    serialContext = sendContextual(listCK);
    serialInteraction = sendInteractionKnowledge(listInc);

    ostringstream osOutput;
    osOutput << "resetKnowledge : " << serialSpatial << " spatialKnowledge(s) added; " << serialTime << " timeKnowledge(s) added; " << serialBehavior << " behavior(s) added; " << serialPlan << " plan(s) added; " << serialContext << " contextualKnowledge(s) added; " << serialInteraction << " interaction(s) added";
    bOutput.addString(osOutput.str().c_str());

    return bOutput;
}


//////////      RETRO REASONING
/*


*/
void interlocutor::setMentalOPC(int instance)
{
    ostringstream osAllOPCid, osAgent;
    Bottle  bAllEntity,     // result of all the entities
        bAgent,
        bObject,
        bRTObject,
        bCurrentEntity;
    // get the opcID of all the entity in the OPC
    osAllOPCid << "SELECT opcid, type, subtype FROM contentopc WHERE instance = " << instance;
    bAllEntity = requestFromStream(osAllOPCid.str());
    yInfo() << "\t" << "All entities are : \n\t\t" << bAllEntity.toString()  ;

    int iNbEntotyError = 0;

    // FOR EACH ENTITY :

    for (int iEnt = 0; iEnt < bAllEntity.size(); iEnt++)
    {
        bCurrentEntity = *bAllEntity.get(iEnt).asList();
        yInfo() << "\t" << "bCurrentEntity : " << bCurrentEntity.toString() << "\t size : " << bCurrentEntity.size()  ;
        // check size of bottle
        if (bCurrentEntity.size() == 3)
        {

        }
        // error : 
        else
        {
            yInfo() << "\t" << "Error in interlocutor::setMentalOPC | problem with currentEntity.size() "  ;
            iNbEntotyError++;
        }


    }


    // GET the Agents : 

    int opcIDofAgent = 2;

    osAgent << "SELECT name, position, orientation, color, presence FROM agent WHERE instance = " << instance << " AND opcid = " << opcIDofAgent;

}



/*
* send the command to OPCManager to update the beliefs of one opc
* true will update the real opc, and false, the mental
*/
Bottle interlocutor::updateBeliefs(bool bOPC)
{
    Bottle bMessenger, bReply;
    bMessenger.addString("updateBeliefs");
    if (bOPC)
    {
        bMessenger.addString(abmReasoningFunction::s_realOPC.c_str());
    }
    else
    {
        bMessenger.addString(abmReasoningFunction::s_mentalOPC.c_str());
    }
    port_to_OPCManager.write(bMessenger, bReply);

    return bReply;
}










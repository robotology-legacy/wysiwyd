
/*
*
* Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Grégoire Pointeau
* email:   gregoire.pointeau@inserm.fr
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

#include <abmReasoning.h>


using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


#define THRESHOLD_CONFIDENCE 5.0



// Send a SQL query (within a bottle) to AutobiographicalMemory. bRequest must be the complete request
Bottle abmReasoning::request(Bottle bRequest)
{
    return iCub->getABMClient()->requestFromString(bRequest.toString());
}


Bottle abmReasoning::requestFromStream(string sInput)
{
    return iCub->getABMClient()->requestFromString(sInput);
}



Bottle abmReasoning::findActivity(string actionName, string beginOrEnd, string select)
{
    //build the SQL query
    ostringstream os;
    os << "SELECT " << select << " FROM main WHERE activityname = '" << actionName << "'";

    //complete sqlQuery if want begin or end
    if (beginOrEnd != "both") {

        //begin
        string isBegin = "true";
        if (beginOrEnd == "end") {
            //end
            isBegin = "false";
        }
        os << " AND begin = " << isBegin;
    }

    return requestFromStream(os.str().c_str());
}


Bottle abmReasoning::findActivityById(int id, string select)
{
    //build the SQL query
    ostringstream os;
    os << "SELECT " << select << " FROM main WHERE instance = '" << id << "'";

    return requestFromStream(os.str().c_str());
}

/*
* Return the consequence on the drive of a given behavior
* input format : queryBehavior number name argument
* if no argument given all the behavior are asked
*/
Bottle abmReasoning::queryBehavior(Bottle bInput)
{
    Bottle bOutput;
    string sName, sArgument;
    int last;
    string sError = "Error in abmReasoning::queryBehavior | ";
    if (bInput.size() < 3)
    {
        sError += "Wrong number of input (<2).";
        yInfo() << "\t" << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    sName = bInput.get(2).toString();
    last = atoi(bInput.get(1).toString().c_str());
    if (last < 1)
    {
        sError += "number of last action to low";
        yInfo() << "\t" << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (sName == "all")
    {
        for (unsigned int b = 0; b < listBehaviors.size(); b++)
        {
            bOutput.addList() = listBehaviors[b].getConsequence(last);
        }
        return bOutput;
    }

    bOutput.addString(sName.c_str());
    // if the argument is specified :
    if (bInput.size() == 4)
    {
        sArgument = bInput.get(3).toString();
        for (unsigned int b = 0; b < listBehaviors.size(); b++)
        {
            if (listBehaviors[b].sName == sName && listBehaviors[b].sArgument == sArgument)
            {
                return listBehaviors[b].getConsequence(last);
            }
        }
    }


    //if the argument is not specified
    vector <pair <string, vector <double> > >   vDrive;

    // for every behavior
    for (unsigned int b = 0; b < listBehaviors.size(); b++)
    {
        if (listBehaviors[b].sName == sName)
        {
            int loop = 0,
                legal = listBehaviors[b].vEffect.size() - last;

            // for each time the behavior has been seen
            for (vector < vector< pair <string, double> > >::iterator it_occurence = listBehaviors[b].vEffect.begin(); it_occurence != listBehaviors[b].vEffect.end(); it_occurence++)
            {
                if (loop >= legal)
                {
                    // for each drive changed
                    for (vector< pair <string, double> >::iterator it_drive = it_occurence->begin(); it_drive != it_occurence->end(); it_drive++)
                    {

                        bool driveKnown = false;

                        // for each known effect
                        for (vector <pair <string, vector <double> > >::iterator it_effect = vDrive.begin(); it_effect != vDrive.end(); it_effect++)
                        {

                            // if the drive is already know
                            if (it_effect->first == it_drive->first && !driveKnown)
                            {
                                it_effect->second.push_back(it_drive->second);
                                driveKnown = true;
                            }
                        }
                        // if first time we see this drive
                        if (!driveKnown)
                        {
                            pair<string, vector <double> > newDrive;
                            newDrive.first = it_drive->first;
                            newDrive.second.push_back(it_drive->second);
                            vDrive.push_back(newDrive);
                        }
                    }
                }
                loop++;
            }
        }
    }


    //unsigned int sizeMin = 99999;
    if (vDrive.size() == 0)
    {
        return bOutput;
    }
    Bottle bDrive;
    for (unsigned int i = 0; i < vDrive.size(); i++)
    {
        bDrive.clear();
        bDrive.addString(vDrive[i].first.c_str());
        double sum = 0.;
        for (unsigned int j = 0; j < vDrive[i].second.size(); j++)
        {
            sum += vDrive[i].second[j];
        }
        bDrive.addDouble(sum / (1.*vDrive[i].second.size()));
        bOutput.addList() = bDrive;
    }

    return bOutput;
}

/*   -------------------------   GET ID (main.instance)  ------------------------------   */


vector<pair<int, int> > abmReasoning::getIdFromActivity(string actionName, Bottle bRoleAndRoleValue)
{
    //bottle bOutput to reply to the outside module
    Bottle bOutput;

    //bottle bQuery to ask autobiographicalMemory for ABM data 
    Bottle bQuery;

    bQuery.addString("request");

    //build the SQL query - first part is always the same
    ostringstream os;
    if (actionName != abmReasoningFunction::TAG_DB_NONE) {
        os << "SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << actionName << "'";
    }
    else {
        os << "SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance";
    }

    //go through the vector to build the ohter part of the SQL query
    for (int i = 0; i < bRoleAndRoleValue.size(); i++)
    {
        os << " AND contentarg.instance IN (SELECT contentarg.instance FROM contentarg WHERE contentarg.role = '" << bRoleAndRoleValue.get(i).asList()->get(0).toString() << "' AND contentarg.argument = '" << bRoleAndRoleValue.get(i).asList()->get(1).toString() << "') ";
    }

    os << " ORDER BY main.instance ASC";

    bOutput = requestFromStream(os.str().c_str());

    yInfo() << "\t" << "Output Bottle in getIdFromActivity : " << bOutput.toString();

    return getIdPairsFromBottle(bOutput);

}

//Assuming the bottle have a list of sorted id (done in getIdFromActivity with the ORDER BY)
vector<pair<int, int> > abmReasoning::getIdPairsFromBottle(Bottle idBottle)
{
    vector <pair <int, int> > vOutput;

    //  yInfo() << "\t" << "Bottle received in getIdPairsFromBottle : " << idBottle.toString()   ;

    //-1 because we have iEnd which is iBegin+1
    for (int iBegin = 0; iBegin < idBottle.size() - 1; iBegin++) {
        int iEnd = iBegin + 1;
        pair<int, int> pActivity;
        Bottle bActBegin = findActivityById(atoi(idBottle.get(iBegin).asList()->get(0).toString().c_str()), "activitytype, begin");

        //look for an activity which is begin
        if (bActBegin.get(0).asList()->get(1).toString() == "t" || bActBegin.get(0).asList()->get(1).toString() == "TRUE") {

            int nbComplexWithin = 0;
            bool endComplexFound = 0;

            //          yInfo() << "\t" << "-----> Next activity (" << idBottle.get(iEnd).asList()->get(0).toString() << ")"   ;

            //if complex : maybe it is not the next one
            if (bActBegin.get(0).asList()->get(0).toString() == "complex") {

                //              yInfo() << "\t" << "=======> Begin activity is a complex"  ;

                while (endComplexFound == 0){

                    Bottle bActEnd = findActivityById(atoi(idBottle.get(iEnd).asList()->get(0).toString().c_str()), "activitytype, begin");
                    yInfo() << "\t" << "Is " << atoi(idBottle.get(iEnd).asList()->get(0).toString().c_str()) << " the end of this complex?";

                    //go to find the next complex activity
                    if (bActEnd.get(0).asList()->get(0).toString() == "complex"){
                        yInfo() << "\t" << "=======> End activity (id" << idBottle.get(iEnd).asList()->get(0).toString() << ")  is a complex";

                        //this complex action is a begin : it is a complex within
                        if (bActEnd.get(0).asList()->get(1).toString() == "t" || bActBegin.get(0).asList()->get(1).toString() == "TRUE"){
                            nbComplexWithin += 1;
                            yInfo() << "\t" << "=======> End activity is a complex but which Begin : nbComplexWithin == " << nbComplexWithin;
                        }
                        else {
                            nbComplexWithin -= 1; //include the -1 for the END of the complex we want so nbComplexWithin will be -1
                            yInfo() << "\t" << "==========> Found an END COMPLEX  (id " << idBottle.get(iEnd).asList()->get(0).toString() << ")";
                        }

                        if (nbComplexWithin < 0) {
                            endComplexFound = 1;
                            yInfo() << "\t" << "==========> Found the proper END COMPLEX  (id " << idBottle.get(iEnd).asList()->get(0).toString() << ")";
                        }

                    }

                    //incremente iEnd if endComplex is not found
                    if (endComplexFound == 0){
                        iEnd += 1;
                    }

                }

            }

            //if activity is action, the next one is the end, or iEnd has been modified if complex
            //          yInfo() << "\t" << "(idBottle)  Pair is ("<< idBottle.get(iBegin).asList()->get(0).toString() << ", " << idBottle.get(iEnd).asList()->get(0).toString() << ")"  ;
            pActivity.first = atoi(idBottle.get(iBegin).asList()->get(0).toString().c_str());
            pActivity.second = atoi(idBottle.get(iEnd).asList()->get(0).toString().c_str());
            //          yInfo() << "\t" << "(pActivity) Pair is ("<< pActivity.first << ", " << pActivity.second << ")"  ;

            vOutput.push_back(pActivity);
        }
    }

    yInfo() << "\t" << "____________________________________ FINAL _____________________________________________";
    for (vector< pair< int, int> >::iterator it = vOutput.begin(); it != vOutput.end(); it++)
    {
        pair<int, int> pActivity(it->first, it->second);

        yInfo() << "\t" << "Pair is (" << pActivity.first << ", " << pActivity.second << ")";
    }
    yInfo() << "\t" << "________________________________________________________________________________________";

    return vOutput;
}


vector<pair<int, int> > abmReasoning::getIDfromTime(Bottle bInput)
{
    vector <pair <int, int> > vOutput;  // output
    string  sBetween,                   // date between which the search will be done
        sRequest;
    if (bInput.size() != 3)
    {
        yInfo() << "\t" << "Error in abmReasoning::getIDfromTime | Wrong number of input  (!=3)";
        return vOutput;
    }

    if (bInput.get(0).toString() == "between")
    {
        sBetween = bInput.get(1).toString() + "' AND '";
        sBetween += bInput.get(2).toString();
    }
    else if (bInput.get(0).toString() == "ago")
    {
        pair<int, string> pAgo(bInput.get(1).asInt(), bInput.get(2).toString());
        pair<string, string> pDates = abmReasoningFunction::ago2string(pAgo);
        sBetween = pDates.first + "' AND '";
        sBetween += pDates.second;
    }
    else
    {
        yInfo() << "\t" << "Error in abmReasoning::getIDfromTime | can't recognize temporal argument";
        //  return vOutput;
    }

    Bottle  bRequest,       // Use for request to ABM
        bActivity,      // Temporary Bottle with the activity
        bIdBegin;       // List of activity begin

    // Get all the activities that begin during the laps of time, with ID and activitytype
    bIdBegin = requestFromStream("SELECT instance, activitytype FROM main WHERE begin = true");
    vector< pair<int, string> > listBegin;

    for (int i = 0; i < bIdBegin.size(); i++)
    {
        bActivity = *bIdBegin.get(i).asList();
        pair<int, string> pActivity(atoi(bActivity.get(0).toString().c_str()), bActivity.get(1).toString());
        listBegin.push_back(pActivity);
    }


    for (vector< pair< int, string> >::iterator it = listBegin.begin(); it != listBegin.end(); it++)
    {
        pair<int, int> pActivity;
        pActivity.first = it->first;
        if (it->second == "action")
        {
            pActivity.second = (it->first) + 1;
            vOutput.push_back(pActivity);
        }
        else if (it->second == "complex")
        {
            ostringstream osRequest;
            osRequest << "SELECT activityname FROM main WHERE instance = " << it->first;
            bActivity = requestFromStream(osRequest.str().c_str());
            string sActivityName = bActivity.toString().c_str();

            osRequest.str("");
            osRequest << "SELECT instance FROM main WHERE instance > " << it->first << " AND activityname = '" << sActivityName << "' AND begin = false ORDER BY instance LIMIT 1 ";
            bActivity = requestFromStream(osRequest.str().c_str());
            pActivity.second = atoi(bActivity.toString().c_str());
        }
    }

    return vOutput;
}

/**
* Return the position of the object of focus of an action before and after execution.
* @param name : name of the action to get
*/
Bottle abmReasoning::getActionConsequence(pair<string, string> pNameArg)
{
    string sName = pNameArg.first;
    string sArg = pNameArg.second;
    //bottle bOutput to reply to the outside module
    Bottle bOutput;
    bOutput.addString(sName.c_str());

    Bottle bArguments;

    //bottle bQuery to ask autobiographicalMemory for ABM data 
    Bottle bOpcIdBegin;
    Bottle bOpcIdEnd;

    Bottle bIdArgBegin;
    Bottle bSubTypeArgBegin;

    Bottle bPosArgBegin;
    Bottle bPosArgEnd;

    Bottle bOutputTemp;
    Bottle bQuery;

    //extract the instances of the OPC

    ostringstream osBegin, osEnd;

    osBegin << "SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << sName.c_str() << "' AND '" << sArg.c_str() << "' IN (contentarg.argument) AND main.begin = TRUE";
    osEnd << "SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << sName << "' AND '" << sArg << "' IN (contentarg.argument) AND main.begin = FALSE";

    bOpcIdBegin = requestFromStream(osBegin.str().c_str());
    bOpcIdEnd = requestFromStream(osEnd.str().c_str());

    int numberAction = bOpcIdEnd.size();
    yInfo() << "\t" << "Getting action consequence of : " << sName << " " << sArg << ". " << numberAction << " occurences.";
    for (int i = 0; i < numberAction; i++){

        yInfo() << "\t" << i + 1 << ".. ";

        int opcIdBegin = atoi(bOpcIdBegin.get(i).asList()->get(0).toString().c_str());
        int opcIdEnd = atoi(bOpcIdEnd.get(i).asList()->get(0).toString().c_str());

        //-- 0. extract all the arguments
        ostringstream osArg;
        osArg << "SELECT argument FROM contentarg WHERE instance = " << opcIdBegin;
        bArguments = requestFromStream(osArg.str().c_str());
        //      yInfo() << "\t" << "bArguments : " << bArguments.toString()  ;
        bool bCheck = false;
        for (int k = 0; k < bArguments.size(); k++)
        {
            if (bArguments.get(k).toString().c_str() == sArg){
                bCheck = true;
            }
        }
        if (bCheck)
        {

            bOutput.addList() = bArguments;

            ostringstream osArgObj;
            osArgObj << "SELECT DISTINCT argument FROM contentarg WHERE instance = " << opcIdBegin << " AND role = 'object1'";
            Bottle bArgObj = requestFromStream(osArgObj.str().c_str());

            string argObject = bArgObj.get(0).toString().c_str();

            //-- 1. extract the id of the argument, assuming it is an entity
            ostringstream osEntity;
            osEntity << "SELECT DISTINCT entity.opcid FROM entity, contentarg WHERE entity.name = contentarg.argument AND entity.instance = " << opcIdBegin << " AND contentarg.argument = '" << argObject << "'";
            bIdArgBegin = requestFromStream(osEntity.str().c_str());
            int idArg = atoi(bIdArgBegin.get(0).asList()->get(0).toString().c_str());

            //-- 2. select the subtype of the argument in order to extract it accordingly
            osEntity.str("");
            osEntity << "SELECT contentopc.subtype from contentopc WHERE contentopc.instance = " << opcIdBegin << " AND contentopc.opcid = " << idArg;
            bSubTypeArgBegin = requestFromStream(osEntity.str().c_str());
            string subtypeArg = bSubTypeArgBegin.get(0).asList()->get(0).toString().c_str();

            //-- 3. extract the x, y of the object at begin and end of the activity
            osEntity.str("");
            osEntity << "SELECT " << subtypeArg << ".position FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << opcIdBegin << " AND " << subtypeArg << ".opcid = " << idArg;
            bPosArgBegin = requestFromStream(osEntity.str().c_str());

            osEntity.str("");
            osEntity << "SELECT " << subtypeArg << ".position FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << opcIdEnd << " AND " << subtypeArg << ".opcid = " << idArg;
            bPosArgEnd = requestFromStream(osEntity.str().c_str());

            string posArgBegin = bPosArgBegin.get(0).asList()->get(0).toString().c_str();
            string posArgEnd = bPosArgEnd.get(0).asList()->get(0).toString().c_str();


            osEntity.str("");
            osEntity << "SELECT presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << opcIdBegin << " AND " << subtypeArg << ".opcid = " << idArg;
            bPosArgBegin = requestFromStream(osEntity.str().c_str());
            int ObjectPresentBefore, ObjectPresentAfter;
            string test = "t";
            if (bPosArgBegin.get(0).asList()->get(0).toString().c_str() == test)
                ObjectPresentBefore = 1;
            else
                ObjectPresentBefore = 0;

            osEntity.str("");
            osEntity << "SELECT presence FROM " << subtypeArg << " WHERE " << subtypeArg << ".instance = " << opcIdEnd << " AND " << subtypeArg << ".opcid = " << idArg;
            bPosArgBegin = requestFromStream(osEntity.str().c_str());
            if (bPosArgBegin.get(0).asList()->get(0).toString().c_str() == test)
                ObjectPresentAfter = 1;
            else
                ObjectPresentAfter = 0;

            bOutputTemp.clear();
            bOutputTemp.addString(posArgBegin.c_str());
            bOutputTemp.addString(posArgEnd.c_str());

            Bottle bContextual;
            bContextual.addString(sName.c_str());
            bContextual.addString(sArg.c_str());
            bContextual.addInt(ObjectPresentBefore);
            bContextual.addInt(ObjectPresentAfter);

            ostringstream osAgent;
            osArgObj << "SELECT DISTINCT argument FROM contentarg WHERE instance = " << opcIdBegin << " AND role = 'agent1'";
            bArgObj = requestFromStream(osArgObj.str().c_str());

            string sSubject = bArgObj.get(0).toString().c_str();
            bContextual.addString(sSubject.c_str());


            bContextual = addContextualKnowledge(bContextual);

            bOutput.addList() = bOutputTemp;
        }
    }

    return bOutput;
}




/**
* Return the position of the object of focus of an action before and after execution.
* @param name : name of the action to get and name of the argument (e.g. : put east, push west, play gangnam-style, put none, ...). If arg is none, just the actionName is used
*/
Bottle abmReasoning::getActionConsequenceDrives(pair<string, string> pNameArg)
{
    string sName = pNameArg.first;
    string sArg = pNameArg.second;
    //bottle bOutput to reply to the outside module
    Bottle bOutput;
    bOutput.addString(sName.c_str());

    Bottle bArguments;

    //bottle bQuery to ask autobiographicalMemory for ABM data 
    Bottle bOpcIdBegin;
    Bottle bOpcIdEnd;

    Bottle bIdArgBegin;
    Bottle bSubTypeArgBegin;

    Bottle bDrivesArgBegin;
    Bottle bDrivesArgEnd;

    Bottle bOutputTemp;
    Bottle bQuery;

    //extract the instances of the OPC

    ostringstream osBegin, osEnd;

    //arg of the action is important
    if (sArg != abmReasoningFunction::TAG_DB_NONE) {
        osBegin << "SELECT main.instance, drives.name, drives.value FROM main, drives WHERE main.instance = drives.instance AND main.instance IN (SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << sName.c_str() << "' AND '" << sArg.c_str() << "' IN (contentarg.argument) AND main.begin = TRUE) ORDER BY main.instance";
        osEnd << "SELECT main.instance, drives.name, drives.value FROM main, drives WHERE main.instance = drives.instance AND main.instance IN (SELECT DISTINCT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activityname = '" << sName.c_str() << "' AND '" << sArg.c_str() << "' IN (contentarg.argument) AND main.begin = FALSE) ORDER BY main.instance";
    }
    else {
        osBegin << "SELECT main.instance, drives.name, value FROM drives, main WHERE main.instance = drives.instance AND main.activityname =  '" << sName.c_str() << "' AND main.begin = TRUE ORDER BY main.instance";
        osEnd << "SELECT main.instance, drives.name, value FROM drives, main WHERE main.instance = drives.instance AND main.activityname =  '" << sName.c_str() << "' AND main.begin = FALSE ORDER BY main.instance";
    }


    bOpcIdBegin = requestFromStream(osBegin.str().c_str());
    bOpcIdEnd = requestFromStream(osEnd.str().c_str());

    int numberActionBegin = bOpcIdBegin.size();
    int numberActionEnd = bOpcIdEnd.size();

    //check we have only pair begin/end
    if (numberActionBegin != numberActionEnd) {
        yInfo() << "\t" << "ERROR : there are " << numberActionBegin << " action(s) begin = true and " << numberActionEnd << " action(s) begin = false : CLEAN THE DATABASE";
        bOutput.addString("ERROR");
        return bOutput;
    }

    //here, numberAction = numberActionBegin = numberActionEnd
    yInfo() << "\t" << "Getting action consequence (drives) of : " << sName << " " << sArg << ". " << numberActionBegin << " occurences.";
    for (int i = 0; i < numberActionBegin; i++){

        yInfo() << "\t" << i + 1 << ".. ";

        //bOpcId -> instance, driveName, value
        int opcIdBegin = atoi(bOpcIdBegin.get(i).asList()->get(0).toString().c_str());
        int currentInstance = opcIdBegin;
        int opcIdEnd = atoi(bOpcIdEnd.get(i).asList()->get(0).toString().c_str());

        yInfo() << "\t" << "instance " << currentInstance;
        //print and do something for all the drives of the same instance
        // /!\ drives are not in the same order and defaultDrive is replaced after by curiosity
        while (atoi(bOpcIdBegin.get(i).asList()->get(0).toString().c_str()) == currentInstance){

            opcIdBegin = atoi(bOpcIdBegin.get(i).asList()->get(0).toString().c_str());
            opcIdEnd = atoi(bOpcIdEnd.get(i).asList()->get(0).toString().c_str());

            string driveNameBegin = bOpcIdBegin.get(i).asList()->get(1).toString().c_str();
            string driveNameEnd = bOpcIdEnd.get(i).asList()->get(1).toString().c_str();

            double driveValueBegin = atof(bOpcIdBegin.get(i).asList()->get(2).toString().c_str());
            double driveValueEnd = atof(bOpcIdEnd.get(i).asList()->get(2).toString().c_str());

            yInfo() << "\t" << "(" << driveNameBegin << ")" << " : " << driveValueBegin << " -> " << driveValueEnd << "\n";

            //next line. /!\ if the line was the last, we can't ask the next one, so currentInstance -1 to end the while loop
            if (i < numberActionBegin - 1) {
                i++;
            }
            else {
                currentInstance = -1;
            }
        }

        //instance is the next, we need to go to the last of the same because of the ++ in the for loop (except if last line)
        if (currentInstance != -1) {
            i--;
        }

    }

    return bOutput;
}

Bottle abmReasoning::getActionFromPostCondition(pair<string, int> postCondition)
{
    Bottle bListAction;

    //for all action, check if they could achieve the postCondition
    for (vector<plan>::iterator it_Plan = listPlan.begin(); it_Plan != listPlan.end(); it_Plan++)
    {
        //if(it_Plan->)
        {

        }
    }

    //for all action to achieve postCondition, check if they are doable (preCondition is true)

    return bListAction;
}


/**
* Find all the different complex, and put the consequence in the listTimeKnowledge
* TODO
*/
Bottle abmReasoning::findAllComplex(int from)
{
    yInfo() << "\t" << "Creating temporal knowledge from complex.";

    //check : simple object query :
    Bottle bTemporal, bOutput;
    ostringstream osRequest;
    osRequest << "SELECT DISTINCT contentarg.argument FROM main, contentarg WHERE main.activitytype = 'complex' AND contentarg.role = 'temporal' AND main.instance = contentarg.instance AND main.instance > " << from;
    bTemporal = requestFromStream(osRequest.str().c_str());
    //yInfo() << "\t" << "bTemporal : " << bTemporal.toString()   ;

    if (bTemporal.toString().c_str() == abmReasoningFunction::TAG_NULL)
    {
        yInfo() << "\t" << "0 temporal found.";
        bOutput.addString("no temporal to load.");
        return bOutput;
    }
    string sTemporal;
    ostringstream osOutput;
    osOutput << "Temporal(s) found : ";

    for (int i = 0; i < bTemporal.size(); i++)
    {
        sTemporal = bTemporal.get(i).toString();
        osOutput << sTemporal << " ; ";
        ostringstream osMessenger;
        osMessenger << "SELECT main.instance FROM main, contentarg WHERE main.instance = contentarg.instance AND main.activitytype = 'complex' AND main.begin = true AND contentarg.argument = '" << sTemporal << "'";
        Bottle  bMessenger = requestFromStream(osMessenger.str().c_str());

        for (int j = 0; j < bMessenger.size(); j++)
        {
            int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
            Bottle bComplex = askComplexFromId(Id);
            Bottle bResult = addTimeKnowledge(bComplex);
        }
    }

    bOutput.addString(osOutput.str().c_str());
    yInfo() << "\t" << osOutput.str().c_str();
    return bOutput;
}

/**
* Add all the action in the ABM chronologicaly
*
*/
Bottle abmReasoning::findAllActions(int from)
{
    yInfo() << "\t" << "Getting actions.";
    int iError = 0;
    //check : simple object query :
    Bottle bTemporal, bOutput;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE (activitytype = 'qRM' or activitytype = 'action')AND begin = true AND INSTANCE > " << from;
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());
    int numberAction = bMessenger.size();

    vector<int> vError;
    yInfo() << "\t" << "found " << numberAction << " action(s)";
    pair<double, double> BEGIN,
        MOVE,
        END,
        BEGIN2,
        END2;

    for (int j = 0; j < numberAction; j++)
    {
        yInfo() << "\t" << j + 1 << "..";
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        Bottle bAction = askActionFromId(Id);



        //yInfo() << "\t" << "bAction : "  ;

        //for (int kk = 0 ; kk < bAction.size() ; kk++)
        //{
        //  yInfo() << "\t" << "element " << kk << " : \t" << bAction.get(kk).toString()  ;
        //}



        if (bAction.size() <= 6)
        {
            yInfo() << "\t" << "Error in abmReasoning::addLastAction : wrong size of 'askLastAction' \n";
            vError.push_back(Id);
            iError++;
        }
        else
        {
            string  sName = (*bAction.get(0).asList()).get(0).toString().c_str(),
                sArgument = (*bAction.get(1).asList()).get(0).toString().c_str(),
                sXY = bAction.get(2).toString().c_str(),
                sEND = bAction.get(3).toString().c_str(),
                sXY2,
                sEND2,
                sDependance = abmReasoningFunction::TAG_DB_NONE;
            bool bSpatial2 = false;
            if (bAction.size() > 9) // spatial 2 is an object (expl put near toy)
            {
                bSpatial2 = true;
                sXY2 = bAction.get(7).toString().c_str();
                sEND2 = bAction.get(8).toString().c_str();
                sDependance = bAction.get(9).toString().c_str();
            }


            spatialKnowledge skMove;
            BEGIN = abmReasoningFunction::coordFromString(sXY);
            if (bDreaming && mentalOPC->isConnected())
            {
                string  sObject = "dream...";
                RTObject* LOCATION = mentalOPC->addOrRetrieveEntity<RTObject>(sObject);
                LOCATION->m_ego_position[0] = BEGIN.first;
                LOCATION->m_ego_position[1] = BEGIN.second;
                LOCATION->m_ego_position[2] = 0.01;
                LOCATION->m_dimensions[0] = 0.04;
                LOCATION->m_dimensions[1] = 0.04;
                LOCATION->m_dimensions[2] = 0.08;
                LOCATION->m_present = 1;
                LOCATION->m_color[0] = 100;
                LOCATION->m_color[1] = 255;
                LOCATION->m_color[2] = 0;
                mentalOPC->commit();
            }

            int ObjectPresentBefore = bAction.get(4).asInt(),
                ObjectPresentAfter = bAction.get(5).asInt();
            string sSubject = bAction.get(6).toString();

            Bottle bContextual;
            bContextual.addString(sName.c_str());
            bContextual.addString(sArgument.c_str());
            bContextual.addInt(ObjectPresentBefore);
            bContextual.addInt(ObjectPresentAfter);
            bContextual.addString(sDependance.c_str());
            bContextual.addString(sSubject.c_str());
            bContextual = addContextualKnowledge(bContextual);


            END = abmReasoningFunction::coordFromString(sEND);
            MOVE.first = END.first - BEGIN.first;
            MOVE.second = END.second - BEGIN.second;

            skMove.sName = sName;
            skMove.sArgument = sArgument;
            skMove.sDependance = sDependance;

            if (!bSpatial2)
            {
                skMove.vX.push_back(END.first);
                skMove.vY.push_back(END.second);
                skMove.vDX.push_back(MOVE.first);
                skMove.vDY.push_back(MOVE.second);
            }
            else
            {
                BEGIN2 = abmReasoningFunction::coordFromString(sXY2);
                END2 = abmReasoningFunction::coordFromString(sEND2);

                skMove.vX.push_back(BEGIN2.first - BEGIN.first);
                skMove.vY.push_back(BEGIN2.second - BEGIN.second);

                skMove.vDX.push_back(END2.first - END.first);
                skMove.vDY.push_back(END2.second - END.second);
            }

            if (ObjectPresentAfter == ObjectPresentBefore)
            {
                addSpatialKnowledge(skMove, true);
            }

            if (bDreaming && mentalOPC->isConnected())
            {
                string  sObject = "dream...";
                RTObject* LOCATION = mentalOPC->addOrRetrieveEntity<RTObject>(sObject);
                LOCATION->m_ego_position[0] = END.first;
                LOCATION->m_ego_position[1] = END.second;
                LOCATION->m_ego_position[2] = 0.01;
                LOCATION->m_present = 1;
                LOCATION->m_color[0] = 100;
                LOCATION->m_color[1] = 255;
                LOCATION->m_color[2] = 0;
                mentalOPC->commit();
            }
        }
    }
    if (bDreaming && mentalOPC->isConnected())
    {
        string  sObject = "dream...";
        RTObject* LOCATION = mentalOPC->addOrRetrieveEntity<RTObject>(sObject);
        LOCATION->m_present = 0;
        mentalOPC->commit();
    }
    yInfo() << "\t";
    if (iError != 0)
    {
        yInfo() << "\t" << iError << " errors while getting the actions:";
        for (unsigned int j = 0; j < vError.size(); j++)
        {
            yInfo() << "\t" << vError[j] << "\t ";
        }
        yInfo() << "\t";
    }
    return bOutput;
}



Bottle abmReasoning::findAllActionsV2(int from)
{
    yInfo() << "\t" << "Getting actions.";
    int iError = 0;
    //check : simple object query :
    Bottle bTemporal, bOutput;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE (activitytype = 'qRM' or activitytype = 'action') AND begin = true AND INSTANCE > " << from << " ORDER by instance";
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());
    int numberAction = bMessenger.size();

    vector<int> vError;
    yInfo() << "\t" << "found " << numberAction << " action(s)";
    pair<double, double> BEGIN,
        MOVE,
        END,
        BEGIN2,
        END2;


    //string filepath_sentence = resfind.findFileByName("sentences.txt");
    //ofstream file_sentences(filepath_sentence.c_str(), ios::out | ios::trunc);  // erase previous contents of file

    //file_sentences << "agent\tverb\tobject\tajd1\tadj2"  ;

    double step = 0.1;
    double done = step;

    for (int j = 0; j < numberAction; j++)
    {
        if ((j*1.0) / (numberAction*1.0) > done)
        {
            yInfo() << "\t" << done * 100 << "%";
            done += step;
        }
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());

        Bottle bAction = askActionFromIdV2(Id);

        if (bAction.get(0).asString() == "error")
        {
            yInfo() << "\t" << bAction.get(1).toString();
            iError++;
        }
        else
        {
            if (bAction.size() <= 5)
            {
                yInfo() << "\t" << "Error in abmReasoning::addLastAction : wrong size of 'askLastAction' \n";
                vError.push_back(Id);
                iError++;
            }
            else
            {
                string  sName = bAction.get(0).asString();

                Bottle bArgument = *bAction.get(1).asList();

                //			yInfo() << "\t"   << bAction.toString()  ;

                list<string>	lAdjectives;
                lAdjectives.push_back(bArgument.check("adv1", Value("none")).asString());
                lAdjectives.push_back(bArgument.check("adv2", Value("none")).asString());

                string	sXY = bAction.get(2).toString().c_str();
                string	sEND = bAction.get(3).toString().c_str();

                bool ObjectPresentBefore = bAction.get(4).asInt() == 1;
                bool ObjectPresentAfter = bAction.get(5).asInt() == 1;

                double dTiming = bAction.get(6).asDouble();

                BEGIN = abmReasoningFunction::coordFromString(sXY);

                if (bDreaming && mentalOPC->isConnected())
                {
                    string  sObject = "dream...";
                    RTObject* LOCATION = mentalOPC->addOrRetrieveEntity<RTObject>(sObject);
                    LOCATION->m_ego_position[0] = BEGIN.first;
                    LOCATION->m_ego_position[1] = BEGIN.second;
                    LOCATION->m_ego_position[2] = 0.01;
                    LOCATION->m_dimensions[0] = 0.04;
                    LOCATION->m_dimensions[1] = 0.04;
                    LOCATION->m_dimensions[2] = 0.08;
                    LOCATION->m_present = 1;
                    LOCATION->m_color[0] = 100;
                    LOCATION->m_color[1] = 255;
                    LOCATION->m_color[2] = 0;
                    mentalOPC->commit();
                }

                END = abmReasoningFunction::coordFromString(sEND);
                MOVE.first = END.first - BEGIN.first;
                MOVE.second = END.second - BEGIN.second;

                //file_sentences << bArgument.check("agent", Value("none")).asString() << "\t";
                //file_sentences << bArgument.check("action", Value("none")).asString() << "\t";
                //file_sentences << bArgument.check("object", Value("none")).asString() << "\t";
                //file_sentences << bArgument.check("adv1", Value("none")).asString() << "\t";
                //file_sentences << bArgument.check("adv2", Value("none")).asString()  ;

                if (ObjectPresentAfter == ObjectPresentBefore)
                {
                    for (list<string>::iterator it = lAdjectives.begin(); it != lAdjectives.end(); it++)
                    {
                        if (*it != "none")
                        {
                            addAdverbKnowledge(*it, sName, dTiming, END, MOVE);
                        }
                    }
                }

                if (bDreaming && mentalOPC->isConnected())
                {
                    string  sObject = "dream...";
                    RTObject* LOCATION = mentalOPC->addOrRetrieveEntity<RTObject>(sObject);
                    LOCATION->m_ego_position[0] = END.first;
                    LOCATION->m_ego_position[1] = END.second;
                    LOCATION->m_ego_position[2] = 0.01;
                    LOCATION->m_present = 1;
                    LOCATION->m_color[0] = 100;
                    LOCATION->m_color[1] = 255;
                    LOCATION->m_color[2] = 0;
                    mentalOPC->commit();
                }
            }
        }
    }
    if (bDreaming && mentalOPC->isConnected())
    {
        string  sObject = "dream...";
        RTObject* LOCATION = mentalOPC->addOrRetrieveEntity<RTObject>(sObject);
        LOCATION->m_present = 0;
        mentalOPC->commit();
    }
    yInfo() << "\t";

    if (iError != 0)
    {
        yInfo() << "\t" << iError << " errors while getting the actions:";
        for (unsigned int j = 0; j < vError.size(); j++)
        {
            yInfo() << "\t" << vError[j] << "\t ";
        }
        yInfo() << "\t";
    }


    //for (vector<adjKnowledge>::iterator it = listKnownAdverb.begin(); it != listKnownAdverb.end(); it++)
    //{
    //    determineTimingInfluence(*it);
    //    if (it->bothtails > 0.1)
    //    {
    //        it->determineSpatialInfluence();
    //    }

    //    if (print_in_file)
    //    {

    //        // writing data in a file
    //        string filepath_time_relative = "time_";
    //        filepath_time_relative += it->sLabel.c_str();
    //        filepath_time_relative += ".txt";
    //        string filepath_time = resfind.findFileByName(filepath_time_relative);
    //        ofstream file_time(filepath_time.c_str(), ios::out | ios::trunc);  // erase previous contents of file
    //        if (file_time){
    //            for (vector<double>::iterator itD = it->vdGnlTiming.begin(); itD != it->vdGnlTiming.end(); itD++)
    //            {
    //                file_time << *itD << endl;
    //            }
    //            yInfo() << "\t" << "file_time " << filepath_time << " written"  ;
    //        }
    //        else
    //        {
    //            yInfo() << "\t" << "cannot write " << filepath_time  ;
    //        }

    //        string filepath_space_relative = "space_";
    //        filepath_space_relative += it->sLabel.c_str();
    //        filepath_space_relative += ".txt";
    //        string filepath_space = resfind.findFileByName(filepath_space_relative);
    //        ofstream file_space(filepath_space.c_str(), ios::out | ios::trunc);  // erase previous contents of file

    //        if (file_space)
    //        {
    //            file_space << "X\tY\tDX\tDY" << endl;

    //            if (it->vdGnlDelta.size() != it->vdGnlXY.size())
    //            {
    //                yInfo() << "\t" << "problem of size dude !"  ;
    //            }
    //            else
    //            {
    //                for (unsigned int i = 0; i < it->vdGnlDelta.size(); i++)
    //                {
    //                    file_space << it->vdGnlXY[i].first << "\t" << it->vdGnlXY[i].second << "\t" << it->vdGnlDelta[i].first << "\t" << it->vdGnlDelta[i].second << endl;
    //                }
    //            }
    //            yInfo() << "\t" << "file " << filepath_space << " written"  ;


    //            if (it->mActionAbsolut.size() != it->mActionDelta.size())
    //            {
    //                yInfo() << "\t" << "problem of size and verb dude !"  ;
    //            }
    //            else
    //            {
    //                string filepath_space_verb_relative = "space_verb_";
    //                filepath_space_verb_relative += it->sLabel.c_str();
    //                filepath_space_verb_relative += ".txt";
    //                string filepath_space_verb = resfind.findFileByName(filepath_space_verb_relative);

    //                ofstream file_space_verb(filepath_space_verb.c_str(), ios::out | ios::trunc);  // erase previous contents of file

    //                file_space_verb << "X\tY\tVerb" << endl;

    //                for (map<string, vector<pair<double, double> > >::iterator itMap = it->mActionAbsolut.begin(); itMap != it->mActionAbsolut.end(); itMap++)
    //                {
    //                    for (unsigned int i = 0; i < itMap->second.size(); i++)
    //                    {
    //                        file_space_verb << itMap->second[i].first + 0.68 << "\t" << itMap->second[i].second << "\t" << itMap->first << "XY" << endl;
    //                    }

    //                    for (unsigned int i = 0; i < itMap->second.size(); i++)
    //                    {
    //                        file_space_verb << it->mActionDelta[itMap->first][i].first << "\t" << it->mActionDelta[itMap->first][i].second << "\t" << itMap->first << "DELTA" << endl;
    //                    }

    //                    yInfo() << "\t" << "file_space_verb " << filepath_space_verb << " written"  ;
    //                }
    //            }
    //        }
    //    }
    //}
    return bOutput;
}




/**
* Add all the sentences in the ABM chronologicaly
*
*/
Bottle abmReasoning::findAllSentence(int from)
{
    yInfo() << "\t" << "Getting sentence.";
    //int iError = 0;
    //check : simple object query :
    Bottle  bOutput;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE activitytype = 'sentence' or activityname = 'sentence' AND begin = true AND INSTANCE > " << from << " ORDER by instance";
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());
    int numberSentence = bMessenger.size();

    vector<int> vError;
    yInfo() << "\t" << "found " << numberSentence << " sentence(s)";
    pair<double, double> BEGIN,
        MOVE,
        END,
        BEGIN2,
        END2;

    for (int j = 0; j < numberSentence; j++)
    {
        yInfo() << "\t" << j + 1 << "..";
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        Bottle bSentence = askSentenceFromId(Id);

        if (bSentence.size() != 4)
            yInfo() << "\t" << "Error in abmReasoning::FindAllSentence - instance " << Id << ".";
        else
        {
            // add the grammar knowledge:
            listGrammarKnowledge.addInteraction(bSentence);
        }
    }

    return bOutput;
}

/**
* Find all the different behavior, and put the consequence in the listBehavior
* TODO
*/
Bottle abmReasoning::findAllBehaviors(int from)
{
    yInfo() << "\t" << "Creating drives knowledge from behaviors";

    //check : simple object query :
    Bottle  bOutput;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE activitytype = 'behavior' AND begin = true AND INSTANCE > " << from;
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());

    if (bMessenger.toString().c_str() == abmReasoningFunction::TAG_NULL)
    {
        yInfo() << "\t" << "0 behavior found.";
        bOutput.addString("no behavior to load.");
        return bOutput;
    }
    for (int j = 0; j < bMessenger.size(); j++)
    {
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        behavior beBehavior = askBehaviorFromId(Id);
        bOutput.addList() = addBehavior(beBehavior);
    }

    yInfo() << "\t" << listBehaviors.size() << " behavior(s) found.";

    return bOutput;
}

/*
* check if the last interaction is an action of a complex and add it.
* @param bInput, bottle of the last interaction
*
*/
Bottle abmReasoning::askLastActivity(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.get(1).isString())
    {
        if (bInput.get(1).toString() == abmReasoningFunction::TAG_DB_ACTION.c_str())
        {
            return  askLastAction();
        }

        if (bInput.get(1).toString() == abmReasoningFunction::TAG_DB_COMPLEX.c_str())
        {
            return  askLastComplex();
        }

        if (bInput.get(1).toString() == abmReasoningFunction::TAG_DB_SHARED_PLAN.c_str())
        {
            addLastPlan();
            return bInput;
        }
    }

    bOutput.addString("Error, ask unknown");
    return bOutput;
}

/**
* Find all the different sharedPlan, and put the consequence in the listSharedPlan
* TODO
*/
Bottle abmReasoning::findAllSharedPlan(int from)
{
    yInfo() << "\t" << "Getting known shared plans.";

    //check : simple object query :
    Bottle bTemporal, bOutput;
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE activitytype = 'sharedplan' AND begin = true AND INSTANCE > " << from;
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());

    listPlan.clear();
    listSharedPlan.clear();
    if (bMessenger.toString().c_str() == abmReasoningFunction::TAG_NULL)
    {
        yInfo() << "\t" << "0 shared plan found.";
        bOutput.addString("no shared plan to load.");
        return bOutput;
    }
    for (int j = 0; j < bMessenger.size(); j++)
    {
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        plan pSharedPlan = askSharedPlanFromId(Id);
        plan test;
        test = addPlan(pSharedPlan);

        bool PlanExisting = false,
            SPExisting = false;

        for (vector<sharedPlan>::iterator it_SP = listSharedPlan.begin(); it_SP != listSharedPlan.end(); it_SP++)
        {
            SPExisting = false;
            if ((it_SP->sManner == pSharedPlan.sManner) && (it_SP->sName == pSharedPlan.sName))
            {
                SPExisting = true;
                PlanExisting = false;
                for (vector< pair <plan, int> >::iterator it_Plan = it_SP->listPlanPossible.begin(); it_Plan != it_SP->listPlanPossible.end(); it_Plan++)
                {
                    if ((it_Plan->first.vActivityArguments == test.vActivityArguments) && (it_Plan->first.vActivityname == test.vActivityname) && (it_Plan->first.vActivitytype == test.vActivitytype) && (it_Plan->first.vArguments == test.vArguments))
                    {
                        PlanExisting = true;
                        it_Plan->second++;
                        bOutput.addString("plan already known. socre updated");
                    }
                }
                if (SPExisting && !PlanExisting)
                {
                    bOutput.addString("sharedPlan existing but new plan.");
                    pair<plan, int> pPlan(test, 1);
                    it_SP->listPlanPossible.push_back(pPlan);
                }
            }
        }

        if (!SPExisting)
        {
            bOutput.addString("new sharedPlan");
            pair<plan, int> pPlan(test, 1);

            sharedPlan newSP;
            newSP.listPlanPossible.push_back(pPlan);
            newSP.sManner = pSharedPlan.sManner;
            newSP.sName = pSharedPlan.sName;
            newSP.vArguments = pSharedPlan.vArguments;

            listSharedPlan.push_back(newSP);
        }
        addSharedPlan(test);
    }

    yInfo() << "\t" << listPlan.size() << " plan(s) found.";
    //displaySharedPlan();

    return bOutput;
}


/**
* add the current action to the list of current action and built the corresponding plan. Check if the plan is matching if one of the known SP and return the SP which match

* e.g. : from (7,8), (5,8), (3,8) to (9,10), (7,10), (5,10), (3,10)
* @param idBeginCurrentAction, int of the current Instance, begin = true
* @param idEndCurrentAction,   int of the current Instance, begin = false
*/
Bottle abmReasoning::availableSharedPlan(int idBeginCurrentAction, int idEndCurrentAction)
{
    Bottle bOutput;

    vBuiltSharedPlan.clear();


    //add action at the end of the currentList
    for (vector <pair <int, int> >::iterator it = vCurrentActions.begin(); it < vCurrentActions.end(); it++)
    {
        it->second = idEndCurrentAction;
    }

    //add action as an element of the list
    pair <int, int> solelyCurrentAction(idBeginCurrentAction, idEndCurrentAction);
    vCurrentActions.push_back(solelyCurrentAction);
    vAvailablePlans.push_back(listPlan);


    yInfo() << "\t" << "List of currentActions : ";
    for (vector <pair <int, int> >::iterator it = vCurrentActions.begin(); it < vCurrentActions.end(); it++)
    {
        //print the list
        yInfo() << "\t" << "- { " << it->first << ", " << it->second << "}";

        //build associated SP
        vBuiltSharedPlan.push_back(actionsToPlan(it->first, it->second));
    }

    // iterator to access vector of current actions, SP built with them and the available (and known) plans
    vector <pair <int, int> >::iterator it_actions = vCurrentActions.begin();
    vector <plan>::iterator it_builtSP = vBuiltSharedPlan.begin();
    vector <vector<plan> >::iterator it_plan = vAvailablePlans.begin();

    while (it_builtSP < vBuiltSharedPlan.end())
    {
        //1. just print to know where we are

        //name SP
        yInfo() << "\t" << "\n============================================================================= ";
        yInfo() << "\t" << "name SP : " << it_builtSP->sName;
        yInfo() << "\t" << "=============================================================================\n ";

        //actionName
        yInfo() << "\t" << "name actions : ";
        for (vector< string >::iterator itActivityName = it_builtSP->vActivityname.begin(); itActivityName < it_builtSP->vActivityname.end(); itActivityName++)
        {
            yInfo() << "\t" << *(itActivityName) << " ";
        }
        yInfo() << "\t";
        //arguments
        yInfo() << "\t" << "Arguments of the next SP : ";
        for (vector< list < pair < string, string > > >::iterator itActivityArguments = it_builtSP->vActivityArguments.begin(); itActivityArguments < it_builtSP->vActivityArguments.end(); itActivityArguments++)
        {
            yInfo() << "\t";
            for (list < pair < string, string > >::iterator itListArg = itActivityArguments->begin(); itListArg != itActivityArguments->end(); itListArg++)
            {
                yInfo() << "\t" << " {" << itListArg->first << ", " << itListArg->second << "}";
            }

        }
        yInfo() << "\t";

        //2. For the current built plan, check if the vector of available plan could match it. Erase the ones which don't. return empty == no plan are available

        *(it_plan) = checkPlan(*(it_builtSP), *(it_plan));

        //if there is not any available plan which could be match with the built SP => erase. (Iterator adjust : no incrementation)
        if (it_plan->empty()){
            it_plan = vAvailablePlans.erase(it_plan);
            it_builtSP = vBuiltSharedPlan.erase(it_builtSP);
            it_actions = vCurrentActions.erase(it_actions);
        }
        else {
            //build the bOutput

            //subBottle for the current built plan
            Bottle bCurrentBuiltPlan;


            //subSubBottle for the current available plan
            Bottle bCurrentAvailablePlan;

            for (vector <plan>::iterator it_availableSP = it_plan->begin(); it_availableSP < it_plan->end(); it_availableSP++)
            {

                bCurrentAvailablePlan.clear();

                //********************************* name and manner
                bCurrentAvailablePlan.addString(it_availableSP->sName.c_str());
                bCurrentAvailablePlan.addString(it_availableSP->sManner.c_str());

                //********************************* sub bottle for arg value and arg role
                Bottle bArgValue;
                Bottle bArgRole;

                for (vector <pair <string, string> >::iterator it_arguments = it_availableSP->vArguments.begin(); it_arguments < it_availableSP->vArguments.end(); it_arguments++)
                {
                    bArgValue.addString(it_arguments->first.c_str());
                    bArgRole.addString(it_arguments->second.c_str());
                }

                bCurrentAvailablePlan.addList() = bArgValue;
                bCurrentAvailablePlan.addList() = bArgRole;

                //********************************* predict next action
                Bottle bNextAction;


                bNextAction.addString("executeActivity");
                bNextAction.addString("action");

                //if builtSP.size < availableSP, else no nextAction because the SP is full

                //init 
                vector< list < pair < string, string > > >::iterator it_actArg = it_availableSP->vActivityArguments.begin();
                vector<string>::iterator it_actName = it_availableSP->vActivityname.begin();
                unsigned int i = 0;

                //incremente until the next action

                // /!\ if the SP is finished!!
                while (i < it_builtSP->vActivityname.size())
                {
                    i++;
                    it_actArg++;
                    it_actName++;
                }

                //yInfo() << "\t" << "Next Action : " << *it_actName << "  : " ;
                string actionName = *it_actName;
                bNextAction.addString(actionName.c_str());

                Bottle subNextActionArg;
                Bottle subNextActionRole;

                for (list < pair < string, string > >::iterator it_Arg = it_actArg->begin(); it_Arg != it_actArg->end(); it_Arg++)
                {
                    //yInfo() << "\t" << "argument ( " << it_Arg->first.c_str() << ", " << it_Arg->second.c_str() << ") "   ;
                    subNextActionArg.addString(it_Arg->first.c_str());

                    //change the incrementation of the role to always be 1, just like any solely action. Assuming number < 10
                    string commonRole = it_Arg->second.substr(0, it_Arg->second.size() - 1);
                    //subNextActionRole.addString(it_Arg->second.c_str());

                    stringstream str;
                    string stringNumber;

                    int defaultNumber = 1;
                    str << defaultNumber;
                    str >> stringNumber;

                    commonRole = commonRole + stringNumber;

                    subNextActionRole.addString(commonRole.c_str());
                }

                bNextAction.addList() = subNextActionArg;
                bNextAction.addList() = subNextActionRole;

                bCurrentAvailablePlan.addList() = bNextAction;

            }

            bCurrentBuiltPlan.addList() = bCurrentAvailablePlan;

            bOutput.addList() = bCurrentBuiltPlan;



            //at least one plan is elligible : incrementation
            it_plan++;
            it_builtSP++;
            it_actions++;
        }
    }

    return bOutput;

}

/**
* Find all possible SharedPlan according to the current vCurrentAction list, added with the last action
*/
Bottle abmReasoning::findPossibleSharedPlan(int beginLastAction, int endLastAction)
{
    Bottle bOutput;
    Bottle bQuery;
    //1. extract the last instance in the ABM (end of an action) and initiate the begin of this action

    //findPossibleSharedPlan() 
    //this is just to test

    /*ostringstream os;
    os << "SELECT max(instance) FROM main" ;
    bQuery = requestFromStream(os.str());
    int endLastAction = atoi(bQuery.get(0).asList()->get(0).toString().c_str()) ;
    int beginLastAction = endLastAction-1;*/



    //2. Check if there is no gap between this action and the last one : new vCurrentAction otherwise
    if ((saveEndLastAction == -1) || ((beginLastAction - saveEndLastAction) > 1) || (beginLastAction < saveEndLastAction))
    {
        yInfo() << "\t" << "\n===========================> New vCurrentActions <===========================\n";
        vCurrentActions.clear();
        vAvailablePlans.clear();
        saveEndLastAction = endLastAction;
    }

    //3. Call availableSharedPlan with the new action
    bOutput = availableSharedPlan(beginLastAction, endLastAction);


    //4. return the bottle of available shared plans
    return bOutput;
}

/**
* Add all the action in the ABM chronologicaly
*
*/
Bottle abmReasoning::findAllInteractions(int from)
{
    yInfo() << "\t" << "Getting interactions.";

    Bottle bOutput;
    if (!realOPC->isConnected())
    {
        bOutput.addString("Error in autobiographicalMemory::populateOPC | OpcClient not connected.");
        yInfo() << "\t" << bOutput.toString();
        return bOutput;
    }

    realOPC->checkout();
    realOPC->update();

    Bottle bMessenger, bReply, bDistinctEntity;
    ostringstream osRequest;
    osRequest << "SELECT DISTINCT argument FROM contentarg WHERE subtype IN ( 'object', 'rtobject', 'agent') AND INSTANCE > " << from;
    bDistinctEntity = requestFromStream(osRequest.str().c_str());
    if (bDistinctEntity.toString().c_str() == abmReasoningFunction::TAG_NULL)
    {
        yInfo() << "\t" << "0 shared plan found.";
        bOutput.addString("no shared plan to load.");
        return bOutput;
    }
    for (int iDE = 0; iDE < bDistinctEntity.size(); iDE++)
    {
        string sSubject = bDistinctEntity.get(iDE).toString();
        ostringstream osEntity;
        osEntity << "SELECT argument, subtype, role FROM contentarg WHERE instance IN(SELECT instance FROM contentarg WHERE argument = '" << sSubject << "') AND argument not in ('" << sSubject << "', 'none', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9')";
        bReply = requestFromStream(osEntity.str().c_str());

        knownInteraction tempInteraction;
        tempInteraction.sSubject = sSubject;

        if (bReply.toString() != "NULL")
        {
            for (int i = 0; i < bReply.size(); i++)
            {
                Bottle tempBottle = *bReply.get(i).asList();

                tuple<string, int, string, string> tInteraction;
                get<0>(tInteraction) = tempBottle.get(0).toString().c_str();
                get<1>(tInteraction) = 1;
                get<2>(tInteraction) = tempBottle.get(1).toString().c_str();
                get<3>(tInteraction) = tempBottle.get(2).toString().c_str();

                tempInteraction.addInteraction(tInteraction);
            }
            listKnownInteraction.push_back(tempInteraction);
        }
    }



    ostringstream osOutput;
    osOutput << listKnownInteraction.size() << " interaction(s) added.";
    yInfo() << "\t" << osOutput.str();
    bOutput.addString(osOutput.str().c_str());

    return bOutput;
}



/*   -------------------------   ADDING FUNCTIONS  ------------------------------   */


/**
* check if the last interaction is an action of a complex and add it.
*
*/
Bottle abmReasoning::addLastActivity(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.size() == 2)
    {
        if (bInput.get(1).isString())
        {
            if (bInput.get(1).asString().c_str() == abmReasoningFunction::TAG_DB_ACTION)
            {
                return addLastAction();
            }

            if (bInput.get(1).asString().c_str() == abmReasoningFunction::TAG_DB_COMPLEX)
            {
                return addLastComplex();
            }

            if (bInput.get(1).asString().c_str() == abmReasoningFunction::TAG_DB_BEHAVIOR)
            {
                return addLastBehavior();
            }

            if (bInput.get(1).asString().c_str() == abmReasoningFunction::TAG_DB_SHARED_PLAN)
            {
                return addLastSharedPlan();
            }

        }
    }
    bOutput.addString("Error, add unknown");

    return bOutput;
}

/**
* Add the last action of the autobiographicalMemory in listSpatialKnowledge
* @param none
* @return Bottle with state of the add
*/
Bottle abmReasoning::addLastAction()
{
    Bottle  bAction = askLastAction(),
        bOutput;
    if (bAction.size() <= 5)
    {
        bOutput.addString("Error in abmReasoning::addLastAction : wrong size of 'askLastAction'");
        return bOutput;
    }
    //  yInfo() << "\t" << "bLastAction " << bLastAction.toString()  ;
    string  sName = (*bAction.get(0).asList()).get(0).toString().c_str(),
        sArgument = (*bAction.get(1).asList()).get(0).toString().c_str(),
        sXY = bAction.get(2).toString().c_str(),
        sEND = bAction.get(3).toString().c_str(),
        sXY2,
        sEND2;

    bool bSpatial2 = false;
    if (bAction.size() == 9) // spatial 2 is an object (expl put near toy)
    {
        bSpatial2 = true;
        sXY2 = bAction.get(7).toString().c_str();
        sEND2 = bAction.get(8).toString().c_str();
    }
    string sHanoi = "hanoi";
    bool    isHanoi = false;
    Bottle bRTOpresent;
    if (sName == sHanoi && bAction.size() == 8)
    {
        bRTOpresent = *bAction.get(7).asList();
        isHanoi = true;
    }

    pair<double, double> BEGIN,
        MOVE,
        END,
        BEGIN2,
        END2;

    spatialKnowledge skMove;
    BEGIN = abmReasoningFunction::coordFromString(sXY);
    END = abmReasoningFunction::coordFromString(sEND);
    MOVE.first = END.first - BEGIN.first;
    MOVE.second = END.second - BEGIN.second;

    skMove.sName = sName;
    skMove.sArgument = sArgument;

    if (!bSpatial2)
    {
        skMove.vX.push_back(END.first);
        skMove.vY.push_back(END.second);
        skMove.vDX.push_back(MOVE.first);
        skMove.vDY.push_back(MOVE.second);
    }
    else
    {
        BEGIN2 = abmReasoningFunction::coordFromString(sXY2);
        END2 = abmReasoningFunction::coordFromString(sEND2);

        skMove.vX.push_back(BEGIN2.first - BEGIN.first);
        skMove.vY.push_back(BEGIN2.second - BEGIN.second);

        skMove.vDX.push_back(END2.first - END.first);
        skMove.vDY.push_back(END2.second - END.second);
    }

    addSpatialKnowledge(skMove, true);

    int ObjectPresentBefore = bAction.get(4).asInt(),
        ObjectPresentAfter = bAction.get(5).asInt();
    string sSubject = bAction.get(6).toString();

    Bottle bContextual;
    bContextual.addString(sName.c_str());
    bContextual.addString(sArgument.c_str());
    bContextual.addInt(ObjectPresentBefore);
    bContextual.addInt(ObjectPresentAfter);
    bContextual = addContextualKnowledge(bContextual);

    return bOutput;
}

/*
* Add the last complex of the autobiographicalMemory in the list of time knowledge.
*
*/
Bottle abmReasoning::addLastComplex()
{
    Bottle  bLastComplex = askLastComplex(),
        bOutput;
    //  yInfo() << "\t" << "bLastComplex " << bLastComplex.toString()  ;
    if (bLastComplex.size() <= 2)
    {
        bOutput.addString("Error in abmReasoning::addLastAction : wrong size of 'askLastAction'");
        return bOutput;
    }

    return  addTimeKnowledge(bLastComplex);

}

/*
* Add the last plan of the autobiographicalMemory in the list of plan.
*
*/
plan abmReasoning::addLastPlan()
{
    plan    pLastPlan = askLastSharedPlan();
    return addPlan(pLastPlan);
}


Bottle abmReasoning::addLastSharedPlan()
{
    yInfo() << "\t" << "Adding last Shared plan";

    //check : simple object query :
    Bottle bTemporal, bOutput;

    Bottle  bMessenger = requestFromStream("SELECT instance FROM main WHERE activitytype = 'sharedplan' AND begin = true ORDER BY instance DESC LIMIT 1");

    int Id = atoi(bMessenger.get(0).asList()->get(0).toString().c_str());
    plan pSharedPlan = askSharedPlanFromId(Id);
    plan test;
    test = addPlan(pSharedPlan);

    bool PlanExisting = false,
        SPExisting = false;

    for (vector<sharedPlan>::iterator it_SP = listSharedPlan.begin(); it_SP != listSharedPlan.end(); it_SP++)
    {
        SPExisting = false;
        if ((it_SP->sManner == pSharedPlan.sManner) && (it_SP->sName == pSharedPlan.sName))
        {
            SPExisting = true;
            PlanExisting = false;
            for (vector< pair <plan, int> >::iterator it_Plan = it_SP->listPlanPossible.begin(); it_Plan != it_SP->listPlanPossible.end(); it_Plan++)
            {
                if ((it_Plan->first.vActivityArguments == test.vActivityArguments) && (it_Plan->first.vActivityname == test.vActivityname) && (it_Plan->first.vActivitytype == test.vActivitytype) && (it_Plan->first.vArguments == test.vArguments))
                {
                    PlanExisting = true;
                    it_Plan->second++;
                    bOutput.addString("plan already known. socre updated");
                }
            }
            if (SPExisting && !PlanExisting)
            {
                bOutput.addString("sharedPlan existing but new plan.");
                pair<plan, int> pPlan(test, 1);
                it_SP->listPlanPossible.push_back(pPlan);
            }
        }
    }

    if (!SPExisting)
    {
        bOutput.addString("new sharedPlan");
        pair<plan, int> pPlan(test, 1);

        sharedPlan newSP;
        newSP.listPlanPossible.push_back(pPlan);
        newSP.sManner = pSharedPlan.sManner;
        newSP.sName = pSharedPlan.sName;
        newSP.vArguments = pSharedPlan.vArguments;

        listSharedPlan.push_back(newSP);
    }
    addSharedPlan(test);

    yInfo() << "\t" << " plan added.";
    //displaySharedPlan();

    return bOutput;
}

/*
* Add the last behavior of the autobiographicalMemory in the list of plan.
*
*/
Bottle abmReasoning::addLastBehavior()
{
    behavior beLastBehavior = askLastBehavior();
    return addBehavior(beLastBehavior);
}


/**
* Return the last commplex stored in the ABM
*
*/
plan abmReasoning::actionsToPlan(int idBegin, int idEnd)
{
    Bottle  bOutput;
    plan newPlan;

    //ostringstream osOpcEnd,   osArg, osName;
    ostringstream oss;
    oss << idBegin;
    oss << "-";
    oss << idEnd;
    string sName = oss.str();
    string sManner = abmReasoningFunction::TAG_DEFAULT;

    //put the begin/end before and after the list of action
    idBegin -= 1;
    idEnd += 1;

    int NbActivity = ((idEnd - idBegin) / 2);

    int agentNumber = 0;
    int objectNumber = 0;

    // extracting activity of the plan
    for (int acti = 0; acti < NbActivity; acti++)
    {
        //std::cout  << "NEW ACTIVITYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"  ;

        // get type and name of activity
        ostringstream osActivity;
        osActivity << "SELECT activitytype, activityname FROM main WHERE instance = " << idBegin + 1 + 2 * acti;
        Bottle bActivity = *(requestFromStream(osActivity.str().c_str()).get(0).asList());

        // fill newPlan 
        newPlan.vActivitytype.push_back(bActivity.get(0).toString().c_str());
        newPlan.vActivityname.push_back(bActivity.get(1).toString().c_str());

        // get argument of activity
        osActivity.str("");
        osActivity << "SELECT argument, role FROM contentarg WHERE instance = " << idBegin + 1 + 2 * acti;

        bActivity = requestFromStream(osActivity.str().c_str());
        list<pair<string, string> > lArgument;

        for (int arg = 0; arg < bActivity.size(); arg++)
        {
            Bottle bRole = *bActivity.get(arg).asList();
            string sArgument = bRole.get(0).toString().c_str(),
                sRole = bRole.get(1).toString().c_str();

            pair <string, string> pRole(sArgument, sRole);

            if (sRole != "spatial1")
            {
                bool isFound = false;
                bool roleFound = false;

                //check'
                for (vector< pair <string, string > >::iterator it_p = newPlan.vArguments.begin(); it_p != newPlan.vArguments.end(); it_p++)
                {
                    //yInfo() << "\t" << "currentArg = " << it_p->first << "-" << it_p->second   ;
                    //yInfo() << "\t" << "sArg = " << sArgument << " and sRole = " << sRole   ;
                    //if the name is found is vArgument -> put the proper role
                    if (it_p->first == sArgument)
                    {
                        sRole = it_p->second;
                        isFound = true;
                    }

                    if (it_p->second == sRole)
                    {
                        roleFound = true;
                    }
                }

                //not in the argument : put it
                if (isFound == false)
                {
                    //role not already defined
                    if (roleFound == false)
                    {
                        pair <string, string>   currentArg(sArgument, sRole);

                        if (sRole == "agent1")
                        {
                            agentNumber += 1;
                        }
                        else {
                            objectNumber += 1;
                        }

                        //yInfo() << "\t" << "Add in vArguments : " << currentArg.first << "-" << currentArg.second   ;
                        newPlan.vArguments.push_back(currentArg);

                        //add in vActivityType the corresponding role
                        pRole.second = currentArg.second;

                    }
                    else {

                        pair <string, string>   currentArg;
                        currentArg.first = sArgument;

                        //role : more difficult : need to check if already exist and create it accordingly if not

                        //common part : agent, object, ...
                        string commonRole = sRole.substr(0, sRole.size() - 1);

                        //number of iteration : 1, 2, 3, ...
                        //char stringNumber [1];
                        stringstream str;
                        string stringNumber;

                        if (commonRole == "agent")
                        {
                            agentNumber += 1;
                            //itoa(agentNumber, stringNumber, 10);
                            str << agentNumber;
                            str >> stringNumber;

                            //yInfo() << "\t" << "agent : stringNumber = " << stringNumber   ;

                        }
                        else {
                            objectNumber += 1;
                            //itoa(objectNumber, stringNumber, 10);
                            str << objectNumber;
                            str >> stringNumber;

                            //yInfo() << "\t" << "object : stringNumber = " << stringNumber   ;
                        }

                        commonRole = commonRole + stringNumber;
                        //yInfo() << "\t" << "commonRole = " << commonRole  ;
                        currentArg.second = commonRole;

                        //yInfo() << "\t" << "Add in vArguments : " << currentArg.first << "-" << currentArg.second   ;
                        newPlan.vArguments.push_back(currentArg);

                        //add in vActivityType the corresponding role
                        pRole.second = currentArg.second;

                    }
                }
                //add vArguments
                //if role = role in vArguments => if name != => role = role+1

            }
            //not necessarily sRole => role of the thing when vArgument.first and sArgument match in vArguments

            lArgument.push_back(pRole);
        }
        newPlan.vActivityArguments.push_back(lArgument);
    }

    newPlan.sManner = sManner;
    newPlan.sName = sName;

    return newPlan;
}


/*   -------------------------   DISCRIMINATE FUNCTIONS  ------------------------------   */


/**
* Discriminate an action of an object
* @param bInput (discriminateAction Xt-1 Yt-1 Xt Yt)
*/
Bottle abmReasoning::discriminateLastAction()
{
    //bottle bQuery to ask autobiographicalMemory for ABM data the instance of OPC
    Bottle bOpcIdBegin = requestFromStream("SELECT instance FROM main WHERE (activitytype = 'qRM' or activitytype = 'action') AND begin = TRUE ORDER BY instance DESC LIMIT 1"),
        bQuery;

    int opcIdBegin = atoi(bOpcIdBegin.get(0).asList()->get(0).toString().c_str());

    bQuery.addString("discriminateAction");
    bQuery.addInt(opcIdBegin);

    return discriminateAction(bQuery);

}

/**
* Discriminate an action of an object
* @param bInput (discriminateAction <instance>)
*/
Bottle abmReasoning::discriminateAction(Bottle bInput)
{
    Bottle  bOutput,
        bMove;

    if (bInput.size() < 2)
    {
        yInfo() << "\t" << "Error in abmReasoning::discriminateAction | wrong number of inputs";
        bOutput.addString("Error in abmReasoning::discriminateAction | wrong number of inputs");
        return bOutput;
    }
    int Id = bInput.get(1).asInt();

    bMove = askActionFromId(Id);
    spatialKnowledge skMove;
    pair<double, double> BEFORE,
        XY,
        MOVE;

    BEFORE = abmReasoningFunction::coordFromString(bMove.get(2).toString().c_str());
    XY = abmReasoningFunction::coordFromString(bMove.get(3).toString().c_str());
    MOVE.first = XY.first - BEFORE.first;
    MOVE.second = XY.second - BEFORE.second;

    string  sAction,
        sArgument;

    double  dMin = 1000000,
        dDist;

    vector<double> vcScore;
    vector< pair <string, string> > vcAction;
    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
    {
        if (it->sName != abmReasoningFunction::TAG_DB_UNKNOWN)
        {

            dDist = fabs((it->distFromMove(XY, MOVE)).second);
            vcScore.push_back(dDist);
            pair <string, string> pAction;
            pAction.first = it->sName;
            pAction.second = it->sArgument;
            vcAction.push_back(pAction);
        }
    }

    // Detect min
    int indiceMin = 0;
    for (unsigned int i = 0; i < vcScore.size(); i++)
    {
        if (vcScore[i] < dMin)
        {
            indiceMin = i;
            dMin = vcScore[i];
        }
    }

    vector<double> vcScoreTemp = vcScore;
    sort(vcScoreTemp.begin(), vcScoreTemp.end());

    double  dScore = vcScoreTemp[1] / vcScoreTemp[0];

    // Insert main discrimination
    Bottle bActionMain;
    sAction = vcAction[indiceMin].first;
    sArgument = vcAction[indiceMin].second;
    bActionMain.addString(sAction.c_str());
    bActionMain.addString(sArgument.c_str());
    //  bActionMain.addDouble(vcScore[1]/vcScore[0]);


    bool bConflict = false; // if there is conflict in the discrimination
    for (unsigned int i = 0; i < vcScore.size(); i++)
    {
        if ((int)i != indiceMin)
        {
            if ((vcScore[i] / dMin) < THRESHOLD_CONFIDENCE)
            {
                bConflict = true;
                Bottle bActionConflict;
                bActionConflict.addString(vcAction[i].first.c_str());
                bActionConflict.addString(vcAction[i].second.c_str());
                bActionConflict.addDouble(vcScore[i]);

                bOutput.addList() = bActionConflict;
            }
        }
    }

    if (bConflict)
    {

        yInfo() << "\t" << "conflict in the discrimination";
        bActionMain.addDouble(dMin);
    }
    else
    {
        bActionMain.addDouble(dScore);

        //renameAction (<instance_begin> <activity_name> <activity_type>) (argument <argument_name> <argument_type> <argument_subtype> <argument_role>)

        Bottle bRename,
            bAct,
            bArg;
        bAct.addInt(Id);
        bAct.addString(sAction.c_str());
        bAct.addString(abmReasoningFunction::TAG_DB_ACTION.c_str());
        bArg.addString(abmReasoningFunction::TAG_DB_ARGUMENT.c_str());
        bArg.addString(sArgument.c_str());
        bArg.addString("external");
        bArg.addString(abmReasoningFunction::TAG_DEFAULT.c_str());
        bArg.addString("spatial1");
        bRename.addString("renameAction");
        bRename.addList() = bAct;
        bRename.addList() = bArg;

        renameAction(bRename);

        bRename.clear();
        bAct.clear();
        bArg.clear();
        bAct.addInt(Id + 1);
        bAct.addString(sAction.c_str());
        bAct.addString(abmReasoningFunction::TAG_DB_ACTION.c_str());
        bArg.addString(abmReasoningFunction::TAG_DB_ARGUMENT.c_str());
        bArg.addString(sArgument.c_str());
        bArg.addString("external");
        bArg.addString(abmReasoningFunction::TAG_DEFAULT.c_str());
        bArg.addString("spatial1");
        bRename.addString("renameAction");
        bRename.addList() = bAct;
        bRename.addList() = bArg;

        renameAction(bRename);

        yInfo() << "\t" << "Action modified";
    }

    bOutput.addList() = bActionMain;

    yInfo() << "\t" << "Confidence : " << dScore;
    return bOutput;
}

/*
* Search all the unknown actions in the DB and try to discriminate them
*
*/
Bottle abmReasoning::discriminateUnknownActions()
{
    Bottle bOutput;
    //simple action
    string actionName = abmReasoningFunction::TAG_DB_UNKNOWN;
    Bottle bRoleAndRoleValue;

    vector <pair <int, int> > vOutput = getIdFromActivity(actionName, bRoleAndRoleValue);
    int iModified = 0;
    for (vector <pair <int, int> >::iterator it = vOutput.begin(); it != vOutput.end(); it++)
    {
        Bottle bMessenger;
        bMessenger.addString("discriminateAction");
        bMessenger.addInt(it->first);
        bMessenger = discriminateAction(bMessenger);
        double dScore = bMessenger.get(2).asDouble();
        if (dScore > 10)
        {
            iModified++;
        }
    }
    ostringstream osReturn;
    osReturn << iModified << " action(s) modified on " << vOutput.size() << " unknown action(s)";
    bOutput.addString(osReturn.str().c_str());

    return bOutput;
}




void abmReasoning::determineTimingInfluence(adjKnowledge &adjInput, bool bPrint)
{
    // get timing data all in one

    vector<double>	otherTiming;

    for (vector<adjKnowledge>::iterator it = listKnownAdverb.begin(); it != listKnownAdverb.end(); it++)
    {
        if (it->sLabel != adjInput.sLabel)
        {
            for (vector<double>::iterator dataIT = it->vdGnlTiming.begin(); dataIT != it->vdGnlTiming.end(); dataIT++)
            {
                otherTiming.push_back(*dataIT);
            }

        }
    }

    abmReasoningFunction::studentttest2(adjInput.vdGnlTiming, otherTiming, &(adjInput.bothtails), &(adjInput.lefttail), &(adjInput.righttail));

    /* yInfo() << "\t" << adjInput.sLabel << "\t bothtails: " << adjInput.bothtails  ;
    yInfo() << "\t\t\t " << "lefttail : " << adjInput.lefttail  ;
    yInfo() << "\t\t\t " << "righttail: " << adjInput.righttail  ;*/


    // if from a general point of view, the adjective influence the timing
    if (adjInput.bothtails < abmReasoningFunction::THRESHOLD_PVALUE_INFLUENCE_TIMING || 1 - adjInput.bothtails < abmReasoningFunction::THRESHOLD_PVALUE_INFLUENCE_TIMING)
    {
        adjInput.fTimingInfluence = true;
        if (bPrint) yInfo() << "\t" << adjInput.sLabel << " is time dependant.\t bothtails: " << adjInput.bothtails;
        return;
    }


    // else check for each association action/adjective:
    for (map<string, vector<double> >::iterator itMap = adjInput.mActionTiming.begin(); itMap != adjInput.mActionTiming.end(); itMap++)
    {
        abmReasoningFunction::studentttest2(itMap->second, otherTiming, &(adjInput.bothtails), &(adjInput.lefttail), &(adjInput.righttail));
        if (adjInput.bothtails < abmReasoningFunction::THRESHOLD_PVALUE_INFLUENCE_TIMING)
        {
            if (bPrint) yInfo() << "\t" << adjInput.sLabel << " influences timing when correlated to the action : " << itMap->first;
            adjInput.fTimingInfluence = true;
        }
    }
}



/**
* Add a spatialKnowledge in listSpatialKnowledge. Create a new one if the action in new or add the knowledge to an existing spatialKnowledge.
*
*/
Bottle abmReasoning::addSpatialKnowledge(spatialKnowledge skInput, bool b_Update)
{
    Bottle bOutput;
    pair<double, double> XY(skInput.vX[0], skInput.vY[0]),
        MOVE(skInput.vDX[0], skInput.vDY[0]);

    string sName = skInput.sName,
        sArgument = skInput.sArgument,
        sDependance = skInput.sDependance;

    bool bFound = false;
    if (skInput.sName == abmReasoningFunction::TAG_DB_UNKNOWN.c_str())
    {
        bOutput.addString("in abmReasoning::addSpatialKnowledge : name of action unknown");
        return bOutput;
    }
    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
    {
        if ((it->sName == sName) && (it->sArgument == sArgument) && (it->sDependance == sDependance))
        {
            it->vX.push_back(XY.first);
            it->vY.push_back(XY.second);
            it->vDX.push_back(MOVE.first);
            it->vDY.push_back(MOVE.second);
            it->determineInfluence();
            bFound = true;
            bOutput.addString("action added");
        }
    }

    if (!bFound)
    {
        skInput.iSize = skInput.vDX.size();
        skInput.determineInfluence();
        listSpatialKnowledge.push_back(skInput);
        bOutput.addString("action created");
    }

    if (b_Update)   {
        string updateLoc = updateLocation(sArgument).toString().c_str();
        bOutput.addString(updateLoc.c_str());
    }

    return bOutput;
}


/**
* Add a adjKnowledge in listKnownAdverb. Create a new one if the action in new or add the knowledge to an existing adjKnowledge.
*
*/
Bottle abmReasoning::addAdverbKnowledge(string sLabel, string sTag, double dTiming, pair<double, double> XY, pair<double, double> DXY)
{
    bool bFound = false;
    Bottle bOutput;
    for (vector<adjKnowledge>::iterator it = listKnownAdverb.begin(); it != listKnownAdverb.end(); it++)
    {
        if ((it->sLabel == sLabel))
        {
            it->mActionTiming[sTag].push_back(dTiming);
            it->vdGnlTiming.push_back(dTiming);

            it->mActionAbsolut[sTag].push_back(XY);
            it->vdGnlXY.push_back(XY);

            it->mActionDelta[sTag].push_back(DXY);
            it->vdGnlDelta.push_back(DXY);

            bFound = true;
        }
    }

    if (!bFound)
    {
        adjKnowledge	newADJ;
        newADJ.sLabel = sLabel;

        newADJ.mActionTiming[sTag].push_back(dTiming);
        newADJ.vdGnlTiming.push_back(dTiming);

        newADJ.mActionAbsolut[sTag].push_back(XY);
        newADJ.vdGnlXY.push_back(XY);

        newADJ.mActionDelta[sTag].push_back(DXY);
        newADJ.vdGnlDelta.push_back(DXY);

        listKnownAdverb.push_back(newADJ);
        bOutput.addString("action created");
    }

    return bOutput;
}



/**
* Add a timeKnowledge in listTimeKnowledge. Create a new one if the action in new or add the knowledge to an existing timeKnowledge.
*
*/
Bottle abmReasoning::addTimeKnowledge(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.size() != 3)
    {
        yInfo() << "\t" << "in abmReasoning::addTimeKnowledge : wrong number of inputs";
        bOutput.addString("in abmReasoning::addTimeKnowledge : wrong number of inputs");
        return bOutput;
    }
    if (!(bInput.get(0).isString() && bInput.get(1).isString() && bInput.get(2).isString()))
    {
        yInfo() << "\t" << "in abmReasoning::addTimeKnowledge : wrong number of inputs";
        bOutput.addString("in abmReasoning::addTimeKnowledge : wrong number of inputs");
        return bOutput;
    }
    bool bFound = false;
    for (vector<timeKnowledge>::iterator it = listTimeKnowledge.begin(); it != listTimeKnowledge.end(); it++)
    {
        string sTemp = it->sTemporal,
            sInput = bInput.get(0).toString().c_str();
        bool test = (sTemp == sInput);
        if (test)
        {
            it->addKnowledge(bInput);
            bOutput.addString("complex added");
            bFound = true;
        }
    }

    if (!bFound)
    {
        timeKnowledge tmNew;
        tmNew.fromBottle(bInput);
        listTimeKnowledge.push_back(tmNew);
        bOutput.addString("complex created");
    }

    return bOutput;
}

/**
* Add a plan in listPlan.
*
*/
plan abmReasoning::addPlan(plan pInput)
{
    Bottle bOutput;
    bool found = false,
        escapePlan;
    //  listPlan.push_back(pInput);

    int itePlan = 0;
    // for each existing plan
    for (vector<plan>::iterator current_plan = listPlan.begin(); current_plan != listPlan.end(); current_plan++)
    {
        bool fPlan = false;
        // check size and number of argument
        if ((current_plan->vActivitytype.size() == pInput.vActivitytype.size()) && (current_plan->vArguments.size() == pInput.vArguments.size()))
        {
            escapePlan = false;
            // check if arguments are the same : 
            for (unsigned int role = 0; role < pInput.vArguments.size(); role++)
            {
                // check if each argument if found
                bool fRole = false;
                for (unsigned int j = 0; j < current_plan->vArguments.size(); j++)
                {
                    if (pInput.vArguments[role].second == current_plan->vArguments[j].second)
                    {
                        fRole = true;
                        if (pInput.vArguments[role].second == abmReasoningFunction::TAG_DB_MANNER)
                        {
                            if (pInput.vArguments[role].first != current_plan->vArguments[j].first)
                                fRole = false;
                        }
                    }
                }
                if (!fRole)
                    escapePlan = true;
            }

            // all the arguments are presents, check the activities
            if (!escapePlan)
            {
                fPlan = true;
                int iRankActivity = 0;
                bool fActivity = true;
                // if all the activities are the same :
                if ((pInput.vActivityname == current_plan->vActivityname) && (pInput.vActivitytype == current_plan->vActivitytype))
                {
                    // check if the arguments are the same for each activity

                    // for each activity of the input plan
                    for (vector<list<pair<string, string> > >::iterator input_act = pInput.vActivityArguments.begin(); input_act != pInput.vActivityArguments.end(); input_act++)
                    {

                        // for each argument of the current activity of the input plan
                        for (list<pair<string, string> >::iterator input_arg = input_act->begin(); input_arg != input_act->end(); input_arg++)
                        {
                            bool fRole = false;
                            // each argument of one activity of the plan investigated
                            for (list<pair<string, string> >::iterator investigated_arg = current_plan->vActivityArguments[iRankActivity].begin(); investigated_arg != current_plan->vActivityArguments[iRankActivity].end(); investigated_arg++)
                            {
                                if (input_arg->second == investigated_arg->second)
                                {
                                    fRole = true;
                                    if (input_arg->second == "spatial1")
                                    {
                                        if (input_arg->first != investigated_arg->first)
                                            fRole = false;
                                    }
                                }
                            }

                            // if the argument isn't found : 
                            if (!fRole)
                                fActivity = false;
                        }
                        iRankActivity++;
                    }
                    if (!fActivity)
                        fPlan = false;
                }
            }
        }
        if (fPlan)
        {
            //          yInfo() << "\t" << "plan already existing"  ;
            bOutput.addString("plan already existing");
            found = true;
            plan pReturn;
            pReturn = (listPlan[itePlan]);
            return pReturn;
        }
        itePlan++;
    }

    if (!found)
    {
        //      yInfo() << "\t" << "new plan. Added in listPlan"  ;
        bOutput.addString("new plan. Added in listPlan");
        listPlan.push_back(pInput);
        plan pReturn;
        pReturn = (listPlan[listPlan.size() - 1]);
        return pReturn;
    }
    plan Empty;
    return Empty;
}



/**
* Check if the plan could be a part of an existing plan
*
*/
vector<plan> abmReasoning::checkPlan(plan pInput, vector<plan> listPlanAvailable)
{
    Bottle bOutput;
    //  listPlan.push_back(pInput);


    // for each existing plan
    //for (vector<plan>::iterator current_plan = listPlanAvailable.begin() ; current_plan != listPlanAvailable.end() ; current_plan++)

    vector<plan>::iterator current_plan = listPlanAvailable.begin();
    while (current_plan != listPlanAvailable.end())
    {
        yInfo() << "\t" << "*************************************************************\ncurrent_plan->name : " << current_plan->sName << "\n*************************************************************";
        // check size and number of argument : has to be >= 
        if (current_plan->vActivitytype.size() >= pInput.vActivitytype.size())
        {
            //to check the activityName
            vector<string>::iterator currentActivityName = current_plan->vActivityname.begin();
            vector<string>::iterator pInputActivityName = pInput.vActivityname.begin();
            bool sameActivityName = true;

            //to check the vActivityArguments
            vector< list < pair < string, string > > >::iterator currentActivityArgument = current_plan->vActivityArguments.begin();
            vector< list < pair < string, string > > >::iterator pInputActivityArgument = pInput.vActivityArguments.begin();
            bool sameActivityArguments = true;

            //check for generalization (isRoleGeneral) and sameConstArg if not, roleFound if yes
            bool isRoleGeneral = false;
            bool sameConstArg = false;

            //1. check the activityName are the same
            while (pInputActivityName != pInput.vActivityname.end())
            {

                if (*pInputActivityName != *currentActivityName)
                {
                    yInfo() << "\t" << "STOP : pInputActivityName = " << *pInputActivityName << " is different from currenActivityName = " << *currentActivityName;
                    yInfo() << "\t" << "[NACK] : " << current_plan->sName << " is the weak link because of missmatched in activityName. Bybye! \n";
                    sameActivityName = false;
                    break;
                }

                //incr iterator
                currentActivityName++;
                pInputActivityName++;
            }

            if (sameActivityName == false){

                //remove the plan from the list
                current_plan = listPlanAvailable.erase(current_plan);
            }
            else
            {
                //2. if same activities : check the roles are the same (e.g. : agent1 <action> object 1 spatial1)
                //WARNING : SIZE, should be the same because same activity

                while (pInputActivityArgument != pInput.vActivityArguments.end())
                {
                    yInfo() << "\t" << "-------- new Activity --------";
                    //check the argument role is somewhere in vArgument of the current plan investigated :
                    //     - if yes : name could be switch
                    //     - if not : name/role is always name/role1 and name should be the same


                    // for each argument of the current activity of the input plan
                    for (list<pair<string, string> >::iterator pInputArg = pInputActivityArgument->begin(); pInputArg != pInputActivityArgument->end(); pInputArg++)
                    {
                        //bool for generalization of the argument or not  [init]
                        isRoleGeneral = false;

                        //bool for spatial (no generalization ever) or object (no generalization sometimes)  [init]
                        sameConstArg = false;

                        for (vector< pair < string, string > >::iterator vArg_currentPlan = current_plan->vArguments.begin(); vArg_currentPlan != current_plan->vArguments.end(); vArg_currentPlan++)
                        {
                            if (vArg_currentPlan->second == pInputArg->second)
                            {
                                isRoleGeneral = true;
                                yInfo() << "\t" << "Generalization allowed for " << vArg_currentPlan->second;

                                //change the name according to the generalization
                                //WARNING : COULD BE RESPONSIBLE FOR A BUG
                                //yInfo() << "\t" << "\n******************************************\nWARNING : COULD BE RESPONSIBLE FOR A BUG\n******************************************\n" ; 

                                if (vArg_currentPlan->first != pInputArg->first)
                                {
                                    yInfo() << "\t" << "Change the name of the " << vArg_currentPlan->second << " from " << vArg_currentPlan->first << " to ";
                                    vArg_currentPlan->first = pInputArg->first;
                                    yInfo() << "\t" << vArg_currentPlan->first;
                                }

                                break;
                            }
                        }

                        bool roleFound = false;
                        //each argument of one activity of the current plan investigated
                        for (list<pair<string, string> >::iterator currentArg = currentActivityArgument->begin(); currentArg != currentActivityArgument->end(); currentArg++)
                        {
                            //if no Generalization : the name should be inside the arguments (role is not relevant)
                            if (isRoleGeneral == false)
                            {
                                if (pInputArg->first == currentArg->first)
                                {
                                    yInfo() << "\t" << "constant argument for " << pInputArg->second << " is found (" << currentArg->first << ")";

                                    sameConstArg = true;

                                    break;
                                }
                            }
                            //same role
                            else if (pInputArg->second == currentArg->second)
                            {
                                yInfo() << "\t" << "role " << pInputArg->second << " is found";
                                roleFound = true;

                                break;
                            }
                        }

                        //generalization : error if role not found
                        if ((isRoleGeneral == true) && (roleFound == false))
                        {
                            yInfo() << "\t" << "\nSTOP : role " << pInputArg->second << " is not found";
                            yInfo() << "\t" << "[NACK] : " << current_plan->sName << " is the weak link because of missmatched in a role. Bybye! \n";
                            sameActivityArguments = false;

                            //remove the plan from the list
                            current_plan = listPlanAvailable.erase(current_plan);

                            break;
                        }

                        //no generalization : error if not same argument name
                        if ((isRoleGeneral == false) && (sameConstArg == false))
                        {
                            yInfo() << "\t" << "\nSTOP : Constant Argument  " << pInputArg->first << " is not found";
                            yInfo() << "\t" << "[NACK] : " << current_plan->sName << " is the weak link because of missmatched in a constant argument. Bybye! \n";
                            sameActivityArguments = false;

                            //remove the plan from the list
                            current_plan = listPlanAvailable.erase(current_plan);

                            break;
                        }
                    }

                    yInfo() << "\t" << "------------------------------";

                    if ((sameActivityArguments == false))
                    {
                        break;
                    }

                    yInfo() << "\t" << "go to the next activityArgument";
                    //incr iterator
                    currentActivityArgument++;
                    pInputActivityArgument++;
                }

                if ((sameActivityName == true) && (sameActivityArguments == true))
                {
                    yInfo() << "\t" << "\n[ACK] : Shared Plan " << current_plan->sName << " is elligible \n ";
                    current_plan++;
                }
            }



            //current SP is smaller than the inputPlan
        }
        else {
            yInfo() << "\t" << "\nSTOP : Shared Plan " << current_plan->sName << " is smaller than the plan input ";
            yInfo() << "\t" << "[NACK] : " << current_plan->sName << " is the weak link because of missmatched in a role. Bybye! \n";

            //remove the plan from the list
            current_plan = listPlanAvailable.erase(current_plan);
        }
    }

    return listPlanAvailable;
}

/**
* Add a plan in listPlan.
*
*/
Bottle abmReasoning::addSharedPlan(plan pInput)
{
    Bottle bOutput;


    return bOutput;
}

/**
* Add a behavior in listPlan.
*
*/
Bottle abmReasoning::addBehavior(behavior beInput)
{
    Bottle bOutput;
    bool found = false;

    for (unsigned int b = 0; b < listBehaviors.size(); b++)
    {
        if (listBehaviors[b].sName == beInput.sName && !found && listBehaviors[b].sArgument == beInput.sArgument)
        {
            found = true;
            listBehaviors[b].vEffect.push_back(beInput.vEffect[0]);
            bOutput.addString("behavior found. Behavior updated");
            return bOutput;
        }
    }

    listBehaviors.push_back(beInput);
    bOutput.addString("behavior new. Behavior created");

    return bOutput;
}


/*
* Add some information to a contextualKnowledge existing, or create a new one
* bInput : name , argument , before (int) , after (int)
*/
Bottle abmReasoning::addContextualKnowledge(Bottle bInput)
{
    Bottle bOutput;

    string sName = bInput.get(0).toString().c_str(),
        sArgument = bInput.get(1).toString().c_str(),
        sXY = bInput.get(4).toString().c_str(),
        sEND = bInput.get(5).toString().c_str();
    int Before = bInput.get(2).asInt(),
        After = bInput.get(3).asInt();

    string sDependance = bInput.get(4).toString().c_str();
    pair<bool, bool> pPresence((Before == 1), (After == 1));

    for (vector<contextualKnowledge>::iterator it_CK = listContextualKnowledge.begin(); it_CK != listContextualKnowledge.end(); it_CK++)
    {
        if (it_CK->sName == sName.c_str() && it_CK->sArgument == sArgument.c_str() && it_CK->sDependance == sDependance.c_str())
        {
            it_CK->vObjectPresent.push_back(pPresence);
            it_CK->updatePresence();
            it_CK->mAgentRelated[bInput.get(5).toString().c_str()] += 1;
            //          yInfo() << "\t" << "ContextualKnowledge already existing"  ;
            bOutput.addString("ContextualKnowledge already existing");
            return bOutput;
        }
    }

    contextualKnowledge newCK;
    newCK.sArgument = sArgument;
    newCK.sName = sName;
    newCK.sDependance = sDependance;
    newCK.vObjectPresent.push_back(pPresence);
    newCK.updatePresence();
    newCK.mAgentRelated[bInput.get(5).toString().c_str()] = 1;

    listContextualKnowledge.push_back(newCK);
    //  yInfo() << "\t" << "ContextualKnowledge created"  ;
    bOutput.addString("ContextualKnowledge created");

    return bOutput;
}


bool abmReasoning::createContextualKnowledge(string sName, string sArgument, string sKind)
{
    for (vector<contextualKnowledge>::iterator it_CK = listContextualKnowledge.begin(); it_CK != listContextualKnowledge.end(); it_CK++)
    {
        if (it_CK->sName == sName.c_str()
            && it_CK->sArgument == sArgument.c_str()
            && sKind == it_CK->sType)
        {
            //yInfo() << "\t" << "ContextualKnowledge " << sName << " " << sArgument << " already existing";
            return false;
        }
    }

    contextualKnowledge newCK;
    newCK.sArgument = sArgument;
    newCK.sName = sName;
    newCK.sDependance = "none";
    newCK.sType = sKind;
    newCK.iOccurence = 0;

    listContextualKnowledge.push_back(newCK);
    // yInfo() << "\t" << "ContextualKnowledge " << sName << " " << sArgument << " created";

    return true;
}


/*
*   askGrammar
*   get a bottle with
*   (speaker "speaker")
*   (addressee "addressee")
*   (subject "subject")
*   (agent "agent)
*
*  with one none, and return it filled
*/
Bottle abmReasoning::askGrammar(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.size() != 2)
    {
        yInfo() << "\t" << "Error in abmReasoning::askGrammar - wrong size of input.";
        bOutput.addString("Error in abmReasoning::askGrammar - wrong size of input.");
        return bOutput;
    }

    if (!bInput.get(1).isList())
    {
        yInfo() << "\t" << "Error in abmReasoning::askGrammar - wrong format of input.";
        bOutput.addString("Error in abmReasoning::askGrammar - wrong formats of input.");
        return bOutput;
    }

    Bottle bContent = *bInput.get(1).asList();

    if (bContent.size() != 4)
    {
        yInfo() << "\t" << "Error in abmReasoning::askGrammar - wrong size of arguments.";
        bOutput.addString("Error in abmReasoning::askGrammar - wrong size of arguments.");
        return bOutput;
    }


    string X,
        Y,
        Z,
        P;

    bool    fSpeaker = false,
        fAddressee = false,
        fSubject = false,
        fAgent = false;

    X = bContent.check(abmReasoningFunction::TAG_SPEAKER.c_str(), Value(abmReasoningFunction::TAG_NONE)).asString().c_str();
    Y = bContent.check(abmReasoningFunction::TAG_ADRESSEE.c_str(), Value(abmReasoningFunction::TAG_NONE)).asString().c_str();
    P = bContent.check(abmReasoningFunction::TAG_SUBJECT.c_str(), Value(abmReasoningFunction::TAG_NONE)).toString().c_str();
    Z = bContent.check(abmReasoningFunction::TAG_AGENT.c_str(), Value(abmReasoningFunction::TAG_NONE)).toString().c_str();

    if (X != abmReasoningFunction::TAG_NONE)
        fSpeaker = true;
    if (Y != abmReasoningFunction::TAG_NONE)
        fAddressee = true;
    if (P != abmReasoningFunction::TAG_NONE)
        fSubject = true;
    if (Z != abmReasoningFunction::TAG_NONE)
        fAgent = true;

    // if all arguments are here, or miss 2 or more arguments
    if ((int)fSpeaker + (int)fAddressee + (int)fSubject + (int)fAgent == 4 || (int)fSpeaker + (int)fAddressee + (int)fSubject + (int)fAgent < 3)
    {
        yInfo() << "\t" << "Error in abmReasoning::askGrammar - wrong size of arguments.";
        bOutput.addString("Error in abmReasoning::askGrammar - wrong size of arguments.");
        return bOutput;
    }


    /*------------------------
    IF SUBJECT IS MISSING
    ------------------------*/
    if (!fSubject)
    {
        pair<string, double> pResult = listGrammarKnowledge.findSubject(X, Y, Z);
        X = pResult.first;
    }

    /*------------------------
    END IF SUBJECT IS MISSING
    ------------------------*/


    /*------------------------
    IF AGENT IS MISSING
    ------------------------*/
    if (!fAgent)
    {
        pair<string, double> pResult = listGrammarKnowledge.findAgent(X, Y, P);
        Z = pResult.first;
    }

    /*------------------------
    END IF AGENT IS MISSING
    ------------------------*/


    /*------------------------
    IF ADDRESSEE IS MISSING
    ------------------------*/
    if (!fAddressee)
    {
        pair<string, double> pResult = listGrammarKnowledge.findAddressee(X, Z, P);
        Y = pResult.first;
    }

    /*------------------------
    END IF ADDRESSEE IS MISSING
    ------------------------*/


    /*------------------------
    IF SPEAKER IS MISSING
    ------------------------*/
    if (!fSpeaker)
    {
        pair<string, double> pResult = listGrammarKnowledge.findSpeaker(Y, Z, P);
        X = pResult.first;
    }

    /*------------------------
    END IF SPEAKER IS MISSING
    ------------------------*/


    Bottle bSpeaker,
        bAddressee,
        bSubject,
        bAgent;

    bSpeaker.addString(abmReasoningFunction::TAG_SPEAKER.c_str());
    bSpeaker.addString(X.c_str());

    bAddressee.addString(abmReasoningFunction::TAG_ADRESSEE.c_str());
    bAddressee.addString(Y.c_str());

    bSubject.addString(abmReasoningFunction::TAG_SUBJECT.c_str());
    bSubject.addString(P.c_str());

    bAgent.addString(abmReasoningFunction::TAG_AGENT.c_str());
    bAgent.addString(Z.c_str());

    bOutput.addList() = bSpeaker;
    bOutput.addList() = bAddressee;
    bOutput.addList() = bSubject;
    bOutput.addList() = bAgent;

    return bOutput;

}



Bottle  abmReasoning::askWordKnowledge(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 3)
    {
        yInfo() << "\t" << "Error in abmReasoning::askWordKnowedge -- wrong input number (should be 3)";
    }
    string  sQuestion = bInput.get(1).toString();
    string  sWhat = bInput.get(2).toString();

    vector<string>  vContext;

    list<Entity*> temp = realOPC->EntitiesCache();

    for (list<Entity*>::iterator itE = temp.begin(); itE != temp.end(); itE++)
    {
        string type = (*itE)->entity_type();
        if (type == EFAA_OPC_ENTITY_OBJECT || type == EFAA_OPC_ENTITY_RTOBJECT || type == EFAA_OPC_ENTITY_AGENT)
        {
            Object o;
            o.fromBottle((*itE)->asBottle());
            if (o.m_present)
            {
                vContext.push_back(o.name());
            }
        }
    }

    return WordKnowledge.askWordKnowledge(sQuestion, sWhat, vContext);
}



/**
* Return the last action stored in the ABM
* @b output format : ("action" action_name) ("argument" arg1 arg2 ... argn) ("object1" XtYtn Xt+1Yt+1) facultative : ("object2" XtYtn Xt+1Yt+1)
*/
Bottle abmReasoning::askLastAction()
{
    //extract the instances of the OPC
    Bottle  bOpcIdBegin = requestFromStream("SELECT instance FROM main WHERE (activitytype = 'qRM' or activitytype = 'action') AND begin = TRUE ORDER BY instance DESC LIMIT 1");

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
Bottle abmReasoning::askActionFromId(int Id)
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


        if (bRTOpresent.toString().c_str() != abmReasoningFunction::TAG_NULL)
        {
            bOutput.addList() = bRTOpresent;
        }
        return bOutput;
    }

    //--5. get the other rtobject present


    if (bContent.toString().c_str() == abmReasoningFunction::TAG_NULL)
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

    if (sArgSpatial2 == abmReasoningFunction::TAG_NULL){

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
Bottle abmReasoning::askActionFromIdV2(int Id)
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
    lAdjectives.push_back(bArguments.check("recipient", Value("none")).asString());

    sName = bArguments.check("action", Value("none")).asString();
    if (sName == "none")
        sName = bArguments.check("action1", Value("none")).asString();
    if (sName == "none")
        sName = bArguments.check("predicate", Value("none")).asString();

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
Bottle abmReasoning::askSentenceFromId(int Id)
{
    Bottle  bOutput,    // main output
        bSentence,  // first part of the return: information about grammar
        bName,  // information about the corresponding action
        bRelBef,    // bottle of relation before
        bRelAft,    // bottle of relations after
        bContent,       // content of the sentence
        bPrevious,      // previous action
        bNext,          // next action
        bTemp;


    //    yInfo() << "\t" << "Treatment of sentence instance : " << Id;

    // 1-
    ostringstream osName;
    osName << "SELECT main.time, contentarg.argument, contentarg.role FROM main, contentarg WHERE main.instance = contentarg.instance AND main.instance =" << Id;
    bContent = requestFromStream(osName.str());

    //    yInfo() << "\t" << "bContent : " << bContent.toString();

    if (bContent.toString() == abmReasoningFunction::TAG_NULL)
    {
        return bContent;
    }

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
        yWarning() << "\t" << "Error in abmReasoning::FindSentenceFromId - Id = " << Id << ". Lack of information in the sentence.";
        bOutput.clear();
        bOutput.addString(abmReasoningFunction::TAG_NULL);
        return bOutput;
    }

    // 2-
    int iInstancePrevious, iInstanceNext;
    string sTimePrevious, sTimeNext,
        sAgentPrevious, sAgentNext;

    osName.str("");
    osName << "select main.time,main.instance, contentarg.argument  from main, contentarg where main.instance = contentarg.instance AND contentarg.role = 'agent1' AND (activitytype = 'qRM' or activitytype = 'action') and main.instance < " << Id << " and begin = 'FALSE' order by instance DESC limit 1";
    bTemp = *(requestFromStream(osName.str()).get(0).asList());

    sTimePrevious = bTemp.get(0).toString().c_str();
    iInstancePrevious = atoi(bTemp.get(1).toString().c_str());
    sAgentPrevious = bTemp.get(2).toString().c_str();


    // 3-
    osName.str("");
    osName << "select main.time,main.instance, contentarg.argument  from main, contentarg where main.instance = contentarg.instance AND (contentarg.role = 'agent1' or contentarg.role = 'agent') AND (activitytype = 'qRM' or activitytype = 'action')and main.instance > " << Id << " and begin = 'TRUE' order by instance limit 1";
    Bottle bRequest = requestFromStream(osName.str());
    //yInfo() << "\t" << "brequest : " << bRequest.toString();
    int iDiffTimefromNext = 10000;

    if (!(bRequest.toString() == "NULL"))
    {
        bTemp = *(bRequest.get(0).asList());
        //        yInfo() << "\t" << "bTemp : " << bTemp.toString();

        sTimeNext = bTemp.get(0).toString().c_str();
        iInstanceNext = atoi(bTemp.get(1).toString().c_str());
        sAgentNext = bTemp.get(2).toString().c_str();
        iDiffTimefromNext = abs(abmReasoningFunction::timeDiffSecondFromString(sTimeSentence, sTimeNext));
    }

    // 4-
    int iDiffTimefromPrev = abs(abmReasoningFunction::timeDiffSecondFromString(sTimeSentence, sTimePrevious));

    bSentence.addString(sSpeaker.c_str());
    bSentence.addString(sAddressee.c_str());
    bSentence.addString(sSubject.c_str());

    int iInstanceAction;

    // if next is closer in time
    if (iDiffTimefromNext < iDiffTimefromPrev)    {
        iInstanceAction = iInstanceNext;
        bSentence.addString(sAgentNext.c_str());
    }
    else{
        iInstanceAction = iInstanceNext;
        bSentence.addString(sAgentPrevious.c_str());
    }

    //cout << "\t bSentence: " << bSentence.toString() << endl;

    bOutput.addList() = bSentence;

    // Get name and argument.
    osName.str("");
    osName << "SELECT role, argument FROM contentarg WHERE (role = 'object1' OR role = 'object' OR role = 'recipient' OR role = 'agent'OR role = 'predicate') AND instance = " << Id;
    bName = requestFromStream(osName.str());

    //    yInfo() << "bName is: " << bName.toString();


    if (bName.toString() == abmReasoningFunction::TAG_NULL)
    {
        bName.addString(abmReasoningFunction::TAG_NULL);
        bOutput.addList() = bName;
        return bOutput;
    }

    string sPredicate = bName.check("predicate", Value(abmReasoningFunction::TAG_NULL)).asString();
    string sAgent = bName.check("agent", Value(abmReasoningFunction::TAG_NULL)).asString();
    string sObject = bName.check("object", Value(abmReasoningFunction::TAG_NULL)).asString();
    string sRecipient = bName.check("recipient", Value(abmReasoningFunction::TAG_NULL)).asString();

    ostringstream osRelation;

    bName.clear();
    bName.addString(sPredicate.c_str());
    bName.addString(sAgent.c_str());
    bName.addString(sObject.c_str());
    bName.addString(sRecipient.c_str());

    bOutput.addList() = bName;

    //get location of objects before
    osRelation.str("");
    osRelation << "SELECT subject, verb, object FROM relation WHERE instance = " << iInstanceAction << " AND verb != 'isAtLoc'";
    bRelBef = requestFromStream(osRelation.str().c_str());

    //clear things
    osRelation.str("");

    //get location of objects after
    osRelation << "SELECT subject, verb, object FROM relation WHERE instance = " << iInstanceAction + 1 << " AND verb != 'isAtLoc'";
    bRelAft = requestFromStream(osRelation.str().c_str());

    bOutput.addList() = bRelBef;
    bOutput.addList() = bRelAft;

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
Bottle abmReasoning::askActionForLevel3Reasoning(int Id)
{
    Bottle bOutput,
        bName,
        bRelationsBefore,
        bRelationsAfter;

    // Get name and argument.
    ostringstream osName;
    osName << "SELECT role, argument FROM contentarg WHERE (role = 'object1' OR role = 'object' OR role = 'recipient' OR role = 'spatial1' OR role = 'spatial' OR role = 'agent' OR role = 'agent1' OR role = 'predicate' OR role = 'action' OR role = 'adv1' OR role = 'adv2') AND instance = " << Id;
    bName = requestFromStream(osName.str());

    if (bName.toString() == abmReasoningFunction::TAG_NULL)
    {
        bOutput.addString(abmReasoningFunction::TAG_NULL);
        return bOutput;
    }

    string sPredicate = bName.check("predicate", Value(abmReasoningFunction::TAG_NULL)).asString();
    string sAgent = bName.check("agent1", Value(abmReasoningFunction::TAG_NULL)).asString();
    string sObject = bName.check("object1", Value(abmReasoningFunction::TAG_NULL)).asString();
    string sRecipient = bName.check("spatial1", Value(abmReasoningFunction::TAG_NULL)).asString();

    if (sAgent == abmReasoningFunction::TAG_NULL){
        sAgent = bName.check("agent", Value(abmReasoningFunction::TAG_NULL)).asString();
    }

    if (sObject == abmReasoningFunction::TAG_NULL){
        sObject = bName.check("object", Value(abmReasoningFunction::TAG_NULL)).asString();
    }

    if (sRecipient == abmReasoningFunction::TAG_NULL){
        sRecipient = bName.check("recipient", Value(abmReasoningFunction::TAG_NULL)).asString();
    }

    if (sRecipient == abmReasoningFunction::TAG_NULL){
        sRecipient = bName.check("spatial", Value(abmReasoningFunction::TAG_NULL)).asString();
    }

    if (sRecipient == abmReasoningFunction::TAG_NULL){
        string sAdv1 = bName.check("adv1", Value(abmReasoningFunction::TAG_NULL)).asString();
        string sAdv2 = bName.check("adv2", Value(abmReasoningFunction::TAG_NULL)).asString();

        // IF ONLY ONE IS PRESENT
        if (sAdv1 == abmReasoningFunction::TAG_NULL && sAdv2 != abmReasoningFunction::TAG_NULL){
            sRecipient = sAdv2;
        }
        if (sAdv2 == abmReasoningFunction::TAG_NULL && sAdv1 != abmReasoningFunction::TAG_NULL){
            sRecipient = sAdv1;
        }


        bool fAD1 = false,
            fAD2 = false,
            isAdv1 = false,
            isAdv2 = false;

        // IF BOTH ARE PRESENTS:
        if (sAdv1 != abmReasoningFunction::TAG_NULL && sAdv2 != abmReasoningFunction::TAG_NULL){
            for (vector<adjKnowledge>::iterator itAK = listKnownAdverb.begin();
                itAK != listKnownAdverb.end();
                itAK++){
                if (!fAD1 && itAK->sLabel == sAdv1){
                    fAD1 = true;
                    if (!(itAK->fTimingInfluence)){
                        isAdv1 = true;
                    }
                }
                if (!fAD2 && itAK->sLabel == sAdv2){
                    fAD2 = true;
                    if (!(itAK->fTimingInfluence)){
                        isAdv2 = true;
                    }
                }
            }
        }

        if (!(isAdv1 && isAdv2)){
            if (isAdv1) sRecipient = sAdv1;
            if (isAdv2) sRecipient = sAdv2;
        }
    }

    if (sPredicate == abmReasoningFunction::TAG_NULL){
        osName.str("");
        osName << "SELECT activityname FROM main WHERE instance = " << Id;
        bName = requestFromStream(osName.str());
        sPredicate = bName.get(0).toString();
    }

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
    bName.addString(sPredicate.c_str());
    bName.addString(sAgent.c_str());
    bName.addString(sObject.c_str());
    bName.addString(sRecipient.c_str());

    bOutput.addList() = bName;
    bOutput.addList() = bRelationsBefore;
    bOutput.addList() = bRelationsAfter;

    if (sPredicate == "hanoi")
    {
        ostringstream osSpatial;
        osSpatial << "SELECT argument,role FROM contentarg WHERE role IN ('spatial1', 'spatial2') AND instance = " << Id;
        Bottle bSpatial = requestFromStream(osSpatial.str());
        bOutput.addList() = bSpatial;
    }

    //get location of objects before
    osRelation.str("");
    osRelation << "SELECT subject, verb, object FROM relation WHERE instance = " << Id;
    bRelationsBefore = requestFromStream(osRelation.str().c_str());


    //clear things
    osRelation.str("");

    //get location of objects after
    osRelation << "SELECT subject, verb, object FROM relation WHERE instance = " << Id + 1;
    bRelationsAfter = requestFromStream(osRelation.str().c_str());


    bOutput.addList() = bRelationsBefore;
    bOutput.addList() = bRelationsAfter;

    return bOutput;
}

/**
* Return the last commplex stored in the ABM
*
*/
Bottle abmReasoning::askLastComplex()
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
Bottle abmReasoning::askComplexFromId(int Id)
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
        yInfo() << "\t" << "in askLastComplex : wrong number of argument for the last complex ( != 9)";
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
        yInfo() << "\t" << "in askLastComplex : Action in complex different of the ones discribed by the arguments";
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
        yInfo() << "\t" << "in askLastComplex : Action in complex different of the ones discribed by the arguments";
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


/**
* Return the last commplex stored in the ABM
*
*/
plan abmReasoning::askLastSharedPlan()
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
plan abmReasoning::askSharedPlanFromId(int opcIdBegin)
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
behavior abmReasoning::askLastBehavior()
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
behavior abmReasoning::askBehaviorFromId(int opcIdBegin)
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
        yInfo() << "\t" << "Error in abmReasoning::askBehaviorFromId | not the same drives at begining and end of a behavior";
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

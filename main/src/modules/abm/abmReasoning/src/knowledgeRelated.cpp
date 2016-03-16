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
#include "wrdac/subsystems/subSystem_ABM.h"

using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


#define THRESHOLD_CONFIDENCE 5.0

Bottle abmReasoning::pddlPlannerSolParser(){
    Bottle bOutput;

    //first file is fine
    /*char filename[512] = "";
    strcpy(filename, plannerPath.c_str()) ;
    strcat(filename, pddlOut.c_str());
    strcat(filename, "_1.SOL") ;
    ifstream fileSol;
    fileSol.open(filename);*/


    //to check with multiple actions solution
    char filename[512] = "";
    for (int i = 1; i <= pddlNb; i++){

        pddlSolFileName(i, filename);
        ifstream fileSol(filename);

        //if the file is found (an not the first) we remove the previous file
        if (fileSol && i > 1){
            char fileSolToRemove[512];
            pddlSolFileName(i - 1, fileSolToRemove);
            if (remove(fileSolToRemove) != 0){
                yInfo() << "\t" << "ERROR : " << fileSolToRemove << " NOT DELETED";
            }
            else {
                yInfo() << "\t" << "File : " << fileSolToRemove << " successfully deleted";
            }
        }

        //the solution file does not exist : the previous solution was the one
        if (!fileSol){
            bestSol = i - 1;
            i = pddlNb;
            yInfo() << "\t" << "----> Best solution is in the solution file number " << bestSol;
        }
    }

    ifstream fileSol;
    //open the bestSol file
    pddlSolFileName(bestSol, filename);
    fileSol.open(filename);

    string sLine = "";

    //open the solution file
    if (fileSol.is_open()){

        unsigned int nbAction = 0;

        while (!fileSol.eof()){
            getline(fileSol, sLine);
            yInfo() << "\t" << sLine;
            int actBegin, actEnd;
            string currentAction;

            actBegin = sLine.find("(");
            actEnd = sLine.find(")");

            //if found the action line
            if (actBegin != -1 && actEnd != -1){

                string actionName = abmReasoningFunction::TAG_DB_NONE;
                string locName = abmReasoningFunction::TAG_DB_NONE;
                string objName = abmReasoningFunction::TAG_DB_NONE;
                string agentName = abmReasoningFunction::TAG_DB_NONE;
                string argName = abmReasoningFunction::TAG_DB_NONE;

                nbAction += 1;
                currentAction = sLine.substr(actBegin + 1, actEnd - actBegin - 1);
                yInfo() << "\t" << "=> Action nb : " << nbAction << " is : " << currentAction;

                string bufferString;
                stringstream ss(currentAction);
                vector<string> vSplitAction;

                while (ss >> bufferString) {
                    vSplitAction.push_back(bufferString);
                }

                unsigned int nbArg = 0;

                for (vector<string>::iterator it = vSplitAction.begin(); it != vSplitAction.end(); it++){
                    nbArg += 1;
                    yInfo() << "\t" << "split : " << it->c_str();

                    ///////////////////////////////////////////////////////////////////////
                    //         CHANGE IT TO PARSE ACCORDING TO THE GENERALIZATION        //
                    ///////////////////////////////////////////////////////////////////////

                    //first split if actionName = action-loc (PUT-NORTH, PUSH-SOUTH-EAST, ...)
                    // hanoi-obj to from
                    if (nbArg == 1) {
                        int sepBegin = it->find_first_of("-");
                        if (sepBegin != -1){
                            actionName = it->substr(0, sepBegin);

                            //upper to lower case
                            transform(actionName.begin(), actionName.end(), actionName.begin(), abmReasoning::fixed_tolower);
                            yInfo() << "\t" << "=> actionName : " << actionName;

                            if (actionName == "hanoi"){
                                //upper to lower case

                                objName = it->substr(sepBegin + 1, it->length());
                                transform(objName.begin(), objName.end(), objName.begin(), abmReasoning::fixed_tolower);
                                yInfo() << "\t" << "=> objName : " << objName;
                            }
                            else {
                                //upper to lower case
                                locName = it->substr(sepBegin + 1, it->length());
                                transform(locName.begin(), locName.end(), locName.begin(), abmReasoning::fixed_tolower);
                                yInfo() << "\t" << "=> locName : " << locName;
                            }

                            agentName = "icub";
                        }
                        else {
                            //ADD, ASK, REMOVE, ...
                            actionName = it->c_str();
                            transform(actionName.begin(), actionName.end(), actionName.begin(), abmReasoning::fixed_tolower);
                            yInfo() << "\t" << "=> actionName : " << actionName;

                            //human has to do these kind of actions
                            agentName = "partner";
                        }
                    }

                    ///////////////////////////////////////////////////////////////////////
                    //         CHANGE IT TO PARSE ACCORDING TO THE GENERALIZATION        //
                    ///////////////////////////////////////////////////////////////////////

                    //Put-north obj   => obj
                    //Hanoi-big loc  => loc
                    else if (nbArg == 2){
                        argName = it->c_str();

                        if (actionName == "hanoi"){
                            //upper to lower case
                            transform(argName.begin(), argName.end(), argName.begin(), abmReasoning::fixed_tolower);
                            locName = argName;
                            yInfo() << "\t" << "=> locName : " << locName;

                        }
                        else {
                            //upper to lower case
                            transform(argName.begin(), argName.end(), argName.begin(), abmReasoning::fixed_tolower);
                            objName = argName;
                            yInfo() << "\t" << "=> objName : " << objName;
                        }
                    }
                }//end go through action split

                //store the action in a bottle
                Bottle bCurrentAction, bArg, bRole;
                bCurrentAction.clear();
                bArg.clear();
                bRole.clear();

                bCurrentAction.addString("action");
                bCurrentAction.addString(actionName.c_str());

                if (agentName != ""){
                    bArg.addString(agentName.c_str());
                    bRole.addString("agent1");
                }
                if (objName != ""){
                    bArg.addString(objName.c_str());
                    bRole.addString("object1");
                }
                if (locName != ""){
                    bArg.addString(locName.c_str());
                    bRole.addString("spatial1");
                }

                bCurrentAction.addList() = bArg;
                bCurrentAction.addList() = bRole;

                yInfo() << "\t" << "actionName == (" << actionName.c_str() << ")";

                //if the action is a put/push -> executeActivity to have the (move absolut/relative (x y) (action put/push (arg...) (role...))) subottle 
                if (actionName == "put" || actionName == "push") {
                    Bottle bForExecute;
                    bForExecute.addString("executeActivity");
                    bForExecute.addString("action");
                    bForExecute.addString(actionName.c_str());
                    bForExecute.addList() = bArg;
                    bForExecute.addList() = bRole;


                    bCurrentAction = *executeActivity(bForExecute).get(0).asList();
                }


                bOutput.addList() = bCurrentAction;

            }//end parse the line

        }//end parse the file

    }
    else {
        yInfo() << "\t" << "ERROR : Solution find (" << filename << ") not found";
        fileSol.close();
        return bOutput;
    }

    fileSol.close();

    //not erase the solution : will be done next time we do a reasoning
    //pddlSolDelete(bestSol, bestSol);

    yInfo() << "\t" << "Bottle of actions : " << bOutput.toString().c_str();

    return bOutput;
}

void abmReasoning::pddlSolFileName(int i, char* filename){
    stringstream ss;
    ss << plannerPath << pddlOut << "_" << i << ".SOL";
    strcpy(filename, ss.str().c_str());
}

//remove all the file given in the range
void abmReasoning::pddlSolDelete(unsigned int begin, unsigned int end){

    //check parameters
    if (begin < 1 || end < begin) {
        yInfo() << "\t" << "ERROR : begin has to be > 1, and end >= begin";
        return;
    }

    char filename[512] = "";
    //loop to generate filename from XXX_begin.SOL to XXX_end.SOL
    for (unsigned int i = begin; i <= end; i++){

        pddlSolFileName(i, filename);
        ifstream fileSol(filename);

        bool isFileExist;

        //check if the file is there
        if (fileSol){
            isFileExist = true;

        }
        else {
            isFileExist = false;
        }

        //close file in order to be able to delete it after
        fileSol.close();

        //if file is there, we delete it
        if (isFileExist) {
            char fileSolToRemove[512];
            pddlSolFileName(i, fileSolToRemove);
            if (remove(fileSolToRemove) != 0){
                yInfo() << "\t" << "ERROR : " << fileSolToRemove << " NOT DELETED";
            }
            else {
                yInfo() << "\t" << "File : " << fileSolToRemove << " successfully deleted";
            }
        }
    }
}

/**
* Compute the PDDL planner launching command according to option specified in the conf file
*/
int abmReasoning::pddlPlannerLauncher(){

    //erase the previous solution if there (so that it is not the first time reasoning is called)

    if (bestSol >= 1){
        char solFilename[512];
        pddlSolFileName(bestSol, solFilename);
        yInfo() << "\t" << "-----> Previous solution file was " << solFilename;
        //remove the file if present
        if (remove(solFilename) != 0){
            yInfo() << "\t" << "===> Previous solution file NOT deleted (" << solFilename << ") : ";
            yInfo() << "\t" << "- Wrong path?";
        }
        else {
            yInfo() << "\t" << "Previous Solution file successfully deleted!";
        }
    }

    //build the command to launch the planner
    char plannerLauncher[1024] = "";
    char cdPlanner[1024] = "";
    char plannerCmds[1024] = "";

    //cdPlanner build
    strcpy(cdPlanner, "cd ");
    strcat(cdPlanner, plannerPath.c_str());

    //plannerLauncher build

    //1. executable
    strcpy(plannerLauncher, plannerExec.c_str());
    strcat(plannerLauncher, " ");

    //2. domain option
    strcat(plannerLauncher, plannerOptDom.c_str());
    strcat(plannerLauncher, " ");
    strcat(plannerLauncher, pddlDomain.c_str());
    strcat(plannerLauncher, " ");

    //3. problem option
    strcat(plannerLauncher, plannerOptProb.c_str());
    strcat(plannerLauncher, " ");
    strcat(plannerLauncher, pddlProblem.c_str());
    strcat(plannerLauncher, " ");

    //4. output option (solution file)
    strcat(plannerLauncher, plannerOptOut.c_str());
    strcat(plannerLauncher, " ");
    strcat(plannerLauncher, pddlOut.c_str());
    strcat(plannerLauncher, " ");

    //5. nb max file produced
    strcat(plannerLauncher, plannerOptNb.c_str());
    strcat(plannerLauncher, " ");
    ostringstream sPddlNb;
    sPddlNb << pddlNb;
    strcat(plannerLauncher, sPddlNb.str().c_str());
    strcat(plannerLauncher, " ");

    //6. nb max cpu time taken (seconds)
    strcat(plannerLauncher, plannerOptCpu.c_str());
    strcat(plannerLauncher, " ");
    ostringstream sPddlCpu;
    sPddlCpu << pddlCpu;
    strcat(plannerLauncher, sPddlCpu.str().c_str());

    yInfo() << "\t" << "executing the command : " << " - " << cdPlanner << " - " << plannerLauncher;

    //grouping both : cd plannerPath + plannerLauncher
    strcpy(plannerCmds, cdPlanner);
    strcat(plannerCmds, " && ");
    strcat(plannerCmds, plannerLauncher);

    //launch both cmds
    yInfo() << "\t" << "System(" << plannerCmds << ")";
    return system(plannerCmds);
}

/**
* Print the Domain PDDL file using contextual knowledge (list of action with preconditions/effects)
*/
Bottle abmReasoning::printPDDLContextualKnowledgeDomain()
{
    Bottle bOutput;

    char buffer[512] = "";
    strcpy(buffer, plannerPath.c_str());
    strcat(buffer, pddlDomain.c_str());
    ofstream myfile;
    yInfo() << "\t" << "Domain PDDL written in : " << buffer;
    myfile.open(buffer, ios_base::ate);


    //print the define, requirements, predicates which will be used
    myfile << ";; STRIPS domain automatically generated by ABMReasoning, part of EFAA";

    myfile << "(define (domain efaa)";

    myfile << "\t(:requirements :strips :typing :equality)";

    myfile << "\t(:predicates";
    myfile << "\t\t(object ?obj)";
    myfile << "\t\t(location ?loc)";
    myfile << "\t\t(isPresent ?obj)";
    //myfile << "\t\t(isReachable ?obj)"   ;
    myfile << "\t\t(isAtLoc ?obj ?loc)";
    myfile << "\t)"; //end parenthesis of predicates

    for (vector<contextualKnowledge>::iterator it = listContextualKnowledge.begin(); it != listContextualKnowledge.end(); it++)
    {

        /**************************  PDDL : action  *****************************/
        //action with no argument (spatial) : remove, add, ... : keep the name
        /**************************  PDDL : action  *****************************/

        string obj1, loc1, loc2;

        if (it->sArgument == abmReasoningFunction::TAG_DB_NONE){
            myfile << "\t(:action " << it->sName;
        }
        else {
            //otherwise the name is name-sArg (e.g. : put-north, put-south, put-near, ...) (in particular to managed put-near)

            //hanoi_loc
            if (it->sName == "hanoi"){
                myfile << "\t(:action " << it->sName << "-" << it->sArgument;

                obj1 = it->sArgument;
                loc1 = "?from";
                loc2 = "?to";
            }
            else {
                //move_obj, put_obj
                myfile << "\t(:action " << it->sName << "-" << it->sArgument;

                obj1 = "?obj1";
            }
        }

        /***********************  PDDL : parameters  **************************/
        //assuming action always are applied to an object1, and if another argument it is a spatial -> in the name
        //       WARNING : this will need to changed for put-near (obj1, obj2)
        /**************************  PDDL : action  *****************************/

        //hanoi_loc
        if (it->sName == "hanoi"){
            myfile << "\t\t:parameters (?to ?from)";
        }
        else {
            myfile << "\t\t:parameters (?obj1)";
        }

        /************************************************************************/
        /***********************  PDDL : precondition  **************************/
        /************************************************************************/

        myfile << "\t\t:precondition (and ";

        //----------------------> for arguments
        if (it->sName == "hanoi"){
            myfile << "(location ?to) (location ?from) ";
        }
        else {
            myfile << "(object ?obj1) ";
        }

        //hanoi_loc : presence <obj, <from,to>> to check
        if (it->sName == "hanoi"){

            //----------------------> for isAtLoc
            //For the others objects : mPercentObjectFromTo
            for (map<string, pair<double, double > >::iterator itIntersect = it->mPercentObjectFromTo.begin(); itIntersect != it->mPercentObjectFromTo.end(); itIntersect++)
            {
                //if percentage isAtLoc > treshold_sup : isAtLoc
                if (itIntersect->second.first > abmReasoningFunction::THRESHOLD_INTERSECT_SUP) {
                    myfile << "(isAtLoc " << itIntersect->first << " ?from) ";
                }

                //if percentage isAtLoc < treshold_inf : not(isAtLoc)
                if (itIntersect->second.first < abmReasoningFunction::THRESHOLD_INTERSECT_INF) {
                    myfile << "(not (isAtLoc " << itIntersect->first << " ?from)) ";
                }

                //if percentage isAtLoc > treshold_sup : isAtLoc
                if (itIntersect->second.second > abmReasoningFunction::THRESHOLD_INTERSECT_SUP) {
                    myfile << "(isAtLoc " << itIntersect->first << " ?to) ";
                }

                //if percentage isAtLoc < treshold_inf : not(isAtLoc)
                if (itIntersect->second.second < abmReasoningFunction::THRESHOLD_INTERSECT_INF) {
                    myfile << "(not (isAtLoc " << itIntersect->first << " ?to)) ";
                }

            }

            //For the moved object : mPercentIntersection
            for (map<string, pair<double, double > >::iterator itIntersect = it->mPercentIntersectLocation.begin(); itIntersect != it->mPercentIntersectLocation.end(); itIntersect++)
            {
                //if loc = from
                if (itIntersect->first == "from" || itIntersect->first == "to") {

                    //second.first = begin = precondition

                    //if percentage isAtLoc > treshold_sup : isAtLoc
                    if (itIntersect->second.first > abmReasoningFunction::THRESHOLD_INTERSECT_SUP) {
                        myfile << "(isAtLoc " << it->sArgument << " ?" << itIntersect->first << ") ";
                    }

                    //if percentage isAtLoc < treshold_inf : not(isAtLoc)
                    if (itIntersect->second.first < abmReasoningFunction::THRESHOLD_INTERSECT_INF) {
                        myfile << "(not (isAtLoc " << it->sArgument << " ?" << itIntersect->first << ")) ";
                    }
                }
            }


        }
        else {

            //----------------------> for isPresent
            if (it->PercentPresence.first > abmReasoningFunction::THRESHOLD_PRESENCE){
                myfile << "(isPresent " << obj1 << ") ";
            }
            else if (it->PercentPresence.first < abmReasoningFunction::THRESHOLD_ABSENCE){
                myfile << "(not (isPresent " << obj1 << ") ) ";
            }

            //----------------------> for isAtLoc
            for (map<string, pair<double, double > >::iterator itIntersect = it->mPercentIntersectLocation.begin(); itIntersect != it->mPercentIntersectLocation.end(); itIntersect++)
            {
                //second: <before,after> => second.first : percent of presence in a loc before

                //if percentage isAtLoc > treshold_sup : isAtLoc
                if (itIntersect->second.first > abmReasoningFunction::THRESHOLD_INTERSECT_SUP) {
                    myfile << "(isAtLoc " << obj1 << " " << itIntersect->first << " ) ";
                }

                //if percentage isAtLoc < treshold_inf : not (isAtLoc)
                if (itIntersect->second.first < abmReasoningFunction::THRESHOLD_INTERSECT_INF) {
                    myfile << "(not (isAtLoc " << obj1 << " " << itIntersect->first << " )) ";
                }
            }
        }

        myfile << ")"; //end parenthesis of precondition

        /************************************************************************/
        /**************************  PDDL : effect  *****************************/
        /************************************************************************/

        //----------------------> for isPresent
        myfile << "\t\t:effect (and ";


        //hanoi_loc : presence <obj, <from,to>> to check
        //EFFECT : looking for the "to"
        if (it->sName == "hanoi"){
            //----------------------> for isAtLoc
            for (map<string, pair<double, double > >::iterator itIntersect = it->mPercentObjectFromTo.begin(); itIntersect != it->mPercentObjectFromTo.end(); itIntersect++)
            {



            }

            //For the moved object : mPercentIntersection
            for (map<string, pair<double, double > >::iterator itIntersect = it->mPercentIntersectLocation.begin(); itIntersect != it->mPercentIntersectLocation.end(); itIntersect++)
            {
                //if loc = from
                if (itIntersect->first == "from" || itIntersect->first == "to") {

                    //second.second = end = effect

                    //if percentage isAtLoc > treshold_sup : isAtLoc
                    if (itIntersect->second.second > abmReasoningFunction::THRESHOLD_INTERSECT_SUP) {
                        myfile << "(isAtLoc " << it->sArgument << " ?" << itIntersect->first << ") ";
                    }

                    //if percentage isAtLoc < treshold_inf : not(isAtLoc)
                    if (itIntersect->second.second < abmReasoningFunction::THRESHOLD_INTERSECT_INF) {
                        myfile << "(not (isAtLoc " << it->sArgument << " ?" << itIntersect->first << ")) ";
                    }
                }
            }

        }
        else {

            if (it->PercentPresence.second > abmReasoningFunction::THRESHOLD_PRESENCE){
                myfile << "(isPresent " << obj1 << ") ";
            }
            else if (it->PercentPresence.second < abmReasoningFunction::THRESHOLD_ABSENCE){
                myfile << "(not (isPresent " << obj1 << ") ) ";
            }

            //----------------------> for isAtLoc
            for (map<string, pair<double, double > >::iterator itIntersect = it->mPercentIntersectLocation.begin(); itIntersect != it->mPercentIntersectLocation.end(); itIntersect++)
            {

                //second: <before,after> => second.second : percent of presence in a loc after

                //if percentage isAtLoc > treshold_sup : isAtLoc
                if (itIntersect->second.second > abmReasoningFunction::THRESHOLD_INTERSECT_SUP) {
                    myfile << "(isAtLoc " << obj1 << " " << itIntersect->first << ") ";
                }

                //if percentage isAtLoc < treshold_inf : not (isAtLoc)
                if (itIntersect->second.second < abmReasoningFunction::THRESHOLD_INTERSECT_INF) {
                    myfile << "(not (isAtLoc " << obj1 << " " << itIntersect->first << ")) ";
                }
            }
        }

        myfile << ")"; //end parenthesis of effect

        myfile << "\t)"; //end parenthesis of current Action
    }

    myfile << ")"; //end parenthesis of domain

    myfile.close();

    bOutput.addString("[ack]");

    return bOutput;
}

/**
* Print the Problem PDDL file using bottle request from user and ABM on the current action
* @param bGoal, bottle of goals (i.e. bottles of OPC relation)
*/
Bottle abmReasoning::printPDDLContextualKnowledgeProblem(Bottle bGoal)
{
    Bottle bOutput, bRequest, bRequestRelation;

    char buffer[512] = "";
    strcpy(buffer, plannerPath.c_str());
    strcat(buffer, pddlProblem.c_str());
    ofstream myfile;
    yInfo() << "\t" << "Problem PDDL written in : " << buffer;
    myfile.open(buffer, ios_base::ate);

    /************************ 1. define problem name, (:domain) *****************************/
    myfile << ";; STRIPS problem automatically generated by ABMReasoning, part of EFAA";

    myfile << "(define (problem efaa-prob)";

    myfile << "\t(:domain efaa)";


    /********************************** 2. (:objects) ***************************************/
    //2.1. get the objects name
    vector <string> vObjName;
    ostringstream osRequest, osRequestRelation;

    myfile << "\t(:objects";
    osRequest.str("");
    osRequest << "SELECT distinct argument FROM contentarg WHERE subtype = '" << EFAA_OPC_ENTITY_RTOBJECT << "' AND instance > 656";
    bRequest = requestFromStream(osRequest.str().c_str());
    myfile << "\t\t";
    for (int arg = 0; arg < bRequest.size(); arg++)
    {
        string objectName;
        objectName = bRequest.get(arg).toString();
        myfile << " " << objectName;
        vObjName.push_back(objectName);
    }
    //end objects name
    myfile << endl;

    //2.2 get the locations name
    vector <string> vLocName;

    osRequest.str("");
    osRequest << "SELECT distinct argument FROM contentarg WHERE (role = 'spatial1' OR role = 'spatial2') AND instance > 656";
    bRequest = requestFromStream(osRequest.str().c_str());
    myfile << "\t\t";
    for (int arg = 0; arg < bRequest.size(); arg++)
    {
        string locName;
        locName = bRequest.get(arg).toString();
        myfile << " " << locName;
        vLocName.push_back(locName);
    }
    //end locations name
    myfile << "\t)\t;; end :objects"; //end objects


    /********************************** 3. (:init) ***************************************/
    myfile << "\t(:init";

    //3.1 ;;types
    myfile << "\t\t;;types";
    myfile << "\t\t";

    //3.1.1 init objects types
    for (vector<string>::iterator it = vObjName.begin(); it != vObjName.end(); it++)
    {
        myfile << "(object " << *(it) << ") ";
    }
    myfile << endl; //end of line for objects types

    //3.1.2 init loc types
    myfile << "\t\t";
    for (vector<string>::iterator it = vLocName.begin(); it != vLocName.end(); it++)
    {
        myfile << "(location " << *(it) << ") ";
    }
    myfile << endl; //end of line for (:init

    //3.2 ;;init-conditions
    myfile << "\t\t;;init-conditions";

    //last instance
    Bottle  bOpcLastInstance = requestFromStream("SELECT instance FROM main WHERE activitytype = 'reasoning' AND begin = TRUE ORDER BY instance DESC LIMIT 1");
    //yInfo() << "\t" << "bottle = " << bOpcLastInstance.toString().c_str() << " and bOPCLastReasoningInstance = " << atoi(bOpcLastInstance.get(0).asList()->get(0).toString().c_str())   ;

    //3.2.1 extract for all rtobjects which ones are present (rtobject
    osRequest.str("");
    osRequest << "SELECT rtobject.name FROM rtobject WHERE rtobject.instance = " << atoi(bOpcLastInstance.get(0).asList()->get(0).toString().c_str()) << " AND rtobject.presence = TRUE";
    bRequest = requestFromStream(osRequest.str().c_str());
    myfile << "\t\t";

    //add relation (obj isAtLoc loc) inside the opc
    Bottle b = updateOpcObjectLocation(abmReasoningFunction::s_realOPC);

    yInfo() << "\t" << "bottle reply update = " << b.toString().c_str();

    for (int arg = 0; arg < bRequest.size(); arg++)
    {
        string objPresentName;
        objPresentName = bRequest.get(arg).toString();
        if (objPresentName != "NULL"){
            myfile << "(isPresent " << objPresentName << ") ";

            //check if the present object intersect a/several loc
            osRequestRelation.str("");

            osRequestRelation << "SELECT relation.object  FROM relation WHERE relation.instance = " << atoi(bOpcLastInstance.get(0).asList()->get(0).toString().c_str()) << " AND relation.verb = 'isAtLoc' AND relation.subject = '" << objPresentName << "' ;";
            bRequestRelation = requestFromStream(osRequestRelation.str().c_str());
            yInfo() << "\t" << "================================================  Relation for " << atoi(bOpcLastInstance.get(0).asList()->get(0).toString().c_str()) << " : " << bRequestRelation.toString().c_str();

            for (int argRelation = 0; argRelation < bRequestRelation.size(); argRelation++)
            {
                if (bRequestRelation.get(argRelation).toString() != "NULL") {
                    myfile << "(isAtLoc " << objPresentName << " " << bRequestRelation.get(argRelation).toString() << ") ";
                }
            }

        }
    }
    myfile << "\t)\t;; end :init"; //end (:init

    //3.2.2 extract for all rtobjects where they are (intersect with loc)
    //add relation (obj isAtLoc loc) inside the opc
    //cf inside isPresent check


    /********************************** 4. (:goal) ***************************************/
    myfile << "\t(:goal";
    myfile << "\t\t(and ";

    //use the input bottle to fill the goal part
    // condition : manner verb subject object -> some could be at abmReasoningFunction::TAG_NONE
    for (int condition = 0; condition < bGoal.size(); condition++)
    {
        unsigned int isPositiveCondition = 1;
        string currentCond;
        for (int condition_part = 0; condition_part < bGoal.get(condition).asList()->size(); condition_part++){

            string currentCondWord = bGoal.get(condition).asList()->get(condition_part).toString();
            //first condition_part = manner = positive or negative condition (none/not)
            if (condition_part == 0) {
                if (currentCondWord == "not") {
                    isPositiveCondition = 0;

                    currentCond += "not (";
                }

                //other condition part (except the first one : manner)
            }
            else if (currentCondWord != abmReasoningFunction::TAG_DB_NONE){
                currentCond += " ";
                currentCond += currentCondWord;
            }
        }

        if (isPositiveCondition == 0) {
            currentCond += ")";
        }

        myfile << "(" << currentCond << ") ";
    }

    myfile << "\t\t)\t;; end and";//end and
    myfile << "\t)\t;; end goal";//end goal
    myfile << ")\t;; end define"; //end define

    myfile.close();

    bOutput.addString("[ack]");

    return bOutput;

}


/*
* Print in a text file the spatial konwledges
*   X   Y   TYPE    NAME
*/
void abmReasoning::printSpatialKnowledge()
{
    string spatial_knowledge_path = resfind.findFileByName("spatial_knowledge.txt");
    ofstream file(spatial_knowledge_path.c_str(), ios::out | ios::trunc);  // erase previous contents of file
    file << "X\tY\ttype\tname";

    for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
    {
        string sNameFile = it->sName;
        sNameFile += "_";
        sNameFile += it->sArgument;
        for (unsigned int i = 0; i < it->vX.size(); i++)
        {
            if (it->isAbsolut)
            {
                file << it->vX[i] << "\t" << it->vY[i] << "\tpoint\t" << sNameFile;
            }
            else if (it->isRelative)
            {
                file << it->vDX[i] << "\t" << it->vDY[i] << "\tvector\t" << sNameFile;
            }
        }
    }
    yInfo() << "\t" << "file " << spatial_knowledge_path << " written";
}

/*
* Check for each action is their is a context linked to the known location
* TODO change
*/
void abmReasoning::checkContextLocation()
{

    //mapLocation.clear();

    //for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
    //{
    //    // Is the spatialKnowledge absolut (put) or relative (push)
    //    if (it->isAbsolut && it->vX.size() >= abmReasoningFunction::THRESHOLD_DETERMINE_INFLUENCE && it->sArgument != "near" && it->sName != "hanoi")
    //    {
    //        pair <vector <double>, vector<double> > vData;
    //        vData.first = it->vX;
    //        vData.second = it->vY;
    //        mapLocation[it->sArgument] = vData;
    //    }
    //}

    //// for kind of action :
    //for (vector<spatialKnowledge>::iterator itSK = listSpatialKnowledge.begin(); itSK != listSpatialKnowledge.end(); itSK++)
    //{
    //    int iNbOccurence = itSK->vX.size();

    //    // search for the CK associated
    //    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin(); itCK != listContextualKnowledge.end(); itCK++)
    //    {
    //        // CK associated found
    //        if (itCK->sArgument == itSK->sArgument && itCK->sName == itSK->sName)
    //        {

    //            // For each move
    //            for (int Oc = 0; Oc < iNbOccurence; Oc++)
    //            {
    //                // get the coordinates
    //                pair<double, double>    BEFORE(itSK->vX[Oc] - itSK->vDX[Oc], itSK->vY[Oc] - itSK->vDY[Oc]),
    //                    AFTER(itSK->vX[Oc], itSK->vY[Oc]);

    //                for (map<string, pair<vector<double>, vector<double> > >::iterator itMAP = mapLocation.begin(); itMAP != mapLocation.end(); itMAP++)
    //                {

    //                    bool before = false,
    //                        after = false;

    //                    // BEFORE :
    //                    if (abmReasoningFunction::getMahalaDist(itMAP->second.first, itMAP->second.second, BEFORE) < abmReasoningFunction::THRESHOLD_IS_AT_LOCATION)
    //                        before = true;

    //                    // AFTER
    //                    if (abmReasoningFunction::getMahalaDist(itMAP->second.first, itMAP->second.second, AFTER) < abmReasoningFunction::THRESHOLD_IS_AT_LOCATION)
    //                        after = true;

    //                    pair < bool, bool > pLoc(before, after);

    //                    itCK->mIntersectLocation[itMAP->first].push_back(pLoc);
    //                }
    //            }
    //        }
    //    }
    //}
}


/**
* Take all the acttivities in the database and build the spatial and time knowledges
*/
Bottle abmReasoning::resetKnowledge(int from)
{

    bDreaming = true;
    DeleteKnownLocations();
    Bottle  bOutput,
        bRequest,
        bMessenger;

    yInfo() << "\t" << "starting to reset knowledge";
    yInfo() << iCub->getABMClient()->resetKnowledge().toString();

    listSpatialKnowledge.clear();
    listTimeKnowledge.clear();
    listBehaviors.clear();
    listPlan.clear();
    listKnownInteraction.clear();
    listKnownAdverb.clear();

    findAllActionsV2(from);
    findAllSharedPlan(from);
    findAllComplex(from);
    findAllBehaviors(from);
    findAllInteractions(from);

    int serialAdjective = sendAdjectiveKnowledge(listKnownAdverb),
        serialSpatial = sendSpatialKnowledge(listSpatialKnowledge),
        serialTime = sendTemporalKnowledge(listTimeKnowledge),
        serialBehavior = sendBehaviors(listBehaviors),
        serialPlan = sendPlan(listPlan),
        serialContext = sendContextual(listContextualKnowledge),
        serialInteraction = sendInteractionKnowledge(listKnownInteraction);


    //  checkContextLocation();

    ostringstream osOutput;
    osOutput << "resetKnowledge : " << serialAdjective << " adjectiveKnowledge, " << serialSpatial << " spatialKnowledge(s) added; " << serialTime << " timeKnowledge(s) added; " << serialBehavior << " behavior(s) added; " << serialPlan << " plan(s) added; " << serialContext << " contextualKnowledge(s) added; " << serialInteraction << " interaction(s) added";
    bOutput.addString(osOutput.str().c_str());
    bDreaming = false;

    string sLocation = updateKnownLocations().toString().c_str();
    bOutput.addString(sLocation.c_str());
    //printSpatialKnowledge();

    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin(); itCK != listContextualKnowledge.end(); itCK++)
    {
        itCK->updateAgentRelated();
    }


    yInfo() << "\t" << osOutput.str().c_str();

    return bOutput;
}

/*
* Take the knowledge in the spatialknowledge and timeknowledge DB
*/
Bottle abmReasoning::getKnowledge()
{
    yInfo() << "\t" << "loading knowledge :";
    Bottle bOutput,
        bAdjectiveKnowledge,
        bError,
        bMainInteraction;
    bError.addString("error");

    listSpatialKnowledge.clear();
    listTimeKnowledge.clear();
    listSharedPlan.clear();
    listPlan.clear();
    listBehaviors.clear();
    listContextualKnowledge.clear();
    listKnownInteraction.clear();
    listKnownAdverb.clear();
    DeleteKnownLocations();

    bAdjectiveKnowledge = requestFromStream("select name, argument, x, y, dx, dy from adjectivespatial");
    //bSpatialKnowledge = requestFromStream("SELECT DISTINCT instance, name, argument, dependance FROM spatialknowledge");

    if (bAdjectiveKnowledge.toString() == abmReasoningFunction::TAG_NULL)
    {
        bError.addString("no adjective data to upload");
    }
    else
    {
        for (int ii = 0; ii < bAdjectiveKnowledge.size(); ii++)
        {

            //yInfo() << "\t" << bAdjectiveKnowledge.get(ii).asList()->toString()  ;
            Bottle bInstance = *bAdjectiveKnowledge.get(ii).asList();

            string sName = bInstance.get(0).toString();
            string sArgument = bInstance.get(1).toString();

            // ADD XY
            if (bInstance.get(2).toString() != "" && bInstance.get(3).toString() != "")
            {
                pair<double, double> pXY((atof(bInstance.get(2).toString().c_str())), (atof(bInstance.get(3).toString().c_str())));
                bool bFound = false;
                for (vector<adjKnowledge>::iterator it = listKnownAdverb.begin(); it != listKnownAdverb.end(); it++)
                {
                    if ((it->sLabel == sName))
                    {
                        it->mActionAbsolut[sArgument].push_back(pXY);
                        it->vdGnlXY.push_back(pXY);
                        bFound = true;
                    }
                }
                if (!bFound)
                {
                    adjKnowledge    newADJ;
                    newADJ.sLabel = sName;
                    newADJ.mActionAbsolut[sArgument].push_back(pXY);
                    newADJ.vdGnlXY.push_back(pXY);
                    listKnownAdverb.push_back(newADJ);
                }
            }

            // ADD DXY
            if (bInstance.get(4).toString() != "" && bInstance.get(5).toString() != "")
            {
                pair<double, double> pXY((atof(bInstance.get(4).toString().c_str())), (atof(bInstance.get(5).toString().c_str())));
                bool bFound = false;
                for (vector<adjKnowledge>::iterator it = listKnownAdverb.begin(); it != listKnownAdverb.end(); it++)
                {
                    if ((it->sLabel == sName))
                    {
                        it->mActionDelta[sArgument].push_back(pXY);
                        it->vdGnlDelta.push_back(pXY);
                        bFound = true;
                    }
                }
                if (!bFound)
                {
                    adjKnowledge    newADJ;
                    newADJ.sLabel = sName;
                    newADJ.mActionDelta[sArgument].push_back(pXY);
                    newADJ.vdGnlDelta.push_back(pXY);
                    listKnownAdverb.push_back(newADJ);
                }
            }
        }
    }


    bAdjectiveKnowledge = requestFromStream("select name, argument, timing from adjectivetemporal");
    //bSpatialKnowledge = requestFromStream("SELECT DISTINCT instance, name, argument, dependance FROM spatialknowledge");


    abmReasoningFunction::TAG_NULL = "NULL";
    if (bAdjectiveKnowledge.toString() == abmReasoningFunction::TAG_NULL)
    {
        bError.addString("no adjective data to upload");
    }
    else
    {
        for (int ii = 0; ii < bAdjectiveKnowledge.size(); ii++)
        {

            //yInfo() << "\t" << bAdjectiveKnowledge.get(ii).asList()->toString();
            Bottle bInstance = *bAdjectiveKnowledge.get(ii).asList();

            string sName = bInstance.get(0).toString();
            string sArgument = bInstance.get(1).toString();

            // ADD timing
            if (bInstance.get(2).toString() != "")
            {
                bool bFound = false;
                for (vector<adjKnowledge>::iterator it = listKnownAdverb.begin(); it != listKnownAdverb.end(); it++)
                {
                    if ((it->sLabel == sName))
                    {
                        it->mActionTiming[sArgument].push_back((atof(bInstance.get(2).toString().c_str())));
                        it->vdGnlTiming.push_back((atof(bInstance.get(2).toString().c_str())));
                        bFound = true;
                    }
                }
                if (!bFound)
                {
                    adjKnowledge    newADJ;
                    newADJ.sLabel = sName;
                    newADJ.mActionTiming[sArgument].push_back((atof(bInstance.get(2).toString().c_str())));
                    newADJ.vdGnlTiming.push_back((atof(bInstance.get(2).toString().c_str())));
                    listKnownAdverb.push_back(newADJ);
                }
            }
        }
    }

    updateKnownLocations();

    yInfo() << "\t" << " ... ";


    Bottle bTimeKnowledge = requestFromStream("SELECT DISTINCT temporal FROM timeknowledge");
    string sResult = bTimeKnowledge.toString();

    if (abmReasoningFunction::TAG_NULL == sResult)
    {
        yInfo() << "\t" << " no temporal data to load";
        bOutput.addString("no temporal data");
    }
    else
    {
        for (int i = 0; i < bTimeKnowledge.size(); i++)
        {
            if (bTimeKnowledge.get(0).isList())
            {
                Bottle bComplex = (*bTimeKnowledge.get(i).asList());
                string sTemporal = bComplex.get(0).toString().c_str();

                timeKnowledge tkComplex;
                tkComplex.sTemporal = sTemporal;

                ostringstream osTimeData;
                osTimeData << "SELECT timearg1, timearg2 FROM timedata WHERE temporal = '" << sTemporal << "'";
                Bottle bTimeData = requestFromStream(osTimeData.str().c_str());

                for (int j = 0; j < bTimeData.size(); j++)
                {
                    tkComplex.timeArg1.push_back(abmReasoningFunction::string2Time(((*bTimeData.get(j).asList()).get(0).toString()).c_str()));
                    tkComplex.timeArg2.push_back(abmReasoningFunction::string2Time(((*bTimeData.get(j).asList()).get(1).toString()).c_str()));
                }
                listTimeKnowledge.push_back(tkComplex);
            }
        }
    }


    yInfo() << "\t" << " ... ";

    Bottle  bMainBehavior = requestFromStream("SELECT DISTINCT instance, name, argument FROM behavior");
    sResult = bMainBehavior.toString();
    ostringstream osBehavior;
    if (abmReasoningFunction::TAG_NULL == sResult)
    {
        yInfo() << "\t" << "no behavior data to load";
        bOutput.addString("no behavior data");
    }
    else
    {
        for (int i = 0; i < bMainBehavior.size(); i++)
        {
            if (bMainBehavior.get(i).isList())
            {
                Bottle bAction = (*bMainBehavior.get(i).asList());
                int instance = atoi(bAction.get(0).toString().c_str());
                string sName = bAction.get(1).toString().c_str();
                string sArg = bAction.get(2).toString().c_str();
                behavior newBehavior;
                newBehavior.sArgument = sArg;
                newBehavior.sName = sName;
                osBehavior.str("");
                osBehavior << "SELECT DISTINCT occurence FROM behaviordata WHERE instance = " << instance;
                Bottle bOccurence = requestFromStream(osBehavior.str().c_str());
                vector<int>  vOccurence;

                for (int oc = 0; oc < bOccurence.size(); oc++)
                {
                    vOccurence.push_back(atoi(bOccurence.get(oc).asList()->get(0).toString().c_str()));
                }

                for (vector<int>::iterator it_occu = vOccurence.begin(); it_occu != vOccurence.end(); it_occu++)
                {

                    osBehavior.str("");
                    osBehavior << "SELECT drive, effect FROM behaviordata WHERE instance = " << instance << " AND occurence = " << *it_occu;
                    Bottle bBehaviorData = requestFromStream(osBehavior.str().c_str());

                    vector< pair <string, double> >         vEffect;
                    for (int j = 0; j < bBehaviorData.size(); j++)
                    {
                        // name of the drive, value
                        pair<string, double>  pTemp((bBehaviorData.get(j).asList())->get(0).toString(), (atof((*bBehaviorData.get(j).asList()).get(1).toString().c_str())));

                        vEffect.push_back(pTemp);
                    }
                    newBehavior.vEffect.push_back(vEffect);
                }
                listBehaviors.push_back(newBehavior);
            }
        }
    }

    Bottle bContextualKnowledge = requestFromStream("SELECT DISTINCT instance, name, argument, dependance FROM contextknowledge");
    sResult = bContextualKnowledge.toString();

    if (abmReasoningFunction::TAG_NULL == sResult)
    {
        yInfo() << "\t" << " no contextual data to load";
        bOutput.addString("no contextual data");
    }
    else
    {
        for (int i = 0; i < bContextualKnowledge.size(); i++)
        {
            contextualKnowledge ckAction;
            if (bContextualKnowledge.get(0).isList())
            {
                Bottle bAction = (*bContextualKnowledge.get(i).asList());
                int instance = atoi(bAction.get(0).toString().c_str());
                string sName = bAction.get(1).toString().c_str();
                string sArg = bAction.get(2).toString().c_str();
                string sDependance = bAction.get(3).toString().c_str();

                ckAction.sArgument = sArg;
                ckAction.sName = sName;
                ckAction.sDependance = sDependance;

                //Presence
                ostringstream osContextKnowled;
                osContextKnowled.str("");
                osContextKnowled << "SELECT presencebegin, presenceend FROM contextdata WHERE instance = " << instance;
                Bottle bContextualData = requestFromStream(osContextKnowled.str().c_str());

                for (int j = 0; j < bContextualData.size(); j++)
                {
                    string before = (*bContextualData.get(j).asList()).get(0).toString().c_str(),
                        after = (*bContextualData.get(j).asList()).get(1).toString().c_str();
                    pair <bool, bool > pTemp(before == "t", after == "t");
                    ckAction.vObjectPresent.push_back(pTemp);
                }

                //Loc
            }
            ckAction.updatePresence();
            listContextualKnowledge.push_back(ckAction);
        }
    }


    yInfo() << "\t" << " ... ";

    Bottle  bMainPlan = requestFromStream("SELECT DISTINCT instance, name, manner FROM sharedplan");
    sResult = bMainPlan.toString();
    osBehavior.str("");;
    if (abmReasoningFunction::TAG_NULL == sResult)
    {
        yInfo() << "\t" << "no sharedplan data to load";
        bOutput.addString("no sharedplan data");
    }
    else
    {
        for (int i = 0; i < bMainPlan.size(); i++)
        {
            plan NewPlan;

            // for each plan
            if (bMainPlan.get(i).isList())
            {
                int instance = atoi(bMainPlan.get(i).asList()->get(0).toString().c_str());
                NewPlan.sName = bMainPlan.get(i).asList()->get(1).toString();
                NewPlan.sManner = bMainPlan.get(i).asList()->get(2).toString();

                ostringstream osPlanArgument;
                osPlanArgument << "SELECT argument, role FROM sharedplanarg WHERE instance = " << instance;
                Bottle bRequest = requestFromStream(osPlanArgument.str().c_str());

                // get the argument of the plan
                for (int arg = 0; arg < bRequest.size(); arg++)
                {
                    pair<string, string> pArgument
                        (bRequest.get(arg).asList()->get(0).toString(),
                        bRequest.get(arg).asList()->get(1).toString());

                    NewPlan.vArguments.push_back(pArgument);
                }

                ostringstream osPlanAct,
                    osActArg;
                osPlanAct << "SELECT activitytype, activityname, id FROM sharedplandata WHERE instance = " << instance << "  ORDER BY id ";
                bRequest = requestFromStream(osPlanAct.str().c_str());

                // get the activities of the plan
                for (int act = 0; act < bRequest.size(); act++)
                {
                    NewPlan.vActivitytype.push_back(bRequest.get(act).asList()->get(0).toString().c_str());
                    NewPlan.vActivityname.push_back(bRequest.get(act).asList()->get(1).toString().c_str());
                    osActArg.str("");
                    osActArg << "SELECT argument, role FROM spdataarg WHERE instance = " << instance << " AND id = " << atoi(bRequest.get(act).asList()->get(2).toString().c_str());
                    Bottle bArgAct = requestFromStream(osActArg.str().c_str());
                    list <pair <string, string> > lArgAct;

                    // get the argument of the current activity
                    for (int arg = 0; arg < bArgAct.size(); arg++)
                    {
                        pair<string, string> pArgRole;
                        pArgRole.first = bArgAct.get(arg).asList()->get(0).toString();
                        pArgRole.second = bArgAct.get(arg).asList()->get(1).toString();
                        lArgAct.push_back(pArgRole);
                    }
                    NewPlan.vActivityArguments.push_back(lArgAct);
                } // out of the current activity

            }// out of the current plan

            NewPlan = addPlan(NewPlan);
        }
    }


    yInfo() << "\t" << " ...";

    bMainInteraction = requestFromStream("SELECT DISTINCT subject FROM interactionknowledge");
    sResult = bMainInteraction.toString();
    osBehavior.str("");
    if (abmReasoningFunction::TAG_NULL == sResult)
    {
        yInfo() << "\t" << "no interaction data to load";
        bOutput.addString("no interaction data");
    }
    else
    {
        for (int i = 0; i < bMainInteraction.size(); i++)
        {
            // for each subject
            if (bMainInteraction.get(i).isList())
            {
                string sSubject = bMainInteraction.get(i).asList()->get(0).toString();

                ostringstream osInteraction;
                osInteraction << "SELECT * FROM interactionknowledge WHERE subject = '" << sSubject << "'";
                Bottle bInteraction = requestFromStream(osInteraction.str().c_str());
                knownInteraction TempInt;
                TempInt.sSubject = sSubject;

                // get the argument of the plan
                for (int arg = 0; arg < bInteraction.size(); arg++)
                {
                    tuple<string, int, string, string> tInteraction;
                    get<0>(tInteraction) = bInteraction.get(arg).asList()->get(1).toString();
                    get<1>(tInteraction) = atoi(bInteraction.get(arg).asList()->get(2).toString().c_str());
                    get<2>(tInteraction) = bInteraction.get(arg).asList()->get(3).toString();
                    get<3>(tInteraction) = bInteraction.get(arg).asList()->get(4).toString();

                    TempInt.addInteraction(tInteraction);
                }
                listKnownInteraction.push_back(TempInt);
            }
        }
    }



    checkContextLocation();
    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin(); itCK != listContextualKnowledge.end(); itCK++)
    {
        itCK->updatePresence();
    }


    yInfo() << "\t" << " done ! ";
    yInfo() << "\t" << listSpatialKnowledge.size() << " spatialKnowledge(s) - " << listTimeKnowledge.size() << " temporalKnowledge(s) - " << listBehaviors.size() << " behavior(s) - " << listPlan.size() << " sharedplan(s) - " << listContextualKnowledge.size() << " contextualKnowledge(s) - " << listKnownInteraction.size() << " knownInteraction(s).";;

    bOutput.addString("knowledge added");

    return bOutput;
}

/*
* Search for the spatialKnowledge absolut and store them in the OPC
*
*/
Bottle abmReasoning::updateKnownLocations()
{
    Bottle bOutput;
    if ((realOPC->isConnected() || mentalOPC->isConnected()) && bPopulateOPC)
    {
        int iAdded = 0;

        for (vector<adjKnowledge>::iterator itAdv = listKnownAdverb.begin(); itAdv != listKnownAdverb.end(); itAdv++)
        {
            determineTimingInfluence(*itAdv, true);
            if (!itAdv->fTimingInfluence)
            {
                for (map<string, vector< pair<double, double > > >::iterator itSubAdv = itAdv->mActionAbsolut.begin();
                    itSubAdv != itAdv->mActionAbsolut.end();
                    itSubAdv++)
                {
                    Bottle bEffect = itAdv->getEffect(itSubAdv->first, true);
                    if ((bEffect.get(0).toString()) == "absolute" && itSubAdv->first != "hanoi")
                    {
                        vector <pair <double, double> > vXY = itSubAdv->second;
                        pair<double, double> vData(bEffect.get(1).asDouble(),
                            bEffect.get(2).asDouble());


                        vector<double> CovMat = abmReasoningFunction::getCovMatrix(vXY);
                        double a = CovMat[0],
                            b = CovMat[1],
                            c = CovMat[2],
                            d = CovMat[3];

                        double VP1, VP2;
                        pair<double, double > Vect1, Vect2;

                        if (a < d)
                        {
                            VP1 = .5 * (a + d + sqrt((a - d)*(a - d) + 4 * b*c));
                            VP2 = .5 * (a + d - sqrt((a - d)*(a - d) + 4 * b*c));
                        }
                        else
                        {
                            VP1 = .5 * (a + d - sqrt((a - d)*(a - d) + 4 * b*c));
                            VP2 = .5 * (a + d + sqrt((a - d)*(a - d) + 4 * b*c));
                        }

                        Vect1.first = c;
                        Vect1.second = VP1 - a;

                        Vect2.first = c;
                        Vect2.second = VP2 - a;


                        string sLoc = "loc_";
                        sLoc += itAdv->sLabel;
                        string large = "large_";
                        large += itAdv->sLabel;

                        int colorR = abmReasoningFunction::color_loc_R,
                            colorG = abmReasoningFunction::color_loc_G,
                            colorB = abmReasoningFunction::color_loc_B;

                        if (itSubAdv->first == "move")
                        {
                            colorR = 238;
                            colorG = 221;
                            colorB = 130;
                        }

                        if (realOPC->isConnected())
                        {
                            Object* LOCATION = realOPC->addOrRetrieveEntity<Object>(sLoc);
                            LOCATION->m_ego_position[0] = vData.first;
                            LOCATION->m_ego_position[1] = vData.second;
                            LOCATION->m_ego_position[2] = abmReasoningFunction::height_location;
                            LOCATION->m_dimensions[0] = sqrt(VP2)*abmReasoningFunction::FACTOR_LOCATION;
                            LOCATION->m_dimensions[1] = sqrt(VP1)*abmReasoningFunction::FACTOR_LOCATION;
                            LOCATION->m_dimensions[2] = abmReasoningFunction::size_location * 2;
                            LOCATION->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180. / PI;
                            LOCATION->m_present = 1.0;
                            LOCATION->m_color[0] = colorR / 2;
                            LOCATION->m_color[1] = colorG / 2;
                            LOCATION->m_color[2] = colorB / 2;

                            Object* LOCATION_LARGE = realOPC->addOrRetrieveEntity<Object>(large);
                            LOCATION_LARGE->m_ego_position[0] = vData.first;
                            LOCATION_LARGE->m_ego_position[1] = vData.second;
                            LOCATION_LARGE->m_ego_position[2] = abmReasoningFunction::height_location;
                            LOCATION_LARGE->m_dimensions[0] = sqrt(VP2) * 2 * abmReasoningFunction::FACTOR_LOCATION;
                            LOCATION_LARGE->m_dimensions[1] = sqrt(VP1) * 2 * abmReasoningFunction::FACTOR_LOCATION;
                            LOCATION_LARGE->m_dimensions[2] = abmReasoningFunction::size_location;
                            LOCATION_LARGE->m_present = 1.0;
                            LOCATION_LARGE->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180. / PI;
                            LOCATION_LARGE->m_color[0] = colorR;
                            LOCATION_LARGE->m_color[1] = colorG;
                            LOCATION_LARGE->m_color[2] = colorB;

                            iAdded++;
                            LOCATION->m_present = 1.0;
                            LOCATION_LARGE->m_present = 0.0;
                            realOPC->commit();
                        }


                        if (mentalOPC->isConnected())
                        {
                            Object* LOCATION = mentalOPC->addOrRetrieveEntity<Object>(sLoc);
                            LOCATION->m_ego_position[0] = vData.first;
                            LOCATION->m_ego_position[1] = vData.second;
                            LOCATION->m_ego_position[2] = abmReasoningFunction::height_location;
                            LOCATION->m_dimensions[0] = sqrt(VP2)*abmReasoningFunction::FACTOR_LOCATION;
                            LOCATION->m_dimensions[1] = sqrt(VP1)*abmReasoningFunction::FACTOR_LOCATION;
                            LOCATION->m_dimensions[2] = abmReasoningFunction::size_location * 2;
                            LOCATION->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180. / PI;
                            LOCATION->m_present = 1.0;
                            LOCATION->m_color[0] = colorR / 2;
                            LOCATION->m_color[1] = colorG / 2;
                            LOCATION->m_color[2] = colorB / 2;

                            Object* LOCATION_LARGE = mentalOPC->addOrRetrieveEntity<Object>(large);
                            LOCATION_LARGE->m_ego_position[0] = vData.first;
                            LOCATION_LARGE->m_ego_position[1] = vData.second;
                            LOCATION_LARGE->m_ego_position[2] = abmReasoningFunction::height_location;
                            LOCATION_LARGE->m_dimensions[0] = sqrt(VP2) * 2 * abmReasoningFunction::FACTOR_LOCATION;
                            LOCATION_LARGE->m_dimensions[1] = sqrt(VP1) * 2 * abmReasoningFunction::FACTOR_LOCATION;
                            LOCATION_LARGE->m_dimensions[2] = abmReasoningFunction::size_location;
                            LOCATION_LARGE->m_present = 1.0;
                            LOCATION_LARGE->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180. / PI;
                            LOCATION_LARGE->m_color[0] = colorR;
                            LOCATION_LARGE->m_color[1] = colorG;
                            LOCATION_LARGE->m_color[2] = colorB;

                            iAdded++;
                            LOCATION->m_present = 1.0;
                            LOCATION_LARGE->m_present = 0.0;
                        }
                    }
                }
            }
        }

        ostringstream osReturn;
        osReturn << iAdded << " locations set in the OPC.";
        mentalOPC->commit();
        realOPC->commit();
        bOutput.addString(osReturn.str().c_str());
    }
    yInfo() << "\t" << "update locations : OPC not connected";
    bOutput.addString("update locations : OPC not connected");

    return bOutput;
}


Bottle abmReasoning::updateLocation(string sLocation)
{
    Bottle bOutput;
    if ((realOPC->isConnected() || mentalOPC->isConnected()) && bPopulateOPC)
    {
        int iAdded = 0;
        for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
        {
            // Is the spatialKnowledge absolut (put) or relative (push)
            if (it->isAbsolut && it->sArgument == sLocation  && it->vX.size() >= abmReasoningFunction::THRESHOLD_DETERMINE_INFLUENCE && it->sArgument != "near" && it->sName != "hanoi")
            {
                vector<double> CovMat = abmReasoningFunction::getCovMatrix(it->vX, it->vY);
                double a = CovMat[0],
                    b = CovMat[1],
                    c = CovMat[2],
                    d = CovMat[3];


                double VP1, VP2;

                if (a < d)
                {
                    VP1 = .5 * (a + d + sqrt((a - d)*(a - d) + 4 * b*c));
                    VP2 = .5 * (a + d - sqrt((a - d)*(a - d) + 4 * b*c));
                }
                else
                {
                    VP1 = .5 * (a + d - sqrt((a - d)*(a - d) + 4 * b*c));
                    VP2 = .5 * (a + d + sqrt((a - d)*(a - d) + 4 * b*c));
                }

                pair<double, double > Vect1(c, VP1 - a),
                    Vect2(c, VP2 - a);

                vector<double>  vData = it->determineAbsolut();
                string sLoc = "loc_";
                sLoc += it->sArgument;
                string large = "large_";
                large += it->sArgument;

                int colorR = abmReasoningFunction::color_loc_R,
                    colorG = abmReasoningFunction::color_loc_G,
                    colorB = abmReasoningFunction::color_loc_B;

                if (it->sName == "move")
                {
                    colorR = 238;
                    colorG = 221;
                    colorB = 130;
                }

                if (realOPC->isConnected())
                {
                    Object* LOCATION = realOPC->addOrRetrieveEntity<Object>(sLoc);
                    LOCATION->m_ego_position[0] = vData[0];
                    LOCATION->m_ego_position[1] = vData[1];
                    LOCATION->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION->m_dimensions[0] = sqrt(VP2)*abmReasoningFunction::FACTOR_LOCATION;
                    LOCATION->m_dimensions[1] = sqrt(VP1)*abmReasoningFunction::FACTOR_LOCATION;
                    LOCATION->m_dimensions[2] = 0.04;
                    LOCATION->m_present = 1.0;
                    LOCATION->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180. / PI;
                    LOCATION->m_color[0] = colorR / 2;
                    LOCATION->m_color[1] = colorG / 2;
                    LOCATION->m_color[2] = colorB / 2;

                    Object* LOCATION_LARGE = realOPC->addOrRetrieveEntity<Object>(large);
                    LOCATION_LARGE->m_ego_position[0] = vData[0];
                    LOCATION_LARGE->m_ego_position[1] = vData[1];
                    LOCATION_LARGE->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION_LARGE->m_dimensions[0] = sqrt(VP2) * 2 * abmReasoningFunction::FACTOR_LOCATION;
                    LOCATION_LARGE->m_dimensions[1] = sqrt(VP1) * 2 * abmReasoningFunction::FACTOR_LOCATION;
                    LOCATION_LARGE->m_dimensions[2] = 0.02;
                    LOCATION_LARGE->m_present = 1.0;
                    LOCATION_LARGE->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180. / PI;
                    LOCATION_LARGE->m_color[0] = colorR;
                    LOCATION_LARGE->m_color[1] = colorG;
                    LOCATION_LARGE->m_color[2] = colorB;

                    ostringstream osReturn;
                    osReturn << iAdded << " locations set in the OPC.";
                    realOPC->commit();
                    bOutput.addString(osReturn.str().c_str());
                }

                if (mentalOPC->isConnected())
                {
                    Object* LOCATION = mentalOPC->addOrRetrieveEntity<Object>(sLoc);
                    LOCATION->m_ego_position[0] = vData[0];
                    LOCATION->m_ego_position[1] = vData[1];
                    LOCATION->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION->m_dimensions[0] = sqrt(VP2)*abmReasoningFunction::FACTOR_LOCATION;
                    LOCATION->m_dimensions[1] = sqrt(VP1)*abmReasoningFunction::FACTOR_LOCATION;
                    LOCATION->m_dimensions[2] = 0.04;
                    LOCATION->m_present = 1.0;
                    LOCATION->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180. / PI;
                    LOCATION->m_color[0] = abmReasoningFunction::color_loc_R / 2;
                    LOCATION->m_color[1] = abmReasoningFunction::color_loc_G / 2;
                    LOCATION->m_color[2] = abmReasoningFunction::color_loc_B / 2;

                    Object* LOCATION_LARGE = mentalOPC->addOrRetrieveEntity<Object>(large);
                    LOCATION_LARGE->m_ego_position[0] = vData[0];
                    LOCATION_LARGE->m_ego_position[1] = vData[1];
                    LOCATION_LARGE->m_ego_position[2] = abmReasoningFunction::height_location;
                    LOCATION_LARGE->m_dimensions[0] = sqrt(VP2) * 2 * abmReasoningFunction::FACTOR_LOCATION;
                    LOCATION_LARGE->m_dimensions[1] = sqrt(VP1) * 2 * abmReasoningFunction::FACTOR_LOCATION;
                    LOCATION_LARGE->m_dimensions[2] = 0.02;
                    LOCATION_LARGE->m_present = 1.0;
                    LOCATION_LARGE->m_ego_orientation[2] = sin(Vect2.second / Vect2.first)*180. / PI;
                    LOCATION_LARGE->m_color[0] = colorR;
                    LOCATION_LARGE->m_color[1] = colorG;
                    LOCATION_LARGE->m_color[2] = colorB;

                    ostringstream osReturn;
                    osReturn << iAdded << " locations set in the OPC.";
                    mentalOPC->commit();
                    bOutput.addString(osReturn.str().c_str());
                }

                return bOutput;

            }
        }

        //yInfo() << "\t" << "location unknown"  ;
        bOutput.addString("location unknown");
        return bOutput;
    }
    //yInfo() << "\t" << "update locations : OPC not connected"  ;
    bOutput.addString("update locations : OPC not connected");

    return bOutput;
}

/*
* Delete from the OPC the location (presence = 0)
*
*/
Bottle abmReasoning::DeleteKnownLocations()
{
    Bottle bOutput;
    if (realOPC->isConnected() || mentalOPC->isConnected())
    {
        for (vector<spatialKnowledge>::iterator it = listSpatialKnowledge.begin(); it != listSpatialKnowledge.end(); it++)
        {
            // Is the spatialKnowledge absolut (put) or relative (push)
            if (it->isAbsolut)
            {
                string sLoc = "loc_";
                sLoc += it->sArgument;
                string large = "large_";
                large += it->sArgument;

                if (realOPC->isConnected())
                {
                    Object* LOCATION = realOPC->addOrRetrieveEntity<Object>(sLoc);
                    LOCATION->m_present = 0.0;

                    Object* LOCATION_LARGE = realOPC->addOrRetrieveEntity<Object>(large);
                    LOCATION_LARGE->m_present = 0.0;
                }

                if (mentalOPC->isConnected())
                {
                    Object* LOCATION = mentalOPC->addOrRetrieveEntity<Object>(sLoc);
                    LOCATION->m_present = 0.0;

                    Object* LOCATION_LARGE = mentalOPC->addOrRetrieveEntity<Object>(large);
                    LOCATION_LARGE->m_present = 0.0;
                }
            }
        }

        realOPC->commit();
        mentalOPC->commit();

        return bOutput;
    }

    //  yInfo() << "\t" << "update locations : OPC not connected"  ;
    bOutput.addString("update locations : OPC not connected");

    return bOutput;
}


/*
*   Save the knowledge in the semantical memory
*/
Bottle abmReasoning::saveKnowledge()
{
    Bottle bOutput = saveKnowledge(listSpatialKnowledge, listTimeKnowledge, listBehaviors, listPlan, listContextualKnowledge, listKnownInteraction);

    printSpatialKnowledge();

    return bOutput;
}


/*
*   Include in the OPC at a given time, the relation between the Object and the relations
*
*/
Bottle abmReasoning::updateOpcObjectLocation(string sOPCname)
{
    Bottle bOutput;
    bool bReal = sOPCname == "OPC";

    if (bReal)
        realOPC->checkout();
    else
        mentalOPC->checkout();

    // Set the Bottles for queryOPC to the 

    Bottle isRtobject, condition, present;
    isRtobject.addString(EFAA_OPC_ENTITY_TAG);
    isRtobject.addString("==");
    isRtobject.addString(EFAA_OPC_ENTITY_RTOBJECT);

    Bottle isPresent;
    isPresent.addString(EFAA_OPC_OBJECT_PRESENT_TAG);
    isPresent.addString("==");
    isPresent.addInt(1);

    condition.addList() = isRtobject;
    condition.addString("&&");
    condition.addList() = isPresent;

    list<Entity*> PresentEntities;
    if (!bReal)
        PresentEntities = (mentalOPC->Entities(condition));
    else
        PresentEntities = (realOPC->Entities(condition));

    // update known location
    mapLocation.clear();
    mapTemporalLocation.clear();


    for (vector<adjKnowledge>::iterator itAdv = listKnownAdverb.begin(); itAdv != listKnownAdverb.end(); itAdv++)
    {
        determineTimingInfluence(*itAdv);
        if (!itAdv->fTimingInfluence)
        {
            for (map<string, vector< pair<double, double > > >::iterator itSubAdv = itAdv->mActionAbsolut.begin();
                itSubAdv != itAdv->mActionAbsolut.end();
                itSubAdv++)
            {
                if ((itAdv->getEffect(itSubAdv->first).get(0).toString()) == "absolute" && itSubAdv->first != "hanoi")
                {
                    vector <pair <double, double> > vData = itSubAdv->second;
                    mapLocation[itAdv->sLabel] = vData;
                }
            }
        }

        // relative + dependance (near, above...)
        //if (itAdv->isRelative && it->vX.size() > 2 && it->sDependance != abmReasoningFunction::TAG_DB_NONE)
        //{
        //    for (list<Entity*>::iterator it_E = PresentEntities.begin(); it_E != PresentEntities.end(); it_E++)
        //    {
        //        if ((*it_E)->entity_type() == it->sDependance)
        //        {

        //            pair<double, double>    pObjDep;        // coordinate of the dependent object

        //            if (it->sDependance == EFAA_OPC_ENTITY_RTOBJECT)
        //            {
        //                RTObject *RTOTemp;
        //                if (!bReal)
        //                    RTOTemp = (mentalOPC->addEntity<RTObject>((*it_E)->name()));
        //                else
        //                    RTOTemp = (realOPC->addEntity<RTObject>((*it_E)->name()));

        //                pObjDep = pair<double, double>(RTOTemp->m_ego_position[0], RTOTemp->m_ego_position[1]);

        //            }

        //            if (it->sDependance == EFAA_OPC_ENTITY_OBJECT)
        //            {
        //                Object *OTemp;
        //                if (!bReal)
        //                    OTemp = (mentalOPC->addEntity<Object>((*it_E)->name()));
        //                else
        //                    OTemp = (realOPC->addEntity<Object>((*it_E)->name()));

        //                pObjDep = pair<double, double>(OTemp->m_ego_position[0], OTemp->m_ego_position[1]);

        //            }

        //            if (it->sDependance == EFAA_OPC_ENTITY_AGENT)
        //            {
        //                Agent *ATemp;
        //                if (!bReal)
        //                    ATemp = (mentalOPC->addEntity<Agent>((*it_E)->name()));
        //                else
        //                    ATemp = (realOPC->addEntity<Agent>((*it_E)->name()));

        //                pObjDep = pair<double, double>(ATemp->m_ego_position[0], ATemp->m_ego_position[1]);

        //            }

        //            vector<double> vXTemp = it->vDX,
        //                vYTemp = it->vDY;

        //            for (unsigned int point = 0; point < vXTemp.size(); point++)
        //            {
        //                vXTemp[point] = pObjDep.first - vXTemp[point];
        //                vYTemp[point] = pObjDep.second - vYTemp[point];
        //            }

        //            tuple<string, vector<double>, vector<double> > mapTemp((*it_E)->name(), vXTemp, vYTemp);
        //            string sTemporalLocationName = it->sArgument;
        //            sTemporalLocationName += "_" + (*it_E)->name();
        //            mapTemporalLocation[sTemporalLocationName] = mapTemp;


        //           // vector<double> CovMat = abmReasoningFunction::getCovMatrix(vXTemp, vYTemp);

        //        }
        //    }
        //}
    }

    // for each object present
    for (list<Entity*>::iterator it_E = PresentEntities.begin(); it_E != PresentEntities.end(); it_E++)
    {

        // get all object
        RTObject *RTTemp = dynamic_cast<RTObject*>(*it_E);

        //detect if they are is a location
        for (map<string, vector<pair <double, double > > >::iterator itMAP = mapLocation.begin(); itMAP != mapLocation.end(); itMAP++)
        {
            pair <double, double > pLoc(RTTemp->m_ego_position[0], RTTemp->m_ego_position[1]);

            if (abmReasoningFunction::getMahalaDist(itMAP->second, pLoc) < abmReasoningFunction::THRESHOLD_IS_AT_LOCATION)
            {
                Adjective* presence;
                if (bReal)
                {
                    presence = realOPC->addOrRetrieveEntity<Adjective>(itMAP->first.c_str());
                    presence->m_quality = (abmReasoningFunction::TAG_LOCATION);
                    Action* is = realOPC->addOrRetrieveEntity<Action>(abmReasoningFunction::TAG_IS_AT_LOC);
                    realOPC->commit();
                    realOPC->addRelation(*it_E, is, presence, 2 * abmReasoningFunction::LIFETIME_RELATION);
                }
                else
                {

                    presence = mentalOPC->addOrRetrieveEntity<Adjective>(itMAP->first.c_str());
                    presence->m_quality = (abmReasoningFunction::TAG_LOCATION);
                    Action* is = mentalOPC->addOrRetrieveEntity<Action>(abmReasoningFunction::TAG_IS_AT_LOC);
                    mentalOPC->commit();
                    mentalOPC->addRelation(*it_E, is, presence, 2 * abmReasoningFunction::LIFETIME_RELATION);
                }

            }
        }

        // temporal location (dependance)
        for (map<string, tuple<string, vector<double>, vector<double> > >::iterator itMAP = mapTemporalLocation.begin(); itMAP != mapTemporalLocation.end(); itMAP++)
        {
            if (RTTemp->name() != get<0>(itMAP->second))
            {
                pair <double, double > pLoc(RTTemp->m_ego_position[0], RTTemp->m_ego_position[1]);

                if (abmReasoningFunction::getMahalaDist(get<1>(itMAP->second), get<2>(itMAP->second), pLoc) < abmReasoningFunction::THRESHOLD_IS_AT_TEMPORAL_LOCATION)
                {
                    Adjective* presence;
                    if (bReal)
                    {
                        presence = realOPC->addOrRetrieveEntity<Adjective>(itMAP->first.c_str());
                        presence->m_quality = (abmReasoningFunction::TAG_LOCATION);
                        Action* is = realOPC->addOrRetrieveEntity<Action>(abmReasoningFunction::TAG_IS_AT_LOC);
                        realOPC->commit();
                        realOPC->addRelation(*it_E, is, presence, 2 * abmReasoningFunction::LIFETIME_RELATION);
                    }
                    else
                    {
                        presence = mentalOPC->addOrRetrieveEntity<Adjective>(itMAP->first.c_str());
                        presence->m_quality = (abmReasoningFunction::TAG_LOCATION);
                        Action* is = mentalOPC->addOrRetrieveEntity<Action>(abmReasoningFunction::TAG_IS_AT_LOC);
                        mentalOPC->commit();
                        mentalOPC->addRelation(*it_E, is, presence, 2 * abmReasoningFunction::LIFETIME_RELATION);
                    }
                }
            }
        }
    }

    if (bReal)
    {
        realOPC->commit();
    }
    else
    {
        mentalOPC->commit();
    }

    bOutput.addString("Relation added in the OPC");
    bOutput.addString("Beliefs updated");

    return bOutput;
}


/*
* Return a list of information about an entity.
* input : name of the entity
*/
Bottle abmReasoning::getInfoEntity(string sName)
{
    Bottle bMessenger, bOutput;

    if (realOPC->getEntity(sName) != NULL)
    {
        // The object is not in the OPC, we have to search in the memory to get the type.
        ostringstream osEntity;
        osEntity << "SELECT instance, opcid FROM entity WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1";
        bMessenger = requestFromStream(osEntity.str());

        int Instance = atoi(bMessenger.get(0).asList()->get(0).toString().c_str()),
            Opcid = atoi(bMessenger.get(0).asList()->get(1).toString().c_str());

        osEntity.str("");
        osEntity << "SELECT subtype FROM contentopc WHERE instance = " << Instance << " AND opcid = " << Opcid;
        bMessenger = requestFromStream(osEntity.str());

        string sSubType = bMessenger.get(0).asList()->get(0).toString();

        osEntity.str("");
        osEntity << "SELECT count(*) FROM contentarg WHERE argument = '" << sName << "'";
        bMessenger = requestFromStream(osEntity.str());

        int iNbInteraction = atoi(bMessenger.get(0).asList()->get(0).toString().c_str());
        yInfo() << "\t" << "I have interacted with this " << sSubType << " " << iNbInteraction / 2 << " times ! ";


    }



    return bOutput;

}


/*
* Search in all the knwoledge, information about the input
*/
Bottle abmReasoning::whatIs(string sInput)
{
    Bottle bReturn;
    // 1. Search in CK

    map<string, vector<pair<string, int> > >  lInputRelBef,
        lInputRelAft;
    map<string, int>    mapOccurence;
    bool bFound;
    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin(); itCK != listContextualKnowledge.end(); itCK++)
    {   //begin FOR itCK : listContextualKnowledge

        if (itCK->sName == sInput)
        {
            itCK->updateAgentRelated();
            itCK->updateIntersect();
            itCK->updatePresence();

            mapOccurence[itCK->sType] += itCK->iOccurence;

            for (map<string, pair<double, double>>::iterator itMP = itCK->mPercentObjectFromTo.begin();
                itMP != itCK->mPercentObjectFromTo.end();
                itMP++)
            {
                if (itMP->second.first > abmReasoningFunction::THRESHOLD_CONFIDENCE_RELATION / 100.) {
                    //cout << itMP->first << " From " << itMP->second.first << "%" << endl;
                }
                if (itMP->second.second > abmReasoningFunction::THRESHOLD_CONFIDENCE_RELATION / 100.) {
                    // cout << itMP->first << " To " << itMP->second.second << "%" << endl;
                }

            }

            for (map<string, pair<double, double>>::iterator itMP = itCK->mPercentIntersectLocation.begin();
                itMP != itCK->mPercentIntersectLocation.end();
                itMP++)
            {
                if (itMP->second.first > abmReasoningFunction::THRESHOLD_CONFIDENCE_RELATION / 100.) {
                    //cout << itMP->first << " intersecteced " << itMP->second.first << "%" << endl;
                }
                if (itMP->second.second > abmReasoningFunction::THRESHOLD_CONFIDENCE_RELATION / 100.) {
                    //cout << itMP->first << " intersecteced " << itMP->second.second << "%" << endl;
                }

            }

            if (itCK->PercentPresence.first > abmReasoningFunction::THRESHOLD_CONFIDENCE_RELATION / 100.){
                //cout << itCK->sName << " object present before: " << itCK->PercentPresence.first << endl;
            }

            if (itCK->PercentPresence.second > abmReasoningFunction::THRESHOLD_CONFIDENCE_RELATION / 100.){
                //cout << itCK->sName << " object present after: " << itCK->PercentPresence.second << endl;
            }


            // for all relations before in the CK
            for (vector<pair<string, int>>::iterator itVPCK = itCK->mRelationBefore.begin();
                itVPCK != itCK->mRelationBefore.end();
                itVPCK++){
                // search if already in the relation of the input
                bFound = false;
                for (vector<pair<string, int>>::iterator itVPInput = lInputRelBef[itCK->sType].begin();
                    itVPInput != lInputRelBef[itCK->sType].end();
                    itVPInput++){
                    if (!bFound && itVPCK->first == itVPInput->first){
                        bFound = true;
                        itVPInput->second += itVPCK->second;
                    }
                }
                if (!bFound){
                    lInputRelBef[itCK->sType].push_back(pair<string, int>(itVPCK->first, itVPCK->second));
                }
            }

            // for all relations after in the CK
            for (vector<pair<string, int>>::iterator itVPCK = itCK->mRelationAfter.begin();
                itVPCK != itCK->mRelationAfter.end();
                itVPCK++){
                // search if already in the relation of the input
                bFound = false;
                for (vector<pair<string, int>>::iterator itVPInput = lInputRelAft[itCK->sType].begin();
                    itVPInput != lInputRelAft[itCK->sType].end();
                    itVPInput++){
                    if (!bFound && itVPCK->first == itVPInput->first){
                        bFound = true;
                        itVPInput->second += itVPCK->second;
                    }
                }
                if (!bFound){
                    lInputRelAft[itCK->sType].push_back(pair<string, int>(itVPCK->first, itVPCK->second));
                }
            }
        }
    }

    bReturn.addString(sInput);
    bool bInformation = false;
    // display all relations found:
    /* BEFORE */
    for (map<string, vector<pair<string, int> > > ::iterator itMap = lInputRelBef.begin();
        itMap != lInputRelBef.end();
        itMap++){
        //cout << "When " << sInput << " is a " << itMap->first << ":" << endl;
        for (vector<pair<string, int>>::iterator itVPInput = itMap->second.begin();
            itVPInput != itMap->second.end();
            itVPInput++){
            if ((100.*itVPInput->second) / (1.*mapOccurence[itMap->first]) > abmReasoningFunction::THRESHOLD_CONFIDENCE_RELATION){
                Bottle bTemp;
                bTemp.addString("before");
                bTemp.addString(itMap->first);
                bTemp.addString(itVPInput->first);
                bReturn.addList() = bTemp;
                bInformation = true;
                //               cout << "relation before: " << itVPInput->first << " , " << (100.*itVPInput->second) / (1.*mapOccurence[itMap->first]) << "%" << " (" << mapOccurence[itMap->first] << ")" << endl;
            }
        }
    }

    /* AFTER */
    for (map<string, vector<pair<string, int> > > ::iterator itMap = lInputRelAft.begin();
        itMap != lInputRelAft.end();
        itMap++){
        //cout << "When " << sInput << " is a " << itMap->first << ":" << endl;
        for (vector<pair<string, int>>::iterator itVPInput = itMap->second.begin();
            itVPInput != itMap->second.end();
            itVPInput++){
            if ((100.*itVPInput->second) / (1.*mapOccurence[itMap->first]) > abmReasoningFunction::THRESHOLD_CONFIDENCE_RELATION){
                Bottle bTemp;
                bTemp.addString("after");
                bTemp.addString(itMap->first);
                bTemp.addString(itVPInput->first);
                bReturn.addList() = bTemp;
                bInformation = true;
                //           cout << "relation after: " << itVPInput->first << " , " << (100.*itVPInput->second) / (1.*mapOccurence[itMap->first]) << "%" << " (" << mapOccurence[itMap->first] << ")" << endl;
            }
        }
    }

    if (!bInformation){
        bReturn.clear();
        bReturn.addString(abmReasoningFunction::TAG_NULL);
    }

    return bReturn;
}


/*
* Change a action in the OPC with the rigth name
* @param bottle renameAction : (<instance_begin> <activity_name> <activity_type>) (argument <argument_name> <argument_type> <argument_subtype>) with n bottle argument
*/
Bottle abmReasoning::renameAction(Bottle bInput)
{
    Bottle  bOutput,
        bRequest,
        bMain,
        bArgu;

    bMain = (*bInput.get(1).asList());
    bArgu = (*bInput.get(2).asList());

    if (bMain.size() != 3)
    {
        bOutput.addString("error in renameAction, information missing for main table");
        return bOutput;
    }

    ostringstream osRequest;
    string  sActivityName = bMain.get(1).asString().c_str(),
        sActivityType = bMain.get(2).asString().c_str(),
        sArgument = bArgu.get(1).asString().c_str(),
        sType = bArgu.get(2).asString().c_str(),
        sSubtype = bArgu.get(3).asString().c_str(),
        sRole = bArgu.get(4).asString().c_str();

    int iInstance = bMain.get(0).asInt();

    osRequest << "UPDATE main SET activityname = '" << sActivityName << "', activitytype = '" << sActivityType << "' WHERE instance = " << iInstance;
    bRequest = requestFromStream(osRequest.str().c_str());

    osRequest.str("");
    osRequest << "UPDATE contentarg SET argument = '" << sArgument << "', type = '" << sType << "', subtype = '" << sSubtype << "', role = '" << sRole << "' WHERE instance = " << iInstance << " AND argument = 'unknown' ";
    bRequest = requestFromStream(osRequest.str().c_str());

    osRequest.str("");
    osRequest << "UPDATE contentarg SET argument = '" << sArgument << "', type = '" << sType << "', subtype = '" << sSubtype << "', role = '" << sRole << "' WHERE instance = " << iInstance << " AND argument = 'none' ";
    bRequest = requestFromStream(osRequest.str().c_str());

    return bOutput;
}


int abmReasoning::sendAdjectiveKnowledge(vector<adjKnowledge> listADK)
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
                    if (bFirst)
                        bFirst = false;
                    else
                        osInsertKnowledge << " , ";

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
                if (bFirst)
                    bFirst = false;
                else
                    osInsertKnowledge << " , ";

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
                if (bFirst)
                    bFirst = false;
                else
                    osInsertKnowledge << " , ";

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
int abmReasoning::sendSpatialKnowledge(vector<spatialKnowledge> listSpatialKnowledge)
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
int abmReasoning::sendTemporalKnowledge(vector<timeKnowledge> listTimeKnowledge)
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
int abmReasoning::sendBehaviors(vector<behavior> listBehaviors)
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
int abmReasoning::sendPlan(vector<plan> listPlan)
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
int abmReasoning::sendInteractionKnowledge(vector<knownInteraction> listIN)
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
int abmReasoning::sendContextual(vector<contextualKnowledge> listContextualKnowledge)
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
*   Save the knowledge in the semantical memory
*/
Bottle abmReasoning::saveKnowledge(vector<spatialKnowledge> listSK, vector<timeKnowledge> listTK, vector<behavior> listBehavior, vector<plan> listPlan, vector<contextualKnowledge> listCK, vector<knownInteraction> listInc)
{
    Bottle  bOutput,
        bRequest,
        bMessenger;

    yInfo() << "\t" << "starting to save knowledge ... ";

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

/*
*  Find in the action or sentence which satisfie a situation.
*
*/
Bottle abmReasoning::howTo(string agentGoal, string verbGoal, string objectGoal)
{
    Bottle bOutput;
    cout << "-----------------" << endl;
    cout << "Solving: " << agentGoal << " " << verbGoal << " " << objectGoal << endl;

    Bottle bSolutions;

    // Searching in all known CK if one can solve the goal.
    vector<string> pastCK;
    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin();
        itCK != listContextualKnowledge.end();
        itCK++)
    {
        bool passed = false;
        for (vector<string>::iterator itVS = pastCK.begin();
            itVS != pastCK.end();
            itVS++){
            if (itCK->sName == *itVS) passed = true;
        }
        if (!passed){
            Bottle bCK = whatIs(itCK->sName);
            if (bCK.toString() != abmReasoningFunction::TAG_NULL){
                //  cout << itCK->sName << " has the following  attributes: " << bCK.toString() << endl;
                // for each condition
                map<string, vector<Bottle>> mapKindAttribute;
                for (int ii = 1; ii < bCK.size(); ii++){
                    // check if is an after consequence:
                    Bottle bAttribute = *bCK.get(ii).asList();
                    //                    cout << "bAttribute: " << bAttribute.toString() << endl;
                    Bottle bTemp;
                    bTemp.addString(bAttribute.get(0).asString());

                    //                      cout << "is in after" << endl;
                    string sKind = bAttribute.get(1).asString();
                    string sAttribute = bAttribute.get(2).toString();

                    string agentAttr,
                        verbAttr,
                        objectAttr;

                    //                        cout << "sKind: " << sKind << " , sAttribute: " << sAttribute << endl;

                    string sNameCut = sAttribute;
                    string delimiter = abmReasoningFunction::TAG_SEPARATOR;
                    size_t pos = 0;
                    string token;
                    int jj = 0;
                    while ((pos = sAttribute.find(delimiter)) != string::npos) {
                        token = sAttribute.substr(0, pos);
                        sAttribute.erase(0, pos + delimiter.length());
                        sNameCut = token;
                        switch (jj)
                        {
                        case 0:
                            agentAttr = token;
                            break;
                        case 1:
                            verbAttr = token;
                            break;
                        }
                        jj++;
                    }
                    objectAttr = sAttribute;
                    bTemp.addString(agentAttr);
                    bTemp.addString(verbAttr);
                    bTemp.addString(objectAttr);

                    mapKindAttribute[sKind].push_back(bTemp);
                }


                // REORGANISING THE BOTTLES

                for (map<string, vector<Bottle>>::iterator mSVB = mapKindAttribute.begin();
                    mSVB != mapKindAttribute.end();
                    mSVB++)
                {
                    vector<Bottle> vBotBef,
                        vBotAft;
                    for (vector<Bottle>::iterator itVB = mSVB->second.begin();
                        itVB != mSVB->second.end();
                        itVB++){
                        if (itVB->get(0).asString() == "after"){
                            vBotAft.push_back(*itVB);
                        }
                        else if (itVB->get(0).asString() == "before"){
                            vBotBef.push_back(*itVB);
                        }
                    }

                    //CHECK IF THERE IS AN EXISTING CONSEQUENCE:

                    // for each consequence:
                    bool bCouldWork = false;
                    for (vector<Bottle>::iterator itVBAf = vBotAft.begin();
                        itVBAf != vBotAft.end();
                        itVBAf++){
                        if (itVBAf->get(2).asString() == verbGoal){
                            cout << endl << "the " << mSVB->first << " " << itCK->sName << " could work !" << endl;
                            cout << "I just need to fit: " << agentGoal << " as " << itVBAf->get(1).asString() << ", " << objectGoal << " as " << itVBAf->get(3).asString() << endl;
                            bCouldWork = true;
                        }
                    }

                    // if this CK could work, check for pre conditions:
                    if (bCouldWork){
                        if (vBotBef.size() != 0){
                            cout << "However, I need to fulfil before: " << endl;
                        }
                        for (vector<Bottle>::iterator itVBBf = vBotBef.begin();
                            itVBBf != vBotBef.end();
                            itVBBf++){
                            cout << itVBBf->get(1).asString() << " " << itVBBf->get(2).asString() << " " << itVBBf->get(3).asString() << endl;

                        }
                    }
                }
            }
            pastCK.push_back(itCK->sName);
        }
        //                displayContextual(itCK->sName, itCK->sArgument, itCK->sType);
    }



    return bOutput;
}

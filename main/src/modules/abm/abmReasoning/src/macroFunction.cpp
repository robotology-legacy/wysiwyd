
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


/**
* Search for the given action, and return the properties
* @param bInput : Bottle ("executeAction"       actionName      argument        object      agent)
*/
Bottle abmReasoning::executeAction(Bottle bInput)
{
    Bottle  bOutput, // main output
        bCoord;  // Bottle of coordonates


    // Check format of input
    if (bInput.size() <= 4)
    {
        bOutput.addString("NACK");
        bOutput.addString("Error in the size of the input in executeAction");
        return bOutput;
    }

    if (!bInput.get(1).isString() || !bInput.get(2).isString() || !bInput.get(3).isString())
    {
        bOutput.addString("NACK");
        bOutput.addString("Error in the format of the input in executeAction");
        return bOutput;
    }
    string  sAction = bInput.get(1).toString().c_str(),
        sArgument = bInput.get(2).toString().c_str(),
        sObject = bInput.get(3).toString().c_str(),
        sAgent = bInput.get(4).toString().c_str();


    // Search action in the list of knowledge
    bool bFound = false;
    adjKnowledge skWantedAction;
    for (vector<adjKnowledge>::iterator it = listKnownAdverb.begin(); it != listKnownAdverb.end(); it++)
    {
        if (it->sLabel == sArgument)
        {
            yInfo() << " in abmReasoning::macroFunction.cpp::executeAction : getEffect is: " << it->getEffect(sAction, true).toString();
            Bottle bTemp;
            bTemp.addString("effect");
            bTemp.addList() = it->getEffect(sAction, true);
            bOutput.addList() = bTemp;
            bFound = true;
        }
    }

    if (!bFound)
    {
        sAction = "Cannot find action : " + sAction;
        sAction += " " + sArgument;
        bOutput.addString("NACK");
        bOutput.addString(sAction.c_str());
        return bOutput;
    }

    bFound = false;
    contextualKnowledge ckAction;
    for (vector<contextualKnowledge>::iterator it = listContextualKnowledge.begin(); it != listContextualKnowledge.end(); it++)
    {
        if (it->sName == sAction)
        {
            if (it->sArgument == sArgument)
            {
                ckAction = *it;
                bFound = true;
            }
        }
    }

    /*   if (bFound)
       {
       ckAction.checkConditions();
       }

       Return
       if (skWantedAction.isAbsolut)
       {
       bOutput.addString("move");
       bOutput.addString("absolut");
       double muX = 0, muY = 0;
       for (unsigned int i = 0; i < skWantedAction.vX.size(); i++)
       {
       muX += skWantedAction.vX[i];
       muY += skWantedAction.vY[i];
       }
       muX /= (skWantedAction.vX.size() *1.);
       muY /= (skWantedAction.vX.size() *1.);
       bCoord.addDouble(muX);
       bCoord.addDouble(muY);
       bOutput.addList() = bCoord;
       }
       else if (skWantedAction.isRelative)
       {
       bOutput.addString("move");
       bOutput.addString("relative");
       double muDX = 0, muDY = 0;
       for (unsigned int i = 0; i < skWantedAction.vX.size(); i++)
       {
       muDX += skWantedAction.vDX[i];
       muDY += skWantedAction.vDY[i];
       }
       muDX /= (skWantedAction.vX.size() *1.);
       muDY /= (skWantedAction.vX.size() *1.);

       bCoord.addDouble(muDX);
       bCoord.addDouble(muDY);
       bOutput.addList() = bCoord;
       }*/
    Bottle bPredicate,
        bAgent,
        bObject,
        bRecipient;

    bPredicate.addString("predicate");
    bPredicate.addString(sAction);

    bAgent.addString("agent");
    bAgent.addString(sAgent);

    bObject.addString("object");
    bObject.addString(sObject);

    bRecipient.addString("recipient");
    bRecipient.addString(sArgument);

    bOutput.addList() = bPredicate;
    bOutput.addList() = bAgent;
    bOutput.addList() = bObject;
    bOutput.addList() = bRecipient;

    return bOutput;
}


/**
* Search for the given action, and return the properties
* @param bInput : Bottle ("executeAction"       ("action" actionName)  ("object" object) ("agent" agent) ("argument" arg1) ("argument" arg2))
* return: Bottle (arg1 effect) (arg2 effect)
*/
Bottle abmReasoning::executeActionFromAdv(Bottle bInput)
{
    Bottle  bOutput, // main output
        bCoord;  // Bottle of coordonates

    string sAction = "none";
    string sAgent = "none";
    string sObject = "none";
    string sArg1 = "none";
    string sArg2 = "none";

    sAction = bInput.find("action").toString();
    sAgent = bInput.find("agent").toString();
    sObject = bInput.find("object").toString();
    sArg1 = bInput.find("arg1").toString();
    sArg2 = bInput.find("arg2").toString();

    list<string> listArgument;
    if (sArg1 != "none") listArgument.push_back(sArg1);
    if (sArg2 != "none") listArgument.push_back(sArg2);

    // get the effect of each adverb with the action

    for (list<string>::iterator itArg = listArgument.begin(); itArg != listArgument.end(); itArg++)
    {
        bool	 bFound = false;

        // search for correpondant adverb
        for (vector<adjKnowledge>::iterator itAdvFromList = listKnownAdverb.begin(); itAdvFromList != listKnownAdverb.end(); itAdvFromList++)
        {
            if (itAdvFromList->sLabel == *itArg)
            {
                bFound = true;
                bOutput.addList() = itAdvFromList->getEffect(sAction);
            }
        }

        if (!bFound)
        {
            Bottle bError;
            bError.addString("error");
            bError.addString("argument unknown");
            bOutput.addList() = bError;
        }
    }



    return bOutput;
}


/**
*  Return the command to make, according to the input : complex or action
* @param bInput : Bottle : "executeActivity"  <actionkind> <actionname> (<arg 1>  <arg 2>  ...  <arg n>)  (<role 1>  <role 2 >  ... <role n>)  <isBegin>
*/
Bottle abmReasoning::executeActivity(Bottle bInput)
{
    Bottle bOutput;
    string sErrorFormat = "Error in abmReasoning::executeActivity | wrong format of input";
    Bottle bErrorFormat;
    bErrorFormat.addString(sErrorFormat.c_str());
    if (bInput.size() < 5)
    {
        yInfo() << "\t" << "Error in abmReasoning::executeActivity | wrong number of input";
        bOutput.addString("Error in abmReasoning::executeActivity | wrong number of input");
        return bOutput;
    }

    // Get activity kind
    string sActionKind;
    if (bInput.get(1).isString())
    {
        sActionKind = bInput.get(1).toString();
    }
    else
    {
        yInfo() << "\t" << sErrorFormat;
        return bErrorFormat;
    }

    // if execute action :
    if (sActionKind == abmReasoningFunction::TAG_DB_ACTION.c_str())
    {
        string sAction, sArgument, sObject, sAgent;
        Bottle bArgu, bRole;

        // get action
        if (bInput.get(2).isString())
        {
            sAction = bInput.get(2).toString();
        }
        else
        {
            yInfo() << "\t" << sErrorFormat;
            return bErrorFormat;
        }

        // get the arguments
        if (bInput.get(3).isList())
        {
            bArgu = *bInput.get(3).asList();
        }
        else
        {
            yInfo() << "\t" << sErrorFormat;
            yInfo() << "\t" << "Arguments missing";
            return bErrorFormat;
        }

        // get the Roles
        if (bInput.get(4).isList())
        {
            bRole = *bInput.get(4).asList();
        }
        else
        {
            yInfo() << "\t" << sErrorFormat;
            yInfo() << "\t" << "Roles missing";
            return bErrorFormat;
        }

        // Check size
        int iSizeArg = 0;
        if (bArgu.size() == bRole.size())
        {
            iSizeArg = bArgu.size();
        }
        else
        {
            yInfo() << "\t" << sErrorFormat;
            yInfo() << "\t" << "Argument and role of size different";
            return bErrorFormat;
        }

        // get action object and argument
        bool fObject = false, fArgument = false, fAgent = false;
        for (int i = 0; i < iSizeArg; i++)
        {
            if (bRole.get(i).toString() == "object1")
            {
                sObject = bArgu.get(i).toString();
                fObject = true;
            }
            if (bRole.get(i).toString() == "spatial1")
            {
                sArgument = bArgu.get(i).toString();
                fArgument = true;
            }
            if (bRole.get(i).toString() == "agent1")
            {
                sAgent = bArgu.get(i).toString();
                fAgent = true;
            }
        }
        if (fObject && fArgument)
        {
            Bottle bExecute;
            bExecute.addString("executeAction");
            bExecute.addString(sAction.c_str());
            bExecute.addString(sArgument.c_str());
            bExecute.addString(sObject.c_str());
            bExecute.addString(sAgent.c_str());
            Bottle bOutput;
            bOutput.addList() = (executeAction(bExecute));
            return  bOutput;
        }
        else
        {
            yInfo() << "\t" << sErrorFormat;
            yInfo() << "\t" << "Can't find argument : object1 or spatial1";
            return bErrorFormat;
        }
    }


    // if execute complex : 
    if (sActionKind == "complex")
    {
        return executeComplex(bInput);
    }

    // if execute reason : 
    if (sActionKind == "reasoning")
    {
        Bottle bReasoning;
        bReasoning.addString("executeReasoning");
        Bottle bTemp;
        Bottle bCond;

        bTemp.addString(abmReasoningFunction::TAG_DB_ACTION.c_str());
        bTemp.addString(bInput.get(2).toString().c_str());
        bTemp.addString("reasoning");
        bReasoning.addList() = bTemp;

        //build the condition bottle : ( (sub1 vb1 obj1 manner1) (sub2 vb2 obj2 manner2) ...)
        // from 2 bottles (arg1, arg2, arg3, ..., argn) (role1, role2, role3, ..., rolen)
        //                        bInput.get(3)                 bInput.get(4)
        int nbCond = bInput.get(3).asList()->size() / 4;


        for (int i = 1; i <= nbCond; i++) {
            Bottle bCurrentCond;
            ostringstream ossVb, ossSub, ossObj, ossManner;
            ossManner << abmReasoningFunction::TAG_DB_MANNER << i;
            ossVb << "verb" << i;
            ossSub << "subject" << i;
            ossObj << "object" << i;


            //go to string because cannot do vector of osstringstream
            string sManner = ossManner.str().c_str();
            string sVb = ossVb.str().c_str();
            string sSub = ossSub.str().c_str();
            string sObj = ossObj.str().c_str();


            //vector for searching role : /!\ order is important (vb first, then subject and [object])
            vector <string> vString;
            vString.push_back(sManner);
            vString.push_back(sVb);
            vString.push_back(sSub);
            vString.push_back(sObj);


            //go through the 4 elements role to build a valid condition
            for (vector<string>::iterator itString = vString.begin(); itString != vString.end(); itString++) {
                int j = 0;
                bool roleFound = false;

                //check where is the position of the current role in the role bottle
                while (!roleFound && j < bInput.get(4).asList()->size()){

                    string currentRole = bInput.get(4).asList()->get(j).toString().c_str();

                    //yInfo() << "\t" << "element nb " << j << " in role bottle is : {" << currentRole.c_str() << "} and itString is {" << itString->c_str() << "}" <<endl ; 

                    if (*itString == currentRole){
                        roleFound = true;
                        //put the arg corresponding to this role in the current condition bottle
                        bCurrentCond.addString(bInput.get(3).asList()->get(j).toString().c_str());
                    }

                    j += 1;
                }

                //there is a role missing
                if (roleFound == false) {
                    yInfo() << "\t" << "ERROR when building condition bottle for executeReasoning : one role in condition number " << nbCond << " is missing";
                }
            } //end of the currentBottle build

            //add the currentCondition to the condition bottle (if the condition is valid, 4 elements)
            if (bCurrentCond.size() == 4) {
                bCond.addList() = bCurrentCond;
            }
        }

        //bReasoning.addList() = *bInput.get(3).asList();
        bReasoning.addList() = bCond;

        yInfo() << "\t" << "Bottle sent to executeReasoning : " << bReasoning.toString().c_str();

        return executeReasoning(bReasoning);
    }


    // if execute sharedPlan
    if (sActionKind == abmReasoningFunction::TAG_DB_SHARED_PLAN)
    {
        Bottle bSP;
        bSP.addString("executeSharedPlan");
        Bottle bTemp;
        bTemp.addString(abmReasoningFunction::TAG_DB_ACTION.c_str());
        bTemp.addString(bInput.get(2).toString().c_str());
        bTemp.addString(abmReasoningFunction::TAG_DB_SHARED_PLAN.c_str());
        bSP.addList() = bTemp;
        bSP.addList() = *bInput.get(3).asList();
        bSP.addList() = *bInput.get(4).asList();

        return executeSharedPlan(bSP);
    }


    return bErrorFormat;
}


/*
* Return the list of action to execute for a complex
* @param bInput : Bottle : "executeComplex"  complex <actionname> (<arg 1>  <arg 2>  ...  <arg n>)  (<role 1>  <role 2 >  ... <role n>)  <isBegin>
*/
Bottle abmReasoning::executeComplex(Bottle bInput)
{
    string sErrorFormat = "Error in abmReasoning::executeComplex | wrong format of input";
    Bottle bErrorFormat;
    bErrorFormat.addString(sErrorFormat.c_str());


    // Extract the arguments : 
    string sTemporal,
        sObject1, sObject2,
        sAgent1, sAgent2,
        sAction1, sAction2,
        sArgument1, sArgument2;

    Bottle bArgu, bRole;

    // get the bottle of arguments
    if (bInput.get(3).isList())
    {
        bArgu = *bInput.get(3).asList();
    }
    else
    {
        yInfo() << "\t" << sErrorFormat;
        yInfo() << "\t" << "Arguments missing";
        return bErrorFormat;
    }

    // get the bottle of Roles
    if (bInput.get(4).isList())
    {
        bRole = *bInput.get(4).asList();
    }
    else
    {
        yInfo() << "\t" << sErrorFormat;
        yInfo() << "\t" << "Roles missing";
        return bErrorFormat;
    }

    // Check size
    int iSizeArg = 0;
    if (bArgu.size() == bRole.size())
    {
        iSizeArg = bArgu.size();
    }
    else
    {
        yInfo() << "\t" << sErrorFormat;
        yInfo() << "\t" << "Argument and role of size different";
        return bErrorFormat;
    }

    // get action object and argument
    bool fObject1 = false, fObject2 = false, fAction1 = false, fAction2 = false, fArgument1 = false, fArgument2 = false, fAgent1 = false, fAgent2 = false, fTemporal = false;
    for (int i = 0; i < iSizeArg; i++)
    {
        if (bRole.get(i).toString() == "object1")
        {
            sObject1 = bArgu.get(i).toString();
            fObject1 = true;
        }
        if (bRole.get(i).toString() == "object2")
        {
            sObject2 = bArgu.get(i).toString();
            fObject2 = true;
        }
        if (bRole.get(i).toString() == "action1")
        {
            sAction1 = bArgu.get(i).toString();
            fAction1 = true;
        }
        if (bRole.get(i).toString() == "action2")
        {
            sAction2 = bArgu.get(i).toString();
            fAction2 = true;
        }
        if (bRole.get(i).toString() == "spatial1")
        {
            sArgument1 = bArgu.get(i).toString();
            fArgument1 = true;
        }
        if (bRole.get(i).toString() == "spatial2")
        {
            sArgument2 = bArgu.get(i).toString();
            fArgument2 = true;
        }
        if (bRole.get(i).toString() == "agent1")
        {
            sAgent1 = bArgu.get(i).toString();
            fAgent1 = true;
        }
        if (bRole.get(i).toString() == "agent2")
        {
            sAgent2 = bArgu.get(i).toString();
            fAgent2 = true;
        }
        if (bRole.get(i).toString() == "temporal")
        {
            sTemporal = bArgu.get(i).toString();
            fTemporal = true;
        }
    }
    if (!fObject1 || !fObject2 || !fAction1 || !fAction2 || !fArgument1 || !fArgument2 || !fTemporal)
    {
        yInfo() << "\t" << sErrorFormat;
        yInfo() << "\t" << "some arguments are missing";
        return bErrorFormat;
    }

    Bottle bAction1, bAction2;

    bAction1.addString("executeAction");
    bAction1.addString(sAction1.c_str());
    bAction1.addString(sArgument1.c_str());
    bAction1.addString(sObject1.c_str());
    bAction1.addString(sAgent1.c_str());
    bAction1 = executeAction(bAction1);

    bAction2.addString("executeAction");
    bAction2.addString(sAction2.c_str());
    bAction2.addString(sArgument2.c_str());
    bAction2.addString(sObject2.c_str());
    bAction2.addString(sAgent2.c_str());
    bAction2 = executeAction(bAction2);

    double dProba = -1.;

    for (vector<timeKnowledge>::iterator it = listTimeKnowledge.begin(); it != listTimeKnowledge.end(); it++)
    {
        if (it->sTemporal == sTemporal)
        {
            dProba = it->T1inferiorT2percent();
        }
    }
    yInfo() << "\t" << "Temporal : " << sTemporal << "\ndProba : " << dProba;
    if (dProba == -1)
    {
        yInfo() << "\t" << sErrorFormat;
        yInfo() << "\t" << "temporal not found";
        return bErrorFormat;
    }

    Bottle bComplex;

    if (dProba > 0.5)
    {
        bComplex.addList() = bAction1;
        bComplex.addList() = bAction2;
    }
    else
    {
        bComplex.addList() = bAction2;
        bComplex.addList() = bAction1;
    }


    return bComplex;
}


/*
* Return the list of action to execute for a complex
* @param bInput : Bottle : "executeSharedPlan"  ('action' name_action 'sharedplan') (<arg 1>  <arg 2>  ...  <arg n>)  (<role 1>  <role 2 >  ... <role n>)
*/
Bottle abmReasoning::executeSharedPlan(Bottle bInput)
{
    string sErrorFormat = "Error in abmReasoning::executeSharedPlan | wrong format of input";
    Bottle bErrorFormat, bOutput, bTemp;
    bErrorFormat.addString(sErrorFormat.c_str());

    // check input
    if (bInput.size() != 4)
    {
        string sError = "Error in abmReasoning::executeSharedPlan | Wrong number of input (!= 4)";
        yInfo() << "\t" << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList())
    {
        string sError = "Error in abmReasoning::executeSharedPlan | Wrong format of input";
        yInfo() << "\t" << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    string sName;
    bTemp = *(bInput.get(1).asList());
    if (bTemp.get(0).toString() == abmReasoningFunction::TAG_DB_ACTION.c_str())
    {
        sName = bTemp.get(1).asString();
    }


    int iNbArg = 0;
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | number of argument different of number of role";
        yInfo() << "\t" << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    iNbArg = bInput.get(2).asList()->size();


    // Fill contentArg
    vector< pair<string, string> > vArgument;

    string sManner;
    bool fManner = false;

    // catch the arguments and the role associate

    Bottle bArguments = *bInput.get(2).asList();
    Bottle bRoles = *bInput.get(3).asList();

    for (int i = 0; i < iNbArg; i++)
    {
        pair <string, string > pArgu;
        pArgu.first = (bArguments.get(i).toString().c_str());
        pArgu.second = (bRoles.get(i).toString().c_str());

        vArgument.push_back(pArgu);

        if (bRoles.get(i).toString() == abmReasoningFunction::TAG_DB_MANNER.c_str())
        {
            sManner = pArgu.first;
            fManner = true;
        }
    }

    if (!fManner)
    {
        pair <string, string > pArgu;
        pArgu.first = abmReasoningFunction::TAG_DB_NONE;
        pArgu.second = abmReasoningFunction::TAG_DB_MANNER;
        yInfo() << "\t" << "manner not found. Auto set to : none";
        sManner = abmReasoningFunction::TAG_DB_NONE;
        vArgument.push_back(pArgu);
    }



    // search the shared plan associate
    bool fSP = false, fAgent, fArgu, fObject;
    Bottle bActionTemp;
    string sActionTemp, sArgTemp, sObjectTemp, sAgentTemp;
    for (vector<plan>::iterator it_SP = listPlan.begin(); it_SP != listPlan.end(); it_SP++)
    {
        if (it_SP->sManner == sManner && it_SP->sName == sName && !fSP)
        {
            //yInfo() << "\t" << "sharedPlan found"  ;
            fSP = true;

            for (unsigned int iAction = 0; iAction < it_SP->vActivitytype.size(); iAction++)
            {
                fAgent = false;
                fArgu = false;
                fObject = false;
                bActionTemp.clear();
                bActionTemp.addString("executeAction");
                bActionTemp.addString(it_SP->vActivityname[iAction].c_str());
                for (list < pair < string, string > >::iterator it_ACTI = it_SP->vActivityArguments[iAction].begin(); it_ACTI != it_SP->vActivityArguments[iAction].end(); it_ACTI++)
                {

                    for (vector < pair < string, string > >::iterator it_input_arg = vArgument.begin(); it_input_arg != vArgument.end(); it_input_arg++)
                    {
                        if (it_ACTI->second == it_input_arg->second)
                        {
                            string stempX = it_ACTI->second;
                            string stemp = stempX;
                            if (stempX.length()>1){
                                stemp = stempX.substr(0, stempX.length() - 1);
                            }

                            if (stemp == "object")
                            {
                                sObjectTemp = it_input_arg->first;
                                fObject = true;
                            }

                            if (stemp == "agent")
                            {
                                sAgentTemp = it_input_arg->first;
                                fAgent = true;
                            }
                        }
                        if (it_ACTI->second == "spatial1")
                        {
                            sArgTemp = it_ACTI->first;
                            fArgu = true;
                        }
                    }
                }
                if (!fObject)
                {
                    for (list < pair < string, string > >::iterator it_ACTI = it_SP->vActivityArguments[iAction].begin(); it_ACTI != it_SP->vActivityArguments[iAction].end(); it_ACTI++)
                    {
                        string stempX = it_ACTI->second;
                        string stemp = stempX;
                        if (stempX.length() > 1){
                            stemp = stempX.substr(0, stempX.length() - 1);
                        }

                        if (stemp == "object")
                        {
                            sObjectTemp = it_ACTI->first;
                            fObject = true;
                        }
                    }
                }
                if (!fAgent)
                {
                    for (list < pair < string, string > >::iterator it_ACTI = it_SP->vActivityArguments[iAction].begin(); it_ACTI != it_SP->vActivityArguments[iAction].end(); it_ACTI++)
                    {
                        string stempX = it_ACTI->second;
                        string stemp = stempX;
                        if (stempX.length() > 1){
                            stemp = stempX.substr(0, stempX.length() - 1);
                        }

                        if (stemp == "agent")
                        {
                            sAgentTemp = it_ACTI->first;
                            fAgent = true;
                        }
                    }
                }
                if (!fAgent || !fObject || !fArgu)
                {
                    bOutput.clear();
                    string sError = "Error in abmReasoning::executeSharedPlan | wrong argument of sharedplan";
                    yInfo() << "\t" << sError;
                    bOutput.addString(sError.c_str());
                    return bOutput;
                }
                bActionTemp.addString(sArgTemp.c_str());
                bActionTemp.addString(sObjectTemp.c_str());
                bActionTemp.addString(sAgentTemp.c_str());
                bOutput.addList() = executeAction(bActionTemp);
            }
        }
    }

    if (!fSP)
    {
        yInfo() << "\t" << "in abmReasoning::executeSharedPlan : can't find shared plan : " << sName << "_" << sManner;
    }
    return bOutput;
}


/*
* Return the list of action to execute for a reasoning request
* @param bInput : Bottle : "executeReasoning"  ('action' want 'reasoning') ( (sub1 verb1 obj1) (sub2 verb2 obj2) ...) where (sub verb obj) defined a condition/goal
*/
Bottle abmReasoning::executeReasoning(Bottle bInput)
{
    string sErrorFormat = "Error in abmReasoning::executeReasoning | wrong format of input";
    Bottle bErrorFormat, bOutput, bTemp;
    bErrorFormat.addString(sErrorFormat.c_str());

    // check input
    if (bInput.size() != 3)
    {
        string sError = "Error in abmReasoning::executeReasoning | Wrong number of input (!= 3)";
        yInfo() << "\t" << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList())
    {
        string sError = "Error in abmReasoning::executeReasoning | Wrong format of input";
        yInfo() << "\t" << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    string sName;
    bTemp = *(bInput.get(1).asList());
    if (bTemp.get(0).toString() == abmReasoningFunction::TAG_DB_ACTION.c_str())
    {
        sName = bTemp.get(1).asString();
    }


    int iNbCond = 0;
    if (bInput.get(2).asList()->size() <= 0)
    {
        string sError = "Error in abmReasoning::executeReasoning | no goal are given";
        yInfo() << "\t" << sError;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    iNbCond = bInput.get(2).asList()->size();


    // Fill contentArg
    vector< pair<string, string> > vArgument;

    // catch the arguments and the role associate
    Bottle bConditions = *bInput.get(2).asList();

    //---------------- > create the domain PDDL file
    printPDDLContextualKnowledgeDomain();

    //---------------- > create the problem PDDL file
    printPDDLContextualKnowledgeProblem(bConditions);

    //---------------- > launch the planner solver
    pddlPlannerLauncher();

    //---------------- > launch the planner solver
    bOutput = pddlPlannerSolParser();

    //bool planPossible = false ;

    //format the plan in bOutput
    //if (!planPossible)
    //{
    //  yInfo() << "\t" << "in abmReasoning::executeReasoning : can't find a proper plan :  the goal is not achievable"  ;
    //}
    return bOutput;
}


/**
* Retro Reasoning
* set retroactivly the semantical knowledge into the ABM
*/
Bottle abmReasoning::retroReasoning(int from)
{
    Bottle bOutput;
    if (!mentalOPC->isConnected())
    {
        yInfo() << "\t" << "Error in abmReasoning::retroReasoning | mentalOPC not connect, retroReasoning impossible.";
        bOutput.addString("Error in abmReasoning::retroReasoning | mentalOPC not connect, retroReasoning impossible.");
        return bOutput;
    }

    yInfo() << "\t" << "Retro Reasoning engaged.";
    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE instance > " << from << " ORDER BY instance";
    Bottle  bMessenger = requestFromStream(osRequest.str());
    int numberAction = bMessenger.size();

    yInfo() << "\t" << "found " << numberAction << " action(s)";


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
        //      yInfo() << "\t" << j+1 << "\t" << Id << "\t" ;

        imagineOPC(Id);
        updateOpcObjectLocation(abmReasoningFunction::s_mentalOPC);
        sendRelation(Id);
    }

    yInfo() << "\n" << "retroReasoning Done on " << numberAction << " situations.";
    bOutput.addString("retroReasoning Done");

    return bOutput;
}


/**
* Third level of retro Reasoning
* Building of the map of properties coming from actions information
*/
string abmReasoning::level3Actions(int from)
{
    string bOutput;

    ostringstream osRequest;
    osRequest << "SELECT instance FROM main WHERE (activitytype = 'qRM' or activitytype = 'action')AND begin = true AND INSTANCE > " << from << " ORDER by INSTANCE";
    Bottle  bMessenger = requestFromStream(osRequest.str().c_str());
    int numberAction = bMessenger.size();

    vector<int> vErrorAction;
    yInfo() << "\t" << "found " << numberAction << " action(s)";

    for (int j = 0; j < numberAction; j++)
    {   // begin for each action
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        cout << j + 1 << " (" << Id << ")..";

        Bottle bAction = askActionForLevel3Reasoning(Id);

        if (bAction.toString() == abmReasoningFunction::TAG_NULL)
        {
            //          yWarning() << " level3: error on instance " << Id << ", wrong format in ABM";
            vErrorAction.push_back(Id);
        }
        else
        {
            Bottle bName = *bAction.get(0).asList();
            Bottle bRelationsBefore = *bAction.get(1).asList();
            Bottle bRelationsAfter = *bAction.get(2).asList();

            Bottle bAllRelationsBefore = *bAction.get(3).asList();
            Bottle bAllRelationsAfter = *bAction.get(4).asList();

            string sNameAct = bName.get(0).toString().c_str(),
                sAgentAct = bName.get(1).toString().c_str(),
                sObjectAct = bName.get(2).toString().c_str(),
                sRecipientAct = bName.get(3).toString().c_str();

            if (sNameAct == abmReasoningFunction::TAG_NULL ||
                sAgentAct == abmReasoningFunction::TAG_NULL ||
                sObjectAct == abmReasoningFunction::TAG_NULL ||
                sRecipientAct == abmReasoningFunction::TAG_NULL){
                yWarning() << " Error in instance" << Id << "missing information.";
                vErrorAction.push_back(Id);
                yInfo() << "instance is: " << sAgentAct << sNameAct << sObjectAct << sRecipientAct;
            }
            else{



                //               yInfo() << " bName : \t" << bName.toString() << "\t" << " bRelationsBefore : \t" << bRelationsBefore.toString() << "\t" << " bRelationsAfter  : \t" << bRelationsAfter.toString();

                // Before
                vector<string>  vLocFocusBefore, vLocFocusAfter;       // Vector with the location of the focus object
                vector< pair <string, string> > vObjectLocBefore,
                    vObjectLocAfter;    // vector with the location of the other objects
                vector<pair <string, bool> > vObjectBoolBefore,
                    vObjectBoolAfter;       // vector is the object was on the same location

                // begin if bRelationBefore is Null
                if (bRelationsBefore.toString().c_str() != abmReasoningFunction::TAG_NULL)
                {
                    string sObject,
                        sLocation;
                    for (int i = 0; i < bRelationsBefore.size(); i++)
                    {   // begin decomposition bRelationBefore
                        Bottle bTemp = *bRelationsBefore.get(i).asList();
                        sObject = bTemp.get(0).toString().c_str();
                        sLocation = bTemp.get(1).toString().c_str();
                        if (sObject == sObjectAct)
                        {
                            vLocFocusBefore.push_back(sLocation);
                        }
                        else
                        {
                            pair<string, string> pTemp(sObject, sLocation);
                            vObjectLocBefore.push_back(pTemp);
                        }
                    }   // end decomposition bRelationBefore


                    // for each object create a pair object-bool
                    for (vector<pair<string, string> >::iterator pItLocation = vObjectLocBefore.begin(); pItLocation != vObjectLocBefore.end(); pItLocation++)
                    {   // begin for each object create a pair object-bool
                        bool bFound = false;
                        for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolBefore.begin(); pItBool != vObjectBoolBefore.end(); pItBool++)
                        {
                            if (pItLocation->first == pItBool->first)
                            {
                                bFound = true;
                                pItBool->second = false;
                            }
                        }
                        if (!bFound)
                        {   // begin IF the pair doesn't exist yet
                            pair <string, bool> pTemp(pItLocation->first, false);
                            vObjectBoolBefore.push_back(pTemp);
                        }   // end IF
                    }   // end for each object create a pair object-bool


                    // begin for each object-location check
                    for (vector<pair<string, string> >::iterator pItLocation = vObjectLocBefore.begin(); pItLocation != vObjectLocBefore.end(); pItLocation++)
                    {
                        // for any location where the object of focus is
                        for (vector<string>::iterator itLocFocus = vLocFocusBefore.begin(); itLocFocus != vLocFocusBefore.end(); itLocFocus++)
                        {
                            // is it the location is the same that the object of focus
                            if (pItLocation->second == *itLocFocus)
                            {
                                // search the appropriate pair
                                for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolBefore.begin(); pItBool != vObjectBoolBefore.end(); pItBool++)
                                {
                                    if (pItLocation->first == pItBool->first)
                                    {
                                        pItBool->second = true;
                                    }
                                }
                            }
                        }   // end FOR any location where the object of focus is
                    }   // end FOR each object-location check
                }   // end IF bRelationBefore is Null


                // After
                if (bRelationsAfter.toString().c_str() != abmReasoningFunction::TAG_NULL)
                {   // begin IF bRelationAfter is Null
                    string sObject,
                        sLocation;

                    for (int i = 0; i < bRelationsAfter.size(); i++)
                    {   // begin decomposition of bRelationAfter
                        Bottle bTemp = *bRelationsAfter.get(i).asList();
                        sObject = bTemp.get(0).toString().c_str();
                        sLocation = bTemp.get(1).toString().c_str();
                        if (sObject == sObjectAct)
                        {
                            vLocFocusAfter.push_back(sLocation);
                        }
                        else
                        {
                            pair<string, string> pTemp(sObject, sLocation);
                            vObjectLocAfter.push_back(pTemp);
                        }
                    }   // end FOR : decomposition of bRelationAfter

                    // for each object create a pair object-bool
                    for (vector<pair<string, string> >::iterator pItLocation = vObjectLocAfter.begin(); pItLocation != vObjectLocAfter.end(); pItLocation++)
                    {
                        bool bFound = false;
                        for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolAfter.begin(); pItBool != vObjectBoolAfter.end(); pItBool++)
                        {
                            if (pItLocation->first == pItBool->first)
                            {
                                bFound = true;
                                pItBool->second = false;
                            }
                        }
                        if (!bFound)
                        {   // begin IF the pair doesn't exist yet
                            pair <string, bool> pTemp(pItLocation->first, false);
                            vObjectBoolAfter.push_back(pTemp);
                        }   // end IF

                    }   // end FOR each object create a pair object-bool

                    // FOR each object-location check
                    for (vector<pair<string, string> >::iterator pItLocation = vObjectLocAfter.begin(); pItLocation != vObjectLocAfter.end(); pItLocation++)
                    {
                        for (vector<string>::iterator itLocFocus = vLocFocusAfter.begin(); itLocFocus != vLocFocusAfter.end(); itLocFocus++)
                        {   // begin FOR any location where the object of focus is

                            if (pItLocation->second == *itLocFocus)
                            {   // begin IF the location is the same that the object of focus

                                for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolAfter.begin(); pItBool != vObjectBoolAfter.end(); pItBool++)
                                {   // FOR search the appropriate pair
                                    if (pItLocation->first == pItBool->first)
                                    {
                                        pItBool->second = true;
                                    }
                                }   // end FOR search the appropriate pair
                            }   // end IF the location is the same that the object of focus
                        }   // end FOR any location where the object of focus is
                    }   // end FOR each object-location check
                }   // end IF bRelationAfter is Null


                list<string> lRelationBefore;
                // OTHER RELATIONS BEFORE
                if (bAllRelationsBefore.toString().c_str() != abmReasoningFunction::TAG_NULL)
                {   // begin IF bAllRelationAfter is not Null

                    // for each relation not locative before the action:
                    for (int ii = 0; ii < bAllRelationsBefore.size(); ii++)
                    {
                        Bottle bTmpRelation = *bAllRelationsBefore.get(ii).asList();

                        //if the relation has enought component
                        if (bTmpRelation.size() >2){
                            string sAgentRel = bTmpRelation.get(0).toString();
                            string sActionRel = bTmpRelation.get(1).toString();
                            string sObjectRel = bTmpRelation.get(2).toString();
                            string sConcatenateRelation;

                            //yInfo() << "instance is: " << sAgentAct << sNameAct << sObjectAct << sRecipientAct;
                            //yInfo() << "relation: " << sAgentRel << "," << sActionRel << "," << sObjectRel;

                            // check if the agent of the action is part of the action arguments
                            if (sAgentRel == sAgentAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_AGENT;
                            }
                            else if (sAgentRel == sNameAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sAgentRel == sObjectAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sAgentRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else{
                                sConcatenateRelation = sAgentRel;
                            }

                            // add separator for relation in one string.
                            sConcatenateRelation += abmReasoningFunction::TAG_SEPARATOR;


                            // check if the verb of the action is part of the action arguments
                            if (sActionRel == sAgentAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_AGENT;
                            }
                            else if (sActionRel == sNameAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sActionRel == sObjectAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sActionRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else {
                                sConcatenateRelation += sActionRel;
                            }

                            // add separator for relation in one string.
                            sConcatenateRelation += abmReasoningFunction::TAG_SEPARATOR;
                            // check if the verb of the action is part of the action arguments
                            if (sObjectRel == sAgentAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_AGENT;
                            }
                            else if (sObjectRel == sNameAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sObjectRel == sObjectAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sObjectRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else {
                                sConcatenateRelation += sObjectRel;
                            }

                            //add the relation to the list of relations
                            lRelationBefore.push_back(sConcatenateRelation);
                        } //end if the relation has enought component
                    } // END for each relation not locative before the action:
                } // END OTHER RELATION BEFORE


                list<string> lRelationAfter;
                // OTHER RELATIONS AFTER
                if (bAllRelationsAfter.toString().c_str() != abmReasoningFunction::TAG_NULL)
                {   // begin IF bAllRelationAfter is not Null

                    // for each relation not locative before the action:
                    for (int ii = 0; ii < bAllRelationsAfter.size(); ii++)
                    {
                        Bottle bTmpRelation = *bAllRelationsAfter.get(ii).asList();

                        //if the relation has enought component
                        if (bTmpRelation.size() >2){
                            string sAgentRel = bTmpRelation.get(0).toString();
                            string sActionRel = bTmpRelation.get(1).toString();
                            string sObjectRel = bTmpRelation.get(2).toString();

                            string sConcatenateRelation;

                            // check if the agent of the action is part of the action arguments
                            if (sAgentRel == sAgentAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_AGENT;
                            }
                            else if (sAgentRel == sNameAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sAgentRel == sObjectAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sAgentRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else{
                                sConcatenateRelation += sAgentRel;
                            }

                            // add separator for relation in one string.
                            sConcatenateRelation += abmReasoningFunction::TAG_SEPARATOR;


                            // check if the verb of the action is part of the action arguments
                            if (sActionRel == sAgentAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_AGENT;
                            }
                            else if (sActionRel == sNameAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sActionRel == sObjectAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sActionRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else {
                                sConcatenateRelation += sActionRel;
                            }

                            // add separator for relation in one string.
                            sConcatenateRelation += abmReasoningFunction::TAG_SEPARATOR;
                            // check if the verb of the action is part of the action arguments
                            if (sObjectRel == sAgentAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_AGENT;
                            }
                            else if (sObjectRel == sNameAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sObjectRel == sObjectAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sObjectRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else {
                                sConcatenateRelation += sObjectRel;
                            }

                            //add the relation to the list of relations
                            lRelationAfter.push_back(sConcatenateRelation);
                        } //end if the relation has enought component
                    } // END for each relation not locative before the action:
                } // END OTHER RELATIONS AFTER


                // create vector before / after
                vector<tuple<string, bool, bool> > vTObjects;

                // BEFORE
                for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolBefore.begin(); pItBool != vObjectBoolBefore.end(); pItBool++)
                {
                    tuple<string, bool, bool> tuTemp(pItBool->first, pItBool->second, false);
                    vTObjects.push_back(tuTemp);
                }

                // AFTER
                for (vector<pair<string, bool> >::iterator pItBool = vObjectBoolAfter.begin(); pItBool != vObjectBoolAfter.end(); pItBool++)
                {   //  FOR each object present after
                    bool bFound = false;

                    for (vector<tuple<string, bool, bool> >::iterator itTu = vTObjects.begin(); itTu != vTObjects.end(); itTu++)
                    {   // FOR each object present before

                        if (get<0>(*itTu) == pItBool->first)
                        {   // IF match
                            bFound = true;
                            get<2>(*itTu) = pItBool->second;
                        }   // end IF match
                    }   // end FOR each object present before

                    if (!bFound)
                    {
                        tuple<string, bool, bool> tuTemp(pItBool->first, false, pItBool->second);
                        vTObjects.push_back(tuTemp);
                    }
                }

                createContextualKnowledge(sNameAct, sObjectAct, abmReasoningFunction::TAG_ACTION);
                // updating the contextual knowledge
                for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin();
                    itCK != listContextualKnowledge.end();
                    itCK++)
                {   //begin FOR itCK : listContextualKnowledge

                    if (itCK->sName == sNameAct && itCK->sArgument == sObjectAct)
                    {   // IF CK found
                        // cout << "itCK->sName: " << itCK->sName << ", sNameAct: " << sNameAct << "; add occurence on instance: " << Id << endl;
                        bool bFound = false;
                        itCK->iOccurence++;
                        // add all relations before:
                        for (list<string>::iterator itS = lRelationBefore.begin();
                            itS != lRelationBefore.end();
                            itS++){
                            bFound = false;
                            // finding the corresponding relation in the CK
                            for (vector<pair<string, int> >::iterator itM = itCK->mRelationBefore.begin();
                                itM != itCK->mRelationBefore.end(); itM++) {
                                if (!bFound && itM->first == *(itS))
                                {
                                    itM->second++;
                                    bFound = true;
                                }
                            }

                            // if the relation doesn't exist in the CK
                            if (!bFound){
                                pair<string, int> tmp(*itS, 1);
                                itCK->mRelationBefore.push_back(tmp);
                            }
                        }// END ADD ALL RELATIONS BEFORE

                        bFound = false;

                        // add all relations AFTER:
                        for (list<string>::iterator itS = lRelationAfter.begin();
                            itS != lRelationAfter.end();
                            itS++){

                            bFound = false;
                            // finding the corresponding relation in the CK
                            for (vector<pair<string, int> >::iterator itM = itCK->mRelationAfter.begin();
                                itM != itCK->mRelationAfter.end();
                                itM++) {
                                if (!bFound && itM->first == *(itS))
                                {
                                    itM->second++;
                                    bFound = true;
                                }
                            }

                            // if the relation doesn't exist in the CK
                            if (!bFound){
                                pair<string, int> tmp(*itS, 1);
                                itCK->mRelationAfter.push_back(tmp);
                            }
                        }// END ADD ALL RELATIONS AFTER

                        for (vector<tuple<string, bool, bool> >::iterator itTu = vTObjects.begin(); itTu != vTObjects.end(); itTu++)
                        {   // FOR itTu: vOTobject each object present 
                            bFound = false;

                            for (map<string, vector <pair <bool, bool> > >::iterator itMap = itCK->mObjectFromTo.begin(); itMap != itCK->mObjectFromTo.end(); itMap++)
                            {   // FOR each object already in the CK

                                if (itMap->first == get<0>(*itTu))
                                {   // IF the object of the CK is the same of the object present
                                    bFound = true;
                                    pair<bool, bool>    pTemp(get<1>(*itTu), get<2>(*itTu));
                                    itMap->second.push_back(pTemp);
                                }   // endIF the object of the CK is the same of the object present
                            }   // end FOR each object already in the CK

                            if (!bFound)
                            {   // IF object wasn't present
                                pair<bool, bool>    pTemp(get<1>(*itTu), get<2>(*itTu));
                                itCK->mObjectFromTo[get<0>(*itTu)].push_back(pTemp);
                            }   // end IF object wasn't present
                        }   // end FOR itTu: vOTobject each object present 

                    }   // end IF CK found
                }   //end FOR itCK : listContextualKnowledge

                // check if Hanoi
                if (bAction.size() == 4)
                {   // begin IF has spatial1 and spatial 2
                    Bottle bSpatial = *bAction.get(3).asList();

                    string sFrom = "none",
                        sTo = "none"; // locations from and to of the move
                    // Get the argument FROM and TO
                    for (int i = 0; i < 2; i++)
                    {
                        if (bSpatial.size() > i)
                        {
                            Bottle bTemp = *bSpatial.get(i).asList();
                            if (bTemp.get(1).toString() == "spatial1")
                            {   //  IF bottle is TO (spatial1)
                                sTo = bTemp.get(0).toString().c_str();
                            }
                            else
                            {   // If bottle is FROM (spatial2)
                                sFrom = bTemp.get(0).toString().c_str();
                            }
                        }
                    }


                    pair<bool, bool>    pbFROM(false, false),   // if the focus was at the location FROM before and after
                        pbTO(false, false);                 // if the focus was at the location TO before and after

                    // CHECK IF FOCUS AT LOCATION FROM AND TO

                    //before
                    for (vector<string>::iterator itLocFoc = vLocFocusBefore.begin(); itLocFoc != vLocFocusBefore.end(); itLocFoc++)
                    {   // begin FOR each location where the focus was before 
                        if (*itLocFoc == sFrom)
                        {
                            pbFROM.first = true;
                        }

                        if (*itLocFoc == sTo)
                        {
                            pbTO.first = true;
                        }
                    }   // end FOR each location where the focus was before

                    //after
                    for (vector<string>::iterator itLocFoc = vLocFocusAfter.begin(); itLocFoc != vLocFocusAfter.end(); itLocFoc++)
                    {   // begin FOR each location where the focus was after
                        if (*itLocFoc == sFrom)
                        {
                            pbFROM.second = true;
                        }

                        if (*itLocFoc == sTo)
                        {
                            pbTO.second = true;
                        }
                    }   // end FOR each location where the focus was after

                    // updating the contextual knowledge
                    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin(); itCK != listContextualKnowledge.end(); itCK++)
                    {   //begin FOR itCK : listContextualKnowledge

                        if (itCK->sName == sNameAct && itCK->sArgument == sObjectAct)
                        {   // IF CK found

                            // update FROM
                            itCK->mIntersectLocation["from"].push_back(pbFROM);
                            // update TO
                            itCK->mIntersectLocation["to"].push_back(pbTO);

                        }   // end IF CK found
                    }   //end FOR itCK : listContextualKnowledge
                }   // end IF has spatial1 and spatial 2
            }
        }
    } // end for each action




    ostringstream osErrorMessage;
    osErrorMessage << "\n" << "level3 finished with " << vErrorAction.size() << " action errors." << endl;
    if (vErrorAction.size() != 0)
    {
        osErrorMessage << "Actions Instances errors are ";
        for (vector<int>::iterator itErr = vErrorAction.begin(); itErr != vErrorAction.end(); itErr++)
        {
            osErrorMessage << ", " << *itErr;
        }
        osErrorMessage << ".";
    }


    string sErrorMessage = osErrorMessage.str();

    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin(); itCK != listContextualKnowledge.end(); itCK++)
    {
        itCK->updatePresence();
    }

    bOutput = sErrorMessage;
    return bOutput;
}


/**
* Third level of retro Reasoning
* Building of the map of properties coming for speech information
*/
string abmReasoning::level3Sentences(int from)
{
    string bOutput;
    Bottle        bMessenger;

    ostringstream osRequest;
    // GETTING SENTENCES
    yInfo() << "\t" << "Getting sentences.";
    //int iError = 0;
    //check : simple object query :

    osRequest.str("");
    osRequest << "SELECT instance FROM main WHERE activitytype = 'sentence' or activityname = 'sentence' AND begin = true AND INSTANCE > " << from << " ORDER by instance";
    bMessenger = requestFromStream(osRequest.str().c_str());
    int numberSentence = bMessenger.size();

    vector<int> vErrorSentence;
    yInfo() << "\t" << "found " << numberSentence << " sentence(s)";
    pair<double, double> BEGIN,
        MOVE,
        END,
        BEGIN2,
        END2;

    for (int j = 0; j < numberSentence; j++)
    {
        cout << j + 1 << "..";
        int Id = atoi(bMessenger.get(j).asList()->get(0).toString().c_str());
        Bottle bSentence = askSentenceFromId(Id);

        //yInfo() << " Instance:" << Id << bSentence.toString();
        if (bSentence.toString() == abmReasoningFunction::TAG_NULL){
            yWarning() << " level3: error on sentence " << Id << ", wrong format in ABM";
            vErrorSentence.push_back(Id);
        }
        else
        {
            // add the GRAMMAR KNOWLEDGE:
            listGrammarKnowledge.addInteraction(*bSentence.get(1).asList());

            if (bSentence.size() == 4){

                Bottle bDeixis = *bSentence.get(0).asList();
                Bottle bName = *bSentence.get(1).asList();
                Bottle bAllRelationsBefore = *bSentence.get(2).asList();
                Bottle bAllRelationsAfter = *bSentence.get(3).asList();

                string sSpeaker = bDeixis.get(0).toString().c_str(),
                    sAddressee = bDeixis.get(1).toString().c_str(),
                    sSubject = bDeixis.get(2).toString().c_str();

                string sNameAct = bName.get(0).toString().c_str(),
                    sObjectAct = bName.get(2).toString().c_str(),
                    sRecipientAct = bName.get(3).toString().c_str();

                //yInfo() << "\t" << sSpeaker << sNameAct << sAddressee << sObjectAct;

                list<string> lRelationBefore;
                // EXTRACT OTHER RELATIONS BEFORE
                if (bAllRelationsBefore.toString().c_str() != abmReasoningFunction::TAG_NULL)
                {   // begin IF bAllRelationAfter is not Null

                    // for each relation not locative before the action:
                    for (int ii = 0; ii < bAllRelationsBefore.size(); ii++)
                    {
                        Bottle bTmpRelation = *bAllRelationsBefore.get(ii).asList();

                        //if the relation has enought component
                        if (bTmpRelation.size() >2){
                            string sAgentRel = bTmpRelation.get(0).toString();
                            string sActionRel = bTmpRelation.get(1).toString();
                            string sObjectRel = bTmpRelation.get(2).toString();
                            string sConcatenateRelation;

                            //yInfo() << "relation: " << sAgentRel << "," << sActionRel << "," << sObjectRel;

                            // check if the agent of the action is part of the action arguments
                            if (sAgentRel == sSpeaker){
                                sConcatenateRelation = abmReasoningFunction::TAG_SPEAKER;
                            }
                            else if (sAgentRel == sAddressee){
                                sConcatenateRelation = abmReasoningFunction::TAG_ADRESSEE;
                            }
                            else if (sAgentRel == sSubject){
                                sConcatenateRelation = abmReasoningFunction::TAG_SUBJECT;
                            }
                            else if (sAgentRel == sNameAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sAgentRel == sObjectAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sAgentRel == sRecipientAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else{
                                sConcatenateRelation = sAgentRel;
                            }

                            // add separator for relation in one string.
                            sConcatenateRelation += abmReasoningFunction::TAG_SEPARATOR;


                            // check if the verb of the action is part of the action arguments
                            if (sActionRel == sSpeaker){
                                sConcatenateRelation += abmReasoningFunction::TAG_SPEAKER;
                            }
                            else if (sActionRel == sAddressee){
                                sConcatenateRelation += abmReasoningFunction::TAG_ADRESSEE;
                            }
                            else if (sActionRel == sSubject){
                                sConcatenateRelation += abmReasoningFunction::TAG_SUBJECT;
                            }
                            else if (sActionRel == sNameAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sActionRel == sObjectAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sActionRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else {
                                sConcatenateRelation += sActionRel;
                            }

                            // add separator for relation in one string.
                            sConcatenateRelation += abmReasoningFunction::TAG_SEPARATOR;
                            // check if the verb of the action is part of the action arguments
                            if (sObjectRel == sSpeaker){
                                sConcatenateRelation += abmReasoningFunction::TAG_SPEAKER;
                            }
                            else if (sObjectRel == sAddressee){
                                sConcatenateRelation += abmReasoningFunction::TAG_ADRESSEE;
                            }
                            else if (sObjectRel == sSubject){
                                sConcatenateRelation += abmReasoningFunction::TAG_SUBJECT;
                            }
                            else if (sObjectRel == sNameAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sObjectRel == sObjectAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sObjectRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else {
                                sConcatenateRelation += sObjectRel;
                            }

                            //add the relation to the list of relations
                            lRelationBefore.push_back(sConcatenateRelation);
                        } //end if the relation has enought component
                    } // END for each relation not locative before the action:
                } // END OTHER RELATION BEFORE


                list<string> lRelationAfter;
                // OTHER RELATIONS AFTER
                if (bAllRelationsAfter.toString().c_str() != abmReasoningFunction::TAG_NULL)
                {   // begin IF bAllRelationAfter is not Null

                    // for each relation not locative before the action:
                    for (int ii = 0; ii < bAllRelationsAfter.size(); ii++)
                    {
                        Bottle bTmpRelation = *bAllRelationsAfter.get(ii).asList();

                        //if the relation has enought component
                        if (bTmpRelation.size() >2){
                            string sAgentRel = bTmpRelation.get(0).toString();
                            string sActionRel = bTmpRelation.get(1).toString();
                            string sObjectRel = bTmpRelation.get(2).toString();

                            string sConcatenateRelation;

                            // check if the agent of the action is part of the action arguments
                            if (sAgentRel == sSpeaker){
                                sConcatenateRelation = abmReasoningFunction::TAG_SPEAKER;
                            }
                            else if (sAgentRel == sAddressee){
                                sConcatenateRelation = abmReasoningFunction::TAG_ADRESSEE;
                            }
                            else if (sAgentRel == sSubject){
                                sConcatenateRelation = abmReasoningFunction::TAG_SUBJECT;
                            }
                            else if (sAgentRel == sNameAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sAgentRel == sObjectAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sAgentRel == sRecipientAct){
                                sConcatenateRelation = abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else{
                                sConcatenateRelation = sAgentRel;
                            }

                            // add separator for relation in one string.
                            sConcatenateRelation += abmReasoningFunction::TAG_SEPARATOR;


                            // check if the verb of the action is part of the action arguments
                            if (sActionRel == sSpeaker){
                                sConcatenateRelation += abmReasoningFunction::TAG_SPEAKER;
                            }
                            else if (sActionRel == sAddressee){
                                sConcatenateRelation += abmReasoningFunction::TAG_ADRESSEE;
                            }
                            else if (sActionRel == sSubject){
                                sConcatenateRelation += abmReasoningFunction::TAG_SUBJECT;
                            }
                            else if (sActionRel == sNameAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sActionRel == sObjectAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sActionRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else {
                                sConcatenateRelation += sActionRel;
                            }

                            // add separator for relation in one string.
                            sConcatenateRelation += abmReasoningFunction::TAG_SEPARATOR;
                            // check if the verb of the action is part of the action arguments
                            if (sObjectRel == sSpeaker){
                                sConcatenateRelation += abmReasoningFunction::TAG_SPEAKER;
                            }
                            else if (sObjectRel == sAddressee){
                                sConcatenateRelation += abmReasoningFunction::TAG_ADRESSEE;
                            }
                            else if (sObjectRel == sSubject){
                                sConcatenateRelation += abmReasoningFunction::TAG_SUBJECT;
                            }
                            else if (sObjectRel == sNameAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_ACTION;
                            }
                            else if (sObjectRel == sObjectAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_OBJECT;
                            }
                            else if (sObjectRel == sRecipientAct){
                                sConcatenateRelation += abmReasoningFunction::TAG_RECIPIENT;
                            }
                            else {
                                sConcatenateRelation += sObjectRel;
                            }

                            //add the relation to the list of relations
                            lRelationAfter.push_back(sConcatenateRelation);
                        } //end if the relation has enought component
                    } // END for each relation not locative before the action:
                } // END OTHER RELATIONS AFTER


                //  ADD AS CONTEXTUAL KNOWLEDGE
                createContextualKnowledge(sNameAct, sObjectAct, abmReasoningFunction::TAG_SENTENCE);
                // updating the contextual knowledge
                for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin(); itCK != listContextualKnowledge.end(); itCK++)
                {   //begin FOR itCK : listContextualKnowledge

                    if (itCK->sName == sNameAct && itCK->sArgument == sObjectAct && itCK->sType == abmReasoningFunction::TAG_SENTENCE)
                    {   // IF CK found

                        itCK->iOccurence++;
                        bool bFound = false;
                        // add all relations before:
                        for (list<string>::iterator itS = lRelationBefore.begin();
                            itS != lRelationBefore.end();
                            itS++){
                            bFound = false;
                            // finding the corresponding relation in the CK
                            for (vector<pair<string, int> >::iterator itM = itCK->mRelationBefore.begin();
                                itM != itCK->mRelationBefore.end(); itM++) {
                                if (!bFound && itM->first == *(itS))
                                {
                                    itM->second++;
                                    bFound = true;
                                }
                            }

                            // if the relation doesn't exist in the CK
                            if (!bFound){
                                pair<string, int> tmp(*itS, 1);
                                itCK->mRelationBefore.push_back(tmp);
                            }
                        }// END ADD ALL RELATIONS BEFORE

                        bFound = false;

                        // add all relations AFTER:
                        for (list<string>::iterator itS = lRelationAfter.begin();
                            itS != lRelationAfter.end();
                            itS++){

                            bFound = false;
                            // finding the corresponding relation in the CK
                            for (vector<pair<string, int> >::iterator itM = itCK->mRelationAfter.begin();
                                itM != itCK->mRelationAfter.end(); itM++) {
                                if (!bFound && itM->first == *(itS))
                                {
                                    itM->second++;
                                    bFound = true;
                                }
                            }

                            // if the relation doesn't exist in the CK
                            if (!bFound){
                                pair<string, int> tmp(*itS, 1);
                                itCK->mRelationAfter.push_back(tmp);
                            }
                        }// END ADD ALL RELATIONS AFTER
                    }   // end IF CK found
                }   //end FOR itCK : listContextualKnowledge
            }
        }
    }


    ostringstream osErrorMessage;
    osErrorMessage << "\n" << "level3 finished with " << vErrorSentence.size() << " sentence errors." << endl;
    if (vErrorSentence.size() != 0)
    {
        osErrorMessage << "Sentence Instances errors are ";
        for (vector<int>::iterator itErr = vErrorSentence.begin(); itErr != vErrorSentence.end(); itErr++)
        {
            osErrorMessage << ", " << *itErr;
        }
        osErrorMessage << ".";
    }

    string sErrorMessage = osErrorMessage.str();

    //yInfo() << "\t " << sErrorMessage;

    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin(); itCK != listContextualKnowledge.end(); itCK++)
    {
        itCK->updatePresence();
    }

    bOutput = sErrorMessage;
    return bOutput;
}


Bottle abmReasoning::level3Reasoning(int from)
{

    listContextualKnowledge.clear();
    Bottle bOutput;

    string sAct = level3Actions(from);
    string sSent = level3Sentences(from);

    bOutput.addString(sAct);
    bOutput.addString(sSent);

    yInfo() << "\n \n" << sAct << "\n" << sSent << "\n";

    return bOutput;
}


/////             DISPLAY FUNCTION

/**
* Display the result of a sql Query on the console
* @param bInput : Bottle to display
*/
void abmReasoning::displayResult(Bottle bInput)
{
    //print the result of the query, decompose elements/column
    yInfo() << "\t" << "query answer : " << bInput.toString();
    for (int i = 0; i < bInput.size(); i++){
        yInfo() << "\t" << "***************************************";
        //yInfo() << "\t" << "opc instance  " << bOpcId.get(i).asList()->get(0).asString() << " : " << bOutput.get(i).toString()  ;
        // yInfo() << "\t" << "opc instance  " << bOpcId.get(i).asList()->get(0).asString() << " : "  ;
        if (bInput.get(i).toString() != "NULL") {
            for (int j = 0; j < bInput.get(i).asList()->size(); j++){
                yInfo() << "\t" << "---" << opcNameTable[j] << " : " << bInput.get(i).asList()->get(j).toString();
            }
        }
    }
}


/*
* Internal function
* display the content of listSharedPlan
*/
void abmReasoning::displaySharedPlan()
{

    for (vector<sharedPlan>::iterator it_SP = listSharedPlan.begin(); it_SP != listSharedPlan.end(); it_SP++)
    {
        yInfo() << "\t" << "Shared plan : name =  " << it_SP->sName << " and manner = " << it_SP->sManner;

        for (vector < pair <plan, int > >::iterator it_Plan = it_SP->listPlanPossible.begin(); it_Plan != it_SP->listPlanPossible.end(); it_Plan++)
        {
            yInfo() << "\t" << "\tPlan : name = " << (it_Plan->first).sName << " and manner = " << it_Plan->first.sManner << ". score = " << it_Plan->second;
        }
    }
}


void abmReasoning::displayContextual(string sInput, string sArgument, string sType)
{

    bool bFound = false;
    for (vector<contextualKnowledge>::iterator itCK = listContextualKnowledge.begin();
        itCK != listContextualKnowledge.end();
        itCK++)
    {
        if (!bFound)
        {
            if (itCK->sName == sInput && itCK->sArgument == sArgument && itCK->sType == sType)
            {
                bFound = true;

                cout << endl << "\t" << sInput << ":" << endl;
                cout << "\t\t" << "sArgument   is: " << itCK->sArgument << endl;
                cout << "\t\t" << "sDependance is: " << itCK->sDependance << endl;
                cout << "\t\t" << "sType is      : " << itCK->sType << endl;

                itCK->updateAgentRelated();
                itCK->updateIntersect();
                itCK->updatePresence();

                cout << "\t\t" << "Percent Presence: " << itCK->PercentPresence.first << "\t" << itCK->PercentPresence.second << endl;
                cout << "\t\t" << "Percent Location: " << endl;
                for (map<string, pair<double, double > >::iterator itMap = itCK->mPercentIntersectLocation.begin();
                    itMap != itCK->mPercentIntersectLocation.end();
                    itMap++)
                {
                    cout << "\t\t\t" << itMap->first << ": " << itMap->second.first << " ; " << itMap->second.second << endl;
                }
                cout << "\t\t" << "Percent Obj from to : " << endl;
                for (map<string, pair<double, double > >::iterator itMap = itCK->mPercentObjectFromTo.begin();
                    itMap != itCK->mPercentObjectFromTo.end();
                    itMap++)
                {
                    cout << "\t\t\t" << itMap->first << ": " << itMap->second.first << " ; " << itMap->second.second << endl;
                }
                cout << "\t\t" << "Percent Agent : " << endl;
                for (map<string, double>::iterator itMap = itCK->mPercentAgentRelated.begin();
                    itMap != itCK->mPercentAgentRelated.end();
                    itMap++)
                {
                    cout << "\t\t\t" << itMap->first << ": " << itMap->second << endl;
                }
                cout << "\t\t" << "Global Relation Before: " << endl;
                for (vector<pair<string, int>>::iterator itMap = itCK->mRelationBefore.begin();
                    itMap != itCK->mRelationBefore.end();
                    itMap++)
                {
                    cout << "\t\t\t" << itMap->first << ": " << itMap->second << endl;
                }
                cout << "\t\t" << "Global Relation After: " << endl;
                for (vector<pair<string, int>>::iterator itMap = itCK->mRelationAfter.begin();
                    itMap != itCK->mRelationAfter.end();
                    itMap++)
                {
                    cout << "\t\t\t" << itMap->first << ": " << itMap->second << endl;
                }
            }
        }
    }

    if (!bFound)
    {
        yInfo() << " impossible to find contextualKnowledge " << sInput << ".";
    }

}


/*
* imagine the state of the OPC according to its ID in the mentalOPC
*
*/
Bottle abmReasoning::imagineOPC(int Id)
{
    Bottle bOutput,
        bMessenger;

    mentalOPC->isVerbose = false;

    //    yInfo() << "\t" << "in imagination"  ;

    if (!mentalOPC->isConnected())
    {
        yInfo() << "\t" << "Problem in abmReasoning::imagineOPC | mentalOPC not connected";
        bOutput.addString("Problem in abmReasoning::imagineOPC | mentalOPC not connected");
        return bOutput;
    }


    mentalOPC->checkout();

    //clean GUI :
    list<Entity*> lMental = mentalOPC->EntitiesCache();
    for (list<Entity*>::iterator it_E = lMental.begin(); it_E != lMental.end(); it_E++)
    {
        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_OBJECT)   {
            Object *Ob = dynamic_cast<Object*>(*it_E);
            Ob->m_present = 0;
        }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_AGENT)    {
            Agent *Ag = dynamic_cast<Agent*>(*it_E);
            Ag->m_present = 0;
        }

        if ((*it_E)->entity_type() == EFAA_OPC_ENTITY_RTOBJECT) {
            RTObject *Rt = dynamic_cast<RTObject*>(*it_E);
            Rt->m_present = 0;
        }
    }
    mentalOPC->commit();
    Time::delay(0.1);
    mentalOPC->clear();
    mentalOPC->checkout();

    // ADD Agent iCub
    Agent *icub = mentalOPC->addEntity<Agent>("icub");
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

        RTObject *RTOtemp = mentalOPC->addOrRetrieveEntity<RTObject>(sName);

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


/*
* send the command to OPCManager to update the beliefs of one opc
* true will update the real opc, and false, the mental
*/
Bottle abmReasoning::updateBeliefs(bool bOPC)
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


/*
*   Send the relation in the mentalOPC to the ABM at a given instance
*   input : instance
*/
Bottle abmReasoning::sendRelation(int instance)
{
    Bottle bOutput;
    list<Relation>  lRelations = mentalOPC->getRelations();
    int iNbRel = getNumberRelation(instance);
    for (list<Relation>::iterator it_R = lRelations.begin(); it_R != lRelations.end(); it_R++)
    {
        Bottle bTemp;
        ostringstream osRelation;

        osRelation << "INSERT INTO contentopc( type , instance , opcid , subtype) VALUES ('relation', " << instance << " , " << iNbRel + 1 << " , 'relation') ; ";
        osRelation << "INSERT INTO relation( opcid, instance, subject, verb, object, time, manner, place ) VALUES ";
        osRelation << " ( " << iNbRel + 1 << " , " << instance << " , '" << it_R->subject().c_str() << "' , '" << it_R->verb().c_str() << "' , '" << it_R->object().c_str() << "' , '" << it_R->complement_time().c_str() << "' , '" << it_R->complement_manner().c_str() << "' , '" << it_R->complement_place().c_str() << "' ) ";
        bTemp = requestFromStream(osRelation.str());
        iNbRel++;
        bOutput.addList() = bTemp;
    }

    return bOutput;
}


/**
*   Get the number that a relation should take at minimum for a given instance
*
*/
int abmReasoning::getNumberRelation(int instance)
{
    ostringstream osRequest;
    osRequest << "SELECT opcid FROM contentopc WHERE instance = " << instance << " ORDER BY opcid DESC LIMIT 1";

    Bottle bRequest = requestFromStream(osRequest.str());

    int iOutput = 0;

    if (bRequest.toString().c_str() != abmReasoningFunction::TAG_NULL)
    {
        iOutput += atoi(bRequest.get(0).asList()->get(0).toString().c_str());
    }

    return iOutput;
}


void abmReasoning::setMentalOPC(int instance)
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
    yInfo() << "\t" << "All entities are : \n\t\t" << bAllEntity.toString();

    int iNbEntotyError = 0;

    // FOR EACH ENTITY :

    for (int iEnt = 0; iEnt < bAllEntity.size(); iEnt++)
    {
        bCurrentEntity = *bAllEntity.get(iEnt).asList();
        yInfo() << "\t" << "bCurrentEntity : " << bCurrentEntity.toString() << "\t size : " << bCurrentEntity.size();
        // check size of bottle
        if (bCurrentEntity.size() == 3)
        {

        }
        // error : 
        else
        {
            yInfo() << "\t" << "Error in abmReasoning::setMentalOPC | problem with currentEntity.size() ";
            iNbEntotyError++;
        }


    }


    // GET the Agents : 

    int opcIDofAgent = 2;

    osAgent << "SELECT name, position, orientation, color, presence FROM agent WHERE instance = " << instance << " AND opcid = " << opcIDofAgent;

}

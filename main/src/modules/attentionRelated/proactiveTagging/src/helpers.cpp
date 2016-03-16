/* 
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau, Tobias Fischer, Maxime Petit
 * email:   greg.pointeau@gmail.com, t.fischer@imperial.ac.uk, m.petit@imperial.ac.uk
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

#include "proactiveTagging.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;
using namespace std;

/*
*   Get the context path of a .grxml grammar, and return it as a string
*
*/
string proactiveTagging::grammarToString(string sPath)
{
    string sOutput = "";
    ifstream isGrammar(sPath.c_str());

    cout << "path is: " << sPath << endl;

    if (!isGrammar)
    {
        cout << "Error in proactiveTagging::grammarToString. Couldn't open file : " << sPath << "." << endl;
        return "Error in proactiveTagging::grammarToString. Couldn't open file";
    }

    string sLine;
    while (getline(isGrammar, sLine))
    {
        sOutput += sLine;
        sOutput += "\n";
    }

    return sOutput;
}

void proactiveTagging::subPopulateObjects(Bottle* objectList, bool addOrRetrieve) {
    if (objectList)
    {
        for (int d = 0; d < objectList->size(); d++)
        {
            std::string name = objectList->get(d).asString().c_str();
            wysiwyd::wrdac::Object* o;
            if(addOrRetrieve) {
                o = iCub->opc->addOrRetrieveEntity<Object>(name);
            } else {
                o = iCub->opc->addEntity<Object>(name);
            }
            yInfo() << " [configureOPC] object " << o->name() << "added" ;
            o->m_present = 0.0;
            iCub->opc->commit(o);
        }
    }
}

void proactiveTagging::subPopulateBodyparts(Bottle* bodyPartList, Bottle* bodyPartJointList, bool addOrRetrieve) {
    if (bodyPartList)
    {
        for (int d = 0; d < bodyPartList->size(); d++)
        {
            std::string name = bodyPartList->get(d).asString().c_str();
            wysiwyd::wrdac::Bodypart* o;
            if(addOrRetrieve) {
                o = iCub->opc->addOrRetrieveEntity<Bodypart>(name);
            } else {
                o = iCub->opc->addEntity<Bodypart>(name);
            }
            yInfo() << " [configureOPC] Bodypart " << o->name() << "added";
            o->m_present = 0.0;
            //apply the joint number if available. protect for the loop because using d from bodyPartList. should be same number of element between bodyPartList and bodyPartJointList
            if(d < bodyPartJointList->size()){
                o->m_joint_number = bodyPartJointList->get(d).asInt();
                yInfo() << " [configureOPC] Bodypart " << o->name() << " has now a joint " << o->m_joint_number ;
            }
            iCub->opc->commit(o);
        }
    }
}

void proactiveTagging::configureOPC(yarp::os::ResourceFinder &rf)
{
    //Populate the OPC if required
    std::cout << "Populating OPC...";

    //1. Populate AddOrRetrieve part
    Bottle grpOPC_AOR = rf.findGroup("OPC_AddOrRetrieve");
    bool shouldPopulate_AOR = grpOPC_AOR.find("populateOPC").asInt() == 1;
    if (shouldPopulate_AOR)
    {
        Bottle *objectList = grpOPC_AOR.find("objectName").asList();
        subPopulateObjects(objectList, true);

        Bottle *bodyPartList = grpOPC_Add.find("bodypartName").asList();
        Bottle *bodyPartJointList = grpOPC_Add.find("bodypartJoint").asList();
        subPopulateBodyparts(bodyPartList, bodyPartJointList, true);
    }

    //2. Populate Add part (allows several object with same base name, e.g. object, object_1, object_2, ..., object_n)
    Bottle grpOPC_Add = rf.findGroup("OPC_Add");
    bool shouldPopulate_Add = grpOPC_Add.find("populateOPC").asInt() == 1;
    if (shouldPopulate_Add)
    {
        Bottle *objectList = grpOPC_Add.find("objectName").asList();
        subPopulateObjects(objectList, false);

        Bottle *bodyPartList = grpOPC_Add.find("bodypartName").asList();
        Bottle *bodyPartJointList = grpOPC_Add.find("bodypartJoint").asList();
        subPopulateBodyparts(bodyPartList, bodyPartJointList, false);
    }

    std::cout << "done" << endl;
}

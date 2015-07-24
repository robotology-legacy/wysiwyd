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

void proactiveTagging::configureOPC(yarp::os::ResourceFinder &rf)
{
    //Populate the OPC if required
    std::cout << "Populating OPC";

    //1. Populate AddOrRetrieve part
    Bottle grpOPC_AOR = rf.findGroup("OPC_AddOrRetrieve");
    bool shouldPopulate_AOR = grpOPC_AOR.find("populateOPC").asInt() == 1;
    if (shouldPopulate_AOR)
    {
        Bottle *objectList = grpOPC_AOR.find("object").asList();
        if (objectList)
        {
            for (int d = 0; d < objectList->size(); d++)
            {
                std::string name = objectList->get(d).asString().c_str();
                wysiwyd::wrdac::Object* o = iCub->opc->addOrRetrieveObject(name);
                yInfo() << " [configureOPC] object " << o->name() << "added" ;
                o->m_present = false;
                iCub->opc->commit(o);
            }
        }
    }

    //2. Populate Add part (allows several object with same base name, e.g. object, object_1, object_2, ..., object_n)
    Bottle grpOPC_Add = rf.findGroup("OPC_Add");
    bool shouldPopulate_Add = grpOPC_Add.find("populateOPC").asInt() == 1;
    if (shouldPopulate_Add)
    {
        Bottle *objectList = grpOPC_Add.find("object").asList();
        if (objectList)
        {
            for (int d = 0; d < objectList->size(); d++)
            {
                std::string name = objectList->get(d).asString().c_str();
                wysiwyd::wrdac::Object* o = iCub->opc->addObject(name);
                yInfo() << " [configureOPC] object " << o->name() << "added" ;
                o->m_present = false;
                iCub->opc->commit(o);
            }
        }
    }

    std::cout << "done" << endl;
}
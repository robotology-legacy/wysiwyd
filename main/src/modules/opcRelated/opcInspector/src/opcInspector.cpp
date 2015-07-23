/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com
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

#include <string>
#include <iostream>
#include <iomanip>
#include <math.h>

#include "opcInspector.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace wysiwyd::wrdac;


bool OpcInspector::configure(yarp::os::ResourceFinder &rf)
{
    /* initialize random generator */
    srand ( time(NULL) );
    string name = rf.check("name",Value("opcInspector")).asString().c_str();
    string opcName = rf.check("opcName",Value("OPC")).asString().c_str();
    setName(name.c_str());

    //Create an iCub Client and check that all dependencies are here before starting
    opc = new OPCClient(name);
    opc->isVerbose = false;

    //Open the reader
    rpc.open("/"+name+"/rpc");
    attach(rpc);

    while (!opc->connect(opcName))
    {
        cout<<"OPC is not running yet..."<<endl;
        Time::delay(1.0);
    }
    cout<<"Connected, starting.."<<endl;

    opc->checkout();

    cout<<"OPC content : "<<endl
        <<opc->print()<<endl;

    cout<<"Up and running"<<endl;
    cout<<"Format is : relation [add/rem/check] subject verb object"<<endl;
    
    return true;
}

bool OpcInspector::updateModule()
{
        //opc->checkout();
        
        return true;
}

bool OpcInspector::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    string key1 = command.get(0).asString().c_str();
    if( key1 == "relation")
    {
        string key2 = command.get(1).asString().c_str();
        if( key2 == "add")
        {
            string subject = command.get(2).asString().c_str();
            string verb = command.get(3).asString().c_str();
            string object = command.get(4).asString().c_str();
            string ccplace = "none";
            string cctime = "none";
            string ccmanner = "none";
            if (command.size()>5)
                ccplace = command.get(5).asString().c_str();
            if (command.size()>6)
                cctime = command.get(6).asString().c_str();
            if (command.size()>7)
                ccmanner = command.get(7).asString().c_str();

            if(opc->getEntity(subject) == NULL)
            {
                reply.addString("Warning: " + subject + "not present in OPC. Created as an adjective");
                opc->addAdjective(subject);
            }
            if(opc->getEntity(verb) == NULL)
            {
                reply.addString("Warning: " + verb + "not present in OPC. Created as an action");
                opc->addAction(verb);
            }
            if(opc->getEntity(object) == NULL)
            {
                reply.addString("Warning: " + object + "not present in OPC. Created as an adjective");
                opc->addAdjective(object);
            }

            if(ccplace!= "none" && opc->getEntity(ccplace) == NULL)
            {
                reply.addString("Warning: " + ccplace + "not present in OPC. Created as an adjective");
                opc->addAdjective(ccplace);
            }
            if(cctime!= "none" && opc->getEntity(cctime) == NULL)
            {
                reply.addString("Warning: " + cctime + "not present in OPC. Created as an adjective");
                opc->addAdjective(cctime);
            }
            if(ccmanner!= "none" && opc->getEntity(ccmanner) == NULL)
            {
                reply.addString("Warning: " + ccmanner + "not present in OPC. Created as an adjective");
                opc->addAdjective(ccmanner);
            }
            Relation r(subject,verb,object,ccplace,cctime,ccmanner);
            opc->addRelation(r);
            reply.addString("Relation addded");
        }
        if( key2 == "rem")
        {
            string subject = command.get(2).asString().c_str();
            string verb = command.get(3).asString().c_str();
            string object = command.get(4).asString().c_str();
            string ccplace = "none";
            string cctime = "none";
            string ccmanner = "none";
            if (command.size()>5)
                ccplace = command.get(5).asString().c_str();
            if (command.size()>6)
                cctime = command.get(6).asString().c_str();
            if (command.size()>7)
                ccmanner = command.get(7).asString().c_str();

            if(opc->getEntity(subject) == NULL)
            {
                reply.addString("Warning: " + subject + "not present in OPC. Created as an adjective");
                opc->addAdjective(subject);
            }
            if(opc->getEntity(verb) == NULL)
            {
                reply.addString("Warning: " + verb + "not present in OPC. Created as an action");
                opc->addAction(verb);
            }
            if(opc->getEntity(object) == NULL)
            {
                reply.addString("Warning: " + object + "not present in OPC. Created as an adjective");
                opc->addAdjective(object);
            }

            if(ccplace!= "none" && opc->getEntity(ccplace) == NULL)
            {
                reply.addString("Warning: " + ccplace + "not present in OPC. Created as an adjective");
                opc->addAdjective(ccplace);
            }
            if(cctime!= "none" && opc->getEntity(cctime) == NULL)
            {
                reply.addString("Warning: " + cctime + "not present in OPC. Created as an adjective");
                opc->addAdjective(cctime);
            }
            if(ccmanner!= "none" && opc->getEntity(ccmanner) == NULL)
            {
                reply.addString("Warning: " + ccmanner + "not present in OPC. Created as an adjective");
                opc->addAdjective(ccmanner);
            }

            Relation r(subject,verb,object,ccplace,cctime,ccmanner);
            opc->removeRelation(r);
            reply.addString("Relation removed");
        }       
        if( key2 == "check")
        {
            string subject = command.get(2).asString().c_str();
            string verb = command.get(3).asString().c_str();
            string object = command.get(4).asString().c_str();
            string ccplace = "any";
            string cctime = "any";
            string ccmanner = "any";
            if (command.size()>5)
                ccplace = command.get(5).asString().c_str();
            if (command.size()>6)
                cctime = command.get(6).asString().c_str();
            if (command.size()>7)
                ccmanner = command.get(7).asString().c_str();

            if(opc->getEntity(subject) == NULL)
            {
                reply.addString("Warning: " + subject + "not present in OPC. Created as an adjective");
                opc->addAdjective(subject);
            }
            if(opc->getEntity(verb) == NULL)
            {
                reply.addString("Warning: " + verb + "not present in OPC. Created as an action");
                opc->addAction(verb);
            }
            if(opc->getEntity(object) == NULL)
            {
                reply.addString("Warning: " + object + "not present in OPC. Created as an adjective");
                opc->addAdjective(object);
            }

            if(ccplace!= "any" && opc->getEntity(ccplace) == NULL)
            {
                reply.addString("Warning: " + ccplace + "not present in OPC. Created as an adjective");
                opc->addAdjective(ccplace);
            }
            if(cctime!= "any" && opc->getEntity(cctime) == NULL)
            {
                reply.addString("Warning: " + cctime + "not present in OPC. Created as an adjective");
                opc->addAdjective(cctime);
            }
            if(ccmanner!= "any" && opc->getEntity(ccmanner) == NULL)
            {
                reply.addString("Warning: " + ccmanner + "not present in OPC. Created as an adjective");
                opc->addAdjective(ccmanner);
            }

            Relation r(subject,verb,object,ccplace,cctime,ccmanner);
            bool result = opc->containsRelation(r);
            if (result)
                reply.addString("OPC contains " + r.toString());
            else
                reply.addString("OPC does not contain " + r.toString());
        }
    }
    if( key1 == "object")
    {
        string key2 = command.get(1).asString().c_str();
        string objectName = command.get(2).asString().c_str();
        Object* objPtr = dynamic_cast<Object*>(opc->getEntity(objectName));
        if( objPtr == NULL)
        {
            reply.addString("Warning: " + objectName + "not present in OPC. Created as a generic object");
            objPtr = opc->addObject(objectName);
        }
        if( key2 == "display")
        {
            reply.addString(objPtr->toString());
        }
        if( key2 == "show")
        {
            objPtr->m_present = true;
        opc->commit(objPtr);
        }
        if( key2 == "hide")
        {
            objPtr->m_present = false;
        opc->commit(objPtr);
        }
    }


    return true;
}



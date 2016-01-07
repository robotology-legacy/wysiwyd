/*
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
 * website: http://wysiwyd.upf.edu/
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

#include "touchingOrder.h"

void TouchingOrder::configure() {
    // Todo: set the value beow from a config file (but we are not in a module here)
    name = "touchingOrder";
    external_port_name = "/proactiveTagging/rpc";
    from_sensation_port_name = "/ears/target:o";
}

void TouchingOrder::run(Bottle args/*=Bottle()*/) {
    yInfo() << "TouchingOrder::run";

    string type = sensation_port_in.read()->get(0).asString();
    string target = sensation_port_in.read()->get(1).asString();
    yInfo() << target;
    if (target != "none"){
        handleSearch(type, target);
        handleTouch(type, target);
    }
}

bool TouchingOrder::handleTouch(string type, string target)
{
    // Point an object (from human order). Independent of proactivetagging
    iCub->opc->checkout();
    yInfo() << " [handleTouch]: opc checkout";

    Entity* e = iCub->opc->getEntity(target);
    if(e!=NULL) {
        if(e->entity_type() == "bodypart") {
            iCub->say("I am going to move my " + target);
            Bodypart* b = dynamic_cast<Bodypart*>(e);
            //TODO: iCub->moveJoint(b->m_joint_number)
        }
    }

    return true;
}

bool TouchingOrder::handleSearch(string type, string target)
{
    // look if the object (from human order) exist and if not, trigger proactivetagging

    iCub->opc->checkout();
    yInfo() << " [handleSearch] : opc checkout";

    Object *o = dynamic_cast<Object*>(iCub->opc->getEntity(target));
    if(o!=NULL) {
        yInfo() << "I found the entity in the opc: " << target << " and thus I'll leave handleSearch";
        return true;
    }

    yInfo() << "I need to call proactiveTagging!";// << endl;

    //If there is an unknown object (to see with agents and rtobjects), add it to the rpc_command bottle, and return true
    Bottle cmd;
    Bottle rply;

    cmd.addString("searchingEntity");
    cmd.addString(type);
    cmd.addString(target);
    yDebug() << "Send to proactiveTagging: " << cmd.toString();
    rpc_out_port.write(cmd,rply);
    yDebug() << rply.toString(); //<< endl;

    return true;
}

/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Ilaria Gori and Maxime Petit
 * email:   ilaria.gori@iit.it, maxime.petit@inserm.fr
 * website: http://efaa.upf.edu/ 
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


#include <wrdac/knowledge/action.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

Action::Action():Entity()
{
    m_entity_type = EFAA_OPC_ENTITY_ACTION;
}

Action::Action(const Action &b):Entity(b)
{
    this->initialDescription = b.initialDescription;
    this->subActions = b.subActions;
    this->estimatedDriveEffects = b.estimatedDriveEffects;
}


Bottle Action::asBottle()
{
    Bottle b = this->Entity::asBottle();
    Bottle bSub;
    bSub.addString("description");
    bSub.addList() = initialDescription.asBottle();
    b.addList() = bSub;
    bSub.clear();
    bSub.addString("subactions");
    Bottle& subs = bSub.addList();
    for(list<Action>::iterator sIt = subActions.begin() ; sIt != subActions.end(); sIt++)
    {
        subs.addList()=sIt->asBottle();
    }
    b.addList() = bSub;
        
    bSub.clear();
    bSub.addString("estimatedDriveEffects");
    Bottle &subss = bSub.addList();
    for(map<string, double>::iterator sIt = estimatedDriveEffects.begin() ; sIt != estimatedDriveEffects.end(); sIt++)
    {
        Bottle &ss = subss.addList();
        ss.addString(sIt->first.c_str());
        ss.addDouble(sIt->second);
    }
    b.addList() = bSub;
    return b;
}

bool Action::fromBottle(const Bottle &b)
{
    if (!this->Entity::fromBottle(b))
        return false;

    if (!b.check("description")||!b.check("subactions"))
        return false;

    Bottle* bDesc = b.find("description").asList();
    initialDescription.fromBottle(*bDesc);

    this->subActions.clear();
    Bottle* bSub = b.find("subactions").asList();
    for(int i=0; i<bSub->size(); i++)
    {
        Action a;
        a.fromBottle(*bSub->get(i).asList());
        this->subActions.push_back(a);
    }

    this->estimatedDriveEffects.clear();
    bSub = b.find("estimatedDriveEffects").asList();
    for(int i=0; i<bSub->size(); i++)
    {
        string driveName = bSub->get(i).asList()->get(0).asString().c_str();
        double driveEffect = bSub->get(i).asList()->get(1).asDouble();
        this->estimatedDriveEffects[driveName] = driveEffect;
    }
    return true;
}



Relation mapRelation(map<string,string> dico, Relation child, Relation current)
{

    string newSubject = child.subject();
    map<string,string>::iterator place = dico.find(newSubject);
        if (place != dico.end() )
            newSubject = place->second;

        string newObject =child.object();
        place = dico.find(newObject);
        if (place != dico.end() )
            newObject = place->second;

        string newVerb = child.verb();
        //place = dico.find(newVerb);
        //if (place != dico.end() )
        //    newVerb = place->second;
        
        string newCTime = child.complement_time();
        place = dico.find(newCTime);
        if (place != dico.end() )
            newCTime = place->second;

        string newCManner = child.complement_manner();
        place = dico.find(newCManner);
        if (place != dico.end() )
            newCManner = place->second;

        string newCPlace = child.complement_place();
        place = dico.find(newCPlace);
        if (place != dico.end() )
            newCPlace = place->second;

        return Relation(newSubject,newVerb,newObject,newCPlace,newCTime,newCManner);
}

void Action::setInitialDescription(Relation r)
{
    //for(list<Action>::iterator sub=subActions.begin(); sub!=subActions.end();sub++)
    //{
    //    sub->setInitialDescription(mapRelation(sub->initialDescription,r));
    //}
    this->initialDescription.fromBottle(r.asBottle());
}

Action Action::express(Relation r)
{
    Action a;
    string initial = this->asBottle().toString().c_str();
    string final = initial.c_str();
    replace_all(final,this->initialDescription.subject().c_str(),"___subject");
    replace_all(final,this->initialDescription.object().c_str(),"___object");
    replace_all(final,this->initialDescription.complement_time().c_str(),"___compTime");
    replace_all(final,this->initialDescription.complement_place().c_str(),"___compPlace");
    replace_all(final,this->initialDescription.complement_manner().c_str(),"___compManner");
    replace_all(final,"___subject", r.subject());
    replace_all(final,"___object", r.object());
    replace_all(final,"___compTime", r.complement_time());
    replace_all(final,"___compPlace", r.complement_place());
    replace_all(final,"___compManner", r.complement_manner());
    Bottle b(final.c_str());
    a.fromBottle(b);
    return a;
}

Relation Action::description()
{
    Relation r;
    r.fromBottle(this->initialDescription.asBottle());
    return r;
}

void Action::append(Action a)
{
    subActions.push_back(a);
}

list<Action> Action::asPlan()
{
    list<Action> unrolled;
    if (this->subActions.size() == 0)
        unrolled.push_back(*this);
    else
        for(list<Action>::iterator it = subActions.begin(); it != subActions.end() ; it++)
        {
            list<Action> subUnrolled = it->asPlan();
            unrolled.splice(unrolled.end(), subUnrolled);
        }
    return unrolled;
}

list<Action> Action::asPlan(Relation newDescription)
{            
    Action expressed = this->express(newDescription);
    list<Action> unrolled = expressed.asPlan();
    return unrolled;
}

void Action::getPlanDrivesEffect(map<string,double> &driveEffects)
{
    //Added the effect of the current plan
    for(map<string, double>::iterator sIt = estimatedDriveEffects.begin() ; sIt != estimatedDriveEffects.end(); sIt++)
    {
        driveEffects[sIt->first] += sIt->second;
    }

    //Added recursively the effects of subplans
    for(list<Action>::iterator it = subActions.begin(); it != subActions.end() ; it++)
    {
        it->getPlanDrivesEffect(driveEffects);
    }
}
string Action::toString()
{   
    return toString(this->initialDescription);
}

string Action::toString(Relation newRelation)
{        
    std::ostringstream oss;
    oss<<"Unrolling: "<<newRelation.toString()<<endl;
    list<Action> unrolled = this->asPlan(newRelation);
    int count=0;
    for(list<Action>::iterator it = unrolled.begin() ; it!=unrolled.end(); it++,count++)
    {
        oss<<'\t'<<count<<")"<<it->description().toString()<<endl;
    }
    return oss.str();
}





/* 
* Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Gori Ilaria and Petit Maxime
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

#ifndef __EFAA_ACTION_H__
#define __EFAA_ACTION_H__

#include "entity.h"
#include "relation.h"
#include "wrdac/functions.h"

namespace wysiwyd{namespace wrdac{

    /**
    * \ingroup wrdac_representations
    *
    * Represent an action, composite or not. Stores as well the expected impact on the drives.
    */
    class Action:public Entity
    {friend class OPCClient;

    /*
    * The description used when the action was teached
    */
    Relation initialDescription;
        
    /*
    * Subactions that may compose a composite one
    */
    std::list<Action> subActions;



    public:
        /*
        * Estimated effects on the drive
        */
        std::map<std::string, double> estimatedDriveEffects;

        Action();
        Action(const Action &b);

        virtual bool    isType(std::string _entityType)
        {
            if (_entityType == EFAA_OPC_ENTITY_ACTION)
                return true;
            else
                return this->Entity::isType(_entityType);
        }
                        void    setInitialDescription(Relation r);
                    Relation    description();
                    Action      express(Relation r);


        /**
        * Append a subaction to create a composite one
        * @param a The subaction
        */ 
                        void    append(Action a);
        /**
        * Get an unrolled plan description
        */ 
        virtual std::list<Action>    asPlan();

        /**
        * Get an unrolled plan description based on new arguments
        * @param newDescription The new arguments
        */ 
        virtual std::list<Action>    asPlan(Relation newDescription);
                
        /**
        * Is an action composite or not?
        */ 
        virtual bool            isComposite(){return subActions.size();}
        /**
        * Number of subactions composing this one
        */ 
                int              size(){return subActions.size();}
        virtual yarp::os::Bottle asBottle();
        virtual bool             fromBottle(yarp::os::Bottle b);
        virtual std::string      toString();
                
        /**
        * Estimate the effect of this plan on a specific drive, by summing all the effects of
        * subplans to this specific plan effect.
        */ 
        virtual void          getPlanDrivesEffect(std::map<std::string,double> &driveEffects);

        /**
        * Gives the string according to a current situation
        * @param newRelation The new arguments
        */ 
        virtual std::string   toString(Relation newRelation);
    };

}}//Namespaces

#endif


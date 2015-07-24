#ifndef __EFAA_OPCEARS_H__
#define __EFAA_OPCEARS_H__
/* 
* Copyright (C) 2012 EFAA Consortium, European Commission FP7 Project IST-270490
* Authors: Grégoire Pointeau
* email:   gregoire.pointeau@inserm.fr
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

/**
 * @defgroup wysiwyd_opcears OPC Ears
 *  
 * @ingroup wysiwyd_libraries
 *  
 * Tool for operations on the OPC as : transforming in SQL request for the integration in the DataBase, difference between 2 OPCs...
 * Contact : gregoire.pointeau@inserm.fr
 * @author Grégoire Pointeau
 */ 
#include <wrdac/clients/opcSave.h>


namespace wysiwyd{namespace wrdac{
    /**
    * \ingroup efaa_opcears
    *
    * Main module.
    * @author Grégoire Pointeau
    */

    class opcEars {

    private:

        OPCClient *opcWorld;    // relation to the OPC
        int instance;

    public:
        opcEars() {}
        ~opcEars() {
            for(std::map<std::string, opcSave*>::iterator it = mSave.begin(); it != mSave.end(); ++it) {
                delete it->second;
            }
        }

        /**
        * Set the instance of OPC
        * @param i the instance
        */
        void setInstance(int i) {instance = i;};

        /**
        * Map of the different OPC saved during the run.
        */
        std::map<std::string, opcSave*>   mSave;  // saves of the differents OPC

        /**
        * Take a snapshot of the OPC and save it in mSave
        * @param bName is a Yarp Bottle with the informations needed
        * @param *OPCReal is a pointer to the OPC checked
        */
        yarp::os::Bottle snapshot(yarp::os::Bottle bName, OPCClient *OPCReal);

        /**
        * Take a snapshot of the OPC and save it in mSave
        * @param bName is a Yarp Bottle with the informations needed
        * @param *OPCReal is a pointer to the OPC checked
        */
        yarp::os::Bottle snapshot_string(std::string sName, OPCClient *OPCReal);

        /**
        * Get the list of SQL request for a full OPC. Result is a Bottle with all the SQL request needed for the insertion in the DataBase.
        * @param sName : string name of the OPC to get in SQL
        */
        yarp::os::Bottle insertOPC(std::string sName);

        /**
        * For a given Relation, return the SQL request for the insertion in the DataBase
        * @param R : Relation to transform
        */
        yarp::os::Bottle insertRelation(Relation R);

        /**
        * For a given Entity, return the SQL request for the insertion in the DataBase
        * @param E : Entity to transform
        */
        yarp::os::Bottle insertEntity(Entity *E);

        /**
        * For a given ReDrivelation, return the SQL request for the insertion in the DataBase
        * @param D : Drive to transform
        */
        yarp::os::Bottle insertDrives(Drive D);

        /**
        * For a given Emotion, return the SQL request for the insertion in the DataBase
        * @param Emo  : pair<string, double> with the name and the value of the emotion
        */
        yarp::os::Bottle insertEmotion(std::pair<std::string, double> Emo);


        /**
        * Transform a int to a string
        */
        std::string InttoStr(int input);

        /**
        * Transform a double to a string
        */
        std::string DoutoStr(double input);


        /**
        * return a Bottle with the differencies between two Entities
        */
        yarp::os::Bottle getDifferencies(Entity *A, Entity *B);

        /**
        * return a Bottle with the differencies between two Agents
        */
        yarp::os::Bottle getDiffAgent(Agent *AgA, Agent *AgB);

        /**
        * return a Bottle with the differencies between two Objects
        */
        yarp::os::Bottle getDiffObject(Object *AgA, Object *AgB);

        /**
        * return a Bottle with the differencies between two RTObjects
        */
        yarp::os::Bottle getDiffRTObject(RTObject *AgA, RTObject *AgB);
        
        /**
        * return a Bottle with the differencies between two Adjectives
        */
        yarp::os::Bottle getDiffAdj(Adjective *AgA, Adjective *AgB);

        
        /**
        * return a Bottle with the differencies between two Actions
        */
        yarp::os::Bottle getDiffAction(Action *AgA, Action *AgB);
    };
}}


#endif


//----- end-of-file --- ( next line intentionally left blank ) ------------------


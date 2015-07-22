/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Stéphane Lallée
 * email:   stephane.lallee@gmail.com
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

#ifndef __EFAA_OPCC_H__
#define __EFAA_OPCC_H__

#include "wrdac/knowledge/representations.h"
#include "wrdac/tags.h"
namespace wysiwyd{namespace wrdac{

/**
* \ingroup wrdac_clients
*
* An OPC client using the datastructures defined within wrdac library.
* Using this insure you that your manipulation of the OPC will be done with correct formatting.
*
*/
class OPCClient
{
private:
    yarp::os::Port opc;
    yarp::os::BufferedPort<yarp::os::Bottle> opcBroadcast;
    bool                    write(yarp::os::Bottle &cmd, yarp::os::Bottle &reply, bool Verbose=false);

    std::map<int, Entity*>       entitiesByID;
    Entity*                 addEntity(Entity* e);

    int                     getRelationID(
            Entity* subject,
            Entity* verb,
            Entity* object = NULL,
            Entity* complement_place = NULL,
            Entity* complement_time = NULL,
            Entity* complement_manner = NULL);

public: 
    bool                    isVerbose;


    /**
    * Create an OPC client
    * @param moduleName The port namespace that will precede the client ports names.
    */
    OPCClient(const std::string &moduleName);

    /**
    * Check is the client is already connected to the OPC server.
    */
    bool isConnected() { return opc.getOutputCount() >= 1;}

    /**
    * Try to connect the client to an OPC server
    * @param opcName the name of the OPC module to connect to.
    */
    bool connect(const std::string &opcName)
    {
        return
                (yarp::os::Network::connect(opc.getName().c_str(), ("/" + opcName + "/rpc").c_str()) &&
                 yarp::os::Network::connect(("/" + opcName + "/broadcast:o").c_str(), opcBroadcast.getName().c_str() ));
    }

    /**
    * Interrupt communications of the client ports
    */
    void interrupt();

    /**
    * Close the client ports
    */
    void close();

    /**
    * Clear the OPC content.
    */
    void clear();

    /**
    * Obtains an object with the given name. If this name is already present then the existing object is returned, else a new one is created.
    */
    Object          *addOrRetrieveObject(const std::string &name);

    /**
    * Creates a new object. If an Object with the provided name is already existing, a new Object with an appended number is returned.
    */
    Object          *addObject(const std::string &name);

    /**
    * Obtains an agent with the given name. If this name is already present then the existing agent is returned, else a new one is created.
    */
    Agent           *addOrRetrieveAgent(const std::string &name);

    /**
    * Creates a new agent. If an Agent with the provided name is already existing, a new Agent with an appended number is returned.
    */
    Agent           *addAgent(const std::string &name);

    /**
    * Obtains an action with the given name. If this name is already present then the existing action is returned, else a new one is created.
    */
    Action           *addOrRetrieveAction(const std::string &name);

    /**
    * Creates a new action. If an Action with the provided name is already existing, a new Action with an appended number is returned.
    */
    Action           *addAction(const std::string &name);

    /**
    * Obtains a reactable object with the given name. If this name is already present then the existing object is returned, else a new one is created.
    */
    RTObject        *addOrRetrieveRTObject(const std::string &name);

    /**
    * Creates a new RTObject. If an RTObject with the provided name is already existing, a new RTObject with an appended number is returned.
    */
    RTObject       *addRTObject(const std::string &name);

    /**
    * Obtains an adjective with the given name. If this name is already present then the existing adjective is returned, else a new one is created.
    */
    Adjective       *addOrRetrieveAdjective(const std::string &name);

    /**
    * Creates a new adjective. If an adjective with the provided name is already existing, a new adjective with an appended number is returned.
    */
    Adjective       *addAdjective(const std::string &name);

    /**
    * Try to assign a property from an entity to another entity, using a specific property name
    */
    bool            setEntityProperty(std::string sourceEntityName, std::string propertyName, std::string targetEntityName);

    /**
    * Obtains a relation between two entities. If this relation already present on the server side then the local relations are updated, else a new one is created.
    * The relation is directly added on the server side.
    * @return true in case of success (relation was not present and was successfully added), false in other cases.
    */
    bool            addRelation(Relation r, double lifeTime = -1);

    /**
    * Obtains a relation between two entities. If this relation already present on the server side then the local relations are updated, else a new one is created.
    * The relation is directly added on the server side.
    * @return true in case of success (relation was not present and was successfully added), false in other cases.
    */
    bool addRelation(
            Entity* subject,
            Entity* verb,
            Entity* object = NULL,
            double lifeTime = -1,
            Entity* complement_place = NULL,
            Entity* complement_time = NULL,
            Entity* complement_manner = NULL
            );

    /**
    * Try to remove a relation between two entities.
    * The relation is directly removed on the server side.
    * @return true in case of success (final state no relation locally/server), false in other cases.
    */
    bool            removeRelation(Relation r);
    
    /**
    * Try to remove a relation between two entities.
    * The relation is directly removed on the server side.
    * @return true in case of success (final state no relation locally/server), false in other cases.
    */
    bool            removeRelation(
            Entity* subject,
            Entity* verb,
            Entity* object = NULL,
            Entity* complement_place = NULL,
            Entity* complement_time = NULL,
            Entity* complement_manner = NULL
            );
    
    /**
    * Check if a relation exist or not.
    */
    bool            containsRelation(
            Entity* subject,
            Entity* verb,
            Entity* object = NULL,
            Entity* complement_place = NULL,
            Entity* complement_time = NULL,
            Entity* complement_manner = NULL
            );

    /**
    * Check if a relation exist or not.
    */
    bool            containsRelation(Relation r);

    /**
    * Returns a list of all existing relations between entities.
    */
    std::list<Relation>  getRelations();
    
    /**
    * Returns a list of all existing relations involving a specific entities.
    */
    std::list<Relation>  getRelations(std::string entity);

    /**
    * Returns a list of all existing relations involving a specific entities.
    */
    std::list<Relation>  getRelations(Entity* entity);

    /**
    * Returns a list of all existing relations between entities.
    * All the roles with something else than "any" will act as filters for the relation.
    */
    std::list<Relation>  getRelationsMatching(std::string subject = "any",std::string verb = "any", std::string object = "any", std::string c_place = "any", std::string c_time = "any", std::string c_manner = "any" );

    /**
    * Returns a list of all existing relations between entities.
    * All the roles with something else than "any" will act as filters for the relation.
    * All relations with at least one role matching will be returned.
    */
    std::list<Relation>  getRelationsMatchingLoosly(std::string subject = "any", std::string verb = "any", std::string object = "any", std::string c_place = "any", std::string c_time = "any", std::string c_manner = "any");

    /**
    * Set the lifetimer property of a relation in the OPC
    * @return true in case of success, false in other cases.
    */
    bool            setLifeTime(int id, double lifeTime);

    /**
    * Gets an entity based on its name, but do no create it if it doesn't exist yet.
    * @param name The name of the entity to retrieve.
    * @param forceUpdate Update the entity instead of retrieving from the cache. (false by default)
    */
    Entity *getEntity(const std::string &name, bool forceUpdate = false);

    /**
    * Gets an entity based on its id, but do no create it if it doesn't exist yet.
    * @param id The ID of the entity to retrieve.
    * @param forceUpdate Update the entity instead of retrieving from the cache. (false by default)
    */
    Entity *getEntity(int id, bool forceUpdate = false);

    /**
    * Poll the OPC for all entities and relations and store them locally.
    * @param updateCache If this option is turned off the local cache will be wiped
    */
    void checkout(bool updateCache=true, bool useBroadcast=false);

    /**
    * Update the properties of all locally stored entities & relations.
    */
    void update();

    /**
    * Poll the OPC for updating a given entity properties
    * @param e A pointer on the entity to update.
    */
    void update(Entity *e);

    /**
    * Commit all the entities & relations stored locally, erasing the server copies.
    */
    void commit();

    /**
    * Commit a single local entity, erasing the server copies.
    */
    void commit(Entity *e);

    /**
    * Getter of the list of entities of matching a complex condition (e.g "(entity "==" agent) "&&" (isPresent "==" 1) ) and update them locally
    */
    std::list<Entity*> Entities(const yarp::os::Bottle &condition);

    /**
    * Getter of the list of entities matching a condition and update them locally
    */
    std::list<Entity*> Entities(const std::string &prop, const std::string &op, const std::string &value);

    /**
    * Getter of the list of entities stored locally
    */
    std::list<Entity*> EntitiesCache();

    /**
    * Getter of the list of the copies of the entities stored locally
    */
    std::list<Entity*> EntitiesCacheCopy();

    /**
    * Returns a human readable description of the client content (Entities & Relations)
    */
    std::string print();
};

}}//Namespaces
#endif


/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer, Grégoire Pointeau, Maxime Petit
 * email:   t.fischer@imperial.ac.uk, greg.pointeau@gmail.com, m.petit@imperial.ac.uk
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


#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include "autobiographicalMemory.h"
#include "templates.h"
#include <limits>
#include <thread>

using namespace yarp::sig;
using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;

/* configure the module */
bool autobiographicalMemory::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("autobiographicalMemory"), "module name (string)").asString();
    processInsertDelayed = rf.check("processInsertDelayed", Value(1)).asInt() > 0;

    setName(moduleName.c_str());

    //conf group for database properties
    Bottle &bDBProperties = rf.findGroup("database_properties");
    server = bDBProperties.check("server", Value("127.0.0.1")).asString();
    user = bDBProperties.check("user", Value("postgres")).asString();
    password = bDBProperties.check("password", Value("postgres")).asString();
    dataB = bDBProperties.check("dataB", Value("ABM")).asString();
    savefile = rf.findFileByName("saveRequest.txt");

    try {
        ABMDataBase = new DataBase<PostgreSql>(server, user, password, dataB);
    }
    catch (DataBaseError e) {
        yError() << "Could not connect to database. Reason: " << e.what();
        return false;
    }

    //conf group for image storing properties
    Bottle &bISProperties = rf.findGroup("image_storing");
    storingPath = bISProperties.check("storingPath", Value("C:/robot/ABMStoring")).asString();
    storingTmpSuffix = bISProperties.check("storingTmpSuffix", Value("tmp")).asString();
    imgFormat = bISProperties.check("imgFormat", Value("tif")).asString();
    portPrefixForStreaming = bISProperties.check("portPrefix", Value("/" + getName())).asString();

    // conf group for data providers
    Bottle &bDataProviders = rf.findGroup("data_providers");
    Bottle *defaultImgStreamProviders = bDataProviders.find("defaultImgStreamProviders").asList();
    Bottle *defaultDataStreamProviders = bDataProviders.find("defaultDataStreamProviders").asList();

    // TODO: streamStatus should be changed to enum
    streamStatus = "none"; //none, record, stop, send
    imgLabel = "defaultLabel";
    imgInstance = -1;
    frameNb = 0;
    sendStreamIsInitialized = false;

    augmentedTime = getCurrentTime();

    shouldClose = false;
    bPutObjectsOPC = false;

    handlerPort.open("/" + getName() + "/rpc");
    abm2reasoning.open("/" + getName() + "/to_reasoning");
    abm2augmented.open("/" + getName() + "/to_augmented");
    portAugmentedImagesIn.open("/" + getName() + "/augmented:i");
    portAugmentedImagesIn.setStrict(true);

    //create the storingPath and the tmp also
    string fullTmpPath = storingPath + "/" + storingTmpSuffix;
    yarp::os::mkdir_p(storingPath.c_str());
    yarp::os::mkdir_p(fullTmpPath.c_str());

    isconnected2reasoning = false;

    attach(handlerPort);

    connectOPC();

    //populateOPC();
    //storeImageOIDs();

    if (defaultImgStreamProviders) {
        for (int i = 0; i < defaultImgStreamProviders->size(); i++) {
            addStreamProvider(mapImgStreamInput, defaultImgStreamProviders->get(i).toString());
        }
    }
    if (defaultDataStreamProviders) {
        for (int i = 0; i < defaultDataStreamProviders->size(); i++) {
            addStreamProvider(mapDataStreamInput, defaultDataStreamProviders->get(i).toString());
        }
    }

    //connect to augmented
    //Network::connect(abm2augmented.getName().c_str(), "/ABMAugmentionExample/rpc");
    //if(!Network::isConnected(abm2augmented.getName().c_str(), "/ABMAugmentionExample/rpc")) {

    Network::connect(abm2augmented.getName().c_str(), "/matlab/kinematicStructure/rpc");
    if (!Network::isConnected(abm2augmented.getName().c_str(), "/matlab/kinematicStructure/rpc")) {
        yWarning("Could not connect to augmention module!");
    }

    //sound
    portSoundStreamInput.open(("/" + getName() + "/sound:i").c_str());

    int trials = 0;
    Network::connect("/speechRecognizer/recog/sound:o", portSoundStreamInput.getName().c_str());

    while ((!Network::isConnected("/speechRecognizer/recog/sound:o", portSoundStreamInput.getName().c_str())) && (trials < 5)) {
        trials += 1;
        yInfo() << "trying to connect to /speechRecognizer/recog/sound:o ...";

        Network::connect("/speechRecognizer/recog/sound:o", portSoundStreamInput.getName().c_str());
        if (trials == 5 && (!Network::isConnected("/speechRecognizer/recog/sound:o", portSoundStreamInput.getName().c_str()))){
            yInfo() << "Seems to be no sound, quit trying!";
        }
        else if (Network::isConnected("/speechRecognizer/recog/sound:o", portSoundStreamInput.getName().c_str())){
            yInfo() << "ABM is connected to /speechRecognizer/recog/sound:o !!!";
        }
        yarp::os::Time::delay(0.3);
    }

    yInfo() << "----------------------------------------------";
    yInfo() << "autobiographicalMemory ready ! ";

    return true;
}

/* Create a new database (erase the current one) */
Bottle autobiographicalMemory::newDB(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.size() != 2)
    {
        yWarning() << "password required";
        bOutput.addString("Error");
        bOutput.addString("password required");
        return bOutput;
    }
    else
    {
        if (bInput.get(1).asString().c_str() != password)
        {
            yError() << "Wrong database password!";
            bOutput.addString("Error");
            bOutput.addString("wrong password");
            return bOutput;
        }
        else
        {
            //to start the Database from scratch
            *ABMDataBase << "DROP TABLE IF EXISTS main CASCADE;";
            *ABMDataBase << "DROP TABLE IF EXISTS contentarg CASCADE;";
            *ABMDataBase << "DROP TABLE IF EXISTS contentopc CASCADE;";
            *ABMDataBase << "DROP TABLE IF EXISTS entity CASCADE;";
            //herits from entity
            *ABMDataBase << "DROP TABLE IF EXISTS action CASCADE;";
            *ABMDataBase << "DROP TABLE IF EXISTS adjective CASCADE;";
            *ABMDataBase << "DROP TABLE IF EXISTS object CASCADE;";
            //herits from object
            *ABMDataBase << "DROP TABLE IF EXISTS rtobject CASCADE;";
            *ABMDataBase << "DROP TABLE IF EXISTS agent CASCADE;";
            *ABMDataBase << "DROP TABLE IF EXISTS relation CASCADE;";

            bOutput.clear();
            bOutput.addString("new done");
            return bOutput;
        }
    }
}

/* load a predefine DB. INTERNAL USE ONLY */
Bottle  autobiographicalMemory::load(Bottle bInput)
{
    Bottle bOutput;

    //to start the Database from scratch

    /****************************** Main Table ******************************/
    *ABMDataBase << "DROP TABLE IF EXISTS main CASCADE;";
    *ABMDataBase << "CREATE TABLE main(idActivity serial NOT NULL, time timestamp without time zone NOT NULL,activityname text, activitytype text, instance integer NOT NULL UNIQUE, opcname text DEFAULT 'OPC'::text, begin boolean NOT NULL,CONSTRAINT main_pkey PRIMARY KEY (time)) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE main OWNER TO postgres;";

    /**************************** contentopc Table **************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contentopc CASCADE;";
    *ABMDataBase << "CREATE TABLE contentopc(  instance integer NOT NULL,  opcid integer,  type text,  subtype text,  UNIQUE (instance, opcid),  CONSTRAINT contentopc_pkey PRIMARY KEY (instance, opcid),  FOREIGN KEY (instance) REFERENCES main (instance)) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contentopc OWNER TO postgres;";

    /**************************** beliefs Table **************************/
    *ABMDataBase << "DROP TABLE IF EXISTS beliefs CASCADE;";
    *ABMDataBase << "CREATE TABLE beliefs (  instance integer NOT NULL,  idagent integer NOT NULL,  subject text NOT NULL,  verb text NOT NULL,  \"object\" text NOT NULL,  \"time\" text,  place text,  manner text,  CONSTRAINT beliefs_key FOREIGN KEY (instance) REFERENCES main (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE beliefs OWNER TO postgres;";

    /**************************** contentarg Table **************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contentarg CASCADE;";
    *ABMDataBase << "CREATE TABLE contentarg(  instance integer NOT NULL,  argument text, type text, subtype text, role text, UNIQUE (instance, role, argument),  CONSTRAINT contentarg_pkey PRIMARY KEY (instance, role, argument),  FOREIGN KEY (instance) REFERENCES main (instance)) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contentarg OWNER TO postgres;";

    /****************************** entity Table ****************************/
    *ABMDataBase << "DROP TABLE IF EXISTS entity CASCADE;";
    *ABMDataBase << "CREATE TABLE entity(  instance int NOT NULL,  opcid integer NOT NULL,  name text NOT NULL,  CONSTRAINT entity_pkey PRIMARY KEY (instance, opcid),  UNIQUE (instance, opcid),  FOREIGN KEY (instance, opcid) REFERENCES contentopc (instance, opcid)) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE entity OWNER TO postgres;";

    /****************************** action Table ****************************/
    //herits from entity
    *ABMDataBase << "DROP TABLE IF EXISTS action CASCADE;";
    *ABMDataBase << "CREATE TABLE action(  argument text,  CONSTRAINT action_pkey PRIMARY KEY (instance, opcid),  UNIQUE (opcid, instance),  FOREIGN KEY (instance, opcid) REFERENCES contentopc (instance, opcid)) INHERITS (entity) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE action OWNER TO postgres;";

    /**************************** adjective Table **************************/
    //herits from entity
    *ABMDataBase << "DROP TABLE IF EXISTS adjective CASCADE;";
    *ABMDataBase << "CREATE TABLE adjective(  quality text,  CONSTRAINT adjective_pkey PRIMARY KEY (instance, opcid),  UNIQUE (opcid, instance),  FOREIGN KEY (instance, opcid) REFERENCES contentopc (instance, opcid)) INHERITS (entity) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE adjective OWNER TO postgres;";

    /****************************** object Table ***************************/
    //herits from entity
    *ABMDataBase << "DROP TABLE IF EXISTS object CASCADE;";
    *ABMDataBase << "CREATE TABLE object(  presence boolean NOT NULL,  position real [],  orientation real[],  dimension real[],  color int[], saliency real, CONSTRAINT object_pkey PRIMARY KEY (instance, opcid),  UNIQUE (instance, opcid),  FOREIGN KEY (instance, opcid) REFERENCES contentopc (instance, opcid)) INHERITS (entity) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE object OWNER TO postgres;";

    /****************************** rtobject Table ***************************/
    //herits from object
    *ABMDataBase << "DROP TABLE IF EXISTS rtobject CASCADE;";
    *ABMDataBase << "CREATE TABLE rtobject(  rtposition real[],  CONSTRAINT rtobject_pkey PRIMARY KEY (instance, opcid),  UNIQUE (opcid, instance),  FOREIGN KEY (instance, opcid) REFERENCES contentopc (instance, opcid)) INHERITS (object) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE rtobject OWNER TO postgres;";

    /****************************** agent Table ***************************/
    //herits from entity
    *ABMDataBase << "DROP TABLE IF EXISTS agent CASCADE;";
    *ABMDataBase << "CREATE TABLE agent(  emotion text[],  CONSTRAINT agent_pkey PRIMARY KEY (instance, opcid),  UNIQUE (opcid, instance),  FOREIGN KEY (instance, opcid) REFERENCES contentopc (instance, opcid)) INHERITS (object) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE agent OWNER TO postgres;";

    /****************************** relation Table ***************************/
    *ABMDataBase << "DROP TABLE IF EXISTS relation CASCADE;";
    *ABMDataBase << "CREATE TABLE relation(  instance integer NOT NULL,  opcid integer NOT NULL,  subject text NOT NULL,  verb text NOT NULL, object text, time text,  place text,  manner text,  CONSTRAINT relation_pkey PRIMARY KEY (instance, opcid),  UNIQUE (instance,opcid),  FOREIGN KEY (instance, opcid) REFERENCES contentopc (instance, opcid) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE relation OWNER TO postgres;";

    /***************************** Drives table *****************************/
    *ABMDataBase << "DROP TABLE IF EXISTS drives CASCADE;";
    *ABMDataBase << "CREATE TABLE drives (     instance integer NOT NULL,  name     text NOT NULL, value double precision NOT NULL, homeomax double precision NOT NULL, homeomin double precision NOT NULL, UNIQUE (instance, name ) ,    CONSTRAINT drives_pkey     PRIMARY KEY (instance, name),     FOREIGN KEY (instance) REFERENCES main (instance),  UNIQUE (name, instance) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE drives OWNER TO postgres;";

    /***************************** Emotions table *****************************/
    *ABMDataBase << "DROP TABLE IF EXISTS emotions CASCADE;";
    *ABMDataBase << "CREATE TABLE emotions ( instance integer NOT NULL,  name text NOT NULL, value double precision NOT NULL, UNIQUE (name, instance ) ,    CONSTRAINT emotion_pkey     PRIMARY KEY (instance, name),     FOREIGN KEY (instance) REFERENCES main (instance),  UNIQUE (instance, name) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE drives OWNER TO postgres;";

    /****************************** spatialknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS spatialknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE spatialknowledge ( name text NOT NULL,  argument text NOT NULL, dependance text NOT NULL , instance integer NOT NULL,  CONSTRAINT spatialknowledge_pkey PRIMARY KEY (instance),  CONSTRAINT spatialknowledge_name_key UNIQUE (name, argument, dependance) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE spatialknowledge OWNER TO postgres;";

    /****************************** spatialdata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS spatialdata CASCADE;";
    *ABMDataBase << "CREATE TABLE spatialdata ( vx double precision, vy double precision, vdx double precision, vdy double precision, instance integer NOT NULL, id serial NOT NULL, CONSTRAINT spatialdata_instance_fkey FOREIGN KEY (instance) REFERENCES spatialknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE spatialdata OWNER  TO postgres;";

    /****************************** contextknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE contextknowledge ( name text NOT NULL,  argument text NOT NULL,  dependance text NOT NULL, instance integer NOT NULL,  CONSTRAINT contextknowledge_pkey PRIMARY KEY (instance),  CONSTRAINT contextknowledge_name_key UNIQUE (name, argument, dependance) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextknowledge OWNER TO postgres;";

    /****************************** contextdata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextdata CASCADE;";
    *ABMDataBase << "CREATE TABLE contextdata (  presencebegin boolean,  presenceend boolean,  instance integer NOT NULL,  id serial NOT NULL,  CONSTRAINT contextdata_instance_fkey FOREIGN KEY (instance) REFERENCES contextknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextdata OWNER  TO postgres;";

    /****************************** contextagent *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextagent CASCADE;";
    *ABMDataBase << "CREATE TABLE contextagent ( instance integer NOT NULL, agent text NOT NULL, number integer, CONSTRAINT contextagent_pkey PRIMARY KEY (instance, agent), CONSTRAINT contextagent_instance_fkey FOREIGN KEY (instance) REFERENCES contextknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextagent OWNER  TO postgres;";

    /****************************** timeknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS timeknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE timeknowledge ( temporal text NOT NULL,   CONSTRAINT timeknowledge_pkey PRIMARY KEY (temporal) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE timeknowledge OWNER TO postgres;";

    /****************************** timedata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS timedata CASCADE;";
    *ABMDataBase << "CREATE TABLE timedata ( temporal text NOT NULL,   timearg1 timestamp without time zone NOT NULL,   timearg2 timestamp without time zone NOT NULL,   CONSTRAINT timedata_temporal_fkey FOREIGN KEY (temporal)        REFERENCES timeknowledge (temporal) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE timedata OWNER  TO postgres;";

    /****************************** interactionknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS interactionknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE interactionknowledge (subject text NOT NULL, argument text NOT NULL, number integer NOT NULL, type text NOT NULL DEFAULT 'none'::text, role text NOT NULL DEFAULT 'none'::text, CONSTRAINT interactionknowledge_pkey PRIMARY KEY (subject, argument, type, role) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE interactionknowledge OWNER  TO postgres;";

    /****************************** behavior *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS behavior CASCADE;";
    *ABMDataBase << "    CREATE TABLE behavior(  \"name\" text NOT NULL,  argument text NOT NULL,  instance integer NOT NULL,  CONSTRAINT behavior_pkey PRIMARY KEY (instance),  CONSTRAINT behavior_name_key UNIQUE (name, argument)) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE behavior OWNER  TO postgres;";

    /****************************** behaviordata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS behaviordata CASCADE;";
    *ABMDataBase << "    CREATE TABLE behaviordata(  drive text NOT NULL,  effect double precision,  instance integer NOT NULL,  occurence integer NOT NULL,  CONSTRAINT behaviordata_pkey PRIMARY KEY (occurence, instance, drive),  CONSTRAINT behaviordata_instance_fkey FOREIGN KEY (instance)      REFERENCES behavior (instance) MATCH SIMPLE      ON UPDATE NO ACTION ON DELETE NO ACTION)WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE behaviordata OWNER  TO postgres;";

    /****************************** sharedplan *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS sharedplan CASCADE;";
    *ABMDataBase << "       CREATE TABLE sharedplan(  \"name\" text NOT NULL,  manner text NOT NULL,  instance integer NOT NULL,  CONSTRAINT sharedplan_pkey PRIMARY KEY (instance),  CONSTRAINT sharedplan_name_key UNIQUE (name, manner))WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE sharedplan OWNER  TO postgres;";

    /****************************** sharedplanarg *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS sharedplanarg CASCADE;";
    *ABMDataBase << "     CREATE TABLE sharedplanarg(  instance integer NOT NULL,  argument text NOT NULL,  \"role\" text NOT NULL,  CONSTRAINT sharedplanarg_pkey PRIMARY KEY (instance, role, argument),  CONSTRAINT sharedplanarg_instance_fkey FOREIGN KEY (instance)      REFERENCES sharedplan (instance) MATCH SIMPLE      ON UPDATE NO ACTION ON DELETE NO ACTION)WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE sharedplanarg OWNER  TO postgres;";

    /****************************** sharedplandata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS sharedplandata CASCADE;";
    *ABMDataBase << "     CREATE TABLE sharedplandata (  activitytype text NOT NULL,  activityname text NOT NULL,  instance integer NOT NULL,  id integer NOT NULL,  CONSTRAINT sharedplandata_pkey PRIMARY KEY (instance, id),  CONSTRAINT sharedplandata_instance_fkey FOREIGN KEY (instance)      REFERENCES sharedplan (instance) MATCH SIMPLE      ON UPDATE NO ACTION ON DELETE NO ACTION)WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE sharedplandata OWNER  TO postgres;";

    /****************************** spdataarg *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS spdataarg CASCADE;";
    *ABMDataBase << "CREATE TABLE spdataarg (id integer NOT NULL, instance integer NOT NULL, argument text NOT NULL, \"role\" text NOT NULL, CONSTRAINT spdataarg_pkey PRIMARY KEY (id, instance, role, argument), CONSTRAINT spdataarg_instance_fkey FOREIGN KEY (instance, id) REFERENCES sharedplandata (instance, id) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE spdataarg OWNER  TO postgres;";

    /****************************** visualdata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS visualdata CASCADE;";
    *ABMDataBase << "CREATE TABLE visualdata(\"time\" timestamp without time zone NOT NULL, img_provider_port text NOT NULL, instance integer NOT NULL, frame_number integer NOT NULL, relative_path text NOT NULL, augmented text, img_oid oid, augmented_time timestamp without time zone NOT NULL DEFAULT '2000-01-01 00:00:00'::timestamp without time zone, CONSTRAINT img_pkey PRIMARY KEY (\"time\", img_provider_port, augmented_time), CONSTRAINT visualdata_instance_fkey FOREIGN KEY (instance) REFERENCES main (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH (OIDS = FALSE);";
    *ABMDataBase << "ALTER TABLE visualdata OWNER TO postgres;";
    *ABMDataBase << "CREATE INDEX visualdata_instance_time ON visualdata (instance, time);";

    /****************************** proprioceptivedata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS proprioceptivedata CASCADE;";
    *ABMDataBase << "CREATE TABLE proprioceptivedata(instance integer NOT NULL, \"time\" timestamp without time zone NOT NULL, label_port text NOT NULL, subtype text NOT NULL, frame_number integer NOT NULL, value text NOT NULL, CONSTRAINT cont_pkey PRIMARY KEY (\"time\", label_port, subtype), CONSTRAINT proprio_instance_fkey FOREIGN KEY (instance) REFERENCES main (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH ( OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE proprioceptivedata OWNER TO postgres;";
    *ABMDataBase << "CREATE INDEX proprioceptivedata_instance_time ON proprioceptivedata (instance, time);";

    /****************************** adjectivespatial *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS adjectivespatial CASCADE;";
    *ABMDataBase << "CREATE TABLE adjectivespatial ( \"name\" text NOT NULL, argument text NOT NULL, x double precision, y double precision, dx double precision, dy double precision ) WITH(OIDS = FALSE) ";
    *ABMDataBase << "ALTER TABLE adjectivespatial OWNER TO postgres;";

    /****************************** adjectivetemporal *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS adjectivetemporal CASCADE;";
    *ABMDataBase << "CREATE TABLE adjectivetemporal ( \"name\" text NOT NULL, argument text NOT NULL, timing double precision) WITH(OIDS = FALSE) ";
    *ABMDataBase << "ALTER TABLE adjectivetemporal OWNER TO postgres;";

    /****************************** sounddata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS sounddata CASCADE;";
    *ABMDataBase << "CREATE TABLE sounddata(\"time\" timestamp without time zone NOT NULL, snd_provider_port text NOT NULL, instance integer NOT NULL, relative_path text, snd_oid oid, CONSTRAINT snd_pkey PRIMARY KEY (\"time\", snd_provider_port), CONSTRAINT sound_instance_fkey FOREIGN KEY (instance) REFERENCES main (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (  OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE sounddata OWNER TO postgres;";

    /****************************** sentencedata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS sentencedata CASCADE;";
    *ABMDataBase << "CREATE TABLE sentencedata ( instance integer NOT NULL, word text, \"role\" text, \"level\" integer NOT NULL, CONSTRAINT sentencedata_pkey PRIMARY KEY(instance, level), CONSTRAINT sentencedata_instance_fkey FOREIGN KEY(instance) REFERENCES main(instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH(OIDS = FALSE);";
    *ABMDataBase << "ALTER TABLE sentencedata OWNER TO postgres;";


    string sFilename;

    //if filename after, load the database through this
    if (bInput.size() > 1)
    {
        bOutput.addString("loaded");
        bOutput.addString(bInput.get(1).asString().c_str());

        sFilename = bInput.get(1).asString();
        //filename format : each line is a SQL sequence of INSERT INTO
    }
    //else, load the default database (the last one)
    else
    {
        bOutput.addString("loaded");
        bOutput.addString("default");
        sFilename = "autobiographicalMemory.db";
    }

    // build the database startup
    bool bInsert = true;
    if (bInsert)
    {
        //main table : 2 actions (begin : true then false) => 4 OPC
        *ABMDataBase << "INSERT INTO main(time, activityname, activitytype, instance, begin) VALUES('2012-01-01 15:53:10','grasp','action',1,TRUE);";

        //contentarg
        *ABMDataBase << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES(1,'ball', 'entity', 'rtobject', 'object1');";
        *ABMDataBase << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES(1,'blue', 'external', 'color', 'argument');";

        //contentopc table

        //------------> OPC 1
        *ABMDataBase << "INSERT INTO contentopc(instance, type, subtype, opcid) VALUES(1,'entity','object',81);";
        *ABMDataBase << "INSERT INTO contentopc(instance, type, subtype, opcid) VALUES(1,'entity','entity',82);";
        *ABMDataBase << "INSERT INTO contentopc(instance, type, subtype, opcid) VALUES(1,'relation','relation',83);";

        //entity table
        *ABMDataBase << "INSERT INTO entity(opcid, name, instance) VALUES(82, 'thing', 1);";

        //object table
        *ABMDataBase << "INSERT INTO object(opcid, name, instance, presence, position) VALUES(81, 'ball', 1, TRUE,'{1.1,2.2,1}');";

        //relation table
        *ABMDataBase << "INSERT INTO relation(opcid, instance, subject, verb, object) VALUES(83, 1, 'ball', 'isAtLoc', 'left');";
    }

    return bOutput;
}


Bottle autobiographicalMemory::addInteraction(Bottle bInteraction)
{
    //prepare the ResultSet of the query and the reply
    Bottle bReply;

    try
    {
        *ABMDataBase << bInteraction.get(1).asString().c_str();
        writeInsert(bInteraction.get(1).asString().c_str());
        bInteraction.addString("add done");

    }
    catch (DataBaseError& e)
    {
        yError() << "Exception during request : " << e.what();
        string sExcept = "Exception during request : "; sExcept += e.what();
        bReply.addString(sExcept.c_str());
    }

    //send the request to the database
    return bReply;
}

double autobiographicalMemory::getPeriod()
{
    return 0.005; //module periodicity (seconds)
}

/* rpc respond module */
bool autobiographicalMemory::respond(const Bottle& bCommand, Bottle& bReply)
{
    Bottle bError;
    bReply.clear();
    string helpMessage = string(getName().c_str()) +
        " commands are: \n" +
        "help \n" +
        "quit/close \n" +
        "interrupt \n" +
        "read \n" +
        "new \n" +
        "snapshot \n" +
        "snapshotSP \n" +
        "snapshotBE \n" +
        "load \n" +
        "connect \n" +
        "request \n" +
        "snapshotSP \n" +
        "insert \n" +
        "resetKnowledge \n" +
        "eraseInstance \n" +
        "getStoringPath \n" +
        "triggerStreaming \n" +
        "addImgStreamProvider \n" +
        "removeImgStreamProvider \n" +
        "listImgStreamProviders \n" +
        "processInsertQueue \n" +
        "saveAugmentedImages \n" +
        "requestAugmentedImages \n" +
        "getStreamStatus \n"
        ;

    bError.addString("ERROR");

    if (bCommand.get(0).isString())
    {
        if (bCommand.get(0) == "quit" || bCommand.get(0) == "close")
        {
            bReply.addString("close module");
            shouldClose = true;
        }
        else if (bCommand.get(0) == "interrupt")
        {
            bReply.addString("interrupt module");
            interruptModule();
        }
        // Read a file, and load the requests
        else if (bCommand.get(0) == "read")
        {
            if (readInsert())
            {
                bReply.addString("File loaded");
            }
            else
            {
                bError.addString("Cannot open the file");
                bReply = bError;
            }
        }
        //database from sratch considering the history [but load the other tables from the OPC]
        else if (bCommand.get(0) == "new")
        {
            bReply = newDB(bCommand);
        }
        else if (bCommand.get(0) == "snapshot")
        {
            //if(streamStatus=="none" || streamStatus=="begin" || streamStatus=="record")
                bReply = snapshot(bCommand);
            //else
            //    bReply.addString("[nack]");
        }
        else if (bCommand.get(0) == "snapshotSP")
        {
            //if(streamStatus=="none" || streamStatus=="begin" || streamStatus=="record")
                bReply = snapshotSP(bCommand);
            //else
            //    bReply.addString("[nack]");
        }
        else if (bCommand.get(0) == "snapshotBE")
        {
            //if(streamStatus=="none" || streamStatus=="begin" || streamStatus=="record")
                bReply = snapshotBehavior(bCommand);
            //else
            //    bReply.addString("[nack]");
        }
        else if (bCommand.get(0) == "load")
        {
            bReply = load(bCommand);
        }
        else if (bCommand.get(0) == "connect")
        {
            bReply = connect2reasoning();
        }
        else if (bCommand.get(0) == "request")
        {
            if (bCommand.size() > 1)
            {
                bReply = request(bCommand);
            }
            else
            {
                bError.addString("in request : number of element insufficient");
                bReply = bError;
            }
        }
        else if (bCommand.get(0) == "insert")
        {
            if (bCommand.size() > 1)
            {
                bReply.addString("insert");
                bReply.addList() = addInteraction(bCommand);
            }
            else
            {
                bError.addString("in insert : number of element insufficient");
                bReply = bError;
            }
        }
        else if (bCommand.get(0) == "resetKnowledge")
        {
            bReply = resetKnowledge();
        }
        else if (bCommand.get(0) == "eraseInstance")
        {
            bReply = eraseInstance(bCommand);
        }
        else if (bCommand.get(0) == "getStoringPath")
        {
            bReply.addString(storingPath);
        }
        // ask for streaming data : they will be sent through ports opened by autobiographicalMemory based on the original image provider port (/autobiographicalMemory/imgProviderPortName)
        else if (bCommand.get(0) == "triggerStreaming")
        {
            if (bCommand.size() > 1 && bCommand.get(1).isInt())
            {
                yDebug() << "Command: " << bCommand.toString();
                int instance = bCommand.get(1).asInt();
                bool blocking = false;
                bool realtime = false;
                bool includeAugmented = true;
                double speedMultiplier = 1.0;
                string robot = "icubSim";

                Value vRealtime = bCommand.find("realtime");
                if (!vRealtime.isNull() && vRealtime.isInt()) {
                    realtime = vRealtime.asInt() > 0;
                }

                Value vIncludeAugmented = bCommand.find("includeAugmented");
                if (!vIncludeAugmented.isNull() && vIncludeAugmented.isInt()) {
                    includeAugmented = vIncludeAugmented.asInt() > 0;
                }

                Value vSpeedMultiplier = bCommand.find("speedMultiplier");
                if (!vSpeedMultiplier.isNull() && vSpeedMultiplier.isDouble()) {
                    speedMultiplier = vSpeedMultiplier.asDouble();
                }

                Value vRobot = bCommand.find("robot");
                if (!vRobot.isNull() && vRobot.isString()) {
                    robot = vRobot.asString();
                }

                Value vBlocking = bCommand.find("blocking");
                if (!vBlocking.isNull() && vBlocking.isInt()) {
                    blocking = vBlocking.asInt() > 0;
                }

                yDebug() << "instance: " << instance;
                yDebug() << "realtime: " << realtime;
                yDebug() << "includeAugmented: " << includeAugmented;
                yDebug() << "speedMultiplier: " << speedMultiplier;
                yDebug() << "robot: " << robot;
                yDebug() << "blocking: " << blocking;

                bReply = triggerStreaming(instance, realtime, includeAugmented, speedMultiplier, robot);
                if(blocking) {
                    while(streamStatus=="send") {
                        yarp::os::Time::delay(0.2);
                    }
                }
            }
            else
            {
                bError.addString("[triggerStreaming]: number of element incorrect: triggerStreaming instance (realtime true/false) (includeAugmented true/false) (speedMultiplier 1.0) (robot icubSim)");
                bReply = bError;
            }
        }
        // add an image provider for the following stream recording : addImgStreamProvider (label /yarp/port/img/provider)
        else if (bCommand.get(0) == "addImgStreamProvider")
        {
            if (bCommand.size() == 2 && bCommand.get(1).isString())
            {
                bReply = addStreamProvider(mapImgStreamInput, bCommand.get(1).toString().c_str());
            }
            else
            {
                bError.addString("[addImgStreamProvider]: wrong number of element -> addImgStreamProvider /yarp/port/img/provider");
                bReply = bError;
            }
        }

        //remove an image provider from the list of available stream images provider
        else if (bCommand.get(0) == "removeImgStreamProvider")
        {
            if (bCommand.size() == 2 && bCommand.get(1).isString())
            {
                bReply = removeStreamProvider(mapImgStreamInput, bCommand.get(1).toString().c_str());
            }
            else
            {
                bError.addString("[removeImgStreamProvider]: wrong number of element -> removeImgStreamProvider /yarp/port/img/provider");
                bReply = bError;
            }
        }
        else if (bCommand.get(0) == "listImgStreamProviders")
        {
            bReply = listProviders(mapImgStreamInput);
        }
        else if (bCommand.get(0) == "addDataStreamProvider")
        {
            if (bCommand.size() == 2 && bCommand.get(1).isString())
            {
                bReply = addStreamProvider(mapDataStreamInput, bCommand.get(1).toString().c_str());
            }
            else {
                bError.addString("[addDataStreamProvider]: wrong number of elements -> addDataStreamProvider /yarp/port/contdata/provider");
                bReply = bError;
            }
        }
        else if (bCommand.get(0) == "removeDataStreamProvider")
        {
            if (bCommand.size() == 2 && bCommand.get(1).isString())
            {
                bReply = removeStreamProvider(mapDataStreamInput, bCommand.get(1).toString().c_str());
            }
            else
            {
                bError.addString("[removeDataStreamProvider]: wrong number of elements -> removeDataStreamProvider /yarp/port/contdata/provider");
                bReply = bError;
            }
        }
        else if (bCommand.get(0) == "listDataStreamProviders")
        {
            bReply = listProviders(mapDataStreamInput);
        }
        else if (bCommand.get(0) == "processInsertQueue")
        {
            requestInsertProcessQueue();
            if (bCommand.size() == 2 && bCommand.get(1).isInt()) {
                int instance = (atoi((bCommand.get(1)).toString().c_str()));
                storeImageOIDs(instance);
            }
            else {
                storeImageOIDs();
            }
            bReply.addString("ack");
        }
        else if (bCommand.get(0) == "saveAugmentedImages")
        {
            yDebug() << "start saveAugmentedImages";
            augmentedTime = getCurrentTime();
            saveAugmentedImages();
            bReply.addString("ack");
        }
        else if (bCommand.get(0) == "requestAugmentedImages")
        {
            int instance = -1;
            int quantity = 5;
            string activity = "babbling";

            Value vInstance = bCommand.find("instance");
            if (!vInstance.isNull() && vInstance.isInt()) {
                instance = vInstance.asInt();
            }

            Value vQuantity = bCommand.find("quantity");
            if (!vQuantity.isNull() && vQuantity.isInt()) {
                quantity = vQuantity.asInt();
            }

            Value vActivity = bCommand.find("activity");
            if (!vActivity.isNull() && vActivity.isString()) {
                activity = vActivity.asString();
            }

            if (requestAugmentedImages(activity, quantity, instance)) {
                bReply.addString("ack");
            }
            else {
                bReply.addString("nack");
            }
            }
        else if (bCommand.get(0) == "getStreamStatus")
        {
            bReply.addString(streamStatus);
        }
        else
        {
            bError.addString(helpMessage);
            yInfo() << "\n" << helpMessage;
            bReply = bError;
        }
        }
    else
    {
        bError.addString(helpMessage);
        yInfo() << "\n" << helpMessage;
        bReply = bError;
    }

    return true;
    }

void autobiographicalMemory::storeImagesAndData(const string &synchroTime, bool forSingleInstance, string fullSentence) {
    std::thread dataStreamThread(&autobiographicalMemory::storeDataStreamAllProviders, this, synchroTime);
    std::thread imageThread(&autobiographicalMemory::storeInfoAllImages, this, synchroTime, forSingleInstance, fullSentence);
    imageThread.join();
    dataStreamThread.join();

    if (increaseFrameNb) {
        frameNb++;
        increaseFrameNb = false;
    }
}

/* rpc update module */
bool autobiographicalMemory::updateModule() {
    //yDebug() << "Update loop";
    yarp::sig::Sound *s;
    s = portSoundStreamInput.read(false);
    if (s != NULL && s->getRawDataSize() > 0)
    {
        yInfo() << "I have received a sound!";
        string soundPath = storingPath + "/" + storingTmpSuffix + "/sound/";
        if (yarp::os::mkdir_p(soundPath.c_str()) == -1) {
            yDebug() << "Folder " << soundPath << " already exists or could not be created!";
        }
        if (!yarp::sig::file::write(*s, (soundPath + "default.wav").c_str()))
        {
            yError() << " [Sound] Sound file not written!!";
        }
    }
    else {
        //yDebug() << "no sound?";
    }

    saveAugmentedImages();

    //we have received a snapshot command indicating an activity that take time so streaming is needed
    //currently it is when activityType == action
    if (streamStatus == "begin") {
        yDebug() << "[mutexStreamRecord] trying to lock in stream begin";
        mutexStreamRecord.lock();
        yDebug() << "[mutexStreamRecord] locked in stream begin";
        yInfo() << "============================= STREAM BEGIN =================================";

        imgInstance = currentInstance; //currentInstance is different from begin/end : imgInstance instanciated just at the beginning and use for the whole stream to assure the same instance id

        string currentPathFolder = storingPath + "/" + std::to_string(imgInstance);
        if (yarp::os::mkdir_p(currentPathFolder.c_str()) == -1) {
            yWarning() << "Folder " << currentPathFolder << " already exists or could not be created!";
        }

        string synchroTime = getCurrentTime();
        storeImagesAndData(synchroTime);

        //init of the stream record done: go through the classic record phase
        streamStatus = "record";
        yDebug() << "[mutexChangeover] unlocked in begin";
        mutexChangeover.unlock();
    }
    else if (streamStatus == "record") {
        string synchroTime = getCurrentTime();
        storeImagesAndData(synchroTime);
    }
    else if (streamStatus == "send") { //stream to send, because rpc port receive a sendStreamImage query
        //select all the images (through relative_path and image provider) corresponding to a precise instance
        if (sendStreamIsInitialized == false) {
            yDebug() << "[mutexStreamRecord] trying to lock in stream send";
            mutexStreamRecord.lock();
            yDebug() << "[mutexStreamRecord] locked in stream send";
            yInfo() << "============================= STREAM SEND =================================";
            timeLastImageSent = -1;

            long timeVeryLastImage = getTimeLastStream(imgInstance, "visualdata");
            long timeVeryLastContData = getTimeLastStream(imgInstance, "proprioceptivedata");

            if (timeVeryLastImage >= timeVeryLastContData) {
                timeVeryLastStream = timeVeryLastImage;
            }
            else {
                timeVeryLastStream = timeVeryLastContData;
            }
            timeStreamStart = getCurrentTimeInMS();

            sendStreamIsInitialized = true;
            yDebug() << "[mutexChangeover] unlocked in send";
            mutexChangeover.unlock();
        }

        // Calculate time in update method since first image/contdata was sent
        long timeStreamCurrent = getCurrentTimeInMS();
        double updateTimeDifference = (timeStreamCurrent - timeStreamStart) * speedMultiplier;
        long timeLastImageSentCurrentIteration = 0;

        // Find which images to send
        Bottle bListImages = getStreamWithinEpoch(long(updateTimeDifference), "visualdata");

        // Save images in temp folder and send them to ports
        if (bListImages.toString() != "NULL") {
            for (int i = 0; i < bListImages.size(); i++) {
                string relative_path = bListImages.get(i).asList()->get(0).asString();
                string portname = bListImages.get(i).asList()->get(1).asString();
                string time = bListImages.get(i).asList()->get(2).asString();
                string frame_number = bListImages.get(i).asList()->get(4).asString();
                string augmented = bListImages.get(i).asList()->get(5).asString();
                string augmented_time = bListImages.get(i).asList()->get(6).asString();
                string concatenated_port = portname + augmented + augmented_time;

                // remove spaces
                std::string::iterator end_pos1 = std::remove(concatenated_port.begin(), concatenated_port.end(), ' ');
                concatenated_port.erase(end_pos1, concatenated_port.end());

                std::string::iterator end_pos2 = std::remove(augmented_time.begin(), augmented_time.end(), ' ');
                augmented_time.erase(end_pos2, augmented_time.end());

                //concatenation of the storing path
                stringstream fullPath;
                fullPath << storingPath << "/" << storingTmpSuffix << "/" << relative_path << augmented_time;

                try {
                    auto* port = mapImgStreamPortOut.at(concatenated_port);
                    Bottle env;
                    env.addInt(imgInstance);
                    env.addString(portname);
                    env.addString(time);
                    env.addString(frame_number);
                    yDebug() << "Envelope: " << env.toString();
                    port->setEnvelope(env);
                    if (atol(bListImages.get(i).asList()->get(3).asString().c_str()) > timeLastImageSentCurrentIteration) {
                        timeLastImageSentCurrentIteration = atol(bListImages.get(i).asList()->get(3).asString().c_str());
                        //yDebug() << "Set new timeLastImageSentCurrentIteration " << timeLastImageSentCurrentIteration;
                    }

                    yInfo() << "Send image: " << fullPath.str() << " to " << port->getName();
                    writeImageToPort(fullPath.str(), port);
                }
                catch (const std::out_of_range&) {
                    yWarning() << "No corresponding port to" << concatenated_port << ", not going to send image for this port!";
                    yWarning() << "Available ports:";
                    for(auto& port : mapImgStreamPortOut) {
                        yWarning() << port.first;
                    }
                }
            }
        }

        for (const auto& dataStreamPortOut : mapDataStreamPortOut)
        {
            // Find which data stream to send
            Bottle bListContData = getStreamWithinEpoch(long(updateTimeDifference), "proprioceptivedata", dataStreamPortOut.first);

            if (bListContData.toString() != "NULL") {
                Bottle &bCmd = dataStreamPortOut.second->prepare();
                bCmd.clear();
                // for joints, begin with the command to set the position
                if (dataStreamPortOut.first.find("state:o") != std::string::npos) {
                    bCmd.fromString("[set] [poss]");
                }

                Bottle bJoints;

                // Append bottle of ports for all the subtypes
                for (int i = 0; i < bListContData.size(); i++) {
                    if (atol(bListContData.get(i).asList()->get(4).asString().c_str()) > timeLastImageSentCurrentIteration) {
                        timeLastImageSentCurrentIteration = atol(bListContData.get(i).asList()->get(4).asString().c_str());
                        //yDebug() << "Set new timeLastImageSentCurrentIteration " << timeLastImageSentCurrentIteration;
                    }
                    if (dataStreamPortOut.first.find("skeleton:o") != std::string::npos) {
                        bJoints.addString(bListContData.get(i).asList()->get(3).asString());
                    } else {
                        bJoints.addDouble(atof(bListContData.get(i).asList()->get(3).asString().c_str()));
                    }
                }

                bCmd.addList() = bJoints;

                // Send concatenated bottles to ports
                yInfo() << "Write port: " << dataStreamPortOut.second->getName();
                yInfo() << dataStreamPortOut.second->prepare().toString();
                dataStreamPortOut.second->writeStrict();
            }
        }

        if (timeLastImageSentCurrentIteration > timeLastImageSent) {
            timeLastImageSent = timeLastImageSentCurrentIteration;
            //yDebug() << "Set new timeLastImageSent " << timeLastImageSent;
        }

        // Are we done?
        bool done = false;
        if (realtimePlayback && updateTimeDifference >= timeVeryLastStream) {
            yDebug() << "realtime on, end of stream:" << timeLastImageSent << " >= " << timeVeryLastStream;
            done = true;
        }
        else if (!realtimePlayback && timeLastImageSent >= timeVeryLastStream) {
            yDebug() << "realtime off, end of stream:" << timeLastImageSent << " >= " << timeVeryLastStream;
            done = true;
        }

        if (done) {
            //Close ports which were opened in openSendContDataPorts / openStreamImgPorts
            yInfo() << "streamStatus = end, closing ports";
            for (auto const& imgStreamPortOut : mapImgStreamPortOut) {
                imgStreamPortOut.second->waitForWrite();
                imgStreamPortOut.second->interrupt();
                imgStreamPortOut.second->close();
                if (!imgStreamPortOut.second->isClosed()) {
                    yError() << "Error, port " << imgStreamPortOut.first << " could not be closed";
                }
            }
            for (auto const& dataStreamPortOut : mapDataStreamPortOut) {
                dataStreamPortOut.second->waitForWrite();
                dataStreamPortOut.second->interrupt();
                dataStreamPortOut.second->close();
                if (!dataStreamPortOut.second->isClosed()) {
                    yError() << "Error, port " << dataStreamPortOut.first << " could not be closed";
                }
            }

            mapImgStreamPortOut.clear();
            mapDataStreamPortOut.clear();

            yDebug() << "[mutexChangeover] trying to lock in end of send";
            mutexChangeover.lock();
            yDebug() << "[mutexChangeover] unlocked in end of send";

            streamStatus = "end";
        }
    }

    //go back to default global value
    if (streamStatus == "end") {
        yInfo() << "============================= STREAM STOP =================================";

        // wait for threads to be finished
        yarp::os::Time::delay(0.5);

        //TODO: startThread with storeOID()

        //close folder and SQL entry
        imgLabel = "defaultLabel";
        imgInstance = -1;
        frameNb = 0;
        sendStreamIsInitialized = false;
        streamStatus = "none";

        yDebug() << "[mutexStreamRecord] unlocked in end of stop";
        mutexStreamRecord.unlock();
        yDebug() << "[mutexChangeover] unlocked in end of stop";
        mutexChangeover.unlock();
    }

    return !shouldClose;
}

bool autobiographicalMemory::interruptModule()
{
    yInfo() << "Interrupting your module, for port cleanup";

    storeImageOIDs();
    opcWorldReal->interrupt();
    opcWorldMental->interrupt();

    portAugmentedImagesIn.interrupt();
    portSoundStreamInput.interrupt();
    handlerPort.interrupt();
    abm2reasoning.interrupt();
    abm2augmented.interrupt();

    return true;
}

bool autobiographicalMemory::close()
{
    yInfo() << "Calling close function";

    disconnectStreamProviders(mapDataStreamInput);
    disconnectStreamProviders(mapImgStreamInput);

    opcWorldReal->interrupt();
    opcWorldReal->close();
    opcWorldMental->interrupt();
    opcWorldMental->close();

    portAugmentedImagesIn.interrupt();
    portAugmentedImagesIn.close();

    portSoundStreamInput.interrupt();
    portSoundStreamInput.close();

    handlerPort.interrupt();
    handlerPort.close();

    abm2reasoning.interrupt();
    abm2reasoning.close();

    abm2augmented.interrupt();
    abm2augmented.close();

    requestInsertProcessQueue();
    storeImageOIDs();

    for(auto& input : mapDataStreamInput) {
        delete input.second;
    }

    for(auto& input : mapImgStreamInput) {
        delete input.second;
    }

    for(auto& outport : mapDataStreamPortOut) {
        delete outport.second;
    }

    for(auto& outport : mapImgStreamPortOut) {
        delete outport.second;
    }

    delete opcWorldReal;
    delete opcWorldMental;
    delete ABMDataBase;

    yInfo() << "ABM Successfully finished!";

    return true;
}

/*
* Reset the tables : timeknowledge, timedata, spatialknowledge, spatialdata in the DataBase
*/
Bottle autobiographicalMemory::resetKnowledge()
{
    Bottle bOutput;

    /****************************** spatialknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS spatialknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE spatialknowledge ( name text NOT NULL,  argument text NOT NULL, dependance text NOT NULL, instance integer NOT NULL,  CONSTRAINT spatialknowledge_pkey PRIMARY KEY (instance),  CONSTRAINT spatialknowledge_name_key UNIQUE (name, argument, dependance) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE spatialknowledge OWNER TO postgres;";

    /****************************** spatialdata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS spatialdata CASCADE;";
    *ABMDataBase << "CREATE TABLE spatialdata ( vx double precision, vy double precision, vdx double precision, vdy double precision, instance integer NOT NULL, id serial NOT NULL, CONSTRAINT spatialdata_instance_fkey FOREIGN KEY (instance) REFERENCES spatialknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE spatialdata OWNER  TO postgres;";

    /****************************** timeknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS timeknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE timeknowledge ( temporal text NOT NULL,   CONSTRAINT timeknowledge_pkey PRIMARY KEY (temporal) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE timeknowledge OWNER TO postgres;";

    /****************************** timedata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS timedata CASCADE;";
    *ABMDataBase << "CREATE TABLE timedata ( temporal text NOT NULL,   timearg1 timestamp without time zone NOT NULL,   timearg2 timestamp without time zone NOT NULL,   CONSTRAINT timedata_temporal_fkey FOREIGN KEY (temporal)        REFERENCES timeknowledge (temporal) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE timedata OWNER  TO postgres;";

    /****************************** contextknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE contextknowledge ( name text NOT NULL,  argument text NOT NULL, dependance text NOT NULL,  instance integer NOT NULL,  CONSTRAINT contextknowledge_pkey PRIMARY KEY (instance),  CONSTRAINT contextknowledge_name_key UNIQUE (name, argument, dependance) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextknowledge OWNER TO postgres;";

    /****************************** contextdata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextdata CASCADE;";
    *ABMDataBase << "CREATE TABLE contextdata (  presencebegin boolean,  presenceend boolean,  instance integer NOT NULL,  id serial NOT NULL,  CONSTRAINT contextdata_instance_fkey FOREIGN KEY (instance) REFERENCES contextknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextdata OWNER  TO postgres;";

    /****************************** contextagent *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextagent CASCADE;";
    *ABMDataBase << "CREATE TABLE contextagent ( instance integer NOT NULL, agent text NOT NULL, number integer, CONSTRAINT contextagent_pkey PRIMARY KEY (instance, agent), CONSTRAINT contextagent_instance_fkey FOREIGN KEY (instance) REFERENCES contextknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextagent OWNER  TO postgres;";

    /*************************** sharedplan *******************************/
    *ABMDataBase << "DROP TABLE IF EXISTS sharedplan CASCADE;";
    *ABMDataBase << "CREATE TABLE sharedplan ( name text NOT NULL,  manner text NOT NULL,  instance integer NOT NULL,  CONSTRAINT sharedplan_pkey PRIMARY KEY (instance),  CONSTRAINT haredplan_name_key UNIQUE (name, manner) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE sharedplan OWNER TO postgres;";

    /**************************** sharedplandata **************************/
    *ABMDataBase << "DROP TABLE IF EXISTS sharedplandata CASCADE;";
    *ABMDataBase << "CREATE TABLE sharedplandata ( activitytype text NOT NULL, activityname text NOT NULL,   instance integer NOT NULL,   id integer NOT NULL,  CONSTRAINT sharedplandata_pkey PRIMARY KEY (instance, id ) , CONSTRAINT  sharedplandata_instance_fkey FOREIGN KEY (instance) REFERENCES sharedplan (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE sharedplandata OWNER TO postgres;";

    /**************************** sharedplanarg ***************************/
    *ABMDataBase << "DROP TABLE IF EXISTS sharedplanarg CASCADE";
    *ABMDataBase << "CREATE TABLE sharedplanarg ( instance integer NOT NULL, argument text NOT NULL, role text NOT NULL, CONSTRAINT sharedplanarg_pkey PRIMARY KEY (instance, role, argument),  CONSTRAINT sharedplanarg_instance_fkey FOREIGN KEY (instance) REFERENCES sharedplan (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE sharedplandata OWNER TO postgres";

    /************************** spadataarg ********************************/
    *ABMDataBase << "DROP TABLE IF EXISTS spdataarg CASCADE";
    *ABMDataBase << "CREATE TABLE spdataarg ( id integer NOT NULL, instance integer NOT NULL, argument text NOT NULL, role text NOT NULL, CONSTRAINT spdataarg_pkey PRIMARY KEY (id, instance, role, argument) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE spdataarg OWNER TO postgres,  ADD FOREIGN KEY (instance, id) REFERENCES sharedplandata (instance, id);";

    /************************  BEHAVIOR ***********************************/
    *ABMDataBase << " DROP TABLE IF EXISTS behavior CASCADE";
    *ABMDataBase << "CREATE TABLE behavior ( name text NOT NULL, argument text NOT NULL, instance integer NOT NULL, CONSTRAINT behavior_pkey PRIMARY KEY (instance), CONSTRAINT behavior_name_key UNIQUE (name, argument) ) WITH (OIDS=FALSE)";
    *ABMDataBase << "ALTER TABLE spatialknowledge OWNER TO postgres";

    /*********************** BEHAVIORDATA **********************************/
    *ABMDataBase << " DROP TABLE IF EXISTS behaviordata CASCADE";
    *ABMDataBase << " CREATE TABLE behaviordata ( drive text, effect double precision, instance integer NOT NULL, occurence integer NOT NULL, CONSTRAINT behaviordata_pkey PRIMARY KEY (occurence, instance, drive), CONSTRAINT behaviordata_instance_fkey FOREIGN KEY (instance) REFERENCES behavior (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE)";
    *ABMDataBase << " ALTER TABLE behaviordata OWNER TO postgres";

    /****************************** interactionknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS interactionknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE interactionknowledge (subject text NOT NULL, argument text NOT NULL, number integer NOT NULL, type text NOT NULL DEFAULT 'none'::text, role text NOT NULL DEFAULT 'none'::text, CONSTRAINT interactionknowledge_pkey PRIMARY KEY (subject, argument, type, role) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE interactionknowledge OWNER  TO postgres;";

    /****************************** adjectivespatial *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS adjectivespatial CASCADE;";
    *ABMDataBase << "CREATE TABLE adjectivespatial ( \"name\" text NOT NULL, argument text NOT NULL, x double precision, y double precision, dx double precision, dy double precision ) WITH(OIDS = FALSE) ";
    *ABMDataBase << "ALTER TABLE adjectivespatial OWNER TO postgres;";

    /****************************** adjectivetemporal *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS adjectivetemporal CASCADE;";
    *ABMDataBase << "CREATE TABLE adjectivetemporal ( \"name\" text NOT NULL, argument text NOT NULL, timing double precision) WITH(OIDS = FALSE) ";
    *ABMDataBase << "ALTER TABLE adjectivetemporal OWNER TO postgres;";

    bOutput.addString("knowledge database reset");
    return bOutput;
}

/*
* Erase of all the tables the data with instance given
* format input : eraseInstance n m ...
*/
Bottle autobiographicalMemory::eraseInstance(const Bottle &bInput)
{
    vector<int> vecToErase;
    Bottle bOutput, bInstances;

    if (bInput.size() < 2)
    {
        bOutput.addString("in autobiographicalMemory::eraseInstance | wrong number of input");
        return bOutput;
    }
    if (bInput.get(1).isList())
    {
        bInstances = *bInput.get(1).asList();
        int begin = 0, end = -1;

        if (bInstances.size() == 1)
        {
            vecToErase.push_back(atoi(bInstances.get(0).toString().c_str()));
        }
        else if (bInstances.size() == 2)
        {
            begin = atoi(bInstances.get(0).toString().c_str());
            end = atoi(bInstances.get(1).toString().c_str());
            for (int inst = begin; inst < end + 1; inst++)
            {
                vecToErase.push_back(inst);
            }
        }
        else
        {
            bOutput.addString("in autobiographicalMemory::eraseInstance | wrong number of input");
            return bOutput;
        }
    }
    else
    {
        for (int i = 1; i < bInput.size(); i++)
        {
            vecToErase.push_back(bInput.get(i).asInt());
        }
    }
    Bottle bRequest;
    ostringstream osRequest;
    for (vector<int>::iterator it = vecToErase.begin(); it != vecToErase.end(); it++)
    {
        osRequest.str("");
        osRequest << "DELETE FROM action WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM adjective WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM agent WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM drives WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM emotions WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM rtobject WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM object WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM relation WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM entity WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM contentarg WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM contentopc WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        //visualdata : remove from pg_largeobjects = unlink
        osRequest.str("");
        osRequest << "SELECT lo_unlink (img_oid) FROM (SELECT DISTINCT img_oid FROM visualdata WHERE instance = " << *it << ") AS visualdata_subquery ;";
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        //remove from images table
        osRequest.str("");
        osRequest << "DELETE FROM visualdata WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        //delete folders where images were stored in
        delete_directory(storingPath + "/" + std::to_string(*it) + "/");
        delete_directory(storingPath + "/" + storingTmpSuffix + "/" + std::to_string(*it) + "/");

        //remove from proprioceptivedata table
        osRequest.str("");
        osRequest << "DELETE FROM proprioceptivedata WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM sentencedata WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM sounddata WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM main WHERE instance = " << *it;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

    }
    bOutput.addString("instance(s) erased");
    return bOutput;
}

/*
* Put in the OPC all the words known in the ABM.
*/
// Not used, but can be seen as example how to use the OPC Client
/*Bottle autobiographicalMemory::populateOPC()
{
// 0. check if connected to the OPC.
OPCClient *opcWorld;
//Connection to the OPC
opcWorld = opcWorldReal;

Bottle bOutput;
if (!opcWorld->isConnected())
{
bOutput.addString("Error in autobiographicalMemory::populateOPC | OpcClient not connected.");
yError() << bOutput.toString();
return bOutput;
}

opcWorld->checkout();
opcWorld->update();

int incrAgent = 0,
incrObject = 0,
incrRTO = 0;

// 1. agent
Bottle bReply, bDistinctAgent;
bDistinctAgent = requestFromString("SELECT DISTINCT name FROM agent");

for (int iDA = 0; iDA < bDistinctAgent.size() && bDistinctAgent.toString() != "NULL"; iDA++)
{
string sName = bDistinctAgent.get(iDA).toString();
ostringstream osAgent;
if (opcWorld->getEntity(sName.c_str()) == NULL)
{
osAgent << "SELECT instance FROM agent WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1";
bReply = requestFromString(osAgent.str());
yInfo() << bReply.get(0).toString();
int instance = atoi(bReply.get(0).asList()->get(0).toString().c_str());

osAgent.str("");
osAgent << "SELECT color FROM agent WHERE (instance = " << instance << " and name = '" << sName << "')";

bReply = requestFromString(osAgent.str());
vector<int> colorAgent = tupleIntFromString(bReply.get(0).asList()->get(0).toString().c_str());

Agent *TempAgent = opcWorld->addEntity<Agent>(sName.c_str());

TempAgent->m_color.clear();
for (vector<int>::iterator it = colorAgent.begin(); it != colorAgent.end(); it++){
TempAgent->m_color.push_back(*it);
//TempAgent->m_color[0] = get<0>(colorAgent);
//TempAgent->m_color[1] = get<1>(colorAgent);
//TempAgent->m_color[2] = get<2>(colorAgent);
}
TempAgent->m_present = 0.0;

opcWorld->commit(TempAgent);
incrAgent++;
}
}

// 2. RTObjects :
Bottle bDistincRto = requestFromString("SELECT DISTINCT name FROM rtobject");

for (int iDTRO = 0; iDTRO < bDistincRto.size() && bDistincRto.toString() != "NULL"; iDTRO++)
{
string sName = bDistincRto.get(iDTRO).toString();
ostringstream osRto;
if (opcWorld->getEntity(sName.c_str()) == NULL)
{
osRto << "SELECT instance FROM rtobject WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1";
bReply = requestFromString(osRto.str());
int instance = atoi(bReply.get(0).asList()->get(0).toString().c_str());

osRto.str("");
osRto << "SELECT color FROM rtobject WHERE (instance = " << instance << " and name = '" << sName << "')";

bReply = requestFromString(osRto.str());
vector<int> colorRto = tupleIntFromString(bReply.get(0).asList()->get(0).toString().c_str());

RTObject *RTO = opcWorld->addEntity<RTObject>(sName.c_str());

//RTO->m_color[0] = get<0>(colorRto);
//RTO->m_color[1] = get<1>(colorRto);
//RTO->m_color[2] = get<2>(colorRto);
//RTO->m_present = 0.0;
RTO->m_color.clear();
for (vector<int>::iterator it = colorRto.begin(); it != colorRto.end(); it++){
RTO->m_color.push_back(*it);
}
RTO->m_present = 0.0;

opcWorld->commit(RTO);
incrRTO++;
}
}

// 3. Objects :
if (bPutObjectsOPC)
{
Bottle bDistinctObject = requestFromString("SELECT DISTINCT name FROM object");

for (int iDO = 0; iDO < bDistinctObject.size() && bDistinctObject.toString() != "NULL"; iDO++)
{
string sName = bDistinctObject.get(iDO).toString();
ostringstream osObject;
if (opcWorld->getEntity(sName.c_str()) == NULL)
{
osObject << "SELECT instance FROM object WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1";
bReply = requestFromString(osObject.str());
int instance = atoi(bReply.get(0).asList()->get(0).toString().c_str());

osObject.str("");
osObject << "SELECT color FROM object WHERE (instance = " << instance << " and name = '" << sName << "')";

bReply = requestFromString(osObject.str());
vector<int> colorObject = tupleIntFromString(bReply.get(0).asList()->get(0).toString().c_str());

Object *TempObj = opcWorld->addEntity<Object>(sName.c_str());
//TempObj->m_color[0] = get<0>(colorObject);
//TempObj->m_color[1] = get<1>(colorObject);
//TempObj->m_color[2] = get<2>(colorObject);
TempObj->m_color.clear();
for (vector<int>::iterator it = colorObject.begin(); it != colorObject.end(); it++){
TempObj->m_color.push_back(*it);
}

osObject.str("");
osObject << "SELECT dimension FROM object WHERE (instance = " << instance << " and name = '" << sName << "')";

bReply = requestFromString(osObject.str());
vector<double> sizeObject = tupleDoubleFromString(bReply.get(0).asList()->get(0).toString().c_str());

//------------------------------------------------- SHOULD BE DIMENSION!!!!!!
//TempObj->m_color[0] = get<0>(sizeObject);
//TempObj->m_color[1] = get<1>(sizeObject);
//TempObj->m_color[2] = get<2>(sizeObject);
TempObj->m_dimensions.clear();
for (vector<double>::iterator it = sizeObject.begin(); it != sizeObject.end(); it++){
TempObj->m_color.push_back(*it);
}

osObject.str("");
osObject << "SELECT orientation FROM object WHERE (instance = " << instance << " and name = '" << sName << "')";

bReply = requestFromString(osObject.str());
vector<double> orientationObject = tupleDoubleFromString(bReply.get(0).asList()->get(0).toString().c_str());

//TempObj->m_ego_orientation[0] = get<0>(orientationObject);
//TempObj->m_ego_orientation[1] = get<1>(orientationObject);
//TempObj->m_ego_orientation[2] = get<2>(orientationObject);
TempObj->m_ego_orientation.clear();
for (vector<double>::iterator it = orientationObject.begin(); it != orientationObject.end(); it++){
TempObj->m_color.push_back(*it);
}

TempObj->m_present = 0.0;

opcWorld->commit(TempObj);
incrObject++;
}
}
}

ostringstream osOutput;
osOutput << endl << incrAgent << " Agent(s) - " << incrObject << " Object(s) - " << incrRTO << " RTObject(s) added in the OPC." << endl;

yInfo() << osOutput.str();
bOutput.addString(osOutput.str().c_str());

return bOutput;
}*/

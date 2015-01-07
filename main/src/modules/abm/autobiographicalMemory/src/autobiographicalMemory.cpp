#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include "autobiographicalMemory.h"

using namespace yarp::sig;
using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;

/* configure the module */
bool autobiographicalMemory::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name", Value("autobiographicalMemory"), "module name (string)").asString();

    setName(moduleName.c_str());

    //conf group for database properties
    Bottle &bDBProperties = rf.findGroup("database_properties");
    server = bDBProperties.check("server", Value("127.0.0.1")).asString();
    user = bDBProperties.check("user", Value("postgres")).asString();
    password = bDBProperties.check("password", Value("postgres")).asString();
    dataB = bDBProperties.check("dataB", Value("ABM")).asString();
    savefile = (rf.getContextPath() + "/saveRequest.txt").c_str();

    ABMDataBase = new DataBase<PostgreSql>(server, user, password, dataB);

    //conf group for image storing properties
    Bottle &bISProperties = rf.findGroup("image_storing");
    storingPath = bISProperties.check("storingPath", Value("C:/robot/ABMStoring")).asString();
    storingTmpSuffix = bISProperties.check("storingTmpSuffix", Value("tmp")).asString();
    imgFormat = bISProperties.check("imgFormat", Value("tif")).asString();

    // TODO: streamStatus should be changed to enum
    streamStatus = "none"; //none, record, stop, send
    imgLabel = "defaultLabel";
    imgInstance = -1;
    imgNb = 0;

    bPutObjectsOPC = false;

    portEventsName = "/" + getName() + "/request:i";
    portEventsIn.open(portEventsName.c_str());

    string portHandlerName = "/" + getName() + "/rpc";
    handlerPort.open(portHandlerName.c_str());

    string name_abm2reasoning = "/" + getName() + "/to_reasoning";
    abm2reasoning.open(name_abm2reasoning.c_str());

    //port for images:
    string name_imagePortOut = "/" + getName() + "/images/out";
    imagePortOut.open(name_imagePortOut.c_str());

    //Temp to test data : will be send without checking connection after
    if (Network::connect(imagePortOut.getName().c_str(), "/yarpview/img:i")) {
        cout << endl << "ABM is now connected to Yarpview!" << endl;
    } else {
        cout << endl << "ABM failed to connect to Yarpview!" << endl;
    }

    //create the storingPath and the tmp also
    string fullTmpPath = storingPath + "/" + storingTmpSuffix;
    yarp::os::mkdir(storingPath.c_str());
    yarp::os::mkdir(fullTmpPath.c_str());

    isconnected2reasoning = false;

    attach(handlerPort);

    Bottle bConnect;
    bConnect.addString("connect");
    bConnect.addString("OPC");
    connectOPC(bConnect);

    //populateOPC();
    storeOID();

    cout << endl << endl << "----------------------------------------------";
    cout << endl << endl << "autobiographicalMemory ready ! " << endl << endl;

    return true;
}

/* Create a new database (erase the current one) */
Bottle autobiographicalMemory::newDB(Bottle bInput)
{
    Bottle bOutput;
    if (bInput.size() != 2)
    {
        cout << "password required" << endl;
        bOutput.addString("Error");
        bOutput.addString("password required");
        return bOutput;
    }
    else
    {
        if (bInput.get(1).asString().c_str() != password)
        {
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
    *ABMDataBase << "CREATE TABLE main(idActivity serial NOT NULL, time timestamp without time zone NOT NULL,activityname text, activitytype text, instance integer NOT NULL UNIQUE,begin boolean NOT NULL,CONSTRAINT main_pkey PRIMARY KEY (time)) WITH (OIDS=FALSE);";
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

    /****************************** images *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS images CASCADE;";
    *ABMDataBase << "CREATE TABLE images(\"time\" timestamp without time zone, img_provider_port text, instance integer NOT NULL, label text, relative_path text, img_oid oid, CONSTRAINT img_pkey PRIMARY KEY(\"time\", img_provider_port), CONSTRAINT images_instance_fkey FOREIGN KEY(instance) REFERENCES main(instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH(OIDS = FALSE); ALTER TABLE images OWNER TO postgres;";
    // DEPRECATED *ABMDataBase << "CREATE TABLE images (instance integer NOT NULL, label text, img_oid oid NOT NULL, filename text, CONSTRAINT img_id PRIMARY KEY (img_oid), CONSTRAINT images_instancee_fkey FOREIGN KEY (instance) REFERENCES main (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE images OWNER  TO postgres;";

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
        cerr << "Exception during request : " << e.what() << endl;
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
    cout << endl << "Got something, echo is on" << endl;
    cout << bCommand.toString() << endl << endl;
    bError.addString("ERROR");

    if (bCommand.get(0).isString())
    {
        if (bCommand.get(0) == "quit" || bCommand.get(0) == "close")
        {
            bReply.addString("close module");
            portEventsIn.reply(bReply);
            interruptModule();
            return false;
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
            bReply = snapshot(bCommand);
        }
        else if (bCommand.get(0) == "snapshotSP")
        {
            bReply = snapshotSP(bCommand);
        }
        else if (bCommand.get(0) == "snapshotBE")
        {
            bReply = snapshotBehavior(bCommand);
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

        // ask for streaming data : they will be sent through ports opened by autobiographicalMemory based on the original image provider port (/autobiographicalMemory/imgProviderPortName)
        else if (bCommand.get(0) == "sendStreamImage") // TODO: Check for timingEnabled
        {
            if ((bCommand.size() > 1) && (bCommand.get(1).isList()))
            {
                imgInstance = bCommand.get(1).asList()->get(0).asInt();
                int nbSentImages = sendStreamImage(imgInstance);
                if (nbSentImages > 0){
                    bReply.addString(streamStatus);
                    bReply.addInt(nbSentImages);
                }
            }
            else
            {
                bError.addString("in sendStreamImage : number of element incorrect : sendStreamImage (instance)");
                bReply = bError;
            }
        }

        //ask for a single image : a single image for EACH image provider so you may end up by several of them
        // bReply: ack ( (labelProvider1 (image1.1)) (labelProvider2 (image1.2)) (labelProvider3 (image1.3)) )
        else if (bCommand.get(0) == "askImage")
        {
            if (bCommand.size() > 1)
            {
                int instance = (atoi((bCommand.get(1)).toString().c_str()));
                if (instance > 0){
                    bReply = askImage(instance);
                }
                else {
                    bError.addString("in askImage : not valid int number for the instance");
                    bReply = bError;
                }
            }
            else
            {
                bError.addString("ERROR in askImage : wrong number of element -> askImage instanceNb");
                bReply = bError;
            }
        }
        // add an image provider for the following stream recording : addImgProvider (label /yarp/port/img/provider)
        else if (bCommand.get(0) == "addImgProvider")
        {
            if (bCommand.size() > 1 && bCommand.get(1).isList())
            {
                if (bCommand.get(1).asList()->size() == 2){
                    bReply = addImgProvider(bCommand.get(1).asList()->get(0).toString().c_str(), bCommand.get(1).asList()->get(1).toString().c_str());
                }
                else {
                    bError.addString("[addImgProvider] : wrong number of element -> addImgProvider (label /yarp/port/img/provider)");
                    bReply = bError;
                }
            }
            else
            {
                bError.addString("[addImgProvider] : wrong number of element -> addImgProvider (label /yarp/port/img/provider)");
                bReply = bError;
            }
        }

        //remove an image provider from the list of available stream images provider
        else if (bCommand.get(0) == "removeImgProvider")
        {
            if (bCommand.size() > 1)
            {
                string labelImgProvider = bCommand.get(1).toString().c_str();
                bReply = removeImgProvider(labelImgProvider);
            }
            else
            {
                bError.addString("[removeImgProvider] : wrong number of element -> removeImgProvider label");
                bReply = bError;
            }
        }
        else if (bCommand.get(0) == "storeOID")
        {
            if(storeOID())
                bReply.addString("ack");
            else
                bReply.addString("storeOID failed");
        }
    }
    else
    {
        bError.addString("wrong input bottle format");
        bReply = bError;
    }

    portEventsIn.reply(bReply);

    return true;
}

/* rpc update module */
bool autobiographicalMemory::updateModule() {
    //we have received a snapshot command indicating an activity that take time so streaming is needed
    //currently it is everything BUT a sentence
    if (streamStatus == "begin") {
        cout << "============================= STREAM BEGIN =================================" << endl;

        imgInstance = currentInstance; //currentInstance is different from begin/end : imgInstance instanciated just at the beginning and use for the whole stream to assure the same instance id

        stringstream imgInstanceString; imgInstanceString << imgInstance;
        string currentPathFolder = storingPath + "/" + imgInstanceString.str();
        if (yarp::os::mkdir(currentPathFolder.c_str()) == -1) {
            cout << "WARNING: folder " << currentPathFolder << " already exists or could not be created!" << endl;
        }

        storeImageAllProviders();

        //init of the stream record done: go through the classic record phase
        streamStatus = "record";
    }
    else if (streamStatus == "record") {
        imgNb += 1;
        //cout << "Image Nb " << imgNb << endl;
        storeImageAllProviders();
    }
    else if (streamStatus == "send") { //stream to send, because rpc port receive a sendStreamImage query

        //select all the images (through relative_path and image provider) corresponding to a precise instance
        if (imgNb == 0) {
            cout << "============================= STREAM SEND =================================" << endl;
            bListImages.clear();
            bListImages.addString("request");
            ostringstream osArg;
            osArg << "SELECT relative_path, img_provider_port, time, ";
            osArg << "EXTRACT(EPOCH FROM time-(SELECT time FROM images WHERE instance = '" << imgInstance << "'  ORDER BY time LIMIT 1)) * 1000000 as time_difference ";
            osArg << "FROM images WHERE instance = '" << imgInstance << "' ORDER BY time;";
            bListImages.addString(osArg.str());
            bListImages = request(bListImages);

            //cout << "bListImages : " << bListImages.toString() << endl ;
            //cout << "bListImages size : " << bListImages.size() << endl ;

            timeStreamStart = getCurrentTimeInMS();
        }

        //If we currently have images left to be send
        if (imgNb < bListImages.size()) {
            // hack to make sure we don't send images where
            // not all providers have an image stored
            bool timesMatch = false;
            // loop until we have a match found
            while(!timesMatch && imgNb < bListImages.size()) {
                timesMatch = true;
                // store time of the imgNb-th image
                string imgNbTimeFirstImage = bListImages.get(imgNb).asList()->get(2).asString();
                // compare the time of the imgNb-th image with the imgNb+i-th image
                // if they are different, there is something wrong. in this case,
                // just increase imgNb by one and we go back to the start of the loop
                for(unsigned int i=1; i<mapStreamImgPortOut.size(); i++) {
                    string imgNbTime = bListImages.get(imgNb+i).asList()->get(2).asString();
                    if(imgNbTime != imgNbTimeFirstImage) {
                        cout << "Skip image " << imgNb << " because matching image is not present" << endl;
                        timesMatch = false;
                        imgNb++;
                        continue;
                    }
                }
            }
            // hack end


            long timeStreamCurrent = getCurrentTimeInMS();

            long updateTimeDifference = timeStreamCurrent - timeStreamStart;
            long streamTimeDifference = atol(bListImages.get(imgNb).asList()->get(3).toString().c_str());

            if(updateTimeDifference >= streamTimeDifference || !timingEnabled) {
                cout << "Send out image " << imgNb << endl;
                for(unsigned int i=0; i<mapStreamImgPortOut.size(); i++) {
                    //concatenation of the storing path
                    stringstream fullPath;
                    fullPath << storingPath << "/" << storingTmpSuffix << "/" << bListImages.get(imgNb).asList()->get(0).asString().c_str();
                    BufferedPort<ImageOf<PixelRgb> >* port = mapStreamImgPortOut.at(bListImages.get(imgNb).asList()->get(1).asString().c_str());

                    //cout << "Send image " << imgNb << ": " << fullPath.str() << endl;
                    sendImage(fullPath.str(), port);

                    //next image
                    imgNb += 1;
                }
            } else {
                cout << "Image not send yet, due to time control" << endl;
            }
        } else {
            //Close ports which were opened in sendStreamImage
            for (std::map<string, BufferedPort<ImageOf<PixelRgb> >*>::const_iterator it = mapStreamImgPortOut.begin(); it != mapStreamImgPortOut.end(); ++it)
            {
                it->second->interrupt();
                it->second->close();
                if(!it->second->isClosed()) {
                    cout << "Error, port " << it->first << " could not be closed" << endl;
                }
            }

            mapStreamImgPortOut.clear();

            streamStatus = "end";
        }
    }

    //go back to default global value
    if (streamStatus == "end") {
        cout << "============================= STREAM STOP =================================" << endl;

        //TODO:
        //startThread with storeOID()

        //close folder and SQL entry
        streamStatus = "none";
        imgLabel = "defaultLabel";
        imgInstance = -1;
        imgNb = 0;
    }

    return true;
}

bool autobiographicalMemory::interruptModule()
{
    cout << "Interrupting your module, for port cleanup" << endl;

    storeOID();

    opcWorld->interrupt();
    handlerPort.interrupt();
    portEventsIn.interrupt();
    abm2reasoning.interrupt();
    imagePortOut.interrupt();

    return true;
}

bool autobiographicalMemory::close()
{
    cout << "Calling close function" << endl;

    disconnectImgProviders();

    opcWorld->interrupt();
    opcWorld->close();

    handlerPort.interrupt();
    handlerPort.close();

    portEventsIn.interrupt();
    portEventsIn.close();

    abm2reasoning.interrupt();
    abm2reasoning.close();

    imagePortOut.interrupt();
    imagePortOut.close();

    storeOID();

    delete opcWorld;
    delete ABMDataBase;

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

    bOutput.addString("knowledge database reset");
    return bOutput;
}

/*
* Erase of all the tables the data with instance given
* format input : eraseInstce n m ...
*/
Bottle autobiographicalMemory::eraseInstance(Bottle bInput)
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

        if (bInstances.size() == 2)
        {
            begin = atoi(bInstances.get(0).toString().c_str());
            end   = atoi(bInstances.get(1).toString().c_str());
        }
        for (int inst = begin; inst < end + 1; inst++)
        {
            vecToErase.push_back(inst);
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

        //images : remove from pg_largeobjects = unlink
        osRequest.str("");
        osRequest << "SELECT lo_unlink (img_oid) FROM (SELECT DISTINCT img_oid FROM images WHERE instance = " << *it << ") AS images_subquery ;";
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        //remove from images table
        osRequest.str("");
        osRequest << "DELETE FROM images WHERE instance = " << *it;
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
Bottle autobiographicalMemory::populateOPC()
{
    // 0. check if connected to the OPC.

    Bottle bOutput;
    if (!opcWorld->isConnected())
    {
        bOutput.addString("Error in autobiographicalMemory::populateOPC | OpcClient not connected.");
        cout << bOutput.toString() << endl;
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

    for (int iDA = 0; iDA < bDistinctAgent.size(); iDA++)
    {
        string sName = bDistinctAgent.get(iDA).toString();
        ostringstream osAgent;
        if (opcWorld->getEntity(sName.c_str()) == NULL)
        {
            osAgent << "SELECT instance FROM agent WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1";
            bReply = requestFromString(osAgent.str());
            cout << bReply.get(0).toString() << endl;
            int instance = atoi(bReply.get(0).asList()->get(0).toString().c_str());

            osAgent.str("");
            osAgent << "SELECT color FROM agent WHERE (instance = " << instance << " and name = '" << sName << "')";

            bReply = requestFromString(osAgent.str());
            vector<int> colorAgent = tupleIntFromString(bReply.get(0).asList()->get(0).toString().c_str());

            Agent *TempAgent = opcWorld->addAgent(sName.c_str());

            TempAgent->m_color.clear();
            for (vector<int>::iterator it = colorAgent.begin(); it != colorAgent.end(); it++){
                TempAgent->m_color.push_back(*it);
                //TempAgent->m_color[0] = get<0>(colorAgent);
                //TempAgent->m_color[1] = get<1>(colorAgent);
                //TempAgent->m_color[2] = get<2>(colorAgent);
            }
            TempAgent->m_present = false;

            opcWorld->commit(TempAgent);
            incrAgent++;
        }
    }

    // 2. RTObjects : 
    Bottle bDistincRto = requestFromString("SELECT DISTINCT name FROM rtobject");

    for (int iDTRO = 0; iDTRO < bDistincRto.size(); iDTRO++)
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

            RTObject *RTO = opcWorld->addRTObject(sName.c_str());

            //RTO->m_color[0] = get<0>(colorRto);
            //RTO->m_color[1] = get<1>(colorRto);
            //RTO->m_color[2] = get<2>(colorRto);    
            //RTO->m_present = false;
            RTO->m_color.clear();
            for (vector<int>::iterator it = colorRto.begin(); it != colorRto.end(); it++){
                RTO->m_color.push_back(*it);
            }
            RTO->m_present = false;

            opcWorld->commit(RTO);
            incrRTO++;
        }
    }

    // 3. Objects : 
    if (bPutObjectsOPC)
    {
        Bottle bDistinctObject = requestFromString("SELECT DISTINCT name FROM object");

        for (int iDO = 0; iDO < bDistinctObject.size(); iDO++)
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

                Object *TempObj = opcWorld->addObject(sName.c_str());
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

                TempObj->m_present = false;

                opcWorld->commit(TempObj);
                incrObject++;
            }
        }
    }

    ostringstream osOutput;
    osOutput << endl << incrAgent << " Agent(s) - " << incrObject << " Object(s) - " << incrRTO << " RTObject(s) added in the OPC." << endl;

    cout << osOutput.str() << endl;
    bOutput.addString(osOutput.str().c_str());

    return bOutput;
}

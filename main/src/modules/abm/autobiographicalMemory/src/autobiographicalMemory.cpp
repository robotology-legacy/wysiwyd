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
using namespace cv;

autobiographicalMemory::autobiographicalMemory(ResourceFinder &rf)
{
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

    inSharedPlan = false;

    // TODO: streamStatus should be changed to enum
    streamStatus = "none"; //none, record, stop, send
    imgLabel = "defaultLabel";
    imgInstance = -1;
    imgNb = 0;
    imgNbInStream = 0;
}

autobiographicalMemory::~autobiographicalMemory()
{
    delete ABMDataBase;
}

/* configure the module */
bool autobiographicalMemory::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("autobiographicalMemory"), "module name (string)").asString();

    setName(moduleName.c_str());

    bPutObjectsOPC = false;

    portEventsName = "/" + getName() + "/request:i";
    portEventsIn.open(portEventsName.c_str());

    string portHandlerName = "/";
    portHandlerName += getName() + "/rpc";
    handlerPort.open(portHandlerName.c_str());

    string name_abm2reasoning = "/";
    name_abm2reasoning += getName() + "/to_reasoning";
    abm2reasoning.open(name_abm2reasoning.c_str());
    //Network::connect(name_abm2reasoning.c_str(), "/efaa/abmReasoning/rpc");

    //port for images:
    string name_imagePortOut = "/";
    name_imagePortOut += getName() + "/images/out";
    imagePortOut.open(name_imagePortOut.c_str());

    string name_imagePortIn = "/";
    name_imagePortIn += getName() + "/images/in";
    imagePortIn.open(name_imagePortIn.c_str());

    //Temp to test data : will be send without checking connection after
    bool isconnected2Yarpview = Network::connect(imagePortOut.getName().c_str(), "/yarpview/img:i");

    if (isconnected2Yarpview) {
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
    //attachTerminal();

    Bottle bConnect;
    bConnect.addString("connect");
    bConnect.addString("OPC");
    connectOPC(bConnect);

    //populateOPC();

    cout << endl << endl << "----------------------------------------------";
    cout << endl << endl << "autobiographicalMemory ready ! " << endl << endl;

    return true;
}

/* transforms an result of query to a bottle */
Bottle autobiographicalMemory::restoBottle(ResultSet bResult)
{
    vector<string> vRow;

    //catch each row of the ResultSet
    Bottle bOutput;
    bool isResult = false;
    //output.addString("request");
    while (bResult.fetch(vRow))
    {
        isResult = true;
        Bottle bLineBottle;
        //parse the current row of the result and add each column values to a bottle
        for (size_t i = 0; i < vRow.size(); i++)
        {
            bLineBottle.addString(vRow[i].c_str());
        }

        //add the line bottle to the reply bottle
        //printf("Bottle lineBottle done : %s\n", lineBottle.toString().c_str());
        //cout << "lineBottle size : " << lineBottle.size() << endl;
        bOutput.addList() = bLineBottle;
    }
    if (!isResult)
    {
        bOutput.clear();
        bOutput.addString("NULL");
    }
    //cout << "output size : " << output.size() << endl;
    return bOutput;
}

/* Send a query to the DB. bRequest must be the complete request */
Bottle autobiographicalMemory::request(Bottle bRequest)
{
    //prepare the ResultSet of the query and the reply
    ResultSet rs1;
    Bottle bReply;
    bReply.clear();
    //cout << "Request : "<< bRequest.get(1).asString().c_str() << endl;

    //send the request to the database
    try
    {
        //verbose debug
        //cout << "Request : "<< bRequest.get(1).asString().c_str() << endl;

        *ABMDataBase << bRequest.get(1).asString().c_str(), rs1;
        bReply = restoBottle(rs1);

        //verbose print reply
        //cout << "bReply = " << bReply.toString().c_str() << endl ;

    }
    catch (DataBaseError& e)
    {
        cerr << "Exception during request: " << e.what() << endl;
        string sExcept;
        sExcept = "Exception during request: ";
        sExcept += e.what();
        bReply.addString(sExcept.c_str());
    }

    return bReply;
}


Bottle autobiographicalMemory::requestFromString(string sInput)
{
    Bottle bReplyRequest;
    //send the SQL query within a bottle to autobiographicalMemory
    Bottle bQuery;
    bQuery.addString("request");
    bQuery.addString(sInput.c_str());
    return request(bQuery);
}

/* TODO */
Bottle autobiographicalMemory::save(Bottle bInput)
{
    //TODO
    Bottle bOutput;
    bOutput = bInput;
    //output.addString("file saved");
    return bOutput;
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
    *ABMDataBase << "CREATE TABLE images (instance integer NOT NULL, label text, img_oid oid NOT NULL, filename text, CONSTRAINT img_id PRIMARY KEY (img_oid), CONSTRAINT images_instancee_fkey FOREIGN KEY (instance) REFERENCES main (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH (OIDS=FALSE);";
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

    /********************************************** build the database startup **********************************************/
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
        string sExcept;
        sExcept = "Exception during request : ";
        sExcept += e.what();
        bReply.addString(sExcept.c_str());
    }

    //send the request to the database
    return bReply;
}

Bottle autobiographicalMemory::addImgProvider(string labelImgProvider, string portImgProvider)
{
    //prepare the ResultSet of the query and the reply
    Bottle bReply;

    if (mapImgProvider.find(labelImgProvider) == mapImgProvider.end()) //key not found
    {
        //creating imgReceiverPort for the current provider
        string portImgReceiver = "/" + getName() + "/images/" + labelImgProvider + "/in";

        //add the imgProvider to the map, imgReceiver to map
        mapImgProvider[labelImgProvider] = portImgProvider;
        //mapImgReceiver[labelImgProvider] = portImgReceiver;
        mapImgReceiver[labelImgProvider] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> > ;
        //mapImgReceiver[labelImgProvider]->open(portImgReceiver);

        bReply.addString("[ack]");
    }
    else { //key found
        cout << "ERROR : addImgProvider : " << labelImgProvider << " is already present! " << endl;
        bReply.addString("ERROR : addImgProvider : " + labelImgProvider + " is already present! ");
    }

    //send the reply
    return bReply;
}

Bottle autobiographicalMemory::removeImgProvider(string labelImgProvider)
{
    //prepare the ResultSet of the query and the reply
    Bottle bReply;

    if (mapImgProvider.find(labelImgProvider) == mapImgProvider.end()) { //key not found
        cout << "ERROR : removeImgProvider : " << labelImgProvider << " is NOT present! " << endl;
        bReply.addString("ERROR : removeImgProvider : " + labelImgProvider + " is NOT present! ");
    }
    else { //key found
        mapImgProvider.erase(labelImgProvider);
        mapImgReceiver[labelImgProvider]->interrupt();
        mapImgReceiver[labelImgProvider]->close();
        mapImgReceiver.erase(labelImgProvider);
        bReply.addString("[ack]");
    }

    //send the reply
    return bReply;
}


double autobiographicalMemory::getPeriod()
{
    return 0.03; //module periodicity (seconds)
}

/* rpc respond module */
bool autobiographicalMemory::respond(const Bottle& bCommand, Bottle& bReply)
{
    Bottle bError;
    bReply.clear();
    cout << endl << "Got something, echo is on" << endl;
    cout << bCommand.toString() << endl << endl;
    bError.addString("ERROR");

    //check connection with the image provider
    //isconnected2Cam = Network::isConnected(robotPortCam, imagePortIn.getName().c_str());
    //check for kinect provider
    //isconnected2Kinect = Network::isConnected(robotPortKin, imagePortIn.getName().c_str());

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
                bError.addString("Canot open the file");
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
        else if (bCommand.get(0) == "save")
        {
            bReply = save(bCommand);
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
        //DEPRECATED = not needed anymore, save stream in snapshot when "action"
        else if (bCommand.get(0) == "testSaveStreamImage")
        {
            if (!isconnected2Cam && !isconnected2Kinect) {
                cout << "ABM failed to connect to Camera and Kinect!" << endl;
                bError.addString("in testSaveStreamImage :  Error, connetion missing between" + robotPortCam + "or" + "/kinectServer/image:o " + "and " + imagePortIn.getName().c_str());
                bReply = bError;
            }
            else if ((bCommand.size() > 1) && (bCommand.get(1).asList()->size() > 1))
            {
                streamStatus = bCommand.get(1).asList()->get(0).asString();
                imgLabel = bCommand.get(1).asList()->get(1).asString();
                bReply.addString(streamStatus);
            }
            else
            {
                bError.addString("in testSaveStreamImage : number of element incorrect : (testSaveStreamImage (begin/end, imgLabel))");
                bReply = bError;
            }
        }
        //sendStreamImage (instance)
        else if (bCommand.get(0) == "sendStreamImage")
        {
            if (!Network::isConnected(imagePortOut.getName(), "/yarpview/img:i")) {
                cout << "ABM failed to connect to Yarpview!" << endl;
                bError.addString("in sendStreamImage :  Error, connetion missing between " + imagePortOut.getName() + " and /yarpview/img:i ");
                bReply = bError;
            }
            else if ((bCommand.size() > 1) && (bCommand.get(1).isList()))
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
                bError.addString("in sendStreamImage : number of element incorrect : testSendStreamImage (instance)");
                bReply = bError;
            }
        }
        //DEPRECATED = in snapshot when "sentence"
        else if (bCommand.get(0) == "testSaveImage")
        {
            if (!isconnected2Cam && !isconnected2Kinect) {
                cout << "ABM failed to connect to Camera or Kinect!" << endl;
                bError.addString("in testSaveImage :  Error, connetion missing between" + robotPortCam + " or /kinectServer/image:o " + " and " + imagePortIn.getName().c_str());
                bReply = bError;
            }
            else if ((bCommand.size() > 1) && (bCommand.get(1).asList()->size() > 1))
            {
                bReply = testSaveImage(bCommand);
            }
            else
            {
                bError.addString("in testSaveImage : number of element insufficient : (testSaveImage (label, filename))");
                bReply = bError;
            }
        }
        //askImage
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
        else if (bCommand.get(0) == "setImgProvider")
        {
            if (bCommand.size() > 1)
            {
                string provider = bCommand.get(1).toString().c_str();

                if (provider != "kinect" && provider != "cam" && provider != "none") {
                    bError.addString("ERROR in setImgProvider : " + provider + " is not a valid imgProvider => kinect or cam or none");
                    bReply = bError;
                }
                else {
                    bReply.addString("ack");
                    imgProvider = provider;
                }
            }
            else
            {
                bError.addString("ERROR in setImgProvider : wrong number of element -> setImgProvider provider");
                bReply = bError;
            }
        }
        // setCustomImgProvider (label /yarp/port/img/provider)
        else if (bCommand.get(0) == "addImgProvider")
        {
            if (bCommand.size() > 1 && bCommand.get(1).isList())
            {
                if (bCommand.get(1).asList()->size() == 2){
                    //TODO : check that the label is not used

                    //TODO : several custom at the same time (fill in dictionnary)                   
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
        else if (bCommand.get(0) == "removeImgProvider")
        {
            if (bCommand.size() > 1)
            {
                string labelImgProvider = bCommand.get(1).toString().c_str();

                //TODO : remove it from the list of imgProvider

                bReply = removeImgProvider(labelImgProvider);
            }
            else
            {
                bError.addString("[removeImgProvider] : wrong number of element -> removeImgProvider label");
                bReply = bError;
            }
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
bool autobiographicalMemory::updateModule()
{
    if (streamStatus == "begin") {
        cout << "============================= STREAM BEGIN =================================" << endl;
        //create folder
        currentPathFolder = "";
        currentPathFolder += storingPath + "/" + imgLabel;

        //if -1 : repo already there
        if (yarp::os::mkdir(currentPathFolder.c_str()) == -1){
            cout << "WARNING :  folder already exist, add getTime to it!" << endl;
            string folderWithTime = imgLabel;
            string currentTime = getCurrentTime();

            //need to change ':' and ' ' by _ for folder name
            replace(currentTime.begin(), currentTime.end(), ':', '_');
            replace(currentTime.begin(), currentTime.end(), ' ', '_');
            folderWithTime += currentTime;

            currentPathFolder = "";
            currentPathFolder += storingPath + "/" + folderWithTime;

            yarp::os::mkdir(currentPathFolder.c_str());
        }

        cout << "Going to create folder : " << currentPathFolder << endl;

        imgInstance = currentInstance; //currentInstance is different from begin/end : imgInstance instanciated just at the beginning and use for the whole stream to assure the same instance id

        //go through the ImgReceiver ports
        for (std::map<string, BufferedPort<ImageOf<PixelRgb>>*>::const_iterator it = mapImgReceiver.begin(); it != mapImgReceiver.end(); ++it)
        {
            //concatenation of the path to store
            char imgName[512] = "";
            char fullPath[512] = "";

            stringstream ssImgName;
            ssImgName << imgLabel << imgNb << "_" << it->first << "." << imgFormat;
            strcpy(imgName, ssImgName.str().c_str());

            stringstream ssPath;
            ssPath << currentPathFolder << "/" << imgName;
            strcpy(fullPath, ssPath.str().c_str());

            if (!createImage(fullPath, it->second)) {
                cout << "Error in Update : image not created from " << it->first << endl;
            }
            else {
                //create SQL entry, register the cam image in specific folder
                storeImage(imgInstance, imgLabel, fullPath, imgName, getCurrentTime(), mapImgProvider[it->first]);
            }
        }

        //init of the stream record done : go through the classic record phase
        streamStatus = "record";
    }
    else if (streamStatus == "record") {
        imgNb += 1;
        //cout << "Image Nb " << imgNb << endl;

        //go through the ImgReceiver ports
        for (std::map<string, BufferedPort<ImageOf<PixelRgb>>*>::const_iterator it = mapImgReceiver.begin(); it != mapImgReceiver.end(); ++it)
        {
            //concatenation of the path to store
            char imgName[512] = "";
            char fullPath[512] = "";

            stringstream ssImgName;
            ssImgName << imgLabel << imgNb << "_" << it->first << "." << imgFormat;
            strcpy(imgName, ssImgName.str().c_str());

            stringstream ssPath;
            ssPath << currentPathFolder << "/" << imgName;
            strcpy(fullPath, ssPath.str().c_str());

            if (!createImage(fullPath, it->second)) {
                cout << "Error in Update : image not created from " << it->first << endl;
            } else {
                //create SQL entry, register the cam image in specific folder
                storeImage(imgInstance, imgLabel, fullPath, imgName, getCurrentTime(), mapImgProvider[it->first]);
            }
        }
    }
    else if (streamStatus == "send") { //stream to send
        //select all the images (through primary key oid) corresponding to a precise instance
        if (imgNb == 0) {
            cout << "============================= STREAM SEND =================================" << endl;
            bListImages.addString("request");
            ostringstream osArg;
            osArg << "SELECT filename FROM images WHERE label = '" << imgLabel << "';";
            bListImages.addString(osArg.str());
            bListImages = request(bListImages);

            //cout << "bListImages : " << bListImages.toString() << endl ;
            //cout << "bListImages size : " << bListImages.size() << endl ;
        }

        //If we currently have images left to be send
        if (imgNb < bListImages.size()) {
            //cout << "image number " << imgNb << endl ;

            //concatenation of the storing path
            char fullPath[512] = "";

            stringstream ss;
            ss << storingPath << "/" << storingTmpSuffix << "/" << bListImages.get(imgNb).asString().c_str();
            strcpy(fullPath, ss.str().c_str());

            sendImage(fullPath);
        } else {
            streamStatus = "end";
            //cout << "============================= STREAM END =================================" << endl;
        }

        //next image
        imgNb += 1;
    }

    //go back to default global value
    if (streamStatus == "end") {
        cout << "============================= STREAM STOP =================================" << endl;
        //close folder and SQL entry

        Network::disconnect(imgProviderPort, imagePortIn.getName().c_str());

        streamStatus = "none";
        imgLabel = "defaultLabel";
        imgInstance = -1;
        imgNb = 0;
        imgNbInStream = 0;
    }

    return true;
}

bool autobiographicalMemory::interruptModule()
{
    cout << "Interrupting your module, for port cleanup" << endl;

    // TODO : save the database
    opcWorld->interrupt();
    handlerPort.interrupt();
    portEventsIn.interrupt();
    abm2reasoning.interrupt();
    imagePortOut.interrupt();
    imagePortIn.interrupt();

    return true;
}


bool autobiographicalMemory::close()
{
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

    imagePortIn.interrupt();
    imagePortIn.close();

    cout << "Calling close function\n";
    delete opcWorld;
    return true;
}

/* each interaction stored in the DB is save in a file text */
void autobiographicalMemory::writeInsert(string sRequest)
{
    ofstream file(savefile.c_str(), ios::out | ios::trunc);
    if (file) {
        file << sRequest << endl;
    }
    else {
        cout << "Error, can not save request in " << savefile << endl;    return;
    }
}

/* read interactions from a file text */
bool autobiographicalMemory::readInsert()
{
    ifstream file(savefile.c_str(), ios::in);
    if (file) {
        cout << endl << "readFile of requests from " << savefile.c_str() << endl << endl;
    }
    else {
        cout << "Error, can not open " << savefile.c_str() << endl;    return false;
    }

    string line;
    Bottle bRequest;
    // for each line of the file:
    while (getline(file, line))
    {
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(line.c_str());
        bRequest = request(bRequest);
        //*ABMDataBase << line;
    }
    return true;
}

Bottle autobiographicalMemory::snapshot(Bottle bInput)
{
    /*
    format of input bottle :
    snapshot (action name_action type_action) (time t_time) (arguments (arg1) (arg2, role) ... argN) (begin 0/1)

    Arguments are from 2 types:
    - something from the OPC : just argN is enough
    - something external : argN + role is mandatory
    */

    //get Instance of the next opc
    string sRequest_instance;
    if (isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("updateObjectLocation");
        b2reasoning.addString(s_real_OPC.c_str());
        abm2reasoning.write(b2reasoning);
    }

    sRequest_instance = "SELECT instance FROM main ORDER BY instance DESC LIMIT 1;";
    Bottle bRequest, bResult, bTemp, bArg;
    bRequest.addString("request");
    bRequest.addString(sRequest_instance.c_str());
    bRequest = request(bRequest);
    bResult = *((bRequest.get(0)).asList());
    int instance = (atoi((bResult.get(0)).toString().c_str())) + 1;
    OPCEARS.setInstance(instance);
    currentInstance = instance;

    // Filling table main:
    Bottle bMain;
    bMain.addString("request");
    ostringstream osMain;
    bool done = false;
    osMain << "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES ('";
    string sName;

    //for streaming image
    string activityName;
    bool isStreamActivity = false;
    string fullSentence;

    //Action
    for (int i = 1; i < bInput.size(); i++)
    {
        bTemp = *(bInput.get(i).asList());
        if (bTemp.get(0) == "action" && !done)
        {
            osMain << bTemp.get(1).asString() << "' , '";
            sName = bTemp.get(1).asString();
            activityName = bTemp.get(1).asString(); //sName is concatenated after...need to save label
            imgLabel = activityName;

            //if it is not a sentence -> stream
            if (activityName != "sentence") {
                isStreamActivity = true;
            }

            osMain << bTemp.get(2).asString() << "' , '";
            done = true;
        }
    }
    if (!done) {
        osMain << "unknown' , '";
    }
    // Time
    done = false;
    string sTime = getCurrentTime();
    osMain << sTime << "' , " << instance << " , ";

    //Begin
    done = false;
    bool bBegin = false;
    //cout << "bInput has a size of " << bInput.size() << " and is : " << bInput.toString().c_str() << endl;
    for (int i = 1; i < bInput.size(); i++)
    {
        bTemp = *(bInput.get(i).asList());
        if (bTemp.get(0) == "begin" && !done)
        {
            if (bTemp.get(1).asInt() == 1)
            {
                osMain << "TRUE ); ";
                bBegin = true;
            }
            else
            {
                osMain << "FALSE ); ";
                bBegin = false;
            }
            done = true;
        }
    }
    if (!done) {
        osMain << "FALSE);";
    }

    bMain.addString(string(osMain.str()).c_str());
    // cout << "\n\n" << string(osMain.str()).c_str() << endl;
    bMain = request(bMain);

    //Connection to the OPC
    Bottle bOutput, bOPC, bRelation, bEntities, bDrives, bEmotions;

    OPCEARS.snapshot(bInput, opcWorld);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();                         //I dont understand this Gregoire : you concatenate the name with nameInstance with itself, producing namenameinstance
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    for (int i = 1; i < bInput.size(); i++)
    {
        bTemp = *(bInput.get(i).asList());
        if (bTemp.get(0) == "arguments" && bTemp.size() > 1)
        {

            for (int j = 1; j < bTemp.size(); j++)
            {
                //check if the argument is an entity in OPC
                Entity* currentEntity = opcWorld->getEntity(bTemp.get(j).asList()->get(0).toString().c_str());

                bRequest.clear();
                bRequest.addString("request");
                ostringstream osArg;
                if (currentEntity == NULL){
                    if (bTemp.get(j).asList()->size() > 1) {
                        osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '" << bTemp.get(j).asList()->get(0).asString() << "', " << "'external', 'default', '" << bTemp.get(j).asList()->get(1).asString() << "');";

                        //add sentence for single img label
                        if (bTemp.get(j).asList()->get(1).asString() == "sentence"){
                            fullSentence = bTemp.get(j).asList()->get(0).asString();
                        }

                    }
                    else {
                        osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '" << bTemp.get(j).asList()->get(0).asString() << "', " << "'external', 'default', 'unknown');";
                    }
                }
                else {
                    if (bTemp.get(j).asList()->size() > 1) {
                        osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', '" << bTemp.get(j).asList()->get(1).asString() << "');";

                        //add sentence for single img label
                        if (bTemp.get(j).asList()->get(1).asString() == "sentence"){
                            fullSentence = bTemp.get(j).asList()->get(0).asString();
                        }

                    }
                    else {
                        osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'unknown');";
                    }
                }
                bRequest.addString(string(osArg.str()).c_str());
                request(bRequest);
            }
        }
    }

    for (int i = 0; i < bSnapShot.size(); i++)
    {
        bTemp.clear();
        bTemp.addString("request");
        bTemp.addString(bSnapShot.get(i).toString().c_str());
        bTemp = request(bTemp);
    }

    if ((!bBegin) && isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("addLastActivity");
        b2reasoning.addString("action");

        abm2reasoning.write(b2reasoning);
    }

    string isConnected = connectImgProvider().toString().c_str();

    if (isConnected != "ack"){
        cout << "ABM failed to connect to one imgProvider" << endl;
        cout << "CAUSE : " << isConnected << endl;
    }
    else if (isStreamActivity == true) //just launch stream images stores when relevant activity
    {
        if (bBegin) {
            streamStatus = "begin";
        }
        else {
            streamStatus = "end";
        }
    }
    else
    {  //just one image (sentence?)
        //go through the ImgReceiver ports
        for (std::map<string, BufferedPort<ImageOf<PixelRgb>>*>::const_iterator it = mapImgReceiver.begin(); it != mapImgReceiver.end(); ++it)
        {
            //concatenation of the path to store
            char imgName[512] = "";
            char fullPath[512] = "";

            stringstream ssImgName;
            if (fullSentence == ""){
                fullSentence = imgLabel;
            }

            //take the full sentence, replace space by _ to have the img name
            replace(fullSentence.begin(), fullSentence.end(), ' ', '_');
            ssImgName << fullSentence << "_" << it->first << "." << imgFormat;
            strcpy(imgName, ssImgName.str().c_str());

            stringstream ssPath;
            ssPath << storingPath << "/" << imgName;
            strcpy(fullPath, ssPath.str().c_str());

            if (!createImage(fullPath, it->second)){
                cout << "Error in snapshot : image not created from " << it->first << endl;
            }
            else {
                //create SQL entry, register the cam image in specific folder
                storeImage(instance, imgLabel, fullPath, imgName, getCurrentTime(), mapImgProvider[it->first]);
            }
        }

        //Network::disconnect(imgProviderPort, imagePortIn.getName().c_str()) ;
        string reply = disconnectImgProvider().toString().c_str();
        if (reply != "ack"){
            cout << "ABM failed to disconnect to one imgProvider" << endl;
        }
    }

    return bSnapShot;
}

Bottle autobiographicalMemory::snapshot2(Bottle bInput)
{
    /*
    format of input bottle :
    snapshot (action name_action type_action) (arg1 arg2 argn) (role1 role 2 rolen) (begin 0/1)

    Arguments are from 2 types:
    - something from the OPC : just argN is enough
    - something external : argN + role is mandatory
    */

    Bottle bOutput;

    // check input
    if (bInput.size() != 5)
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong number of input (!= 5)";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong format of input";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    int iNbArg = 0;
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | number of argument different of number of role";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    iNbArg = bInput.get(2).asList()->size();

    //get Instance of the next opc
    Bottle bRequest, bTemp, bArg, bArguments, bRoles;
    bRequest = requestFromString("SELECT instance FROM main ORDER BY instance DESC LIMIT 1;");
    int instance = atoi(bRequest.get(0).asList()->get(0).toString().c_str()) + 1;
    OPCEARS.setInstance(instance);

    // Filling table main:
    Bottle bMain;
    ostringstream osMain;
    bool done = false;
    osMain << "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES ('";
    string sName;
    bool isBegin = false;
    bool isStreamActivity = false;

    //Action
    bTemp = *(bInput.get(1).asList());
    sName = bTemp.get(1).asString();
    osMain << sName << "' , '";
    osMain << bTemp.get(2).asString() << "' , '";
    //if it is an action, will store stream image after
    if (bTemp.get(2).asString() == "action") {
        isStreamActivity = true;
    }

    // Time
    osMain << getCurrentTime() << "' , " << instance << " , ";

    //Begin
    done = false;
    bTemp = *(bInput.get(4).asList());
    if (bTemp.get(0) == "begin")
    {
        if (bTemp.get(1).asInt() == 1)
        {
            osMain << "TRUE ); ";
            inSharedPlan = true;
            isBegin = true;
        }
        else
        {
            osMain << "FALSE ); ";
            inSharedPlan = false;
        }
        done = true;
    }
    if (!done) {
        osMain << "FALSE);";
    }

    // Fill contentArg
    vector<string> vArgument, vRole;

    // catch the arguments and the role associate
    bArguments = *bInput.get(2).asList();
    bRoles = *bInput.get(3).asList();

    for (int i = 0; i < iNbArg; i++)
    {
        vArgument.push_back(bArguments.get(i).toString().c_str());
        vRole.push_back(bRoles.get(i).toString().c_str());
    }

    //Connetion to the OPC and snapshot
    if (isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("updateObjectLocation");
        b2reasoning.addString(s_real_OPC.c_str());
        abm2reasoning.write(b2reasoning);
    }

    OPCEARS.snapshot(bInput, opcWorld);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    ostringstream osArg;
    osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ";

    // Fill argument:
    for (unsigned int i = 0; i < vArgument.size(); i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vArgument[i]);
        if (i != 0)
            osArg << " , ";
        if (currentEntity == NULL){
            osArg << " ( " << instance << ", '" << vArgument[i] << "', " << "'external', 'default', '" << vRole[i] << "') ";
        }
        else {
            osArg << " ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', '" << vRole[i] << "') ";
        }
    }

    // Insert the main request.
    bMain = requestFromString(osMain.str().c_str());

    // send filling contentarg
    bArg = requestFromString(osArg.str().c_str());

    for (int i = 0; i < bSnapShot.size(); i++)
    {
        bTemp = requestFromString(bSnapShot.get(i).toString().c_str());
    }

    if (!Network::connect(robotPortCam, imagePortIn.getName().c_str())) {
        cout << "ABM failed to connect to Camera!" << endl;
    }
    else if (isStreamActivity == true) //just launch stream images stores when relevant activity
    {
        if (isBegin) {
            streamStatus = "begin";
        }
        else {
            streamStatus = "end";
        }

        imgLabel = sName;
    }

    return bSnapShot;
}


Bottle autobiographicalMemory::snapshotSP(Bottle bInput)
{
    /*
    format of input bottle :
    snapshot (action name_action type_action) (arg1 arg2 argn) (role1 role 2 rolen) (begin 0/1)

    Arguments are from 2 types:
    - something from the OPC : just argN is enough
    - something external : argN + role is mandatory
    */

    Bottle bOutput;

    // check input
    if (bInput.size() != 5)
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong number of input (!= 5)";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong format of input";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    int iNbArg = 0;
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError = "Error in autobiographicalMemory::snapshotSP | number of argument different of number of role";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    iNbArg = bInput.get(2).asList()->size();

    //get Instance of the next opc
    Bottle bRequest, bTemp, bArg, bArguments, bRoles;
    bRequest = requestFromString("SELECT instance FROM main ORDER BY instance DESC LIMIT 1;");
    int instance = atoi(bRequest.get(0).asList()->get(0).toString().c_str()) + 1;
    OPCEARS.setInstance(instance);

    // Filling table main :
    Bottle bMain;
    ostringstream osMain;
    bool done = false;
    osMain << "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES('";
    string sName;

    //Action
    bTemp = *(bInput.get(1).asList());
    if (bTemp.get(0) == "action")
    {
        sName = bTemp.get(1).asString();
        osMain << sName << "' , '";
        osMain << bTemp.get(2).asString() << "' , '";
        done = true;
    }
    if (!done) {
        osMain << "unknown' , 'unknown', '";
    }

    // Time
    osMain << getCurrentTime() << "' , " << instance << " , ";

    //Begin
    done = false;
    bTemp = *(bInput.get(4).asList());
    if (bTemp.get(0) == "begin")
    {
        if (bTemp.get(1).asInt() == 1)
        {
            osMain << "TRUE ); ";
            inSharedPlan = true;
        }
        else
        {
            osMain << "FALSE ); ";
            inSharedPlan = false;
        }
        done = true;
    }
    if (!done) {
        osMain << "FALSE);";
    }

    // Fill contentArg
    vector<string> vAgent, vObject, vSpatial;

    string sManner;
    bool fManner = false;

    // catch the arguments and the role associate
    bArguments = *bInput.get(2).asList();
    bRoles = *bInput.get(3).asList();

    for (int i = 0; i < iNbArg; i++)
    {
        if (bRoles.get(i).toString() == "agent")
            vAgent.push_back(bArguments.get(i).toString().c_str());
        if (bRoles.get(i).toString() == "object")
            vObject.push_back(bArguments.get(i).toString().c_str());
        if (bRoles.get(i).toString() == "spatial")
            vSpatial.push_back(bArguments.get(i).toString().c_str());
        if (bRoles.get(i).toString() == "manner")
        {
            sManner = bArguments.get(i).toString().c_str();
            fManner = true;
        }
    }

    if (!fManner)
    {
        cout << "manner not found. Auto set to : none" << endl;
        sManner = "none";
    }

    //Connection to the OPC and snapshot
    OPCEARS.snapshot(bInput, opcWorld);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    ostringstream osArg;
    osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << " , '" << sManner << "' , 'manner' , 'manner' , 'manner' ) ";

    // Fill agents:
    for (unsigned int i = 0; i < vAgent.size(); i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vAgent[i]);

        if (currentEntity == NULL)
            osArg << ", ( " << instance << ", '" << vAgent[i] << "', " << "'external', 'default', 'agent" << i + 1 << "') ";
        else
            osArg << ", ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'agent" << i + 1 << "') ";
    }

    // Fill objects:
    for (unsigned int i = 0; i < vObject.size(); i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vObject[i]);

        if (currentEntity == NULL)
            osArg << ", ( " << instance << ", '" << vObject[i] << "', " << "'external', 'default', 'object" << i + 1 << "') ";
        else
            osArg << ", ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'object" << i + 1 << "') ";
    }

    // Fill spatials:
    for (unsigned int i = 0; i < vSpatial.size(); i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vSpatial[i]);

        if (currentEntity == NULL)
            osArg << ", ( " << instance << ", '" << vSpatial[i] << "', " << "'external', 'default', 'spatial" << i + 1 << "') ";
        else
            osArg << ", ( " << instance << ", '" << currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'spatial" << i + 1 << "') ";
    }

    // Insert the main request.
    bMain = requestFromString(osMain.str().c_str());

    // send filling contentarg
    bArg = requestFromString(osArg.str().c_str());

    for (int i = 0; i < bSnapShot.size(); i++)
    {
        bTemp = requestFromString(bSnapShot.get(i).toString().c_str());
    }

    if (!inSharedPlan && isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("addLastActivity");
        b2reasoning.addString("sharedplan");

        abm2reasoning.write(b2reasoning);
    }

    return bSnapShot;
}

Bottle autobiographicalMemory::snapshotBehavior(Bottle bInput)
{
    /*
    format of input bottle :
    snapshot (action name_action type_action) (arg1 arg2 argn) (role1 role 2 rolen) (begin 0/1)

    Arguments are from 2 types:
    - something from the OPC : just argN is enough
    - something external : argN + role is mandatory
    */

    Bottle bOutput;

    // check input
    if (bInput.size() != 5)
    {
        string sError = "Error in autobiographicalMemory::snapshotBE | Wrong number of input (!= 5)";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError = "Error in autobiographicalMemory::snapshotBE | Wrong format of input";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    // catch the arguments and the role associate
    int iNbArg = 0;
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError = "Error in autobiographicalMemory::snapshotBE | number of argument different of number of role";
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }
    iNbArg = bInput.get(2).asList()->size();

    //get Instance of the next opc
    Bottle bRequest, bTemp, bArg, bArguments, bRoles;
    bRequest = requestFromString("SELECT instance FROM main ORDER BY instance DESC LIMIT 1;");
    int instance = atoi(bRequest.get(0).asList()->get(0).toString().c_str()) + 1;
    OPCEARS.setInstance(instance);

    // Filling table main :
    Bottle bMain;
    ostringstream osMain;
    bool done = false;
    bool bBegin = false;
    osMain << "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES('";
    string sName;

    //Action
    bTemp = *(bInput.get(1).asList());
    if (bTemp.get(0) == "action")
    {
        sName = bTemp.get(1).asString();
        osMain << sName << "' , '";
        osMain << bTemp.get(2).asString() << "' , '";
        done = true;
    }
    if (!done)
        osMain << "unknown' , 'unknown', '";

    // Time
    osMain << getCurrentTime() << "' , " << instance << " , ";

    //Begin
    done = false;
    bTemp = *(bInput.get(4).asList());
    if (bTemp.get(0) == "begin")
    {
        if (bTemp.get(1).asInt() == 1)
        {
            osMain << "TRUE ); ";
            bBegin = true;
        }
        else
        {
            osMain << "FALSE ); ";
            bBegin = false;
        }
        done = true;
    }
    if (!done)
        osMain << "FALSE);";

    // catch the arguments and the role associate
    string    sArguments = (*bInput.get(2).asList()).get(0).toString();
    string sRole = (*bInput.get(3).asList()).get(0).toString();

    //Connection to the OPC and snapshot
    OPCEARS.snapshot(bInput, opcWorld);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    ostringstream osArg;
    osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << " , '" << sArguments << "' , 'none' , 'none' , 'argument' ) ";

    // Insert the main request.
    bMain = requestFromString(osMain.str().c_str());

    // send filling contentarg
    bArg = requestFromString(osArg.str().c_str());

    for (int i = 0; i < bSnapShot.size(); i++)
    {
        bTemp = requestFromString(bSnapShot.get(i).toString().c_str());
    }

    if (!bBegin && isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("addLastActivity");
        b2reasoning.addString("behavior");

        abm2reasoning.write(b2reasoning);
    }

    return bSnapShot;
}

//test to extract a temp copy of an image by giving the label
//WARNING : label is not primary key, as we could store several picture of the same label (different angle/time)

//with instance nb
Bottle autobiographicalMemory::askImage(int instance)
{
    Bottle bOutput, bRequest, bResult;
    bOutput.clear();

    //export the desired image, doing a temp copy
    bRequest.addString("request");
    ostringstream osArg;
    //osArg << "SELECT filename FROM images WHERE label = '" << bInput.get(1).asString() << "';" ;

    osArg << "SELECT filename FROM images WHERE instance = '" << instance << "';";

    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    //verbose debug
    cout << "Reply : " << bRequest.toString() << endl;

    //if nothing is found : go out with ERROR message
    if (bRequest.toString() == "NULL"){
        cout << "ERROR : no result is found, no image match the label!" << endl;
        bOutput.addString("ERROR : no result is found, no image match the label!");
        return bOutput;
    }

    //if streaming : ERROR (switch to send the 1st image by default?)
    if (bRequest.size() > 1) {
        cout << "ERROR : You ask for a single image for a stream instance!" << endl;
        bOutput.addString("ERROR : You ask for a single image for a stream instance!");
        return bOutput;
    }

    //just one result
    string filename = bRequest.get(0).asList()->get(0).asString();
    bRequest.clear();
    osArg.str("");

    //path of the temp image
    char tmpPath[512] = "";
    stringstream ss;

    //lo_export to make a tmp copy before sending
    ss << storingPath << "/" << storingTmpSuffix << "/" << filename;
    strcpy(tmpPath, ss.str().c_str());
    bRequest.addString("request");

    //retrieve the image from the db and print it to /storingPath/temp folder
    osArg << "SELECT lo_export(img_oid, '" << tmpPath << "') from images WHERE instance = '" << instance << "';";

    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    //clear
    bRequest.clear();
    osArg.str("");

    if (!Network::connect(imagePortOut.getName().c_str(), "/yarpview/img:i"))
    {
        cout << "Error in aubotiographicalMemory::testSendImage : cannot connect to camera." << endl;
    }// hack just to check with a default yarpview
    else
    {
        if (!sendImage(tmpPath)){
            fprintf(stderr, "Cannot load file %s !\n", tmpPath);
            bOutput.addString("Cannot load image");
            return bOutput;
        }

    }

    bOutput.addString("ack");
    return bOutput;
}

Bottle autobiographicalMemory::connectOPC(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 2)
    {
        bOutput.addString("Error in connect, wrong number of input");
    }

    string OPC_name = s_real_OPC;
    if (!bInput.get(1).isString())
    {
        bOutput.addString("Error in connect, wrong format of input");
    }
    else
    {
        OPC_name = bInput.get(1).toString();
    }

    opcWorld = new OPCClient(moduleName.c_str());
    int iTry = 0;
    while (!opcWorld->isConnected())
    {
        cout << "ABM Connecting to " << OPC_name << "..." << opcWorld->connect(OPC_name) << endl;
        Time::delay(0.5);
        iTry++;
        if (iTry > 2)
        {
            bOutput.addString("Connection failed, please check your port");
            return bOutput;
        }
    }
    // opcWorld->checkout();
    // opcWorld->update();
    bOutput.addString("Connection done");
    return bOutput;
}

Bottle autobiographicalMemory::connectImgProvider()
{
    Bottle bOutput;

    if (mapImgProvider.size() == 0){
        bOutput.addString("ERROR [connectImgProvider] the map is NULL");
        return bOutput;
    }

    for (std::map<string, string>::const_iterator it = mapImgProvider.begin(); it != mapImgProvider.end(); ++it)
    {
        string portImgReceiver = "/" + getName() + "/images/" + it->first + "/in";
        mapImgReceiver.find(it->first)->second->open(portImgReceiver);
        //port name of img Provider                                  //portname of imgReceiver which correspond to the label of imgProvider
        //cout << "  [connectImgProvider] : trying to connect " << it->second << " with " <<  mapImgReceiver.find(it->first)->second->getName().c_str() << endl ;
        if (!Network::isConnected(it->second, mapImgReceiver.find(it->first)->second->getName().c_str())) {

            //cout << "Port is NOT connected : we will connect" << endl ;

            if (!Network::connect(it->second, mapImgReceiver.find(it->first)->second->getName().c_str()), "", false) {
                cout << "ERROR OF CONNECTION" << endl;
                bOutput.addString(it->second);
            }

            //cout << "CONNECTION FROM : " << it->second << endl ;
            //cout << "CONNECTION TO   : " << mapImgReceiver.find(it->first)->second->getName().c_str() << endl;
        }
        else {
            //cout << "ALREADY CONNECTED" << endl ;
        }
    }

    if (bOutput.size() == 0){
        bOutput.addString("ack");
    }

    return bOutput;
}

Bottle autobiographicalMemory::disconnectImgProvider()
{
    Bottle bOutput;
    bool isAllDisconnected = true;

    if (mapImgProvider.size() == 0){
        bOutput.addString("ERROR [disconnectImgProvider] the map is NULL");
        return bOutput;
    }

    for (std::map<string, string>::const_iterator it = mapImgProvider.begin(); it != mapImgProvider.end(); ++it)
    {
        //port name of img Provider
        //port name of imgReceiver which correspond to the label of imgProvider
        Network::disconnect(it->second, mapImgReceiver.find(it->first)->second->getName().c_str());
        if (Network::isConnected(it->second, mapImgReceiver.find(it->first)->second->getName().c_str())) {
            cout << "ERROR [disconnectImgProvider] " << it->second << " is NOT disconnected!";
            bOutput.addString(it->second);
            isAllDisconnected = false;
        }
        else {
            //cout << "[disconnectImgProvider] " << it->second << " successfully disconnected!"  << endl ;

            //Have to close/interrupt each time otherwise the port is not responsive anymore
            mapImgReceiver.find(it->first)->second->interrupt();
            mapImgReceiver.find(it->first)->second->close();
        }
    }

    if (isAllDisconnected == true){
        bOutput.addString("ack");
    }

    //cout << "[disconnectImgProvider] bOutput = {" << bOutput.toString().c_str() << "}" << endl ;
    return bOutput;
}

bool autobiographicalMemory::createImage(string fullPath, BufferedPort<ImageOf<PixelRgb>>* imgPort)
{
    //Extract the incoming images from yarp
    ImageOf<PixelRgb> *yarpImage = imgPort->read();

    //cout << "imgPort name : " << imgPort->getName() << endl ; 

    if (yarpImage != NULL) { // check we actually got something
        //user opencv to load the image
        IplImage *cvImage = cvCreateImage(cvSize(yarpImage->width(), yarpImage->height()), IPL_DEPTH_8U, 3);
        cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage, CV_RGB2BGR);
        ImageOf<PixelBgr> yarpReturnImage;
        yarpReturnImage.wrapIplImage(cvImage);

        //create the image
        cvSaveImage(fullPath.c_str(), cvImage);

        //cout << "img created : " << fullPath.c_str() << endl ;

        cvReleaseImage(&cvImage);
    }
    else {
        cout << "ERROR CANNOT SAVE :no image come from sim camera!" << endl;
        return false;
    }

    return true;
}

bool autobiographicalMemory::sendImage(string fullPath)
{
    IplImage* img = NULL;

    //cout << "Going to send : " << fullPath << endl;

    //load image with opencv
    img = cvLoadImage(fullPath.c_str(), CV_LOAD_IMAGE_UNCHANGED);

    if (img == 0)  return false;

    //create a yarp image
    cvCvtColor(img, img, CV_BGR2RGB);
    ImageOf<PixelRgb> &temp = imagePortOut.prepare();
    temp.resize(img->width, img->height);
    cvCopyImage(img, (IplImage *)temp.getIplImage());

    //send the image
    imagePortOut.write();

    cvReleaseImage(&img);

    return true;
}

int autobiographicalMemory::sendStreamImage(int instance)
{
    Bottle bRequest;
    ostringstream osArg;

    //extract label of the instance
    bRequest.addString("request");
    osArg << "SELECT DISTINCT label FROM images WHERE instance = " << instance << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    //put global imgLabel (for full path completion after in update)
    imgLabel = bRequest.get(0).asList()->get(0).asString();

    bRequest.clear();
    osArg.str("");

    //extract oid of all the images (primary key)
    bRequest.addString("request");
    osArg << "SELECT img_oid FROM images WHERE instance = " << instance << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    //cout << "bRequest has " << bRequest.size() << "images : " << bRequest.toString() << endl ;;
    imgNbInStream = bRequest.size();

    //export all the images corresponding to the instance to a tmp folder in order to be sent after (update())
    for (int i = 0; i < bRequest.size(); i++){
        exportImage(atoi(bRequest.get(i).asList()->get(0).toString().c_str()), storingTmpSuffix);
    }

    //streamStatus changed (triggered in update()
    streamStatus = "send";
    //bOutput.addString("ack");

    return bRequest.size();
}

//store an image into the SQL db
bool autobiographicalMemory::storeImage(int instance, string label, string fullPath, string imgName, string imgTime, string currentImgProviderPort)
{
    Bottle bRequest;
    ostringstream osArg;
    //sql request with instance and label, images are stored from their location
    bRequest.addString("request");
    osArg << "INSERT INTO images(instance, label, img_oid, filename, time, img_provider_port) VALUES (" << instance << ", '" << label << "', lo_import('" << fullPath << "'), '" << imgName << "', '" << imgTime << "', '" << currentImgProviderPort << "' );";
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    //bOutput.addString("ack");

    return true;
}

//export (i.e. save) a stored image to hardrive, using oid to identify and the path wanted
bool autobiographicalMemory::exportImage(int img_oid, string myTmpPath)
{
    Bottle bRequest;
    ostringstream osArg;

    //extract filename of the image to export fromn primary key oid
    bRequest.addString("request");
    //SELECT filename from images WHERE img_oid = 33275 ;
    osArg << "SELECT filename from images WHERE img_oid = " << img_oid << " ;";
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    //cout << "filename = " << bRequest.toString() << endl ;
    string filename = bRequest.get(0).asList()->get(0).asString();

    bRequest.clear();
    osArg.str("");

    //path of the temp image
    char tmpPath[512] = "";
    stringstream ss;

    //lo_export to make a tmp copy before sending
    ss << storingPath << "/" << myTmpPath << "/" << filename;
    strcpy(tmpPath, ss.str().c_str());

    bRequest.addString("request");
    //retrieve the image from the db and print it to /storingPath/temp folder
    osArg << "SELECT lo_export(img_oid, '" << tmpPath << "') from images WHERE img_oid = '" << img_oid << "';";

    bRequest.addString(string(osArg.str()).c_str());
    bRequest = request(bRequest);

    //bOutput.addString("ack");

    return true;
}

string autobiographicalMemory::getCurrentTime()
{
    struct tm Time;
    time_t myTime;
    time(&myTime);                    // get unix time
    tm *t = localtime(&myTime);        // conversion in local time

#ifdef WIN32
    SYSTEMTIME st;
    GetSystemTime(&st);
    int iUS = st.wMilliseconds * 1000;
#else
    struct timezone tz;
    struct timeval tv;
    gettimeofday(&tv, &tz);
    int iUS = tv.tv_usec;
#endif

    Time.tm_hour = (*t).tm_hour;
    Time.tm_min = (*t).tm_min;
    Time.tm_sec = (*t).tm_sec;
    Time.tm_mday = (*t).tm_mday;
    Time.tm_mon = (*t).tm_mon;
    Time.tm_year = (*t).tm_year;
    Time.tm_mday = (*t).tm_mday;
    Time.tm_mon = (*t).tm_mon;
    Time.tm_year = (*t).tm_year;

    ostringstream osTime;

    // get information
    int iHH, iMM, iSS, iMonth, iDay, iYear;
    iHH = Time.tm_hour;
    iMM = Time.tm_min;
    iSS = Time.tm_sec;
    iMonth = Time.tm_mon + 1;
    iDay = Time.tm_mday;
    iYear = Time.tm_year + 1900;
    osTime << iYear << "-" << iMonth << "-" << iDay << " " << iHH << ":" << iMM << ":" << iSS << "." << iUS;

    return osTime.str();
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
        int begin, end;
        begin = 0;
        end = -1;
        if (bInstances.size() == 2)
        {
            begin = atoi(bInstances.get(0).toString().c_str());
            end = atoi(bInstances.get(1).toString().c_str());
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
        //SELECT lo_unlink (img_oid) FROM (SELECT DISTINCT img_oid FROM images WHERE instance = 42) AS images_subquery ;
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

Bottle autobiographicalMemory::connect2reasoning()
{
    string name_abm2reasoning = "/";
    name_abm2reasoning += getName() + "/to_reasoning";

    Network::connect(name_abm2reasoning.c_str(), "/abmReasoning/rpc");

    string state;

    isconnected2reasoning = Network::isConnected(name_abm2reasoning.c_str(), "/abmReasoning/rpc");

    (isconnected2reasoning ? state = "ABM is now connected to abmReasoning" : state = "ABM failed to connect to abmReasoning");

    Bottle bOutput;
    bOutput.addString(state.c_str());

    return bOutput;
}

Bottle autobiographicalMemory::detectFailed()
{
    Bottle bOutput;
    Bottle bMessenger = requestFromString("SELECT instance FROM main WHERE activitytype = 'actionfailed' AND begin = true");

    return bOutput;
}

/*
* Put in the OPC all the words known in the ABM.
*/
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

/*
* return a tuple of 3 int from a string input
*/
vector<int> autobiographicalMemory::tupleIntFromString(string sInput)
{
    vector<int>    tOutput;
    char *cInput;
    cInput = (char*)sInput.c_str();
    int    iLevel = 0;
    unsigned int data = 0;
    string sX = "", sY = "", sZ = "";
    while (cInput[data] != '\0')
    {
        char cTemp = cInput[data];
        if (cTemp == ',')
        {
            iLevel++;
        }
        if (cTemp != '{' && cTemp != ',')
        {
            switch (iLevel)
            {
            case 0:
                sX += cTemp;
                break;
            case 1:
                sY += cTemp;
                break;
            case 2:
                sZ += cTemp;
                break;
            }
        }
        data++;
    }

    tOutput.push_back(atoi(sX.c_str()));
    tOutput.push_back(atoi(sY.c_str()));
    tOutput.push_back(atoi(sZ.c_str()));
    //get<0>(tOutput) = atoi(sX.c_str());
    //get<1>(tOutput) = atoi(sY.c_str());
    //get<2>(tOutput) = atoi(sZ.c_str());

    return tOutput;
}

/*
* return a tuple of 3 double from a string input
*/
vector<double> autobiographicalMemory::tupleDoubleFromString(string sInput)
{
    vector<double> tOutput;
    char *cInput;
    cInput = (char*)sInput.c_str();
    int iLevel = 0;
    unsigned int data = 0;
    string sX = "", sY = "", sZ = "";
    while (cInput[data] != '\0')
    {
        char cTemp = cInput[data];
        if (cTemp == ',')
        {
            iLevel++;
        }
        if (cTemp != '{' && cTemp != ',')
        {
            switch (iLevel)
            {
            case 0:
                sX += cTemp;
                break;
            case 1:
                sY += cTemp;
            case 2:
                sZ += cTemp;
                break;
            }
        }
        data++;
    }

    tOutput.push_back(atof(sX.c_str()));
    tOutput.push_back(atof(sY.c_str()));
    tOutput.push_back(atof(sZ.c_str()));
    //get<0>(tOutput) = atof(sX.c_str());
    //get<1>(tOutput) = atof(sY.c_str());
    //get<2>(tOutput) = atof(sZ.c_str());

    return tOutput;
}

Bottle autobiographicalMemory::getInfoAbout(string sName)
{
    Bottle bMessenger, bOutput;

    if (opcWorld->getEntity(sName) != NULL)
    {
        // The object is not in the OPC, we have to search in the memory to get the type.
        ostringstream osEntity;
        osEntity << "SELECT instance, opcid FROM entity WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1";
        bMessenger = requestFromString(osEntity.str());

        cout << "bMessenger : " << bMessenger.toString() << endl;

        int Instance = atoi(bMessenger.get(0).asList()->get(0).toString().c_str()),
            Opcid = atoi(bMessenger.get(0).asList()->get(1).toString().c_str());

        osEntity.str("");
        osEntity << "SELECT subtype FROM contentopc WHERE instance = " << Instance << " AND opcid = " << Opcid;
        bMessenger = requestFromString(osEntity.str());

        string sSubType = bMessenger.get(0).asList()->get(0).toString();

        osEntity.str("");
        osEntity << "SELECT count(*) FROM contentarg WHERE argument = '" << sName << "'";
        bMessenger = requestFromString(osEntity.str());

        int iNbInteraction = atoi(bMessenger.get(0).asList()->get(0).toString().c_str());
        cout << "I have interacted with this " << sSubType << " " << iNbInteraction / 2 << " times ! " << endl;
    }

    return bOutput;
}

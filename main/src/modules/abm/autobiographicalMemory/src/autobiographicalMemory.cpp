#include <autobiographicalMemory.h>


autobiographicalMemory::autobiographicalMemory(ResourceFinder &rf)
{
    Bottle &bDBProperties = rf.findGroup("database_properties");
    server = bDBProperties.check("server",Value("127.0.0.1")).asString();
    user = bDBProperties.check("user",Value("postgres")).asString();
    password = bDBProperties.check("password",Value("rclab")).asString();
    dataB = bDBProperties.check("dataB",Value("ABM")).asString();
    savefile = (rf.getContextPath()+"/saveRequest.txt").c_str();
    ABMDataBase = new DataBase<PostgreSql>(server, user, password, dataB);
    inSharedPlan = false;
}


autobiographicalMemory::~autobiographicalMemory()
{
    delete ABMDataBase;
}


/* configure the module */
bool autobiographicalMemory::configure(ResourceFinder &rf)
{		
    moduleName = rf.check("name", 
        Value("autobiographicalMemory"), 
        "module name (string)").asString();

    setName(moduleName.c_str());

    bPutObjectsOPC = false;

    portEventsName = "/";
    portEventsName +=  getName() + "/request:i";
    portEventsIn.open(portEventsName.c_str());

    string portHandlerName = "/";
    portHandlerName +=  getName() + "/rpc";
    handlerPort.open(portHandlerName.c_str());


    string name_abm2reasoning = "/";
    name_abm2reasoning +=  getName() + "/to_reasoning";
    abm2reasoning.open(name_abm2reasoning.c_str());
    //	Network::connect(name_abm2reasoning.c_str(), "/efaa/abmReasoning/rpc");


    isconnected2reasoning = false;

    attach(handlerPort);
    //attachTerminal();


    Bottle bConnect;
    bConnect.addString("connect");
    bConnect.addString("OPC");
    connectOPC(bConnect);

    //	populateOPC();

    cout << endl << endl << "----------------------------------------------" << endl << endl << "autobiographicalMemory ready ! " << endl << endl;

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
    while(bResult.fetch(vRow))
    {
        isResult = true;
        Bottle bLineBottle ;
        int test = vRow.size();
        //parse the current row of the result and add each column values to a bottle
        for (size_t i = 0; i < vRow.size(); i++)
        {
            bLineBottle.addString(vRow[i].c_str());
        }

        //add the line bottle to the reply bottle
        //printf("Bottle lineBottle done : %s\n", lineBottle.toString().c_str());
        //cout << "lineBottle size : " << lineBottle.size() << endl;
        bOutput.addList() = bLineBottle ;
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
    //cout << "in request : "  << request.toString().c_str() << endl;

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
        cerr << "Exception during request : "<<e.what()<<endl;
        string sExcept;
        sExcept = "Exception during request : ";
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
    //	output.addString("file saved");
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
    *ABMDataBase << "ALTER TABLE spatialdata OWNER  TO postgres;" ; 


    /****************************** contextknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE contextknowledge ( name text NOT NULL,  argument text NOT NULL,  dependance text NOT NULL, instance integer NOT NULL,  CONSTRAINT contextknowledge_pkey PRIMARY KEY (instance),  CONSTRAINT contextknowledge_name_key UNIQUE (name, argument, dependance) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextknowledge OWNER TO postgres;";


    /****************************** contextdata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextdata CASCADE;";
    *ABMDataBase << "CREATE TABLE contextdata (  presencebegin boolean,  presenceend boolean,  instance integer NOT NULL,  id serial NOT NULL,  CONSTRAINT contextdata_instance_fkey FOREIGN KEY (instance) REFERENCES contextknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextdata OWNER  TO postgres;" ; 


    /****************************** contextagent *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextagent CASCADE;";
    *ABMDataBase << "CREATE TABLE contextagent ( instance integer NOT NULL, agent text NOT NULL, number integer, CONSTRAINT contextagent_pkey PRIMARY KEY (instance, agent), CONSTRAINT contextagent_instance_fkey FOREIGN KEY (instance) REFERENCES contextknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextagent OWNER  TO postgres;" ; 


    /****************************** timeknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS timeknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE timeknowledge ( temporal text NOT NULL,   CONSTRAINT timeknowledge_pkey PRIMARY KEY (temporal) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE timeknowledge OWNER TO postgres;";


    /****************************** timedata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS timedata CASCADE;";
    *ABMDataBase << "CREATE TABLE timedata ( temporal text NOT NULL,   timearg1 timestamp without time zone NOT NULL,   timearg2 timestamp without time zone NOT NULL,   CONSTRAINT timedata_temporal_fkey FOREIGN KEY (temporal)        REFERENCES timeknowledge (temporal) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE timedata OWNER  TO postgres;" ; 


    /****************************** interactionknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS interactionknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE interactionknowledge (subject text NOT NULL, argument text NOT NULL, number integer NOT NULL, type text NOT NULL DEFAULT 'none'::text, role text NOT NULL DEFAULT 'none'::text, CONSTRAINT interactionknowledge_pkey PRIMARY KEY (subject, argument, type, role) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE interactionknowledge OWNER  TO postgres;" ; 

    string sFilename ;

    //if filename after, load the database through this
    if (bInput.size()>1)
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
        sFilename = "autobiographicalMemory.db" ;
    }

    /********************************************** build the database startup **********************************************/
    bool bInsert = true;
    if (bInsert)
    {
        //main table : 2 actions (begin : true then false) => 4 OPC
        *ABMDataBase << "INSERT INTO main(time, activityname, activitytype, instance, begin) VALUES('2012-01-01 15:53:10','grasp','action',1,TRUE);" ;

        //contentarg
        *ABMDataBase << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES(1,'ball', 'entity', 'rtobject', 'object1');" ;
        *ABMDataBase << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES(1,'blue', 'external', 'color', 'argument');" ;

        //contentopc table

        //------------> OPC 1
        *ABMDataBase << "INSERT INTO contentopc(instance, type, subtype, opcid) VALUES(1,'entity','object',81);" ;
        *ABMDataBase << "INSERT INTO contentopc(instance, type, subtype, opcid) VALUES(1,'entity','entity',82);" ;
        *ABMDataBase << "INSERT INTO contentopc(instance, type, subtype, opcid) VALUES(1,'relation','relation',83);" ;

        //entity table
        *ABMDataBase << "INSERT INTO entity(opcid, name, instance) VALUES(82, 'thing', 1);" ;

        //object table
        *ABMDataBase << "INSERT INTO object(opcid, name, instance, presence, position) VALUES(81, 'ball', 1, TRUE,'{1.1,2.2,1}');" ;

        //relation table
        *ABMDataBase << "INSERT INTO relation(opcid, instance, subject, verb, object) VALUES(83, 1, 'ball', 'isAtLoc', 'left');" ;
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
        cerr << "Exception during request : "<<e.what()<<endl;
        string sExcept;
        sExcept = "Exception during request : ";
        sExcept += e.what();
        bReply.addString(sExcept.c_str());
    }


    //send the request to the database
    return bReply;
}


double autobiographicalMemory::getPeriod()
{
    return 0.2; //module periodicity (seconds)
}

/* rpc respond module */
bool autobiographicalMemory::respond(const Bottle& bCommand, Bottle& bReply)
{
    Bottle bError;
    bReply.clear();
    cout << endl  << "Got something, echo is on" << endl;
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
        if (bCommand.get(0) == "read")
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
        if (bCommand.get(0) == "new")
        {
            bReply = newDB(bCommand);
        }

        if (bCommand.get(0) == "snapshot")
        {
            bReply = snapshot(bCommand);
        }

        if (bCommand.get(0) == "snapshotSP")
        {
            bReply = snapshotSP(bCommand);
        }

        if (bCommand.get(0) == "snapshotBE")
        {
            bReply = snapshotBehavior(bCommand);
        }

        if (bCommand.get(0) == "save")
        {
            bReply = save(bCommand);
        }


        if (bCommand.get(0) == "load")
        {
            bReply = load(bCommand);
        }

        if (bCommand.get(0) == "connect")
        {
            bReply = connect2reasoning();
        }

        if (bCommand.get(0) == "request")
        {

            if (bCommand.size()>1)
            {
                bReply = request(bCommand);
            }
            else
            {
                bError.addString("in request : number of element insufficient");
                bReply = bError;
            }
        }


        if (bCommand.get(0) == "insert")
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

        if (bCommand.get(0) == "resetKnowledge")
        {
            bReply = resetKnowledge();
        }


        if (bCommand.get(0) == "eraseInstance")
        {
            bReply = eraseInstance(bCommand);
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

    Bottle bCommand, bReply, bError;
    portEventsIn.read(bCommand,true);
    cout << endl  << "Got something, echo is on" << endl;
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
        if (bCommand.get(0) == "read")
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
        if (bCommand.get(0) == "new")
        {
            bReply = newDB(bCommand);
        }

        if (bCommand.get(0) == "snapshot")
        {
            bReply = snapshot2(bCommand);
        }

        if (bCommand.get(0) == "eraseInstance")
        {
            bReply = eraseInstance(bCommand);
        }

        if (bCommand.get(0) == "snapshotSP")
        {
            bReply = snapshotSP(bCommand);
        }		

        if (bCommand.get(0) == "snapshotBE")
        {
            bReply = snapshotBehavior(bCommand);
        }

        if (bCommand.get(0) == "connectOPC")
        {
            bReply = connectOPC(bCommand);
        }

        if (bCommand.get(0) == "connect")
        {
            bReply = connect2reasoning();
        }

        if (bCommand.get(0) == "save")
        {
            bReply = save(bCommand);
        }

        if (bCommand.get(0) == "load")
        {
            bReply = load(bCommand);
        }

        if (bCommand.get(0) == "resetKnowledge")
        {
            bReply = resetKnowledge();
        }


        if (bCommand.get(0) == "request")
        {

            if (bCommand.size()>1)
            {
                bReply = request(bCommand);
            }
            else
            {
                bError.addString("in request : number of element insufficient");
                bReply = bError;
            }
        }


        if (bCommand.get(0) == "insert")
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

    }
    else
    {
        bError.addString("wrong input bottle format");
        bReply = bError;
    }

    portEventsIn.reply(bReply);

    return true;
}

bool autobiographicalMemory::interruptModule()
{
    cout<<"Interrupting your module, for port cleanup"<<endl;


    // TODO : save the database
    opcWorld->interrupt();
    opcWorld->close();

    handlerPort.interrupt();
    handlerPort.close();

    portEventsIn.interrupt();
    portEventsIn.close();

    abm2reasoning.interrupt();
    abm2reasoning.close();
    return true;
}


bool autobiographicalMemory::close()
{
    cout<<"Calling close function\n";
    delete opcWorld;
    return true;
}

/* each interaction stored in the DB is save in a file text */
void autobiographicalMemory::writeInsert(string sRequest)
{
    ofstream file(savefile.c_str(),ios::out | ios::trunc);
    if (file)	{
        file << sRequest <<endl;
    }
    else	{
        cout<<"Error, can not save request in "<<savefile<<endl;	return;
    }
}

/* read interactions from a file text */
bool autobiographicalMemory::readInsert()
{
    ifstream file(savefile.c_str(),ios::in);
    if (file)	{
        cout<<endl<<"readFile of requests from "<<savefile.c_str()<<endl<<endl;
    }
    else	{
        cout<<"Error, can not open "<<file<<endl;	return false;
    }

    string line;
    Bottle bRequest;
    // for each line of the file :
    while(getline(file, line))
    {
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(line.c_str());
        bRequest = request(bRequest);
        //		*ABMDataBase << line ;
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

    // Filling table main :
    Bottle bMain;
    bMain.addString("request");
    ostringstream osMain;
    bool done = false;
    osMain <<  "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES ('";
    string sName;
    //Action
    for (int i = 1; i < bInput.size() ; i++)
    {
        bTemp = *(bInput.get(i).asList());
        if (bTemp.get(0) == "action" && !done)
        {
            osMain << bTemp.get(1).asString() << "' , '";
            sName = bTemp.get(1).asString();
            osMain << bTemp.get(2).asString() << "' , '";
            done = true;
        }
    }
    if (!done)
        osMain << "unknown' , '" ;

    // Time
    done = false;
    string sTime = getCurrentTime();
    osMain << sTime << "' , " << instance << " , ";

    //Begin
    done = false;
    bool bBegin;
    for (int i = 1; i < bInput.size() ; i++)
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
    if (!done)
        osMain << "FALSE;)";


    bMain.addString(string(osMain.str()).c_str());
    //	cout << "\n\n" << string(osMain.str()).c_str() << endl;
    bMain = request(bMain);

    //Connexion to the OPC
    Bottle bOutput,bOPC, bRelation, bEntities, bDrives, bEmotions;

    OPCEARS.snapshot(bInput, opcWorld);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    for (int i = 1; i < bInput.size() ; i++)
    {
        bTemp = *(bInput.get(i).asList());
        if (bTemp.get(0) == "arguments" && bTemp.size()>1)
        {

            for (int j = 1; j < bTemp.size(); j++)
            {
                //check if the argument is an entity in OPC
                Entity* currentEntity = opcWorld->getEntity(bTemp.get(j).asList()->get(0).toString().c_str());

                bRequest.clear();
                bRequest.addString("request");
                ostringstream osArg;
                if(currentEntity == NULL){
                    if (bTemp.get(j).asList()->size() > 1) {
                        osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '"<< bTemp.get(j).asList()->get(0).asString() << "', " << "'external', 'default', '" << bTemp.get(j).asList()->get(1).asString() << "');";
                    } else {
                        osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '"<< bTemp.get(j).asList()->get(0).asString() << "', " << "'external', 'default', 'unknown');";
                    }
                } else {
                    if (bTemp.get(j).asList()->size() > 1) {
                        osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '"<< currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', '" << bTemp.get(j).asList()->get(1).asString() << "');";	
                    } else {
                        osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << ", '"<< currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'unknown');";	
                    }
                }
                bRequest.addString(string(osArg.str()).c_str());
                request(bRequest);
            }
        }
    }


    for (unsigned int i = 0 ; i < bSnapShot.size() ; i++)
    {

        bTemp.clear();
        bTemp.addString("request");
        bTemp.addString(bSnapShot.get(i).toString().c_str());
        bTemp = request(bTemp);
    }


    if (!bBegin && isconnected2reasoning)
    {
        Bottle b2reasoning;
        b2reasoning.addString("addLastActivity");
        b2reasoning.addString("action");

        abm2reasoning.write(b2reasoning);
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
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong number of input (!= 5)" ;
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError =  "Error in autobiographicalMemory::snapshotSP | Wrong format of input" ;
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }


    // catch the arguments and the role associate
    int iNbArg = 0;
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError =  "Error in autobiographicalMemory::snapshotSP | number of argument different of number of role" ;
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
    osMain <<  "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES ('";
    string sName;

    //Action
    bTemp = *(bInput.get(1).asList());
    sName = bTemp.get(1).asString();
    osMain << sName << "' , '";
    osMain << bTemp.get(2).asString() << "' , '";

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
    if (!done)
        osMain << "FALSE;)";



    // Fill contentArg
    vector<string> vArgument, vRole;

    // catch the arguments and the role associate

    bArguments	= *bInput.get(2).asList();
    bRoles		= *bInput.get(3).asList();

    for (unsigned int i = 0 ; i < iNbArg ; i++)
    {
        vArgument.push_back(bArguments.get(i).toString().c_str());
        vRole.push_back(bRoles.get(i).toString().c_str());
    }

    //Connexion to the OPC and snapshot
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
    osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES " ;

    // Fill argument :
    for (int i = 0 ; i < vArgument.size() ; i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vArgument[i]);
        if (i !=0)
            osArg << " , " ;
        if(currentEntity == NULL){
            osArg << " ( " << instance << ", '"<< vArgument[i] << "', " << "'external', 'default', '" << vRole[i] << "') ";
        }
        else {
            osArg << " ( " << instance << ", '"<< currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', '" << vRole[i] << "') ";	
        }
    }


    // Insert the main request.
    bMain = requestFromString(osMain.str().c_str());

    // send filling contentarg
    bArg =	requestFromString(osArg.str().c_str());

    for (unsigned int i = 0 ; i < bSnapShot.size() ; i++)
    {
        bTemp = requestFromString(bSnapShot.get(i).toString().c_str());
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
        string sError = "Error in autobiographicalMemory::snapshotSP | Wrong number of input (!= 5)" ;
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError =  "Error in autobiographicalMemory::snapshotSP | Wrong format of input" ;
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }


    // catch the arguments and the role associate
    int iNbArg = 0;
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError =  "Error in autobiographicalMemory::snapshotSP | number of argument different of number of role" ;
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
    osMain <<  "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES('";
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
        osMain << "unknown' , 'unknown', '" ;

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
    if (!done)
        osMain << "FALSE;)";



    // Fill contentArg
    vector<string> vAgent,
        vObject,
        vSpatial;

    string sManner;
    bool fManner = false;

    // catch the arguments and the role associate

    bArguments	= *bInput.get(2).asList();
    bRoles		= *bInput.get(3).asList();

    for (unsigned int i = 0 ; i < iNbArg ; i++)
    {
        if (bRoles.get(i).toString() == "agent" )
            vAgent.push_back(bArguments.get(i).toString().c_str());
        if (bRoles.get(i).toString() == "object" )
            vObject.push_back(bArguments.get(i).toString().c_str());
        if (bRoles.get(i).toString() == "spatial" )
            vSpatial.push_back(bArguments.get(i).toString().c_str());
        if (bRoles.get(i).toString() == "manner" )
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

    //Connexion to the OPC and snapshot
    OPCEARS.snapshot(bInput, opcWorld);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    ostringstream osArg;
    osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << " , '" << sManner << "' , 'manner' , 'manner' , 'manner' ) " ;

    // Fill agents :
    for (int i = 0 ; i < vAgent.size() ; i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vAgent[i]);

        if(currentEntity == NULL)
            osArg << ", ( " << instance << ", '"<< vAgent[i] << "', " << "'external', 'default', 'agent" << i+1 << "') ";
        else
            osArg << ", ( " << instance << ", '"<< currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'agent" << i+1 << "') ";	
    }

    // Fill objects :
    for (int i = 0 ; i < vObject.size() ; i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vObject[i]);

        if(currentEntity == NULL)
            osArg << ", ( " << instance << ", '"<< vObject[i] << "', " << "'external', 'default', 'object" << i+1 << "') ";
        else
            osArg << ", ( " << instance << ", '"<< currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'object" << i+1 << "') ";	
    }


    // Fill spatials :
    for (int i = 0 ; i < vSpatial.size() ; i++)
    {
        Entity* currentEntity = opcWorld->getEntity(vSpatial[i]);

        if(currentEntity == NULL)
            osArg << ", ( " << instance << ", '"<< vSpatial[i] << "', " << "'external', 'default', 'spatial" << i+1 << "') ";
        else
            osArg << ", ( " << instance << ", '"<< currentEntity->name() << "', 'entity', '" << currentEntity->entity_type() << "', 'spatial" << i+1 << "') ";	
    }


    // Insert the main request.
    bMain = requestFromString(osMain.str().c_str());

    // send filling contentarg
    bArg =	requestFromString(osArg.str().c_str());

    for (unsigned int i = 0 ; i < bSnapShot.size() ; i++)
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
        string sError = "Error in autobiographicalMemory::snapshotBE | Wrong number of input (!= 5)" ;
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }

    if (!bInput.get(0).isString() || !bInput.get(1).isList() || !bInput.get(2).isList() || !bInput.get(3).isList() || !bInput.get(4).isList())
    {
        string sError =  "Error in autobiographicalMemory::snapshotBE | Wrong format of input" ;
        cout << sError << endl;
        bOutput.addString(sError.c_str());
        return bOutput;
    }


    // catch the arguments and the role associate
    int iNbArg = 0;
    if (bInput.get(2).asList()->size() != bInput.get(3).asList()->size())
    {
        string sError =  "Error in autobiographicalMemory::snapshotBE | number of argument different of number of role" ;
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
    bool done = false,
        bBegin;
    osMain <<  "INSERT INTO main(activityname, activitytype, time, instance, begin) VALUES('";
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
        osMain << "unknown' , 'unknown', '" ;

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
        osMain << "FALSE;)";


    // catch the arguments and the role associate

    string	sArguments	= (*bInput.get(2).asList()).get(0).toString();
    string sRole		= (*bInput.get(3).asList()).get(0).toString();


    //Connexion to the OPC and snapshot
    OPCEARS.snapshot(bInput, opcWorld);
    ostringstream osName;
    osName << sName << instance;
    sName += osName.str();
    Bottle bSnapShot = OPCEARS.insertOPC(sName);

    // Filling contentArg
    ostringstream osArg;
    osArg << "INSERT INTO contentarg(instance, argument, type, subtype, role) VALUES ( " << instance << " , '" << sArguments << "' , 'none' , 'none' , 'argument' ) " ;

    // Insert the main request.
    bMain = requestFromString(osMain.str().c_str());

    // send filling contentarg
    bArg =	requestFromString(osArg.str().c_str());

    for (unsigned int i = 0 ; i < bSnapShot.size() ; i++)
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


Bottle autobiographicalMemory::connectOPC(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() !=2)
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
    while(!opcWorld->isConnected())
    {  
        cout<<"ABM Connecting to " << OPC_name << "..." << opcWorld->connect(OPC_name)<<endl;
        Time::delay(0.5);
        iTry++;
        if (iTry > 2)
        {
            bOutput.addString("Connection failed, please check your port");
            return bOutput;
        }
    }
    //	opcWorld->checkout();
    //	opcWorld->update();
    bOutput.addString("Connection done");
    return bOutput;
}


string autobiographicalMemory::getCurrentTime()
{
    struct tm Time;
    time_t myTime;
    time(&myTime);					// get unix time
    tm *t = localtime(&myTime);		// conversion in local time

    Time.tm_hour	=	(*t).tm_hour;
    Time.tm_min		=	(*t).tm_min;
    Time.tm_sec		=	(*t).tm_sec;
    Time.tm_mday	=	(*t).tm_mday;
    Time.tm_mon		=	(*t).tm_mon;
    Time.tm_year	=	(*t).tm_year;
    Time.tm_mday=(*t).tm_mday;
    Time.tm_mon=(*t).tm_mon;
    Time.tm_year=(*t).tm_year;

    ostringstream osTime;

    // get information
    int iHH, iMM, iSS, iMonth, iDay, iYear;
    iHH = Time.tm_hour;
    iMM = Time.tm_min;
    iSS = Time.tm_sec;
    iMonth = Time.tm_mon + 1;
    iDay = Time.tm_mday;
    iYear = Time.tm_year + 1900;
    osTime << iYear << "-" << iMonth << "-" << iDay << " " << iHH << ":" << iMM << ":" << iSS ;

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
    *ABMDataBase << "ALTER TABLE spatialdata OWNER  TO postgres;" ; 


    /****************************** timeknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS timeknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE timeknowledge ( temporal text NOT NULL,   CONSTRAINT timeknowledge_pkey PRIMARY KEY (temporal) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE timeknowledge OWNER TO postgres;";


    /****************************** timedata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS timedata CASCADE;";
    *ABMDataBase << "CREATE TABLE timedata ( temporal text NOT NULL,   timearg1 timestamp without time zone NOT NULL,   timearg2 timestamp without time zone NOT NULL,   CONSTRAINT timedata_temporal_fkey FOREIGN KEY (temporal)        REFERENCES timeknowledge (temporal) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE timedata OWNER  TO postgres;" ; 


    /****************************** contextknowledge *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextknowledge CASCADE;";
    *ABMDataBase << "CREATE TABLE contextknowledge ( name text NOT NULL,  argument text NOT NULL, dependance text NOT NULL,  instance integer NOT NULL,  CONSTRAINT contextknowledge_pkey PRIMARY KEY (instance),  CONSTRAINT contextknowledge_name_key UNIQUE (name, argument, dependance) ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextknowledge OWNER TO postgres;";


    /****************************** contextdata *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextdata CASCADE;";
    *ABMDataBase << "CREATE TABLE contextdata (  presencebegin boolean,  presenceend boolean,  instance integer NOT NULL,  id serial NOT NULL,  CONSTRAINT contextdata_instance_fkey FOREIGN KEY (instance) REFERENCES contextknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION ) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextdata OWNER  TO postgres;" ; 

    /****************************** contextagent *************************/
    *ABMDataBase << "DROP TABLE IF EXISTS contextagent CASCADE;";
    *ABMDataBase << "CREATE TABLE contextagent ( instance integer NOT NULL, agent text NOT NULL, number integer, CONSTRAINT contextagent_pkey PRIMARY KEY (instance, agent), CONSTRAINT contextagent_instance_fkey FOREIGN KEY (instance) REFERENCES contextknowledge (instance) MATCH SIMPLE ON UPDATE NO ACTION ON DELETE NO ACTION) WITH (OIDS=FALSE);";
    *ABMDataBase << "ALTER TABLE contextagent OWNER  TO postgres;" ; 

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
    *ABMDataBase << "ALTER TABLE interactionknowledge OWNER  TO postgres;" ; 

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

    if (bInput.size() <2)
    {
        bOutput.addString("in autobiographicalMemory::eraseInstance | wrong number of input");
        return bOutput;
    }
    if (bInput.get(1).isList())
    {
        bInstances = *bInput.get(1).asList();
        int begin, end;
        if (bInstances.size() == 2)
        {
            begin = atoi(bInstances.get(0).toString().c_str());
            end = atoi(bInstances.get(1).toString().c_str());
        }
        for (int inst = begin ; inst < end +1 ; inst ++)
        {
            vecToErase.push_back(inst);
        }
    }
    else
    {
        for (unsigned int i = 1; i < bInput.size(); i++)
        {
            vecToErase.push_back(bInput.get(i).asInt());
        }
    }
    Bottle bRequest;
    ostringstream osRequest;
    for (vector<int>::iterator it = vecToErase.begin() ; it != vecToErase.end() ; it++)
    {
        osRequest.str("");
        osRequest << "DELETE FROM action WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM adjective WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM agent WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM drives WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM emotions WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM rtobject WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM object WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM relation WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM entity WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM contentarg WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM contentopc WHERE instance = " << *it ;
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(osRequest.str().c_str());
        request(bRequest);

        osRequest.str("");
        osRequest << "DELETE FROM main WHERE instance = " << *it ;
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
    name_abm2reasoning +=  getName() + "/to_reasoning";

    Network::connect(name_abm2reasoning.c_str(), "/efaa/abmReasoning/rpc");

    string state;

    isconnected2reasoning = Network::isConnected(name_abm2reasoning.c_str(), "/efaa/abmReasoning/rpc");

    if (isconnected2reasoning)
    {
        state = "ABM is now connected to abmReasoning";
    }

    else
    {
        state = "ABM failed to connect to abmReasoning";
    }

    Bottle bOutput;
    bOutput.addString(state.c_str());

    return bOutput;
}


Bottle autobiographicalMemory::detectFailed()
{
    Bottle bOutput;
    Bottle	bMessenger = requestFromString("SELECT instance FROM main WHERE activitytype = 'actionfailed' AND begin = true");


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
    Bottle bMessenger, bReply, bDistinctAgent;
    bDistinctAgent = requestFromString("SELECT DISTINCT name FROM agent");

    for (unsigned int iDA = 0 ; iDA < bDistinctAgent.size() ; iDA++)
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
            osAgent << "SELECT color FROM agent WHERE (instance = " << instance << " and name = '" << sName <<"')";

            bReply = requestFromString(osAgent.str());
            tuple<int, int, int> colorAgent = tupleIntFromString(bReply.get(0).asList()->get(0).toString().c_str());

            Agent *TempAgent = opcWorld->addAgent(sName.c_str());
            TempAgent->m_color[0] = get<0>(colorAgent);
            TempAgent->m_color[1] = get<1>(colorAgent);
            TempAgent->m_color[2] = get<2>(colorAgent);	
            TempAgent->m_present = false;

            opcWorld->commit(TempAgent);
            incrAgent++;
        }
    }


    // 2. RTObjects : 
    Bottle bDistincRto = requestFromString("SELECT DISTINCT name FROM rtobject");

    for (unsigned int iDTRO = 0 ; iDTRO < bDistincRto.size() ; iDTRO++)
    {
        string sName = bDistincRto.get(iDTRO).toString();
        ostringstream osRto;
        if (opcWorld->getEntity(sName.c_str()) == NULL)
        {

            osRto << "SELECT instance FROM rtobject WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1";
            bReply = requestFromString(osRto.str());
            int instance = atoi(bReply.get(0).asList()->get(0).toString().c_str());

            osRto.str("");
            osRto << "SELECT color FROM rtobject WHERE (instance = " << instance << " and name = '" << sName <<"')";

            bReply = requestFromString(osRto.str());
            tuple<int, int, int> colorRto = tupleIntFromString(bReply.get(0).asList()->get(0).toString().c_str());

            RTObject *RTO = opcWorld->addRTObject(sName.c_str());
            RTO->m_color[0] = get<0>(colorRto);
            RTO->m_color[1] = get<1>(colorRto);
            RTO->m_color[2] = get<2>(colorRto);	
            RTO->m_present = false;

            opcWorld->commit(RTO);
            incrRTO++;
        }
    }

    // 3. Objects : 

    if (bPutObjectsOPC)
    {
        Bottle bDistinctObject = requestFromString("SELECT DISTINCT name FROM object");

        for (unsigned int iDO = 0 ; iDO < bDistinctObject.size() ; iDO++)
        {
            string sName = bDistinctObject.get(iDO).toString();
            ostringstream osObject;
            if (opcWorld->getEntity(sName.c_str()) == NULL)
            {

                osObject << "SELECT instance FROM object WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1";
                bReply = requestFromString(osObject.str());
                int instance = atoi(bReply.get(0).asList()->get(0).toString().c_str());

                osObject.str("");
                osObject << "SELECT color FROM object WHERE (instance = " << instance << " and name = '" << sName <<"')";

                bReply = requestFromString(osObject.str());
                tuple<int, int, int> colorObject = tupleIntFromString(bReply.get(0).asList()->get(0).toString().c_str());

                Object *TempObj = opcWorld->addObject(sName.c_str());
                TempObj->m_color[0] = get<0>(colorObject);
                TempObj->m_color[1] = get<1>(colorObject);
                TempObj->m_color[2] = get<2>(colorObject);	


                osObject.str("");
                osObject << "SELECT dimension FROM object WHERE (instance = " << instance << " and name = '" << sName <<"')";

                bReply = requestFromString(osObject.str());
                tuple<double, double, double> sizeObject = tupleDoubleFromString(bReply.get(0).asList()->get(0).toString().c_str());

                TempObj->m_color[0] = get<0>(sizeObject);
                TempObj->m_color[1] = get<1>(sizeObject);
                TempObj->m_color[2] = get<2>(sizeObject);	

                osObject.str("");
                osObject << "SELECT orientation FROM object WHERE (instance = " << instance << " and name = '" << sName <<"')";

                bReply = requestFromString(osObject.str());
                tuple<double, double, double> orientationObject = tupleDoubleFromString(bReply.get(0).asList()->get(0).toString().c_str());

                TempObj->m_ego_orientation[0] = get<0>(orientationObject);
                TempObj->m_ego_orientation[1] = get<1>(orientationObject);
                TempObj->m_ego_orientation[2] = get<2>(orientationObject);	

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
tuple<int, int, int> autobiographicalMemory::tupleIntFromString(string sInput)
{
    tuple<int, int, int>	tOutput;
    char *cInput;
    cInput = (char*)sInput.c_str();
    int	iLevel = 0;
    unsigned int data =0;
    string sX = "", sY = "", sZ ="";
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

    get<0>(tOutput) = atoi(sX.c_str());
    get<1>(tOutput) = atoi(sY.c_str());
    get<2>(tOutput) = atoi(sZ.c_str());

    return tOutput;
}

/*
* return a tuple of 3 double from a string input
*/
tuple<double, double, double> autobiographicalMemory::tupleDoubleFromString(string sInput)
{
    tuple<double, double, double>	tOutput;
    char *cInput;
    cInput = (char*)sInput.c_str();
    int	iLevel = 0;
    unsigned int data =0;
    string sX = "", sY = "", sZ ="";
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

    get<0>(tOutput) = atof(sX.c_str());
    get<1>(tOutput) = atof(sY.c_str());
    get<2>(tOutput) = atof(sZ.c_str());

    return tOutput;
}


Bottle autobiographicalMemory::getInfoAbout(string sName)
{
    Bottle bMessenger, bOutput;

    if (opcWorld->getEntity(sName) != NULL)
    {
        // The object is not in the OPC, we have to search in the memory to get the type.
        ostringstream osEntity;
        osEntity << "SELECT instance, opcid FROM entity WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1" ;
        bMessenger = requestFromString(osEntity.str());

        cout << "bMessenger : " << bMessenger.toString() << endl;

        int Instance = atoi(bMessenger.get(0).asList()->get(0).toString().c_str()),
            Opcid = atoi(bMessenger.get(0).asList()->get(1).toString().c_str());

        osEntity.str("");
        osEntity << "SELECT subtype FROM contentopc WHERE instance = " << Instance << " AND opcid = " << Opcid ;
        bMessenger = requestFromString(osEntity.str());

        string sSubType = bMessenger.get(0).asList()->get(0).toString();

        osEntity.str("");
        osEntity << "SELECT count(*) FROM contentarg WHERE argument = '" << sName << "'" ;
        bMessenger = requestFromString(osEntity.str());

        int iNbInteraction = atoi(bMessenger.get(0).asList()->get(0).toString().c_str());
        cout << "I have interacted with this " << sSubType << " " << iNbInteraction/2 << " times ! " << endl;


    }



    return bOutput;

}

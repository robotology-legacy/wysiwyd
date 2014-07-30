//#include <stdafx.h>
//#include <winsock2.h> 
#include <db/PostgreSQL.h>
#include "wrdac/clients/opcEars.h"
#include <yarp/os/all.h>
#include <tuple>


using namespace yarp::os;
using namespace wysiwyd::wrdac;
using namespace std;


const double threshold_time_sequence = 3.	;		//threshold of a same sequence

const string s_real_OPC = "OPC";


class autobiographicalMemory: public RFModule
{
private :
    string server;
    string user;
    string password;
    string dataB;
    string savefile;
    Bottle bSaveRequest;
    opcEars OPCEARS;
    OPCClient *opcWorld;
    string		getCurrentTime();
    bool		inSharedPlan;
    bool		isconnected2reasoning;
    bool		bPutObjectsOPC;

    Bottle		detectFailed();

    DataBase<PostgreSql>* ABMDataBase;

public : 

    autobiographicalMemory(ResourceFinder &rf);
    ~autobiographicalMemory();

    string moduleName;
    string portEventsName;
    Port portEventsIn;
    Port handlerPort;      //a port to handle messages
    Port abm2reasoning;

    void writeInsert(string request);
    bool readInsert();

    Bottle request(Bottle request);
    Bottle requestFromString(string sInput);
    Bottle save(Bottle bInput);
    Bottle addInteraction(Bottle interaction);
    Bottle newDB(Bottle bInput);
    Bottle load(Bottle bInput);
    Bottle restoBottle(ResultSet result);
    Bottle snapshot(Bottle bInput);
    Bottle snapshot2(Bottle bInput);
    Bottle snapshotSP(Bottle bInput);
    Bottle snapshotBehavior(Bottle bInput);
    Bottle connectOPC(Bottle bInput);
    Bottle resetKnowledge();

    Bottle eraseInstance(Bottle bInput);

    Bottle	connect2reasoning();

    bool respond(const Bottle& command, Bottle& reply);
    bool interruptModule();

    double getPeriod();

    bool updateModule();	//	This is our main function. Will be called periodically every getPeriod() seconds.
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();	//  Close function, to perform cleanup.
    static tuple<int,int,int> tupleIntFromString(string sInput);
    static tuple<double,double,double> tupleDoubleFromString(string sInput);

    Bottle populateOPC();

    Bottle getInfoAbout(string sName);

};
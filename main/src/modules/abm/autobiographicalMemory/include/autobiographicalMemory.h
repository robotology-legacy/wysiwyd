#include <db/PostgreSQL.h>
#include "wrdac/clients/opcEars.h"
#include <yarp/os/all.h>
#include <tuple>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>

const double threshold_time_sequence = 3.    ;        //threshold of a same sequence

const std::string s_real_OPC = "OPC";


class autobiographicalMemory: public yarp::os::RFModule
{
private :
    std::string server;
    std::string user;
    std::string password;
    std::string dataB;
    std::string savefile;
    std::string storingPath;        //context/conf path to store data by default
    std::string storingTmpPath ;      //folder inside storingPath for temp image to transfer
    std::string robotName ;
    std::string camName ;
    std::string camSide ;
    std::string camExtension ;
    std::string imgFormat ;

    //for update camera stream
    std::string streamStatus ;
    std::string currentPathFolder ;
    std::string imgLabel ;
    std::string robotPortCam ;
    int imgNb; 
    int imgNbInStream ;
    int imgInstance ;
    int currentInstance ;

    yarp::os::Bottle bListImages ;

    yarp::os::Bottle bSaveRequest;
    wysiwyd::wrdac::opcEars OPCEARS;
    wysiwyd::wrdac::OPCClient *opcWorld;
    std::string        getCurrentTime();
    bool        inSharedPlan;
    bool        isconnected2reasoning;
    bool        isconnected2Cam ;
    bool        bPutObjectsOPC;

    yarp::os::Bottle        detectFailed();

    DataBase<PostgreSql>* ABMDataBase;

public : 

    autobiographicalMemory(yarp::os::ResourceFinder &rf);
    ~autobiographicalMemory();

    std::string moduleName;
    std::string portEventsName;
    yarp::os::Port portEventsIn;
    yarp::os::Port handlerPort;      //a port to handle messages
    yarp::os::Port abm2reasoning;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortOut;

    void writeInsert(std::string request);
    bool readInsert();

    yarp::os::Bottle request(yarp::os::Bottle request);
    yarp::os::Bottle requestFromString(std::string sInput);
    yarp::os::Bottle save(yarp::os::Bottle bInput);
    yarp::os::Bottle addInteraction(yarp::os::Bottle interaction);
    yarp::os::Bottle newDB(yarp::os::Bottle bInput);
    yarp::os::Bottle load(yarp::os::Bottle bInput);
    yarp::os::Bottle restoBottle(ResultSet result);
    yarp::os::Bottle snapshot(yarp::os::Bottle bInput);
    yarp::os::Bottle snapshot2(yarp::os::Bottle bInput);
    yarp::os::Bottle snapshotSP(yarp::os::Bottle bInput);
    yarp::os::Bottle snapshotBehavior(yarp::os::Bottle bInput);
    yarp::os::Bottle connectOPC(yarp::os::Bottle bInput);
    yarp::os::Bottle resetKnowledge();

    yarp::os::Bottle eraseInstance(yarp::os::Bottle bInput);

    yarp::os::Bottle testSaveImage(yarp::os::Bottle bInput);
    yarp::os::Bottle testSendImage(yarp::os::Bottle bInput);

    bool createImage(std::string fullPath);
    bool sendImage(std::string fullPath);
    bool exportImage(int img_oid, std::string path);
    bool storeImage(int instance, std::string label, std::string fullPath, std::string imgName);
    int sendStreamImage(int instance); //return nb of images that will be sent


    yarp::os::Bottle    connect2reasoning();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool interruptModule();

    double getPeriod();

    bool updateModule();    //    This is our main function. Will be called periodically every getPeriod() seconds.
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();    //  Close function, to perform cleanup.
    static std::tuple<int,int,int> tupleIntFromString(std::string sInput);
    static std::tuple<double,double,double> tupleDoubleFromString(std::string sInput);

    yarp::os::Bottle populateOPC();

    yarp::os::Bottle getInfoAbout(std::string sName);

};
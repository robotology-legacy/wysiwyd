#ifndef _AUTOBIOGRAPHICALMEMORY_
#define _AUTOBIOGRAPHICALMEMORY_

#include <db/PostgreSQL.h>
#include "wrdac/clients/opcEars.h"
#include <yarp/os/all.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#ifdef __linux__
#include <sys/stat.h>
#endif

const std::string s_real_OPC = "OPC";

class autobiographicalMemory: public yarp::os::RFModule
{
private:
    // connection to database
    std::string server;
    std::string user;
    std::string password;
    std::string dataB;
    std::string savefile;

    std::string storingPath;        //context/conf path to store data by default
    std::string storingTmpSuffix;   //folder inside storingPath for temp image to transfer

    DataBase<PostgreSql>* ABMDataBase;
    yarp::os::Mutex database_mutex;

    // visualABM
    std::string imgFormat;
    std::string streamStatus;
    std::string imgLabel;

    int imgNb;
    int imgInstance;
    int currentInstance;

    yarp::os::Bottle bListImages;

    // connection to OPC / reasoning
    wysiwyd::wrdac::opcEars OPCEARS;
    wysiwyd::wrdac::OPCClient *opcWorld;

    bool isconnected2reasoning;
    bool bPutObjectsOPC; // not used!

    // helpers
    yarp::os::Bottle detectFailed();

public:
    yarp::os::Port portEventsIn;
    yarp::os::Port handlerPort;      //a port to handle messages
    yarp::os::Port abm2reasoning;

    // autobiographicalMemory
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool interruptModule();
    double getPeriod();
    bool updateModule(); // This is our main function. Will be called periodically every getPeriod() seconds.
    bool configure(yarp::os::ResourceFinder &rf);
    bool close(); // Close function, to perform cleanup.

    yarp::os::Bottle addInteraction(yarp::os::Bottle interaction);
    yarp::os::Bottle newDB(yarp::os::Bottle bInput);
    yarp::os::Bottle load(yarp::os::Bottle bInput);
    yarp::os::Bottle resetKnowledge();
    yarp::os::Bottle eraseInstance(yarp::os::Bottle bInput);

    // snapshot
    yarp::os::Bottle snapshot(yarp::os::Bottle bInput);
    yarp::os::Bottle snapshotSP(yarp::os::Bottle bInput);
    yarp::os::Bottle snapshotBehavior(yarp::os::Bottle bInput);

    // visualABM
    bool timingEnabled;
    long timeStreamStart;

    int sendStreamImage(int instance, bool timingEnabled=false); //return nb of images that will be sent
    yarp::os::Bottle askImage(int instance);

    bool createImage(std::string fullPath, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);
    bool sendImage(std::string fullPath, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);
    bool sendImage(std::string fullPath);

    int exportImages(int instance, int fromImage=-1, int toImage=-1);
    bool exportImage(int img_oid, std::string path);

    bool storeImage(int instance, std::string label, std::string relativePath, std::string imgTime, std::string currentImgProviderPort);
    bool storeImageAllProviders(bool forSingleInstance=false, std::string fullSentence="");
    bool storeOID();

    yarp::os::Bottle addImgProvider(std::string label, std::string portImgProvider);
    yarp::os::Bottle removeImgProvider(std::string label);

    int openStreamImgPorts(int instance);
    yarp::os::Bottle disconnectImgProviders();
    yarp::os::Bottle connectImgProvider();

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortOut;
    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapStreamImgPortOut;
    std::map <std::string, std::string> mapImgProvider;
    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapImgReceiver;

    // helpers
    std::string getCurrentTime();
    long getCurrentTimeInMS();

    void writeInsert(std::string request);
    bool readInsert();

    yarp::os::Bottle request(yarp::os::Bottle request);
    yarp::os::Bottle requestFromString(std::string sInput);
    yarp::os::Bottle restoBottle(ResultSet result);

    static std::vector<int> tupleIntFromString(std::string sInput);
    static std::vector<double> tupleDoubleFromString(std::string sInput);

    yarp::os::Bottle connect2reasoning();
    yarp::os::Bottle connectOPC(yarp::os::Bottle bInput);
    //yarp::os::Bottle getInfoAbout(std::string sName);

    yarp::os::Bottle populateOPC();
};

#endif

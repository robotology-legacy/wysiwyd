#ifndef _AUTOBIOGRAPHICALMEMORY_
#define _AUTOBIOGRAPHICALMEMORY_

#include <db/PostgreSQL.h>
#include "wrdac/clients/opcEars.h"
#include <yarp/os/all.h>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <map>
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

    unsigned int imgNb;
    int imgInstance;
    int currentInstance;

    // connection to OPC / reasoning
    wysiwyd::wrdac::opcEars OPCEARS;
    wysiwyd::wrdac::OPCClient *opcWorld;

    bool isconnected2reasoning;
    bool bPutObjectsOPC; // not used!

    // helpers
    yarp::os::Bottle detectFailed();

public:
    bool shouldClose;

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
    std::string portPrefix;
    bool timingEnabled;
    long timeStreamStart;
    long timeLastImageSent;
    long timeVeryLastStream;
    unsigned int imgProviderCount;

    yarp::os::Bottle sendStreamImage(int instance, bool timingEnabled=false);
    yarp::os::Bottle askImage(int instance);

    bool createImage(const std::string &fullPath, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);
    bool sendImage(const std::string &fullPath, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);
    bool sendImage(const std::string &fullPath);

    int exportImages(int instance, int fromImage=-1, int toImage=-1);
    int exportImage(int img_oid, const std::string &path);

    bool storeImage(int instance, const std::string &label, const std::string &relativePath, const std::string &imgTime, const std::string &currentImgProviderPort);
    bool storeImageAllProviders(const std::string &synchroTime, bool forSingleInstance=false, std::string fullSentence="");
    bool storeOID();

    yarp::os::Bottle addImgProvider(const std::string &label, const std::string &portImgProvider);
    yarp::os::Bottle removeImgProvider(const std::string &label);

    int openStreamImgPorts(int instance);
    yarp::os::Bottle connectImgProviders();
    yarp::os::Bottle disconnectImgProviders();

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortOut;
    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapStreamImgPortOut;
    std::map <std::string, std::string> mapImgProvider;
    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapImgReceiver;

    unsigned int getImagesProviderCount(int instance);
    long getTimeLastImage(int instance);
    yarp::os::Bottle getListImages(long updateTimeDifference);

    // continuousABM
    bool sendStreamIsInitialized;
    unsigned int contDataProviderCount;

    yarp::os::Bottle addContDataProvider(const std::string &type, const std::string &portContDataProvider);
    yarp::os::Bottle removeContDataProvider(const std::string &type);
    std::map <std::string, yarp::os::BufferedPort< yarp::os::Bottle >*> mapContDataPortOut;
    std::map< std::string, std::string > mapContDataProvider;
    std::map< std::string, yarp::os::BufferedPort< yarp::os::Bottle >*> mapContDataReceiver;

    bool storeContDataAllProviders(const std::string &synchroTime);
    bool storeContData(int instance, const std::string &type, int subtype, const std::string &contDataTime, const std::string &contDataPort, double value);

    int openSendContDataPorts(int instance);
    yarp::os::Bottle connectContDataProviders();
    yarp::os::Bottle disconnectContDataProviders();

    unsigned int getContDataProviderCount(int instance);
    long getTimeLastContData(int instance);
    yarp::os::Bottle getListContData(long updateTimeDifference);

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

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
    std::string server;
    std::string user;
    std::string password;
    std::string dataB;
    std::string savefile;
    std::string storingPath;        //context/conf path to store data by default
    std::string storingTmpSuffix;   //folder inside storingPath for temp image to transfer

    std::string imgFormat;

    //for update camera stream
    std::string streamStatus;
    std::string imgLabel;

    int imgNb;
    int imgInstance;
    int currentInstance;

    yarp::os::Bottle bListImages;

    wysiwyd::wrdac::opcEars OPCEARS;
    wysiwyd::wrdac::OPCClient *opcWorld;

    bool isconnected2reasoning;
    bool bPutObjectsOPC; // not used!

    yarp::os::Bottle detectFailed();

    std::string getCurrentTime();

    DataBase<PostgreSql>* ABMDataBase;
    yarp::os::Mutex database_mutex;

public:
    std::string portEventsName;
    yarp::os::Port portEventsIn;
    yarp::os::Port handlerPort;      //a port to handle messages
    yarp::os::Port abm2reasoning;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortOut;
    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapStreamImgPortOut;

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
    yarp::os::Bottle snapshotSP(yarp::os::Bottle bInput);
    yarp::os::Bottle snapshotBehavior(yarp::os::Bottle bInput);
    yarp::os::Bottle connectOPC(yarp::os::Bottle bInput);
    yarp::os::Bottle resetKnowledge();

    yarp::os::Bottle eraseInstance(yarp::os::Bottle bInput);

    yarp::os::Bottle askImage(int instance);

    bool createImage(std::string fullPath, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);
    bool sendImage(std::string fullPath, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);
    bool sendImage(std::string fullPath);
    int openStreamImgPorts(int instance);
    int exportImages(int instance);
    bool exportImage(int img_oid, std::string path);
    bool storeImage(int instance, std::string label, std::string relativePath, std::string imgTime, std::string currentImgProviderPort);
    bool storeImageAllProviders(bool forSingleInstance=false, std::string fullSentence="");
    bool storeOID();
    int sendStreamImage(int instance); //return nb of images that will be sent

    yarp::os::Bottle addImgProvider(std::string label, std::string portImgProvider);
    yarp::os::Bottle removeImgProvider(std::string label);
    std::map <std::string, std::string> mapImgProvider;
    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapImgReceiver;

    yarp::os::Bottle disconnectImgProviders();
    yarp::os::Bottle connectImgProvider();

    yarp::os::Bottle connect2reasoning();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool interruptModule();

    double getPeriod();

    bool updateModule(); // This is our main function. Will be called periodically every getPeriod() seconds.
    bool configure(yarp::os::ResourceFinder &rf);
    bool close(); // Close function, to perform cleanup.
    static std::vector<int> tupleIntFromString(std::string sInput);
    static std::vector<double> tupleDoubleFromString(std::string sInput);

    yarp::os::Bottle populateOPC();

    //yarp::os::Bottle getInfoAbout(std::string sName);
};

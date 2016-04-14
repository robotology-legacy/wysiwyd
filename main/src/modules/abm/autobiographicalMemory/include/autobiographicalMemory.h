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

#ifndef _AUTOBIOGRAPHICALMEMORY_
#define _AUTOBIOGRAPHICALMEMORY_

#include <cstdio>
#include <map>
#include <vector>

#include <yarp/os/all.h>

#include <db/PostgreSQL.h>
#include <opencv2/opencv.hpp>

#ifdef __linux__
#include <sys/stat.h>
#endif

#include "wrdac/clients/opcEars.h"

const std::string s_real_OPC = "OPC";
const std::string s_mental_OPC = "mentalOPC";

class autobiographicalMemory : public yarp::os::RFModule
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

    unsigned int frameNb;
    int imgInstance;
    int currentInstance;

    // connection to OPC / reasoning
    wysiwyd::wrdac::opcEars OPCEARS;
    wysiwyd::wrdac::OPCClient *opcWorldReal;
    wysiwyd::wrdac::OPCClient *opcWorldMental;

    bool isconnected2reasoning;
    bool bPutObjectsOPC; // not used!

    // helpers
    bool delete_directory(const std::string &dir_to_delete);
    yarp::os::Bottle detectFailed();
    std::ostringstream osInsertTemp;

    bool shouldClose;

    yarp::os::Port handlerPort;      //a port to handle messages
    yarp::os::Port abm2reasoning;

public:
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
    yarp::os::Bottle eraseInstance(const yarp::os::Bottle &bInput);

    // snapshot
    yarp::os::Mutex mutexSnapshot;

    yarp::os::Bottle snapshot(const yarp::os::Bottle &bInput);
    yarp::os::Bottle snapshotSP(const yarp::os::Bottle &bInput);
    yarp::os::Bottle snapshotBehavior(const yarp::os::Bottle &bInput);
    void recogFromGrammarSemantic(const yarp::os::Bottle &bRecogBottle, std::string s_deep, int i_deep, const int iInstance);

    //////////////////////////////////////////////////////////////////////////
    // visual + data streaming
    //////////////////////////////////////////////////////////////////////////
    std::string portPrefixForStreaming;
    bool realtimePlayback;
    long timeStreamStart;
    double speedMultiplier;

    long timeLastImageSent; // will be obsolete
    long timeVeryLastStream; // will be obsolete

    bool sendStreamIsInitialized;
    bool increaseFrameNb;

    yarp::os::Mutex mutexStreamRecord;
    yarp::os::Mutex mutexChangeover;

    yarp::os::Bottle triggerStreaming(int instance, bool realtimePlayback, bool includeAugmented, double speedM, const std::string& robotName);

    // maps to receive / send images
    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapImgStreamPortOut;
    std::map <std::string, yarp::os::BufferedPort<yarp::os::Bottle >*> mapDataStreamPortOut;

    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapImgStreamInput;
    std::map <std::string, yarp::os::BufferedPort<yarp::os::Bottle >*> mapDataStreamInput;

    //sound
    yarp::os::BufferedPort<yarp::sig::Sound> portSoundStreamInput;

    //yarp::os::Bottle provideImagesByFrame(int instance, int frame_number, bool include_augmented = false, std::string provider_port = "");

    bool saveImageFromPort(const std::string &fullPath, const std::string &portFrom, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);
    bool writeImageToPort(const std::string &fullPath, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);

    int saveImagesFromABM(int instance, int fromFrame = -1, int toFrame = -1, std::string provider_port = "");

    // store image / data stream to ABM
    bool processInsertDelayed;
    void requestInsertPushToQueue(const std::string &sRequest);
    void requestInsertProcessQueue();
    std::vector<std::string> requests;

    void storeImagesAndData(const std::string &synchroTime, bool forSingleInstance = false, std::string fullSentence = "");

    bool storeInfoSingleImage(int instance, int frame_number, const std::string &relativePath, const std::string &imgTime, const std::string &currentImgProviderPort);
    bool storeInfoAllImages(const std::string &synchroTime, bool forSingleInstance = false, std::string fullSentence = "");
    bool storeImageOIDs(int instance = -1);

    bool storeDataStreamAllProviders(const std::string &synchroTime);

    // add / remove stream providers
    template<typename T>
    yarp::os::Bottle addStreamProvider(std::map <std::string, yarp::os::BufferedPort<T>* > &map, const std::string &portRemote);
    template<typename T>
    yarp::os::Bottle removeStreamProvider(std::map <std::string, yarp::os::BufferedPort<T>* > &map, const std::string &portStreamProvider);
    template<typename T>
    yarp::os::Bottle listProviders(const std::map <std::string, yarp::os::BufferedPort<T>* > &map);
    template<typename T>
    yarp::os::Bottle disconnectStreamProviders(std::map <std::string, yarp::os::BufferedPort<T>* > &map);

    int openImgStreamPorts(int instance, bool includeAugmented = true);
    int openDataStreamPorts(int instance, std::string robotName = "icubSim");

    //unsigned int getImagesProviderCount(int instance); // obsolete
    //unsigned int getStreamDataProviderCount(int instance); // obsolete

    long getTimeLastStream(int instance, std::string table); // will be obsolete
    yarp::os::Bottle getStreamWithinEpoch(long updateTimeDifference, std::string table, std::string port="");

    // augmented images stuff
    std::string augmentedTime;
    yarp::os::RpcClient abm2augmented;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portAugmentedImagesIn;
    bool requestAugmentedImages(std::string activityname, int number_of_augmentions, int instance = -1);
    void saveAugmentedImages();
    //yarp::os::Bottle getImagesInfo(int instance, bool includeAugmentedImages = true);

    //////////////////////////////////////////////////////////////////////////
    // visual + data streaming end
    //////////////////////////////////////////////////////////////////////////

    // helpers
    std::string getCurrentTime();
    long getCurrentTimeInMS();

    void writeInsert(std::string request);
    bool readInsert();

    yarp::os::Bottle request(const yarp::os::Bottle &request);
    yarp::os::Bottle requestFromString(const std::string &sInput);
    yarp::os::Bottle restoBottle(ResultSet result);

    static std::vector<int> tupleIntFromString(const std::string &sInput);
    static std::vector<double> tupleDoubleFromString(const std::string &sInput);

    yarp::os::Bottle connect2reasoning();
    void connectOPC();
    //yarp::os::Bottle getInfoAbout(std::string sName);

    //yarp::os::Bottle populateOPC();
};

#endif

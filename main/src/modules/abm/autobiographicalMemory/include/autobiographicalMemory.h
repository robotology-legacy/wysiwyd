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

    unsigned int frameNb;
    int imgInstance;
    int currentInstance;

    // connection to OPC / reasoning
    wysiwyd::wrdac::opcEars OPCEARS;
    wysiwyd::wrdac::OPCClient *opcWorld;

    bool isconnected2reasoning;
    bool bPutObjectsOPC; // not used!

    // helpers
    bool delete_directory(const std::string &dir_to_delete);
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

    //////////////////////////////////////////////////////////////////////////
    // visual + data streaming
    //////////////////////////////////////////////////////////////////////////
    std::string portPrefixForStreaming;
    bool realtimePlayback;
    long timeStreamStart;
    double speedMultiplier;

    long timeLastImageSent; // will be obsolete
    long timeVeryLastStream; // will be obsolete
    unsigned int imgProviderCount; // will be obsolete
    unsigned int streamDataProviderCount; // will be obsolete

    bool sendStreamIsInitialized;

    yarp::os::Bottle triggerStreaming(int instance, bool realtimePlayback=true, bool includeAugmented=true, double speedM=1.0);

    // maps to receive / send images
    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapImgStreamPortOut;
    std::map <std::string, yarp::os::BufferedPort< yarp::os::Bottle >*> mapDataStreamPortOut;

    std::map <std::string, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*> mapImgStreamInput;
    std::map< std::string, yarp::os::BufferedPort< yarp::os::Bottle >*> mapDataStreamInput;

    yarp::os::Bottle provideImagesByFrame(int instance, int frame_number, bool include_augmented=false, std::string provider_port="");

    bool saveImageFromPort(const std::string &fullPath, const std::string &portFrom, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);
    bool writeImageToPort(const std::string &fullPath, yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >*);
    void processOneImagePort(const std::string &imagePath, const std::string &relativeImagePath, const std::string &portFrom,
                                                yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* port,
                                                const std::string &synchroTime);

    int saveImagesFromABM(int instance, int fromFrame=-1, int toFrame=-1, std::string provider_port="");

    // store image / data stream to ABM
    void storeImagesAndData(const std::string &synchroTime, bool forSingleInstance=false, std::string fullSentence="");

    bool storeInfoSingleImage(int instance, int frame_number, const std::string &relativePath, const std::string &imgTime, const std::string &currentImgProviderPort);
    bool storeInfoAllImages(const std::string &synchroTime, bool forSingleInstance=false, std::string fullSentence="");
    bool storeImageOIDs(int instance=-1);

    bool storeDataStreamAllProviders(const std::string &synchroTime);

    // add / remove stream providers
    yarp::os::Bottle addImgStreamProvider(const std::string &portImgStreamProvider);
    yarp::os::Bottle removeImgStreamProvider(const std::string &portImgStreamProvider);
    yarp::os::Bottle addDataStreamProvider(const std::string &portDataStreamProvider);
    yarp::os::Bottle removeDataStreamProvider(const std::string &portDataStreamProvider);
    yarp::os::Bottle listImgStreamProviders();
    yarp::os::Bottle listDataStreamProviders();

    int openImgStreamPorts(int instance, bool includeAugmented=true);
    yarp::os::Bottle connectToImgStreamProviders();
    yarp::os::Bottle disconnectFromImgStreamProviders();
    int openDataStreamPorts(int instance);
    yarp::os::Bottle connectDataStreamProviders();
    yarp::os::Bottle disconnectDataStreamProviders();

    unsigned int getImagesProviderCount(int instance); // will be obsolete
    unsigned int getStreamDataProviderCount(int instance);

    long getTimeLastImgStream(int instance); // will be obsolete
    long getTimeLastDataStream(int instance); // will be obsolete
    yarp::os::Bottle getStreamImgWithinEpoch(long updateTimeDifference);
    yarp::os::Bottle getStreamDataWithinEpoch(long updateTimeDifference, std::string port="");

    // augmented images stuff
    yarp::os::Bottle saveAugmentedImages(yarp::os::Bottle bInput);
    yarp::os::Bottle getImagesInfo(int instance, bool includeAugmentedImages=true);

    //////////////////////////////////////////////////////////////////////////
    // visual + data streaming end
    //////////////////////////////////////////////////////////////////////////

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

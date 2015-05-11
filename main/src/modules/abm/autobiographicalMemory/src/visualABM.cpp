/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer, Maxime Petit
 * email:   t.fischer@imperial.ac.uk, m.petit@imperial.ac.uk
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

#ifdef BOOST_AVAILABLE
#include <boost/thread.hpp>
#endif

#include <yarp/sig/all.h>
#include "autobiographicalMemory.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace cv;

Bottle autobiographicalMemory::listImgStreamProviders()
{
    Bottle bReply;

    for (std::map<string, BufferedPort<ImageOf<PixelRgb> >*>::const_iterator it = mapImgStreamInput.begin(); it != mapImgStreamInput.end(); ++it) {
        bReply.addString(it->first);
    }

    return bReply;
}

Bottle autobiographicalMemory::addImgStreamProvider(const string &portImgStreamProvider)
{
    //prepare the ResultSet of the query and the reply
    Bottle bReply;

    if (mapImgStreamInput.find(portImgStreamProvider) == mapImgStreamInput.end()) //key not found
    {
        //add the imgProvider and imgReceiver to the map
        mapImgStreamInput[portImgStreamProvider] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> > ;

        bReply.addString("[ack]");
    }
    else { //key found
        cout << "ERROR : addImgStreamProvider : " << portImgStreamProvider << " is already present!" << endl;
        bReply.addString("ERROR : addImgStreamProvider : " + portImgStreamProvider + " is already present!");
    }

    //send the reply
    return bReply;
}

Bottle autobiographicalMemory::removeImgStreamProvider(const string &portImgStreamProvider)
{
    //prepare the ResultSet of the query and the reply
    Bottle bReply;

    if (mapImgStreamInput.find(portImgStreamProvider) == mapImgStreamInput.end()) { //key not found
        cout << "ERROR : removeImgStreamProvider : " << portImgStreamProvider << " is NOT present! " << endl;
        bReply.addString("ERROR : removeImgStreamProvider : " + portImgStreamProvider + " is NOT present! ");
    }
    else { //key found
        mapImgStreamInput[portImgStreamProvider]->interrupt();
        mapImgStreamInput[portImgStreamProvider]->close();
        mapImgStreamInput.erase(portImgStreamProvider);
        bReply.addString("[ack]");
    }

    return bReply;
}

Bottle autobiographicalMemory::getImagesInfo(int instance, bool includeAugmentedImages) {
    Bottle bRequest, bOutput;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT MAX(frame_number) FROM visualdata WHERE instance = " << instance << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    if(bRequest.get(0).toString() != "NULL") {
        bOutput.addInt(atoi(bRequest.get(0).asList()->get(0).toString().c_str()));

        bRequest.clear();
        osArg.str("");
        bRequest.addString("request");
        osArg << "SELECT DISTINCT img_provider_port FROM visualdata WHERE instance = " << instance;
        if(!includeAugmentedImages) {
            osArg << " AND augmented IS NULL";
        }
        osArg << endl;
        bRequest.addString(osArg.str());
        bRequest = request(bRequest);

        bOutput.addList() = bRequest;
    } else { // no images for given instance
        bOutput.addString("0");
    }

    return bOutput;
}

//Ask for images of a precise instance and frame_number
//If several provider were used, a single image from each image provider will be sent
// Return : ack ( ((imageMeta1.1) (image1.1)) ((imageMeta1.2) (image1.2)) ((imageMeta1.3) (image1.3)) )
Bottle autobiographicalMemory::provideImagesByFrame(int instance, int frame_number, bool include_augmented, string provider_port)
{
    Bottle bSubOutput, bOutput, bRequest;
    ostringstream osArg;

    //export all the images for the specific frame of the instance into the temp folder
    saveImagesFromABM(instance, frame_number, frame_number, provider_port);

    bRequest.addString("request");
    osArg << "SELECT relative_path, img_provider_port, time FROM visualdata";
    osArg << " WHERE instance = " << instance;
    osArg << " AND frame_number = " << frame_number;
    if(provider_port!="") {
        osArg << " AND img_provider_port = '" << provider_port << "'";
    }
    if(!include_augmented) {
        osArg << " AND augmented IS NULL";
    }

    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    if(bRequest.toString()=="NULL") {
        bOutput.addString("nack");
        return bOutput;
    }

    for (int i = 0; i < bRequest.size(); i++) {
        string relative_path = bRequest.get(i).asList()->get(0).toString();
        string imgProviderPort = bRequest.get(i).asList()->get(1).asString();
        string imgTime = bRequest.get(i).asList()->get(2).asString();
        //cout << "Create image " << i << " " << relative_path << endl;

        // create image in the /storePath/tmp/relative_path
        IplImage* img = cvLoadImage((storingPath + "/" + storingTmpSuffix + "/" + relative_path).c_str(), CV_LOAD_IMAGE_UNCHANGED);
        if (img == 0) {
            cout << "Image " << storingPath << "/" << storingTmpSuffix << "/" + relative_path << " could not be loaded.";
            bOutput.addString((storingPath + "/" + storingTmpSuffix + "/" + relative_path).c_str());
            return bOutput;
        }

        //create a yarp image
        cvCvtColor(img, img, CV_BGR2RGB);
        ImageOf<PixelRgb> temp;
        temp.resize(img->width, img->height);
        cvCopyImage(img, (IplImage *)temp.getIplImage());

        //for the current image of the actual image provider, we add meta information to the image
        Bottle bCurrentImageMeta;
        bCurrentImageMeta.addInt(instance);
        bCurrentImageMeta.addInt(frame_number);
        bCurrentImageMeta.addString(imgProviderPort);
        bCurrentImageMeta.addString(imgTime);

        Bottle bCurrentImage;
        bCurrentImage.addList() = bCurrentImageMeta;
        //use this copyPortable in order to be able to send the image as a bottle in the rpc port of the respond method
        yarp::os::Portable::copyPortable(temp, bCurrentImage.addList());
        bSubOutput.addList() = bCurrentImage;

        cvReleaseImage(&img);
    }

    bOutput.addString("ack");
    bOutput.addList() = bSubOutput;

    return bOutput;
}

Bottle autobiographicalMemory::connectToImgStreamProviders()
{
    Bottle bOutput;

    if (mapImgStreamInput.size() == 0){
        bOutput.addString("ERROR [connectToImgStreamProviders] the map is NULL");
        return bOutput;
    }

    for (std::map<string, BufferedPort<ImageOf<PixelRgb> >*>::const_iterator it = mapImgStreamInput.begin(); it != mapImgStreamInput.end(); ++it)
    {
        string portImgStreamReceiver = "/" + getName() + "/images" + it->first + "/in";
        it->second->open(portImgStreamReceiver);
        //it->first: port name of img Provider
        //it->second->getName(): portname of imgReceiver which correspond to the label of imgProvider
        //cout << "  [connectToImgStreamProviders] : trying to connect " << it->first << " with " <<  it->second->getName() << endl;
        if (!Network::isConnected(it->first, it->second->getName().c_str())) {
            //cout << "Port is NOT connected : we will connect" << endl;
            if (!Network::connect(it->first, it->second->getName().c_str())) {
                cout << "Error: Connection could not be setup" << endl;
                bOutput.addString(it->first);
            }
            //cout << "Connection from : " << it->first << endl;
            //cout << "Connection to   : " << it->second->getName() << endl;
        }
        else {
            //cout << "Error: Connection already present!" << endl ;
        }
    }

    if (bOutput.size() == 0){
        bOutput.addString("ack");
    }

    return bOutput;
}

Bottle autobiographicalMemory::disconnectFromImgStreamProviders()
{
    Bottle bOutput;
    bool isAllDisconnected = true;

    if (mapImgStreamInput.size() == 0){
        bOutput.addString("ERROR [disconnectFromImgStreamProviders] the map is NULL");
        return bOutput;
    }

    for (std::map<string, BufferedPort<ImageOf<PixelRgb> >*>::const_iterator it = mapImgStreamInput.begin(); it != mapImgStreamInput.end(); ++it)
    {
        //it->second: port name of img Provider
        //mapImgReceiver.find(it->first)->second: port name of imgReceiver which correspond to the label of imgProvider
        Network::disconnect(it->first, it->second->getName().c_str());
        if (Network::isConnected(it->first, it->second->getName().c_str())) {
            cout << "ERROR [disconnectFromImgStreamProviders] " << it->first << " is NOT disconnected!";
            bOutput.addString(it->first);
            isAllDisconnected = false;
        } else {
            //cout << "[disconnectFromImgStreamProviders] " << it->first << " successfully disconnected!"  << endl ;

            //Have to close/interrupt each time otherwise the port is not responsive anymore
            it->second->interrupt();
            it->second->close();
        }
    }

    if (isAllDisconnected == true){
        bOutput.addString("ack");
    }

    //cout << "[disconnectFromImgStreamProviders] bOutput = {" << bOutput.toString().c_str() << "}" << endl ;
    return bOutput;
}

bool autobiographicalMemory::saveImageFromPort(const string &fullPath, const string &fromPort, BufferedPort<ImageOf<PixelRgb> >* imgPort)
{
    //Extract the incoming images from yarp
    ImageOf<PixelRgb> *yarpImage = imgPort->read(false);
    //cout << "imgPort name : " << imgPort->getName() << endl ;

    if (yarpImage != NULL) { // check we actually got something
        //use opencv to convert the image and save it
        IplImage *cvImage = cvCreateImage(cvSize(yarpImage->width(), yarpImage->height()), IPL_DEPTH_8U, 3);
        cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage, CV_RGB2BGR);
        cvSaveImage(fullPath.c_str(), cvImage);

        //cout << "img created : " << fullPath << endl ;
        cvReleaseImage(&cvImage);
        return true;
    } else {
        //cout << "[saveImageFromPort] No image received from: " << imgPort->getName() << endl;
        return false;
    }
}

bool autobiographicalMemory::writeImageToPort(const string &fullPath, BufferedPort<ImageOf<PixelRgb> >* imgPort)
{
    //cout << "Going to send : " << fullPath << endl;
    IplImage* img = cvLoadImage(fullPath.c_str(), CV_LOAD_IMAGE_UNCHANGED);
    if (img == 0)
        return false;

    //create a yarp image
    cvCvtColor(img, img, CV_BGR2RGB);
    ImageOf<PixelRgb> &temp = imgPort->prepare();
    temp.resize(img->width, img->height);
    cvCopyImage(img, (IplImage *)temp.getIplImage());

    //send the image
    imgPort->write();

    cvReleaseImage(&img);

    return true;
}

Bottle autobiographicalMemory::triggerStreaming(int instance, bool timingE, bool includeAugmented, double speedM, string robotName)
{
    Bottle bReply;
    realtimePlayback = timingE;
    speedMultiplier = speedM;
    openImgStreamPorts(instance, includeAugmented);
    openDataStreamPorts(instance, robotName);
    // make sure images are stored in ABM before saving them
    storeImageOIDs(instance);
    int imageCount = saveImagesFromABM(instance);
    streamStatus = "send"; //streamStatus changed (triggered in update())

    bReply.addString(streamStatus);
    bReply.addInt(imageCount);

    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT DISTINCT img_provider_port FROM visualdata WHERE instance = " << instance;
    if(!includeAugmented) {
        osArg << " AND augmented IS NULL";
    }
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    Bottle bImgProviders;
    for(int i = 0; i < bRequest.size() && bRequest.toString()!="NULL"; i++) {
        bImgProviders.addString(portPrefixForStreaming + bRequest.get(i).asList()->get(0).asString().c_str());
    }

    bReply.addList() = bImgProviders;

    bRequest.clear();
    osArg.str("");

    bRequest.addString("request");
    osArg << "SELECT DISTINCT label_port FROM proprioceptivedata WHERE instance = " << instance << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    Bottle bDataStreamProviders;
    for(int i = 0; i < bRequest.size() && bRequest.toString()!="NULL"; i++) {
        bDataStreamProviders.addString(portPrefixForStreaming + bRequest.get(i).asList()->get(0).asString().c_str());
    }

    bReply.addList() = bDataStreamProviders;

    return bReply;
}

int autobiographicalMemory::openImgStreamPorts(int instance, bool includeAugmented)
{
    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT DISTINCT img_provider_port, augmented FROM visualdata WHERE instance = " << instance << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    for (int i = 0; i < bRequest.size() && bRequest.toString()!="NULL"; i++) {
        string imgProviderPort = bRequest.get(i).asList()->get(0).asString();
        mapImgStreamPortOut[imgProviderPort] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> >;
        mapImgStreamPortOut[imgProviderPort]->open((portPrefixForStreaming+imgProviderPort).c_str());

        if(includeAugmented || (!includeAugmented && bRequest.get(i).asList()->get(1).asString()=="")) {
            //cout << "Connect " << portPrefixForStreaming+imgProviderPort << " with " << "/yarpview"+portPrefixForStreaming+imgProviderPort << endl;
            Network::connect(portPrefixForStreaming+imgProviderPort, "/yarpview"+portPrefixForStreaming+imgProviderPort);
        }
    }

    cout << "[openImgStreamPorts] just created " << mapImgStreamPortOut.size() << " ports." << endl;

    return mapImgStreamPortOut.size();
}

//store an image into the SQL db /!\ no lo_import/oid!! (high frequency streaming needed)
bool autobiographicalMemory::storeInfoSingleImage(int instance, int frame_number, const string &relativePath, const string &imgTime, const string &currentImgProviderPort)
{
    ostringstream osArg;
    //sql request with instance and label, images are stored from their location
    osArg << "INSERT INTO visualdata(instance, frame_number, relative_path, time, img_provider_port) VALUES (" << instance << ", '" << frame_number << "', '" << relativePath << "', '" << imgTime << "', '" << currentImgProviderPort << "' );";

    if(processInsertDelayed) {
        requestInsertPushToQueue(osArg.str());
    } else {
        Bottle bRequest;
        bRequest.addString("request");
        bRequest.addString(osArg.str());
        request(bRequest);
    }

    return true;
}

void autobiographicalMemory::processOneImagePort(const string &imagePath, const string &relativeImagePath, const string &portFrom,
                                            yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* port,
                                            const string &synchroTime) {
    if(saveImageFromPort(imagePath, portFrom, port)) {
        //cout << "Store image " << imagePath << " in database." << endl;
        //create SQL entry, register the cam image in specific folder
        if(!storeInfoSingleImage(imgInstance, frameNb, relativeImagePath, synchroTime, portFrom)) {
            cout << "[storeInfoAllImages] Something went wrong storing image " << relativeImagePath << endl;
        }
    }
}

//fullSentence is only used in case forSingleInstance=true!
bool autobiographicalMemory::storeInfoAllImages(const string &synchroTime, bool forSingleInstance, string fullSentence) {
    bool allGood = true;

    //go through the ImgReceiver ports
    for (std::map<string, BufferedPort<ImageOf<PixelRgb> >*>::const_iterator it = mapImgStreamInput.begin(); it != mapImgStreamInput.end(); ++it)
    {
        //concatenation of the path to store
        stringstream imgInstanceString; imgInstanceString << imgInstance;

        stringstream imgName;
        string port = it->first;
        replace(port.begin(), port.end(), '/', '_');
        if(forSingleInstance) {
            if (fullSentence == ""){
                fullSentence = imgLabel;
            }
            //take the full sentence, replace space by _ to have the img name
            replace(fullSentence.begin(), fullSentence.end(), ' ', '_');
            imgName << fullSentence << port << "." << imgFormat;

            string currentPathFolder = storingPath + "/"; currentPathFolder+=imgInstanceString.str();
            yarp::os::mkdir(currentPathFolder.c_str());
#ifdef __linux__
            // we do this because we use postgres user, so that user does not
            // have sufficient permissions to write
            chmod(currentPathFolder.c_str(), 0777);
#endif
        } else {
            imgName << imgLabel << frameNb << port << "." << imgFormat;
        }

        string relativeImagePath = imgInstanceString.str() + "/" + imgName.str();
        string imagePath = storingPath + "/" + relativeImagePath;

        processOneImagePort(imagePath, relativeImagePath, it->first, it->second, synchroTime);
    }

    // only save storeOID if its a single image instance (otherwise it takes too long)
    // for streaming, we take care of this in the triggerStreaming method
    if(forSingleInstance) {
        storeImageOIDs(imgInstance);
    }

    return allGood;
}

bool autobiographicalMemory::storeImageOIDs(int instance) {
    Bottle bRequest;
    ostringstream osStoreOIDReq;

    osStoreOIDReq << "SELECT \"time\", img_provider_port, relative_path FROM visualdata WHERE img_oid IS NULL";
    if(instance >= 0) {
        osStoreOIDReq << " AND instance = " << instance;
    }
    bRequest = requestFromString(osStoreOIDReq.str());

    if(bRequest.size()>0 && bRequest.get(0).toString() != "NULL") {
        cout << "[storeImageOIDs] This may take a while!" << endl;
    } else {
        return true;
    }

    ostringstream osStoreOID;

    for(int i = 0; i<bRequest.size(); i++) {
        string imgTime = bRequest.get(i).asList()->get(0).toString().c_str();
        string imgProviderPort = bRequest.get(i).asList()->get(1).toString().c_str();
        string imgRelativePath = bRequest.get(i).asList()->get(2).toString().c_str();

        string fullPath = storingPath + "/" + imgRelativePath;
        database_mutex.lock();
        unsigned int new_img_oid = ABMDataBase->lo_import(fullPath.c_str());
        database_mutex.unlock();

        osStoreOID << "UPDATE visualdata SET img_oid=" << new_img_oid;
        osStoreOID << " WHERE time='" << imgTime << "' and img_provider_port = '" << imgProviderPort << "';";

        if((i%100==0 && i!=0) || i==bRequest.size() - 1) {
            requestFromString(osStoreOID.str());
            cout << "[storeImageOIDs] Saved " << i+1 << " images out of " << bRequest.size() << endl;
            osStoreOID.str("");
        }
    }

    return true;
}

// saves all images given an instance
int autobiographicalMemory::saveImagesFromABM(int instance, int fromFrame, int toFrame, string provider_port)
{
    Bottle bRequest;
    ostringstream osArg;

    //extract oid of all the images
    bRequest.addString("request");
    osArg << "SELECT img_oid, relative_path FROM visualdata WHERE";
    osArg << " instance = " << instance;
    if(fromFrame!=-1) {
        osArg << " AND frame_number >= " << fromFrame;
    }
    if(toFrame!=-1) {
        osArg << " AND frame_number <= " << toFrame;
    }
    if(provider_port!="") {
        osArg << " AND img_provider_port = '" << provider_port << "'";
    }
    osArg << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    if(bRequest.toString()!="NULL") {
        //export all the images corresponding to the instance to a tmp folder in order to be sent after (update())
        for (int i = 0; i < bRequest.size(); i++) {
            int imageOID = atoi(bRequest.get(i).asList()->get(0).toString().c_str());
            string relative_path = bRequest.get(i).asList()->get(1).toString();
            if(i==0) { // only create folder to store images once
                string folderName = storingPath + "/" + storingTmpSuffix + "/" + relative_path.substr(0, relative_path.find_first_of("/"));
                yarp::os::mkdir(folderName.c_str());
#ifdef __linux__
                // we do this because we use postgres user, so that user does not
                // have sufficient permissions to write
                chmod(folderName.c_str(), 0777);
 #endif
            }
            cout << "[saveImagesFromABM] Export OID " << imageOID << " to: " << storingPath << "/" << storingTmpSuffix << "/" << relative_path << endl;
            string imgPath = storingPath + "/" + storingTmpSuffix + "/" + relative_path;
             database_mutex.lock();
            ABMDataBase->lo_export(imageOID, imgPath.c_str());
             database_mutex.unlock();
        }

        return bRequest.size(); // return how many images were saved
    } else {
        return 0;
    }
}

unsigned int autobiographicalMemory::getImagesProviderCount(int instance) {
    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT DISTINCT img_provider_port FROM visualdata WHERE instance = " << instance;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    if(bRequest.size()>0 && bRequest.toString()!="NULL") {
        return bRequest.size();
    } else {
        return 0;
    }
}

long autobiographicalMemory::getTimeLastImgStream(int instance) {
    Bottle bRequest;
    ostringstream osArg;

    osArg << "SELECT CAST(EXTRACT(EPOCH FROM time-(SELECT time FROM visualdata WHERE instance = " << instance << " ORDER BY time LIMIT 1)) * 1000000 as INT) as time_difference FROM visualdata WHERE instance = " << instance << " ORDER BY time DESC LIMIT 1;";
    bRequest.addString("request");
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    if(bRequest.size()>0 && bRequest.toString()!="NULL") {
        return atol(bRequest.get(0).asList()->get(0).asString().c_str());
    } else {
        return 0;
    }
}

Bottle autobiographicalMemory::getStreamImgWithinEpoch(long updateTimeDifference) {
    // Find which images to send
    Bottle bListImages;
    bListImages.addString("request");
    ostringstream osArgImages;

    osArgImages << "SELECT * FROM (";
    osArgImages << "SELECT relative_path, img_provider_port, time, ";
    osArgImages << "CAST(EXTRACT(EPOCH FROM time-(SELECT time FROM visualdata WHERE instance = '" << imgInstance << "' ORDER BY time LIMIT 1)) * 1000000 as INT) as time_difference ";
    osArgImages << "FROM visualdata WHERE instance = '" << imgInstance << "' ORDER BY time) s ";
    if(realtimePlayback) {
        osArgImages << "WHERE time_difference <= " << updateTimeDifference << " and time_difference > " << timeLastImageSent << " ORDER BY time DESC LIMIT " << imgProviderCount << ";";
    } else {
        osArgImages << "WHERE time_difference > " << timeLastImageSent << " ORDER BY time ASC LIMIT " << imgProviderCount << ";";
    }

    bListImages.addString(osArgImages.str());
    return request(bListImages);
}

Bottle autobiographicalMemory::saveAugmentedImages(const Bottle &bInput) {
    Bottle bReply;

    for(int i=1; i<bInput.size(); i++) {
        Bottle* bImageWithMeta = bInput.get(i).asList();

        //cout << "extract variables from bottle" << endl;
        Bottle *bMetaInformation = bImageWithMeta->get(0).asList();

        string instanceString = (bMetaInformation->get(0)).toString();
        int instance = atoi(instanceString.c_str());
        string frameNumberString = (bMetaInformation->get(1)).toString();
        int frame_number = atoi(frameNumberString.c_str());
        string providerPort = (bMetaInformation->get(2)).asString().c_str();
        string time = (bMetaInformation->get(3)).asString().c_str();
        string augmentedLabel = bImageWithMeta->get(1).asString();

        //cout << "augmentedLabel: " << augmentedLabel << endl;

        string providerPortSpecifier = providerPort;
        replace(providerPortSpecifier.begin(), providerPortSpecifier.end(), '/', '_');

        //cout << "save image from bottle to file" << endl;
        ImageOf<PixelRgb> yarpImage;
        Bottle* bImage = bImageWithMeta->get(2).asList();
        yarp::os::Portable::copyPortable(*bImage, yarpImage);
        IplImage *cvImage = cvCreateImage(cvSize(yarpImage.width(), yarpImage.height()), IPL_DEPTH_8U, 3);
        cvCvtColor((IplImage*)yarpImage.getIplImage(), cvImage, CV_RGB2BGR);

        string folderName = storingPath + "/" + storingTmpSuffix + "/" + augmentedLabel;
        yarp::os::mkdir(folderName.c_str());
#ifdef __linux__
        // we do this because we use postgres user, so that user does not
        // have sufficient permissions to write
        chmod(folderName.c_str(), 0777);
#endif
        string fullPath = folderName + "/" + frameNumberString + providerPortSpecifier + "." + imgFormat;
        cvSaveImage(fullPath.c_str(), cvImage);
        cvReleaseImage(&cvImage);

        // insert image to database
        database_mutex.lock();
        unsigned int img_oid = ABMDataBase->lo_import(fullPath.c_str());
        database_mutex.unlock();

        // insert new row in database
        string relativePath = instanceString + "/augmented_" + augmentedLabel + "_" + frameNumberString + providerPortSpecifier + "." + imgFormat;
        string fullProviderPort = providerPort + "/" + augmentedLabel;

        Bottle bRequest;
        ostringstream osArg;

        bRequest.addString("request");
        osArg << "INSERT INTO visualdata(instance, frame_number, relative_path, time, img_provider_port, img_oid, augmented) VALUES (" << instance << ", '" << frame_number << "', '" << relativePath << "', '" << time << "', '" << fullProviderPort << "', '" << img_oid << "', '" << augmentedLabel << "');";
        bRequest.addString(osArg.str());
        request(bRequest);
    }
    bReply.addString("ack");
    bReply.addString("[saveAugmentedImages] Success");

    return bReply;
}

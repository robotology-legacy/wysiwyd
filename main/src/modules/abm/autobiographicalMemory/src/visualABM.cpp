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

#include <yarp/sig/all.h>
#include "autobiographicalMemory.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace cv;

Bottle autobiographicalMemory::addImgStreamProvider(const string &labelImgProvider, const string &portImgStreamProvider)
{
    //prepare the ResultSet of the query and the reply
    Bottle bReply;

    if (mapImgStreamProvider.find(labelImgProvider) == mapImgStreamProvider.end()) //key not found
    {
        //creating imgReceiverPort for the current provider (DEPRECATED)
        //string portImgReceiver = "/" + getName() + "/images/" + labelImgProvider + "/in";

        //add the imgProvider and imgReceiver to the map
        mapImgStreamProvider[labelImgProvider] = portImgStreamProvider;
        mapImgStreamReceiver[labelImgProvider] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> > ;

        bReply.addString("[ack]");
    }
    else { //key found
        cout << "ERROR : addImgStreamProvider : " << labelImgProvider << " is already present!" << endl;
        bReply.addString("ERROR : addImgStreamProvider : " + labelImgProvider + " is already present!");
    }

    //send the reply
    return bReply;
}

Bottle autobiographicalMemory::removeImgStreamProvider(const string &labelImgProvider)
{
    //prepare the ResultSet of the query and the reply
    Bottle bReply;

    if (mapImgStreamProvider.find(labelImgProvider) == mapImgStreamProvider.end()) { //key not found
        cout << "ERROR : removeImgStreamProvider : " << labelImgProvider << " is NOT present! " << endl;
        bReply.addString("ERROR : removeImgStreamProvider : " + labelImgProvider + " is NOT present! ");
    }
    else { //key found
        mapImgStreamProvider.erase(labelImgProvider);
        mapImgStreamReceiver[labelImgProvider]->interrupt();
        mapImgStreamReceiver[labelImgProvider]->close();
        mapImgStreamReceiver.erase(labelImgProvider);
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

    //export all the images from the instance into the temp folder
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
    osArg << " ORDER BY time" << endl;

    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

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

    if (mapImgStreamProvider.size() == 0){
        bOutput.addString("ERROR [connectToImgStreamProviders] the map is NULL");
        return bOutput;
    }

    for (std::map<string, string>::const_iterator it = mapImgStreamProvider.begin(); it != mapImgStreamProvider.end(); ++it)
    {
        string portImgStreamReceiver = "/" + getName() + "/images/" + it->first + "/in";
        mapImgStreamReceiver.find(it->first)->second->open(portImgStreamReceiver);
        //it->second: port name of img Provider
        //mapImgReceiver.find(it->first)->second: portname of imgReceiver which correspond to the label of imgProvider
        //cout << "  [connectToImgStreamProviders] : trying to connect " << it->second << " with " <<  mapImgStreamReceiver.find(it->first)->second->getName().c_str() << endl ;
        if (!Network::isConnected(it->second, mapImgStreamReceiver.find(it->first)->second->getName().c_str())) {
            //cout << "Port is NOT connected : we will connect" << endl ;
            if (!Network::connect(it->second, mapImgStreamReceiver.find(it->first)->second->getName().c_str())) {
                cout << "Error: Connection could not be setup" << endl;
                bOutput.addString(it->second);
            }
            //cout << "Connection from : " << it->second << endl ;
            //cout << "Connection to   : " << mapImgReceiver.find(it->first)->second->getName().c_str() << endl;
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

    if (mapImgStreamProvider.size() == 0){
        bOutput.addString("ERROR [disconnectFromImgStreamProviders] the map is NULL");
        return bOutput;
    }

    for (std::map<string, string>::const_iterator it = mapImgStreamProvider.begin(); it != mapImgStreamProvider.end(); ++it)
    {
        //it->second: port name of img Provider
        //mapImgReceiver.find(it->first)->second: port name of imgReceiver which correspond to the label of imgProvider
        Network::disconnect(it->second, mapImgStreamReceiver.find(it->first)->second->getName().c_str());
        if (Network::isConnected(it->second, mapImgStreamReceiver.find(it->first)->second->getName().c_str())) {
            cout << "ERROR [disconnectFromImgStreamProviders] " << it->second << " is NOT disconnected!";
            bOutput.addString(it->second);
            isAllDisconnected = false;
        } else {
            //cout << "[disconnectFromImgStreamProviders] " << it->second << " successfully disconnected!"  << endl ;

            //Have to close/interrupt each time otherwise the port is not responsive anymore
            mapImgStreamReceiver.find(it->first)->second->interrupt();
            mapImgStreamReceiver.find(it->first)->second->close();
        }
    }

    if (isAllDisconnected == true){
        bOutput.addString("ack");
    }

    //cout << "[disconnectFromImgStreamProviders] bOutput = {" << bOutput.toString().c_str() << "}" << endl ;
    return bOutput;
}

bool autobiographicalMemory::saveImageFromPort(const string &fullPath, BufferedPort<ImageOf<PixelRgb> >* imgPort)
{
    //Extract the incoming images from yarp
    ImageOf<PixelRgb> *yarpImage = imgPort->read();
    //cout << "imgPort name : " << imgPort->getName() << endl ;

    if (yarpImage != NULL) { // check we actually got something
        //use opencv to convert the image and save it
        IplImage *cvImage = cvCreateImage(cvSize(yarpImage->width(), yarpImage->height()), IPL_DEPTH_8U, 3);
        cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage, CV_RGB2BGR);
        cvSaveImage(fullPath.c_str(), cvImage);

        //cout << "img created : " << fullPath << endl ;
        cvReleaseImage(&cvImage);
    }
    else {
        cout << "[saveImageFromPort] No image received from: " << imgPort->getName() << endl;
        return false;
    }

    return true;
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

Bottle autobiographicalMemory::triggerStreaming(int instance, bool timingE)
{
    Bottle bReply;
    realtimePlayback = timingE;
    openImgStreamPorts(instance);
    openDataStreamPorts(instance);
    int imageCount = saveImagesFromABM(instance);
    streamStatus = "send"; //streamStatus changed (triggered in update())

    bReply.addString(streamStatus);
    bReply.addInt(imageCount);

    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT DISTINCT img_provider_port FROM visualdata WHERE instance = " << instance << endl;
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

int autobiographicalMemory::openImgStreamPorts(int instance)
{
    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT DISTINCT img_provider_port FROM visualdata WHERE instance = " << instance << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    for (int i = 0; i < bRequest.size() && bRequest.toString()!="NULL"; i++) {
        string imgProviderPort = bRequest.get(i).asList()->get(0).asString();
        mapImgStreamPortOut[imgProviderPort] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> >;
        mapImgStreamPortOut[imgProviderPort]->open((portPrefixForStreaming+imgProviderPort).c_str());

        Network::connect(portPrefixForStreaming+imgProviderPort, "/yarpview"+portPrefixForStreaming+imgProviderPort);
    }

    cout << "[openImgStreamPorts] just created " << mapImgStreamPortOut.size() << " ports." << endl;

    return mapImgStreamPortOut.size();
}

//store an image into the SQL db /!\ no lo_import/oid!! (high frequency streaming needed)
bool autobiographicalMemory::storeInfoSingleImage(int instance, int frame_number, const string &relativePath, const string &imgTime, const string &currentImgProviderPort)
{
    Bottle bRequest;
    ostringstream osArg;

    //sql request with instance and label, images are stored from their location
    bRequest.addString("request");
    osArg << "INSERT INTO visualdata(instance, frame_number, relative_path, time, img_provider_port) VALUES (" << instance << ", '" << frame_number << "', '" << relativePath << "', '" << imgTime << "', '" << currentImgProviderPort << "' );";
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    return true;
}

//fullSentence is only used in case forSingleInstance=true!
bool autobiographicalMemory::storeInfoAllImages(const string &synchroTime, bool forSingleInstance, string fullSentence) {
    bool allGood = true;
    //go through the ImgReceiver ports

    for (std::map<string, BufferedPort<ImageOf<PixelRgb> >*>::const_iterator it = mapImgStreamReceiver.begin(); it != mapImgStreamReceiver.end(); ++it)
    {
        //concatenation of the path to store
        stringstream imgInstanceString; imgInstanceString << imgInstance;

        stringstream imgName;
        if(forSingleInstance) {
            if (fullSentence == ""){
                fullSentence = imgLabel;
            }
            //take the full sentence, replace space by _ to have the img name
            replace(fullSentence.begin(), fullSentence.end(), ' ', '_');
            imgName << fullSentence << "_" << it->first << "." << imgFormat;

            string currentPathFolder = storingPath + "/"; currentPathFolder+=imgInstanceString.str();
            yarp::os::mkdir(currentPathFolder.c_str());
#ifdef __linux__
            // we do this because we use postgres user, so that user does not
            // have sufficient permissions to write
            chmod(currentPathFolder.c_str(), 0777);
#endif
        } else {
            imgName << imgLabel << frameNb << "_" << it->first << "." << imgFormat;
        }

        string relativeImagePath = imgInstanceString.str() + "/" + imgName.str();

        string imagePath = storingPath + "/" + relativeImagePath;
        if (!saveImageFromPort(imagePath, it->second)) {
            cout << "[storeInfoAllImages]: image not created from " << it->first << endl;
            allGood = false;
        }
        else {
            //cout << "Store image " << imagePath << " in database." << endl;
            //create SQL entry, register the cam image in specific folder
            if(!storeInfoSingleImage(imgInstance, frameNb, relativeImagePath, synchroTime, mapImgStreamProvider[it->first])) {
                allGood = false;
                cout << "[storeInfoAllImages] Something went wrong storing image " << relativeImagePath << endl;
            }
        }
    }

    // only save storeOID if its a single image instance
    // for streaming, we take care of this in updateModule at the stream "end"
    if(forSingleInstance) {
        storeImageOIDs();
    }

    return allGood;
}

bool autobiographicalMemory::storeImageOIDs() {
    Bottle bRequest;
    ostringstream osStoreOIDReq;

    osStoreOIDReq << "SELECT \"time\", img_provider_port, relative_path FROM visualdata WHERE img_oid IS NULL";
    bRequest = requestFromString(osStoreOIDReq.str());

    if(bRequest.size()>0 && bRequest.get(0).toString() != "NULL") {
        cout << "[storeImageOIDs] This may take a while!" << endl;
    } else {
        return true;
    }

    for(int i = 0; i<bRequest.size(); i++) {
        string imgTime = bRequest.get(i).asList()->get(0).toString().c_str();
        string imgProviderPort = bRequest.get(i).asList()->get(1).toString().c_str();
        string imgRelativePath = bRequest.get(i).asList()->get(2).toString().c_str();

        string fullPath = storingPath + "/" + imgRelativePath;
        unsigned int new_img_oid = ABMDataBase->lo_import(fullPath.c_str());

        ostringstream osStoreOID;
        osStoreOID << "UPDATE visualdata SET img_oid=" << new_img_oid;
        osStoreOID << " WHERE time='" << imgTime << "' and img_provider_port = '" << imgProviderPort << "'";

        requestFromString(osStoreOID.str());

        if(i%100==0 || i==bRequest.size() - 1) {
            cout << "[storeImageOIDs] Saved " << i+1 << " images out of " << bRequest.size() << endl;
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
            ABMDataBase->lo_export(imageOID, imgPath.c_str());
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

Bottle autobiographicalMemory::saveAugmentedImages(Bottle bInput) {
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

        string providerPortSpecifier = providerPort.substr(providerPort.find_last_of("/")+1);

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
        string fullPath = folderName + "/" + frameNumberString + "_" + providerPortSpecifier + "." + imgFormat;
        cvSaveImage(fullPath.c_str(), cvImage);
        cvReleaseImage(&cvImage);

        // insert image to database
        unsigned int img_oid = ABMDataBase->lo_import(fullPath.c_str());

        // insert new row in database
        string relativePath = instanceString + "/" + augmentedLabel + "_" + frameNumberString + "_" + providerPortSpecifier + "." + imgFormat;
        string fullProviderPort = providerPort + "/" + augmentedLabel;

        Bottle bRequest;
        ostringstream osArg;

        bRequest.addString("request");
        osArg << "INSERT INTO visualdata(instance, frame_number, relative_path, time, img_provider_port, img_oid, augmented) VALUES (" << instance << ", '" << frame_number << "', '" << relativePath << "', '" << time << "', '" << fullProviderPort << "', '" << img_oid << "', '" << augmentedLabel << "');";
        bRequest.addString(osArg.str());
        bRequest = request(bRequest);
    }
    bReply.addString("[saveAugmentedImages] Success");

    return bReply;
}

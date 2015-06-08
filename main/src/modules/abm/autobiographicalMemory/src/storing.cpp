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

bool autobiographicalMemory::saveImageFromPort(const string &fullPath, const string &fromPort, BufferedPort<ImageOf<PixelRgb> >* imgPort)
{
    //Extract the incoming images from yarp
    ImageOf<PixelRgb> *yarpImage = imgPort->read(false);
    //yDebug() << "imgPort name : " << imgPort->getName();

    if (yarpImage != NULL) { // check we actually got something
        //use opencv to convert the image and save it
        IplImage *cvImage = cvCreateImage(cvSize(yarpImage->width(), yarpImage->height()), IPL_DEPTH_8U, 3);
        cvCvtColor((IplImage*)yarpImage->getIplImage(), cvImage, CV_RGB2BGR);
        cvSaveImage(fullPath.c_str(), cvImage);

        //yDebug() << "img created : " << fullPath;
        cvReleaseImage(&cvImage);
        return true;
    }
    else {
        //yWarning() << "[saveImageFromPort] No image received from: " << imgPort->getName();
        return false;
    }
}

//store an image into the SQL db /!\ no lo_import/oid!! (high frequency streaming needed)
bool autobiographicalMemory::storeInfoSingleImage(int instance, int frame_number, const string &relativePath, const string &imgTime, const string &currentImgProviderPort)
{
    ostringstream osArg;
    //sql request with instance and label, images are stored from their location
    osArg << "INSERT INTO visualdata(instance, frame_number, relative_path, time, img_provider_port) VALUES (" << instance << ", '" << frame_number << "', '" << relativePath << "', '" << imgTime << "', '" << currentImgProviderPort << "' );";

    if (processInsertDelayed) {
        requestInsertPushToQueue(osArg.str());
    }
    else {
        Bottle bRequest;
        bRequest.addString("request");
        bRequest.addString(osArg.str());
        request(bRequest);
    }

    return true;
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
        if (forSingleInstance) {
            if (fullSentence == ""){
                fullSentence = imgLabel;
            }
            //take the full sentence, replace space by _ to have the img name
            replace(fullSentence.begin(), fullSentence.end(), ' ', '_');
            imgName << fullSentence << port << "." << imgFormat;

            string currentPathFolder = storingPath + "/"; currentPathFolder += imgInstanceString.str();
            yarp::os::mkdir(currentPathFolder.c_str());
#ifdef __linux__
            // we do this because we use postgres user, so that user does not
            // have sufficient permissions to write
            chmod(currentPathFolder.c_str(), 0777);
#endif
        }
        else {
            imgName << imgLabel << frameNb << port << "." << imgFormat;
        }

        string relativeImagePath = imgInstanceString.str() + "/" + imgName.str();
        string imagePath = storingPath + "/" + relativeImagePath;

        if (saveImageFromPort(imagePath, it->first, it->second)) {
            //yDebug() << "Store image " << imagePath << " in database.";
            //create SQL entry, register the cam image in specific folder
            if (!storeInfoSingleImage(imgInstance, frameNb, relativeImagePath, synchroTime, it->first)) {
                yError() << "[storeInfoAllImages] Something went wrong storing image " << relativeImagePath;
            }
        }
    }

    // only save storeOID if its a single image instance (otherwise it takes too long)
    // for streaming, we take care of this in the triggerStreaming method
    if (forSingleInstance) {
        storeImageOIDs(imgInstance);
    }

    return allGood;
}

bool autobiographicalMemory::storeImageOIDs(int instance) {
    Bottle bRequest;
    ostringstream osStoreOIDReq;

    osStoreOIDReq << "SELECT \"time\", img_provider_port, relative_path FROM visualdata WHERE img_oid IS NULL";
    if (instance >= 0) {
        osStoreOIDReq << " AND instance = " << instance;
    }
    bRequest = requestFromString(osStoreOIDReq.str());

    if (bRequest.size() > 0 && bRequest.get(0).toString() != "NULL") {
        yInfo() << "[storeImageOIDs] This may take a while!";
    }
    else {
        return true;
    }

    ostringstream osStoreOID;

    for (int i = 0; i < bRequest.size(); i++) {
        string imgTime = bRequest.get(i).asList()->get(0).toString().c_str();
        string imgProviderPort = bRequest.get(i).asList()->get(1).toString().c_str();
        string imgRelativePath = bRequest.get(i).asList()->get(2).toString().c_str();

        string fullPath = storingPath + "/" + imgRelativePath;
        database_mutex.lock();
        unsigned int new_img_oid = ABMDataBase->lo_import(fullPath.c_str());
        database_mutex.unlock();

        osStoreOID << "UPDATE visualdata SET img_oid=" << new_img_oid;
        osStoreOID << " WHERE time='" << imgTime << "' and img_provider_port = '" << imgProviderPort << "';";

        if ((i % 100 == 0 && i != 0) || i == bRequest.size() - 1) {
            requestFromString(osStoreOID.str());
            yInfo() << "[storeImageOIDs] Saved " << i + 1 << " images out of " << bRequest.size();
            osStoreOID.str("");
        }
    }

    return true;
}

bool autobiographicalMemory::storeDataStreamAllProviders(const string &synchroTime) {
    bool doInsert = false;
    ostringstream osArg;

    osArg << "INSERT INTO proprioceptivedata(instance, subtype, frame_number, time, label_port, value) VALUES ";

    for (std::map<string, BufferedPort<Bottle>*>::const_iterator it = mapDataStreamInput.begin(); it != mapDataStreamInput.end(); ++it)
    {
        Bottle* lastReading = it->second->read(false); // (false) such that we do not wait until data arrives at port

        if (lastReading != NULL) { // only proceed if we got something
            for (int subtype = 0; subtype < lastReading->size(); subtype++) {
                // go ahead if it is NOT a port related to skin OR it is a skin port and the value is bigger than 5.0
                if (it->first.find("skin") == std::string::npos || lastReading->get(subtype).asDouble() > 5.0) {
                    doInsert = true;

                    osArg << "(" << imgInstance << ", '" << subtype << "', '" << frameNb << "', '" << synchroTime << "', '" << it->first << "', '" << lastReading->get(subtype).asDouble() << "' ),";
                }
            }
        }
    }

    if (doInsert) {
        string strRequest = osArg.str(); // needed so we can cut off last character below, which is a ','
        if (processInsertDelayed) {
            requestInsertPushToQueue(strRequest.substr(0, strRequest.size() - 1));
        }
        else {
            Bottle bRequest;
            bRequest.addString("request");
            bRequest.addString(strRequest.substr(0, strRequest.size() - 1));
            request(bRequest);
        }
    }

    return true;
}

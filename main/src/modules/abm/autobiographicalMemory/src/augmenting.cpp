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

#include "autobiographicalMemory.h"

using namespace std;
using namespace yarp::os;

bool autobiographicalMemory::requestAugmentedImages(string activityname, int number_of_augmentions, int instance) {
    yDebug("Starting requestAugmentedImages");
    if (abm2augmented.getOutputCount() == 0) {
        yError("Connect to module to augment images, abort!");
        return false;
    }

    Bottle bRequest;
    bRequest.addString("request");
    ostringstream osRequest;
    osRequest << "SELECT instance, COUNT(DISTINCT augmented_time) as c from visualdata WHERE instance IN (";
    osRequest << "    SELECT instance FROM main WHERE activityname = '" << activityname << "' ";
    osRequest << "    AND begin='true'";
    if (instance != -1) {
        ostringstream osInstance; osInstance << instance;
        osRequest << " AND instance='" << osInstance.str() << "'";
    }
    osRequest << ") GROUP BY instance ";
    osRequest << "HAVING count(DISTINCT augmented_time) < " << number_of_augmentions;
    osRequest << " ORDER BY instance;";
    bRequest.addString(osRequest.str());

    Bottle bResponse = request(bRequest);
    yDebug() << "Response from database: " << bResponse.toString();

    if (bResponse.get(0).toString() == "NULL") {
        yError("Not a suitable instance!");
        return false;
    }

    bool allOkay = true;
    for (int i = 0; i < bResponse.size(); i++) {
        int instance = atoi(bResponse.get(i).asList()->get(0).toString().c_str());
        int existing_number_of_augmentions = atoi(bResponse.get(i).asList()->get(1).toString().c_str());
        Bottle bReqAugmentingModule, bRespAugmentingModule;
        //bReqAugmentingModule.addString("augmentImages");
        //bReqAugmentingModule.addInt(instance);

        bReqAugmentingModule.addString("startStructureLearning");
        bReqAugmentingModule.addInt(instance);
        bReqAugmentingModule.addString("left");
        bReqAugmentingModule.addInt(0);
        bReqAugmentingModule.addInt(600);

        for (int j = existing_number_of_augmentions; j < number_of_augmentions; j++) {
            yInfo() << "Send request to Augmenting module: " << bReqAugmentingModule.toString();
            abm2augmented.write(bReqAugmentingModule, bRespAugmentingModule);
            if (bRespAugmentingModule.get(0).asString() == "ack") {
                yInfo() << "Augmenting for instance " << instance << " at time " << augmentedTime << " was successful";
            }
            else {
                yWarning() << "Did not get positive reply from augmenting module for instance " << instance;
                allOkay = false;
            }
        }
    }

    return allOkay;
}

void autobiographicalMemory::saveAugmentedImages() {
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *img_augmented = portAugmentedImagesIn.read(false);
    if (img_augmented != NULL) {
        Bottle env;
        portAugmentedImagesIn.getEnvelope(env);
        string instanceString = env.get(0).toString();
        int instance = atoi(instanceString.c_str());
        string providerPort = env.get(1).asString();
        string time = env.get(2).asString();
        string frameNumberString = env.get(3).toString();
        int frame_number = atoi(frameNumberString.c_str());
        string augmentedLabel = env.get(4).asString();

        // this is a way to keep track when the first
        // augmented image was sent
        // it is reset as soon as the frame number received
        // is smaller than the last frame number which was received + 10
        // please note that this is a HACK!!!
        if (frame_number + 10 < augmentedLastFrameNumber) {
            augmentedTime = getCurrentTime();
        }
        augmentedLastFrameNumber = frame_number;

        yDebug() << "instance: " << instance;
        yDebug() << "port: " << providerPort;
        yDebug() << "time: " << time;
        yDebug() << "frame: " << frame_number;
        yDebug() << "augmentedLabel: " << augmentedLabel;

        string providerPortSpecifier = providerPort;
        replace(providerPortSpecifier.begin(), providerPortSpecifier.end(), '/', '_');

        IplImage *cvImage = cvCreateImage(cvSize(img_augmented->width(), img_augmented->height()), IPL_DEPTH_8U, 3);
        cvCvtColor((IplImage*)img_augmented->getIplImage(), cvImage, CV_RGB2BGR);

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
        osArg << "INSERT INTO visualdata(instance, frame_number, relative_path, time, img_provider_port, img_oid, augmented, augmented_time) VALUES (" << instance << ", '" << frame_number << "', '" << relativePath << "', '" << time << "', '" << fullProviderPort << "', '" << img_oid << "', '" << augmentedLabel << "', '" << augmentedTime << "');";
        bRequest.addString(osArg.str());
        request(bRequest);
    }
}

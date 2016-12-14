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
using namespace yarp::sig;
using namespace cv;

Bottle autobiographicalMemory::triggerStreaming(int instance, bool timingE, bool includeAugmented, double speedM, const string &robotName)
{
    Bottle bReply;
    while(streamStatus!=StreamStatuses::NONE) {
        yarp::os::Time::delay(1.0);
    }

    realtimePlayback = timingE;
    speedMultiplier = speedM;
    imgInstance = instance;

    openImgStreamPorts(instance, includeAugmented);
    openDataStreamPorts(instance, robotName);
    // make sure images are stored in ABM before saving them
    storeImageOIDs(instance);
    int imageCount = saveImagesFromABM(instance);

    bReply.addString("send");
    bReply.addInt(imageCount);

    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT img_provider_port, augmented, augmented_time, count(img_provider_port) FROM visualdata WHERE instance = " << instance;
    if (!includeAugmented) {
        osArg << " AND augmented IS NULL";
    }
    osArg << " GROUP BY img_provider_port, augmented, augmented_time";

    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    Bottle bImgProviders;
    for (int i = 0; i < bRequest.size() && bRequest.toString() != "NULL"; i++) {
        Bottle bSingleImgProvider;
        string imgProviderPort = bRequest.get(i).asList()->get(0).asString();
        string augmented = bRequest.get(i).asList()->get(1).asString();
        string augmented_time = bRequest.get(i).asList()->get(2).asString();
        string concatenated_port = imgProviderPort + augmented + augmented_time;

        // remove spaces
        std::string::iterator end_pos = std::remove(concatenated_port.begin(), concatenated_port.end(), ' ');
        concatenated_port.erase(end_pos, concatenated_port.end());

        bSingleImgProvider.addString(portPrefixForStreaming + concatenated_port);
        bSingleImgProvider.addInt(atoi(bRequest.get(i).asList()->get(3).asString().c_str()));
        bImgProviders.addList() = bSingleImgProvider;
    }

    bReply.addList() = bImgProviders;

    bRequest.clear();
    osArg.str("");

    bRequest.addString("request");
    osArg << "SELECT label_port, count(label_port) FROM proprioceptivedata WHERE instance = " << instance << " GROUP BY label_port" << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    Bottle bDataStreamProviders;
    for (int i = 0; i < bRequest.size() && bRequest.toString() != "NULL"; i++) {
        Bottle bSingleDataStreamProvider;
        bSingleDataStreamProvider.addString(portPrefixForStreaming + bRequest.get(i).asList()->get(0).asString().c_str());
        bSingleDataStreamProvider.addInt(atoi(bRequest.get(i).asList()->get(1).asString().c_str()));
        bDataStreamProviders.addList() = bSingleDataStreamProvider;
    }

    bReply.addList() = bDataStreamProviders;

    yDebug() << "[mutexChangeover] try locking in triggerStreaming";
    mutexChangeover.lock();
    yDebug() << "[mutexChangeover] locked in triggerStreaming";

    streamStatus = StreamStatuses::SEND; //streamStatus changed (triggered in update())

    return bReply;
}

int autobiographicalMemory::openImgStreamPorts(int instance, bool includeAugmented)
{
    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT DISTINCT img_provider_port, augmented, augmented_time FROM visualdata WHERE instance = " << instance << " ORDER BY augmented_time DESC" << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    map<string, vector<string>> desired_times_local;
    for (int i = 0; i < bRequest.size() && bRequest.toString() != "NULL"; i++) {
        string augmented = bRequest.get(i).asList()->get(1).asString();
        string augmented_time = bRequest.get(i).asList()->get(2).asString();
        if(augmented != "") {
            desired_times_local[augmented].push_back(augmented_time);
        }
    }
    for (int i = 0; i < bRequest.size() && bRequest.toString() != "NULL"; i++) {
        string imgProviderPort = bRequest.get(i).asList()->get(0).asString();
        string augmented = bRequest.get(i).asList()->get(1).asString();
        string augmented_time = bRequest.get(i).asList()->get(2).asString();
        string concatenated_port = imgProviderPort + augmented + augmented_time;

        // remove spaces
        std::string::iterator end_pos = std::remove(concatenated_port.begin(), concatenated_port.end(), ' ');
        concatenated_port.erase(end_pos, concatenated_port.end());

        if (!includeAugmented && augmented == "") {
            if(mapImgStreamPortOut.find(concatenated_port) == mapImgStreamPortOut.end()) {
                mapImgStreamPortOut[concatenated_port] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> > ;
                mapImgStreamPortOut[concatenated_port]->open((portPrefixForStreaming + concatenated_port).c_str());
                yDebug() << "Connect " << concatenated_port << " with " << "/yarpview" + portPrefixForStreaming + imgProviderPort;
                Network::connect(mapImgStreamPortOut[concatenated_port]->getName(), "/yarpview" + portPrefixForStreaming + imgProviderPort);
            } else {
                yWarning() << concatenated_port << "Already existing, not opening again, this should not happen";
            }
        }
        else if (includeAugmented) {
            if (augmented != "") {
                size_t pos = std::find(desired_times_local[augmented].begin(), desired_times_local[augmented].end(), augmented_time) - desired_times_local[augmented].begin();
                if (pos < desired_times_local.size()) {
                    if(mapImgStreamPortOut.find(concatenated_port) == mapImgStreamPortOut.end()) {
                        mapImgStreamPortOut[concatenated_port] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> > ;
                        mapImgStreamPortOut[concatenated_port]->open((portPrefixForStreaming + concatenated_port).c_str());
                        yDebug() << "Connect " << concatenated_port << " with " << "/yarpview" + portPrefixForStreaming + imgProviderPort + std::to_string(pos);
                        Network::connect(mapImgStreamPortOut[concatenated_port]->getName(), "/yarpview" + portPrefixForStreaming + imgProviderPort + std::to_string(pos));
                    } else {
                        yWarning() << concatenated_port << "Already existing, not opening again, this should not happen";
                    }
                }
            } else {
                if(mapImgStreamPortOut.find(concatenated_port) == mapImgStreamPortOut.end()) {
                    mapImgStreamPortOut[concatenated_port] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> > ;
                    mapImgStreamPortOut[concatenated_port]->open((portPrefixForStreaming + concatenated_port).c_str());
                    yDebug() << "Connect " << concatenated_port << " with " << "/yarpview" + portPrefixForStreaming + imgProviderPort;
                    Network::connect(mapImgStreamPortOut[concatenated_port]->getName(), "/yarpview" + portPrefixForStreaming + imgProviderPort);
                } else {
                    yWarning() << concatenated_port << "Already existing, not opening again, this should not happen";
                }
            }
        }
    }

    yInfo() << "[openImgStreamPorts] just created " << mapImgStreamPortOut.size() << " ports.";

    return mapImgStreamPortOut.size();
}

int autobiographicalMemory::openDataStreamPorts(int instance, string robotName) {
    Bottle bDistLabelPort;
    ostringstream osArg;

    bDistLabelPort.addString("request");
    osArg << "SELECT DISTINCT label_port FROM proprioceptivedata WHERE instance = " << instance << endl;
    bDistLabelPort.addString(osArg.str());
    bDistLabelPort = request(bDistLabelPort);

    for (int i = 0; i < bDistLabelPort.size() && bDistLabelPort.toString() != "NULL"; i++) {
        string dataStreamPortFrom = bDistLabelPort.get(i).asList()->get(0).asString();
        if(mapDataStreamPortOut.find(dataStreamPortFrom)==mapDataStreamPortOut.end()) {
            mapDataStreamPortOut[dataStreamPortFrom] = new yarp::os::BufferedPort < Bottle > ;
            mapDataStreamPortOut[dataStreamPortFrom]->open((portPrefixForStreaming + dataStreamPortFrom).c_str());
        } else {
            yWarning << dataStreamPortFrom << "Already existing, not opening again";
        }

        // in case of position commands, replace state:o with rpc:i; otherwise do nothing
        string toReplace = "state:o";
        string dataStreamPortTo = dataStreamPortFrom;
        if (dataStreamPortTo.find(toReplace) != string::npos) {
            dataStreamPortTo.replace(dataStreamPortTo.find(toReplace), toReplace.length(), "rpc:i");
        }

        // if memory was made not on same robot as replay should be made
        string robotReplacement = "/" + robotName + "/";
        size_t second_slash = dataStreamPortTo.find("/", 1);
        dataStreamPortTo.replace(0, second_slash + 1, robotReplacement);

        Network::connect(portPrefixForStreaming + dataStreamPortFrom, dataStreamPortTo, "tcp");
        if (Network::isConnected(portPrefixForStreaming + dataStreamPortFrom, dataStreamPortTo)) {
            yInfo() << "Successfully connected " << portPrefixForStreaming + dataStreamPortFrom << " and " << dataStreamPortTo;
        }
        else {
            yWarning() << "NOT connected " << portPrefixForStreaming + dataStreamPortFrom << " and " << dataStreamPortTo;
        }
    }

    yInfo() << "[openDataStreamPorts] Just created " << mapDataStreamPortOut.size() << " ports.";

    return mapDataStreamPortOut.size();
}

// saves all images given an instance
int autobiographicalMemory::saveImagesFromABM(int instance, int fromFrame, int toFrame, string provider_port)
{
    Bottle bRequest;
    ostringstream osArg;

    //extract oid of all the images
    bRequest.addString("request");
    osArg << "SELECT img_oid, relative_path, augmented_time FROM visualdata WHERE";
    osArg << " instance = " << instance;
    if (fromFrame != -1) {
        osArg << " AND frame_number >= " << fromFrame;
    }
    if (toFrame != -1) {
        osArg << " AND frame_number <= " << toFrame;
    }
    if (provider_port != "") {
        osArg << " AND img_provider_port = '" << provider_port << "'";
    }
    osArg << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    if (bRequest.toString() != "NULL") {
        //export all the images corresponding to the instance to a tmp folder in order to be sent after (update())
        for (int i = 0; i < bRequest.size(); i++) {
            int imageOID = atoi(bRequest.get(i).asList()->get(0).toString().c_str());
            string relative_path = bRequest.get(i).asList()->get(1).toString();
            string augmented_time = bRequest.get(i).asList()->get(2).toString();
            //remove spaces
            std::string::iterator end_pos = std::remove(augmented_time.begin(), augmented_time.end(), ' ');
            augmented_time.erase(end_pos, augmented_time.end());

            if (i == 0) { // only create folder to store images once
                string folderName = storingPath + "/" + storingTmpSuffix + "/" + relative_path.substr(0, relative_path.find_first_of("/"));
                yarp::os::mkdir_p(folderName.c_str());
#ifdef __linux__
                // we do this because we use postgres user, so that user does not
                // have sufficient permissions to write
                chmod(folderName.c_str(), 0777);
#endif
            }
            string imgPath = storingPath + "/" + storingTmpSuffix + "/" + relative_path + augmented_time;
            std::ifstream infile(imgPath.c_str());
            if (!infile.good()) { // file does not exist yet
                yInfo() << "[saveImagesFromABM] Export OID " << imageOID << " to: " << storingPath << "/" << storingTmpSuffix << "/" << relative_path << augmented_time;
                database_mutex.lock();
                ABMDataBase->lo_export(imageOID, imgPath.c_str());
                database_mutex.unlock();
            }
            infile.close();
        }

        return bRequest.size(); // return how many images were saved
    }
    else {
        return 0;
    }
}

Bottle autobiographicalMemory::getStreamWithinEpoch(long updateTimeDifference, string table, string port) {
    // Find which images to send
    Bottle bListImages;
    bListImages.addString("request");
    ostringstream osArgImages;

    osArgImages << "WITH data AS (";
    osArgImages << "SELECT * FROM (";
    if(table=="visualdata") {
        osArgImages << "SELECT relative_path, img_provider_port, time, ";
    } else if(table=="proprioceptivedata") {
        osArgImages << "SELECT subtype, label_port, time, value, ";
    }
    osArgImages << "CAST(EXTRACT(EPOCH FROM time-(SELECT min(time) FROM " << table << " WHERE instance = '" << imgInstance << "')) * 1000000 as BIGINT) as time_difference ";
    if(table=="visualdata") {
        osArgImages << ", frame_number, augmented, augmented_time ";
    }
    osArgImages << "FROM " << table << " WHERE instance = '" << imgInstance << "' ORDER BY time) s ";
    if (realtimePlayback) {
        osArgImages << "WHERE time_difference <= " << updateTimeDifference << " and time_difference > " << timeLastImageSent;
        if(port!="") {
            osArgImages << " and label_port = '" << port << "'";
        }
        osArgImages << "), max_time as (select max(time_difference) as max from data) ";
        osArgImages << "SELECT * FROM data, max_time WHERE data.time_difference = max_time.max";
    }
    else {
        osArgImages << "WHERE time_difference > " << timeLastImageSent;
        if(port!="") {
            osArgImages << " and label_port = '" << port << "'";
        }
        osArgImages << "), min_time AS (select min(time_difference) as min from data) ";
        osArgImages << "SELECT * FROM data, min_time WHERE data.time_difference = min_time.min;";
    }

    bListImages.addString(osArgImages.str());
    return request(bListImages);
}

long autobiographicalMemory::getTimeLastStream(int instance, string table) {
    // only send request to database if we are actually going to send something
    // to that port!
    if(table=="visualdata" && mapImgStreamPortOut.size()==0) {
        return 0;
    } else if(table=="proprioceptivedata" && mapDataStreamPortOut.size()==0) {
        return 0;
    }

    Bottle bRequest;
    ostringstream osArg;

    osArg << "SELECT CAST(EXTRACT(EPOCH FROM time-(SELECT time FROM " << table << " WHERE instance = " << instance << " ORDER BY time LIMIT 1)) * 1000000 as BIGINT) as time_difference FROM " << table << " WHERE instance = " << instance << " ORDER BY time DESC LIMIT 1;";
    bRequest.addString("request");
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    if (bRequest.size() > 0 && bRequest.toString() != "NULL") {
        return atol(bRequest.get(0).asList()->get(0).asString().c_str());
    }
    else {
        return 0;
    }
}

bool autobiographicalMemory::writeImageToPort(const string &fullPath, BufferedPort<ImageOf<PixelRgb> >* imgPort)
{
    //yDebug() << "Going to send : " << fullPath;
    IplImage* img = cvLoadImage(fullPath.c_str(), CV_LOAD_IMAGE_UNCHANGED);
    if (img == 0)
        return false;

    //create a yarp image
    cvCvtColor(img, img, CV_BGR2RGB);
    ImageOf<PixelRgb> &temp = imgPort->prepare();
    temp.resize(img->width, img->height);
    cvCopy(img, (IplImage *)temp.getIplImage());

    //send the image
    imgPort->writeStrict();

    cvReleaseImage(&img);

    return true;
}

#include "autobiographicalMemory.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace cv;

Bottle autobiographicalMemory::addImgProvider(string labelImgProvider, string portImgProvider)
{
    //prepare the ResultSet of the query and the reply
    Bottle bReply;

    if (mapImgProvider.find(labelImgProvider) == mapImgProvider.end()) //key not found
    {
        //creating imgReceiverPort for the current provider (DEPRECATED)
        //string portImgReceiver = "/" + getName() + "/images/" + labelImgProvider + "/in";

        //add the imgProvider and imgReceiver to the map
        mapImgProvider[labelImgProvider] = portImgProvider;
        mapImgReceiver[labelImgProvider] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> > ;

        bReply.addString("[ack]");
    }
    else { //key found
        cout << "ERROR : addImgProvider : " << labelImgProvider << " is already present!" << endl;
        bReply.addString("ERROR : addImgProvider : " + labelImgProvider + " is already present!");
    }

    //send the reply
    return bReply;
}

Bottle autobiographicalMemory::removeImgProvider(string labelImgProvider)
{
    //prepare the ResultSet of the query and the reply
    Bottle bReply;

    if (mapImgProvider.find(labelImgProvider) == mapImgProvider.end()) { //key not found
        cout << "ERROR : removeImgProvider : " << labelImgProvider << " is NOT present! " << endl;
        bReply.addString("ERROR : removeImgProvider : " + labelImgProvider + " is NOT present! ");
    }
    else { //key found
        mapImgProvider.erase(labelImgProvider);
        mapImgReceiver[labelImgProvider]->interrupt();
        mapImgReceiver[labelImgProvider]->close();
        mapImgReceiver.erase(labelImgProvider);
        bReply.addString("[ack]");
    }

    return bReply;
}

//Ask for a single image of a precise instance. It can be because it is a sentence instance or a stream : in that case the first image of the stream will be sent.
//If several provider were used, a single image from each image provider will be sent
// Return : ack ( (labelProvider1 (image1.1)) (labelProvider2 (image1.2)) (labelProvider3 (image1.3)) )
Bottle autobiographicalMemory::askImage(int instance)
{
    Bottle bSubOutput, bOutput, bRequest;
    ostringstream osArg;

    //get distinct img_provider_port

    bRequest.addString("request");
    osArg << "SELECT DISTINCT img_provider_port FROM images WHERE instance = " << instance << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    //export all the images from the instance into the temp folder
    int savedImages = exportImages(instance, 0, bRequest.size());

    // get relative_path and img_provider_port for requested instance
    bRequest.clear();
    osArg.str("");

    bRequest.addString("request");
    osArg << "SELECT relative_path, img_provider_port FROM images WHERE instance = " << instance << " ORDER BY time LIMIT " << savedImages << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    //go through all the image provider for the instance.
    for (int i = 0; i < bRequest.size(); i++) {
        string relative_path = bRequest.get(i).asList()->get(0).toString();
        string imgProviderPort = bRequest.get(i).asList()->get(1).asString();
        //cout << "Create image " << i << " " << relative_path << endl;

        // create image in the /storePath/tmp/relative_path
        IplImage* img = cvLoadImage((storingPath + "/" + storingTmpSuffix + "/" + relative_path).c_str(), CV_LOAD_IMAGE_UNCHANGED);
        if (img == 0) {
            cout << "Image " << storingPath << "/" << storingTmpSuffix << "/" + relative_path << " could not be loaded.";
            bOutput.addString((storingPath + "/" + storingTmpSuffix + "/" + relative_path).c_str());
            return bOutput;
        }

        //cout << "Image created" << endl;
        //create a yarp image
        cvCvtColor(img, img, CV_BGR2RGB);
        ImageOf<PixelRgb> temp;
        temp.resize(img->width, img->height);
        cvCopyImage(img, (IplImage *)temp.getIplImage());

        //cout << "Image copied to yarp image" << endl;

        //for the current image of the actual image provider, we had the label of the provider to the image
        Bottle bCurrentImageProvider;
        bCurrentImageProvider.addString(imgProviderPort);
        //use this copyPortable in order to be able to send the image as a bottle in the rpc port of the responde method
        yarp::os::Portable::copyPortable(temp, bCurrentImageProvider.addList());
        //add the pair imgProviderPort (image) to the subottle
        bSubOutput.addList() = bCurrentImageProvider;
        //cout << "Image copied to bottle" << endl;

        cvReleaseImage(&img);
        //cout << "Image released" << endl;
    }

    bOutput.addString("ack");
    bOutput.addList() = bSubOutput;

    // Testing to get an image from the bottle
    // This is just for testing purposes, and your own module should implement this!

    // Bottle: ack ( (labelProvider1 (image1.1)) (labelProvider2 (image1.2)) (labelProvider3 (image1.3)) )
    string desiredLabel = "/icubSim/cam/right";

    Bottle bResponse = bOutput;
    Bottle* bImages = bResponse.get(1).asList(); // (labelProvider1 (image1.1)) (labelProvider2 (image1.2)) (labelProvider3 (image1.3))

    //go through each of the subottles, one for each imageProvider
    for(int imageProvider = 0; imageProvider < bImages->size(); imageProvider++) {
        Bottle* bImage = bImages->get(imageProvider).asList(); // (labelProvider1 (image1.1))
        //cout << bImage1->toString() << endl;
        string bImageLabel = bImage->get(0).toString(); // labelProvider1

        if(bImageLabel==desiredLabel) {
            ImageOf<PixelRgb> &temp = imagePortOut.prepare();
            Bottle* bRawImage = bImage->get(1).asList(); //image1.1
            yarp::os::Portable::copyPortable(*bRawImage, temp);
        }

        imagePortOut.write();
   }

    return bOutput;
}

Bottle autobiographicalMemory::connectImgProvider()
{
    Bottle bOutput;

    if (mapImgProvider.size() == 0){
        bOutput.addString("ERROR [connectImgProvider] the map is NULL");
        return bOutput;
    }

    for (std::map<string, string>::const_iterator it = mapImgProvider.begin(); it != mapImgProvider.end(); ++it)
    {
        string portImgReceiver = "/" + getName() + "/images/" + it->first + "/in";
        mapImgReceiver.find(it->first)->second->open(portImgReceiver);
        //it->second: port name of img Provider
        //mapImgReceiver.find(it->first)->second: portname of imgReceiver which correspond to the label of imgProvider
        //cout << "  [connectImgProvider] : trying to connect " << it->second << " with " <<  mapImgReceiver.find(it->first)->second->getName().c_str() << endl ;
        if (!Network::isConnected(it->second, mapImgReceiver.find(it->first)->second->getName().c_str())) {
            //cout << "Port is NOT connected : we will connect" << endl ;
            if (!Network::connect(it->second, mapImgReceiver.find(it->first)->second->getName().c_str())) {
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

Bottle autobiographicalMemory::disconnectImgProviders()
{
    Bottle bOutput;
    bool isAllDisconnected = true;

    if (mapImgProvider.size() == 0){
        bOutput.addString("ERROR [disconnectImgProvider] the map is NULL");
        return bOutput;
    }

    for (std::map<string, string>::const_iterator it = mapImgProvider.begin(); it != mapImgProvider.end(); ++it)
    {
        //it->second: port name of img Provider
        //mapImgReceiver.find(it->first)->second: port name of imgReceiver which correspond to the label of imgProvider
        Network::disconnect(it->second, mapImgReceiver.find(it->first)->second->getName().c_str());
        if (Network::isConnected(it->second, mapImgReceiver.find(it->first)->second->getName().c_str())) {
            cout << "ERROR [disconnectImgProvider] " << it->second << " is NOT disconnected!";
            bOutput.addString(it->second);
            isAllDisconnected = false;
        }
        else {
            //cout << "[disconnectImgProvider] " << it->second << " successfully disconnected!"  << endl ;

            //Have to close/interrupt each time otherwise the port is not responsive anymore
            mapImgReceiver.find(it->first)->second->interrupt();
            mapImgReceiver.find(it->first)->second->close();
        }
    }

    if (isAllDisconnected == true){
        bOutput.addString("ack");
    }

    //cout << "[disconnectImgProvider] bOutput = {" << bOutput.toString().c_str() << "}" << endl ;
    return bOutput;
}

bool autobiographicalMemory::createImage(string fullPath, BufferedPort<ImageOf<PixelRgb> >* imgPort)
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
        cout << "ERROR CANNOT SAVE: no image received for: " << imgPort->getName() << endl;
        return false;
    }

    return true;
}

bool autobiographicalMemory::sendImage(string fullPath)
{
    return sendImage(fullPath, &imagePortOut);
}

bool autobiographicalMemory::sendImage(string fullPath, BufferedPort<ImageOf<PixelRgb> >* imgPort)
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

int autobiographicalMemory::sendStreamImage(int instance)
{
    openStreamImgPorts(instance);
    int imageCount = exportImages(instance);
    streamStatus = "send"; //streamStatus changed (triggered in update())

    return imageCount;
}

int autobiographicalMemory::openStreamImgPorts(int instance)
{
    Bottle bRequest;
    ostringstream osArg;

    //get distinct img_provider_port
    bRequest.addString("request");
    osArg << "SELECT DISTINCT img_provider_port FROM images WHERE instance = " << instance << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    for (int i = 0; i < bRequest.size(); i++) {
        string imgProviderPort = bRequest.get(i).asList()->get(0).asString();
        mapStreamImgPortOut[imgProviderPort] = new yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb> >;
        mapStreamImgPortOut[imgProviderPort]->open(("/abm"+imgProviderPort).c_str());

        Network::connect(("/abm"+imgProviderPort).c_str(), "/yarpview/abm"+imgProviderPort);
    }

    cout << "Just created " << mapStreamImgPortOut.size() << " ports." << endl;

    return mapStreamImgPortOut.size();
}

//store an image into the SQL db /!\ no lo_import/oid!! (high frequency streaming needed)
bool autobiographicalMemory::storeImage(int instance, string label, string relativePath, string imgTime, string currentImgProviderPort)
{
    Bottle bRequest;
    ostringstream osArg;

    //sql request with instance and label, images are stored from their location
    bRequest.addString("request");
    osArg << "INSERT INTO images(instance, label, relative_path, time, img_provider_port) VALUES (" << instance << ", '" << label << "', '" << relativePath << "', '" << imgTime << "', '" << currentImgProviderPort << "' );";
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    return true;
}

//fullSentence is only used in case forSingleInstance=true!
bool autobiographicalMemory::storeImageAllProviders(bool forSingleInstance, string fullSentence) {
    bool allGood = true;
    //go through the ImgReceiver ports
    string synchroTime = getCurrentTime();

    for (std::map<string, BufferedPort<ImageOf<PixelRgb> >*>::const_iterator it = mapImgReceiver.begin(); it != mapImgReceiver.end(); ++it)
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
            chmod(currentPathFolder.c_str(), 0777);
#endif
        } else {
            imgName << imgLabel << imgNb << "_" << it->first << "." << imgFormat;
        }

        string relativeImagePath = imgInstanceString.str() + "/" + imgName.str();

        string imagePath = storingPath + "/" + relativeImagePath;
        if (!createImage(imagePath, it->second)) {
            cout << "Error in Update : image not created from " << it->first << endl;
            allGood = false;
        }
        else {
            //cout << "Store image " << imagePath << " in database." << endl;
            //create SQL entry, register the cam image in specific folder
            if(!storeImage(imgInstance, imgLabel, relativeImagePath, synchroTime, mapImgProvider[it->first])) {
                allGood = false;
                cout << "Something went wrong storing image " << relativeImagePath << endl;
            }
        }
    }

    // only save storeOID if its a single image instance
    // for streaming, we take care of this in updateModule at the stream "end"
    if(forSingleInstance) {
        storeOID();
    }

    return allGood;
}

bool autobiographicalMemory::storeOID() {
    Bottle bRequest;
    ostringstream osStoreOIDReq;

    osStoreOIDReq << "SELECT \"time\", img_provider_port, relative_path FROM images WHERE img_oid IS NULL";
    bRequest = requestFromString(osStoreOIDReq.str());

    if(bRequest.size()>0 && bRequest.get(0).toString() != "NULL") {
        cout << "Storing image OID, this may take a while!" << endl;
    } else {
        return true;
    }

    for(int i = 0; i<bRequest.size(); i++) {
        string imgTime = bRequest.get(i).asList()->get(0).toString().c_str();
        string imgProviderPort = bRequest.get(i).asList()->get(1).toString().c_str();
        string imgRelativePath = bRequest.get(i).asList()->get(2).toString().c_str();

        ostringstream osStoreOID;
        osStoreOID << "UPDATE images SET img_oid=lo_import('" << storingPath << "/" << imgRelativePath << "')";
        osStoreOID << "WHERE time='" << imgTime << "' and img_provider_port = '" << imgProviderPort << "'";

        requestFromString(osStoreOID.str());

        if(i%100==0) {
            cout << "Saved " << i << " images out of " << bRequest.size() << endl;
        }
    }

    return true;
}

// exports all images given an instance
int autobiographicalMemory::exportImages(int instance, int fromImage, int toImage)
{
    Bottle bRequest;
    ostringstream osArg;

    //extract oid of all the images
    bRequest.addString("request");
    osArg << "SELECT img_oid, relative_path FROM images WHERE instance = " << instance << " ORDER BY time" << endl;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    if(fromImage<0)
        fromImage = 0;
    if(toImage==-1)
        toImage = bRequest.size();
    if(toImage > bRequest.size()) {
        cout << "Requested to save up to image " << toImage << ", but only " << bRequest.size() << " available." << endl;
        toImage = bRequest.size();
        cout << "Will only send up to image " << toImage << endl;
    }

    //export all the images corresponding to the instance to a tmp folder in order to be sent after (update())
    for (int i = fromImage; i < toImage; i++) {
        int imageOID = atoi(bRequest.get(i).asList()->get(0).toString().c_str());
        string relative_path = bRequest.get(i).asList()->get(1).toString();
        if(i==0) { // only create folder to store images once
            string folderName = storingPath + "/" + storingTmpSuffix + "/" + relative_path.substr(0, relative_path.find_first_of("/"));
            yarp::os::mkdir(folderName.c_str());
#ifdef __linux__
            chmod(folderName.c_str(), 0777);
#endif
        }
        cout << "Call exportImage with " << imageOID << " : " << storingPath + "/" + storingTmpSuffix + "/" + relative_path << endl;
        exportImage(imageOID, storingPath + "/" + storingTmpSuffix + "/" + relative_path);
    }

    // return how many images were saved
    return toImage-fromImage;
}

//export (i.e. save) a stored image to hardrive, using oid to identify and the path wanted
bool autobiographicalMemory::exportImage(int img_oid, string imgPath)
{
    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    //retrieve the image from the db and print it to /storingPath/temp folder
    osArg << "SELECT lo_export(img_oid, '" << imgPath << "') from images WHERE img_oid = '" << img_oid << "';";

    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    //bOutput.addString("ack");

    return true;
}

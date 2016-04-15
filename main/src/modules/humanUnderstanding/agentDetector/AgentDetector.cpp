
#include <vector>
#include "AgentDetector.h"

clickType AgentDetector::clicked=idle;
float AgentDetector::clickX=0.0F;
float AgentDetector::clickY=0.0F;


bool AgentDetector::showImageParser(string &mode, string &submode)
{
    submode=mode;
    size_t found=mode.find_first_of('+');
    if ((found>0) && (found!=string::npos))
    {
        submode=mode.substr(0,found);
        if (found+1<mode.length())
            mode=mode.substr(found+1,mode.length()-found);
        else
            mode.clear();
    }
    else
        mode.clear();    

    return !submode.empty();
}

bool AgentDetector::configure(ResourceFinder &rf)
{
    period = rf.check("period",Value(0.03)).asDouble();
    int verbosity=rf.check("verbosity",Value(0)).asInt();
    string name=rf.check("name",Value("agentDetector")).asString().c_str();
    useFaceRecognition = rf.check("useFaceRecognition");
    handleMultiplePlayers = rf.check("multiplePlayers");
    isMounted = !rf.check("isFixed");
    string show=rf.check("showImages",Value("false")).asString().c_str();
    showMode=rf.check("showMode",Value("rgb+depth+skeleton+players")).asString().c_str();
    dThresholdDisparition = rf.check("dThresholdDisparition",Value("3.0")).asDouble();    

    // initialise timing in case of misrecognition
    dTimingLastApparition = clock();

    //Open the OPC Client
    partner_default_name=rf.check("partner_default_name",Value("partner")).asString().c_str();

    string opcName=rf.check("opc",Value("OPC")).asString().c_str();
    opc = new OPCClient(name);
    dSince = 0.0;
    while (!opc->connect(opcName))
    {
        cout<<"Waiting connection to OPC..."<<endl;
        Time::delay(1.0);
    }
    opc->checkout();

    list<Entity*> entityList = opc->EntitiesCacheCopy();
    for(auto& e : entityList) {
        if(e->entity_type() == "agent" && e->name() != "icub") {
            partner_default_name = e->name();
        }
    }

    partner = opc->addOrRetrieveEntity<Agent>(partner_default_name);
    partner->m_present = 0.0;
    opc->commit(partner);

    //Retrieve the calibration matrix from RFH
    string rfhName=rf.check("rfh",Value("referenceFrameHandler")).asString().c_str();
    string rfhLocal = "/";
    rfhLocal+=name;
    rfhLocal+="/rfh:o";
    rfh.open(rfhLocal.c_str());

    string rfhRemote = "/";
    rfhRemote += rfhName ;
    rfhRemote += "/rpc";

    while (!Network::connect(rfhLocal.c_str(),rfhRemote.c_str()))
    {
        cout<<"Waiting connection to RFH..."<<endl;
        Time::delay(1.0);
    }

    isCalibrated=false;
    if(!checkCalibration())
        yWarning() << " ========================= KINECT NEED TO BE CALIBRATED ============================" ;

    string clientName = name;
    clientName += "/kinect";

    //Prepare skeleton output port
    string skeletonName = "/";
    skeletonName += name;
    skeletonName += "/skeleton:o";
    outputSkeletonPort.open(skeletonName.c_str());

    depthPort.open( ("/"+clientName+"/depthPort:o").c_str());
    imagePort.open(("/"+clientName+"/imagePort:o").c_str());
    playersPort.open(("/"+clientName+"/playersPort:o").c_str());
    skeletonPort.open(("/"+clientName+"/skeletonPort:o").c_str());

    Property options;
    options.put("carrier","tcp");
    options.put("remote","kinectServer");
    options.put("local",clientName.c_str());
    options.put("verbosity",verbosity);

    if (!client.open(options))
        return false;

    Property opt;
    client.getInfo(opt);

    showImages=(show=="true")?true:false;
    int xPos = rf.check("x",Value(10)).asInt();
    int yPos = rf.check("y",Value(10)).asInt();

    int img_width=opt.find("img_width").asInt();
    int img_height=opt.find("img_height").asInt();
    int depth_width=opt.find("depth_width").asInt();
    int depth_height=opt.find("depth_height").asInt();

    rgb.resize(img_width, img_height);
    depth.resize(depth_width,depth_height);
    depthToDisplay.resize(depth_width,depth_height);
    playersImage.resize(depth_width,depth_height);
    skeletonImage.resize(depth_width,depth_height);

    depthTmp=cvCreateImage(cvSize(depth_width,depth_height),IPL_DEPTH_32F,1);
    rgbTmp=cvCreateImage(cvSize(img_width,img_height),IPL_DEPTH_8U,3);

    if (showImages)
    {
        vector<bool> alreadyOpen(4,false); 
        string mode=showMode;
        string submode;
        while (!mode.empty())
        {
            if (showImageParser(mode,submode))
            {            
                if (submode=="rgb")
                {
                    if (!alreadyOpen[0])
                    {
                        int x_rgb=rf.check("x-rgb",Value(xPos)).asInt(); 
                        int y_rgb=rf.check("y-rgb",Value(yPos)).asInt();
                        yInfo("\"rgb\" window selected in (%d,%d)",x_rgb,y_rgb);

                        cvNamedWindow("rgb",CV_WINDOW_AUTOSIZE);
                        cvMoveWindow("rgb",x_rgb,y_rgb);
                        alreadyOpen[0]=true;
                    }
                    else
                        yWarning("\"rgb\" window already open");
                }
                else if (submode=="depth")
                {
                    if (!alreadyOpen[1])
                    {
                        int x_depth=rf.check("x-depth",Value(xPos+300)).asInt();
                        int y_depth=rf.check("y-depth",Value(yPos)).asInt();
                        yInfo("\"depth\" window selected in (%d,%d)",x_depth,y_depth);

                        cvNamedWindow("depth",CV_WINDOW_AUTOSIZE);
                        cvMoveWindow("depth",x_depth,y_depth);
                        cvSetMouseCallback("depth",AgentDetector::click_callback,
                                           (void*)depthToDisplay.getIplImage());
                        alreadyOpen[1]=true;
                    }
                    else
                        yWarning("\"depth\" window already open");
                }
                else if (submode=="skeleton")
                {
                    if (!alreadyOpen[2])
                    {
                        int x_skeleton=rf.check("x-skeleton",Value(xPos)).asInt(); 
                        int y_skeleton=rf.check("y-skeleton",Value(yPos+300)).asInt();
                        yInfo("\"skeleton\" window selected in (%d,%d)",x_skeleton,y_skeleton);

                        cvNamedWindow("skeleton",CV_WINDOW_AUTOSIZE);
                        cvMoveWindow("skeleton",x_skeleton,y_skeleton);
                        alreadyOpen[2]=true;
                    }
                    else
                        yWarning("\"skeleton\" window already open");
                }
                else if (submode=="players")
                {
                    if (!alreadyOpen[3])
                    {
                        int x_players=rf.check("x-players",Value(xPos+300)).asInt();
                        int y_players=rf.check("y-players",Value(yPos+300)).asInt();
                        yInfo("\"players\" window selected in (%d,%d)",x_players,y_players);

                        cvNamedWindow("players",CV_WINDOW_AUTOSIZE);
                        cvMoveWindow("players",x_players,y_players);
                        alreadyOpen[3]=true;
                    }
                    else
                        yWarning("\"players\" window already open");
                }
                else
                    yError("unrecognized show mode!");
            }
        }
    }

    //Initialise Face Recognizer
    if (useFaceRecognition)
    {    
        string faceRecognName = "/";
        faceRecognName+=name;
        faceRecognName+="/faceRecognizer:rpc";
        faceRecognizerModule.open(faceRecognName.c_str());

        string faceRecognResName = "/";
        faceRecognResName+=name;
        faceRecognResName+="/faceRecognizer/results:i";
        faceRecognizerModuleResults.open(faceRecognResName.c_str());

        // WARNING: Do not use getContextPath if that ever should be used again!
        //recognizer->loadTrainingSet(rf.getContextPath().c_str());
        //recognizer->Train();
        //cout<<"Loading recognizer: "
        //    <<recognizer->loadRecognizer("defaultFaces.tpc")<<endl;
    }
    else
    {
        //Load the skeletonPatterns
    }

    string rpcName = "/";
    rpcName+=name;
    rpcName+="/rpc";
    rpc.open(rpcName.c_str());
    attach(rpc);

    pointsCnt=0;
    return true;
}

bool AgentDetector::checkCalibration()
{
    if (isCalibrated)
        return true;

    if (rfh.getOutputCount()>0)
    {
        Bottle bCmd,reply;
        bCmd.addString("mat");
        bCmd.addString("kinect");
        bCmd.addString("icub");
        rfh.write(bCmd,reply);

        if (reply.get(0)!="nack")
        {
            if (Bottle *bMat=reply.get(1).asList())
            {
                kinect2icub.resize(4,4);
                for (int i=0; i<4; i++)
                    for (int j=0; j<4; j++)
                        kinect2icub(i,j)=bMat->get(4*i+j).asDouble();
                return isCalibrated=true;
            }
        }
    }

    return false;
}

bool AgentDetector::close()
{
    depthPort.interrupt();
    depthPort.close();
    imagePort.interrupt();
    imagePort.close();
    playersPort.interrupt();
    playersPort.close();
    skeletonPort.interrupt();
    skeletonPort.close();
    outputSkeletonPort.interrupt();
    outputSkeletonPort.close();
    client.close();
    cvReleaseImage(&depthTmp);
    cvReleaseImage(&rgbTmp);
    opc->close();
    rfh.close();
    delete opc;

    return true;
}

bool AgentDetector::respond(const Bottle& cmd, Bottle& reply)
{
    if (cmd.get(0).asString() == "train" ) {
        reply.addString("ack");
        cout<<"Received a training order"<<endl;
        currentTrainingFace = cmd.get(1).asString();
    }
    else if (cmd.get(0).asString() == "change_partner_name" ) {
        if(cmd.get(1).isString()) {
            reply.addString("ack");
            partner_default_name=cmd.get(1).asString();
            identities[joint.ID] = partner_default_name;
        } else {
            reply.addString("nack");
        }
    }
    else if (cmd.get(0).asString() == "pause" ) {
        m.lock();
        reply.addString("ack");
    }
    else if (cmd.get(0).asString() == "resume" ) {
        m.unlock();
        reply.addString("ack");
    }
    else
    {
        reply.addString("nack");
    }
    return true;
}

double AgentDetector::getPeriod()
{
    return period;
}

bool AgentDetector::updateModule()
{
    LockGuard lg(m);

    bool isRefreshed = client.getDepthAndPlayers(depth,players);
    client.getRgb(rgb);

    bool tracked;

    if (handleMultiplePlayers)
        tracked=client.getJoints(joints);
    else
        tracked=client.getJoints(joint, EFAA_KINECT_CLOSEST_PLAYER);
    //cout<<"Tracking value = "<<tracked<<endl;

    if (tracked)
    {
        if (handleMultiplePlayers)
            client.getSkeletonImage(joints,skeletonImage);
        else
        {
            client.getSkeletonImage(joint,skeletonImage);
            joints.clear();
            joints.push_back(joint);
        }
    } 

    client.getPlayersImage(players,playersImage);
    client.getDepthImage(depth,depthToDisplay);

    if (depthPort.getOutputCount()>0)
    {
        depthPort.prepare()=depthToDisplay;
        depthPort.write();
    }

    if (imagePort.getOutputCount()>0)
    {
        imagePort.prepare()=rgb;
        imagePort.write();
    }

    if (playersPort.getOutputCount()>0)
    {
        playersPort.prepare()=playersImage;
        playersPort.write();
    }

    if (skeletonPort.getOutputCount()>0)
    {
        skeletonPort.prepare()=skeletonImage;
        skeletonPort.write();
    }

    if (showImages)
    {        
        cvConvertScale((IplImage*)depthToDisplay.getIplImage(),depthTmp,1.0/255);
        cvCvtColor((IplImage*)rgb.getIplImage(),rgbTmp,CV_BGR2RGB);

        string mode=showMode;
        string submode;
        while (!mode.empty())
        {
            if (showImageParser(mode,submode))
            {            
                if (submode=="rgb")
                    cvShowImage("rgb",rgbTmp);
                else if (submode=="depth")
                    cvShowImage("depth",depthTmp);
                else if (submode=="skeleton")
                    cvShowImage("skeleton",(IplImage*)skeletonImage.getIplImage());
                else if (submode=="players")
                    cvShowImage("players",(IplImage*)playersImage.getIplImage());
                else
                    yError("unrecognized show mode!");
            }
        }

        cvWaitKey(1);
    }

    //Send the players information to the OPC
    //Allow click calibration
    if (!checkCalibration())
    {
        if (AgentDetector::clicked==clicked_left)
        {
            AgentDetector::clicked=idle;

            //Get the clicked point coordinate in Kinect space
            Vector clickedPoint(3);
            cout<<"Processing a click on ("<<AgentDetector::clickX<<" "<<AgentDetector::clickY<<") --> ";
            client.get3DPoint((int)AgentDetector::clickX,(int)AgentDetector::clickY,clickedPoint);
            cout<<clickedPoint.toString(3,3)<<endl;

            Bottle bCond;
            Bottle bObject;
            Bottle bRTObject;

            bObject.addString(EFAA_OPC_ENTITY_TAG);
            bObject.addString("==");
            bObject.addString(EFAA_OPC_ENTITY_OBJECT);

            bRTObject.addString(EFAA_OPC_ENTITY_TAG);
            bRTObject.addString("==");
            bRTObject.addString(EFAA_OPC_ENTITY_RTOBJECT);

            Bottle bPresent;
            bPresent.addString(EFAA_OPC_OBJECT_PRESENT_TAG);
            bPresent.addString("==");
            bPresent.addDouble(1.0);

            bCond.addList()=bObject;
            bCond.addString("&&");
            bCond.addList()=bPresent;
            bCond.addString("||");
            bCond.addList()=bRTObject;
            bCond.addString("&&");
            bCond.addList()=bPresent;
            opc->checkout();
            opc->isVerbose=true;
            list<Entity*> presentObjects=opc->Entities(bCond);
            opc->isVerbose=false;
            
            if (presentObjects.size()==1)
            {
                Object* o=(Object*)(presentObjects.front());
                Bottle botRPH, botRPHRep;
                botRPH.addString("add");
                botRPH.addString("kinect");
                Bottle &cooKinect=botRPH.addList();
                cooKinect.addDouble(clickedPoint[0]);
                cooKinect.addDouble(clickedPoint[1]);
                cooKinect.addDouble(clickedPoint[2]);

                Bottle &cooiCub=botRPH.addList();
                cooiCub.addDouble(o->m_ego_position[0]);
                cooiCub.addDouble(o->m_ego_position[1]);
                cooiCub.addDouble(o->m_ego_position[2]);
                rfh.write(botRPH,botRPHRep);
                cout<<"Sent to RFH: "<<botRPH.toString().c_str()<<endl;
                cout<<"Got from RFH: "<<botRPHRep.toString().c_str()<<endl;

                pointsCnt++;
            }
            else
                yWarning("There should be 1 and only 1 object on the table");
        }
        else if (AgentDetector::clicked==clicked_right)
        {
            AgentDetector::clicked=idle;
            if (pointsCnt>=3)
            {
                Bottle calibBottle,calibReply; 
                calibBottle.addString("cal");
                calibBottle.addString("kinect");
                rfh.write(calibBottle,calibReply);
                cout<<"Calibrated ! "<<calibReply.toString().c_str()<<endl;

                calibBottle.clear();
                calibBottle.addString("save");
                rfh.write(calibBottle,calibReply);
                cout<<"Saved to file ! "<<calibReply.toString().c_str()<<endl;
                checkCalibration();
            }
            else
                yWarning("Unable to calibrate with less than 3 points pairs collected");
        }
    }

    if (isRefreshed)
    {
//        yInfo() << " refreshed";
        //////////////////////////////////////////////////////////////////
        //Clear the previous agents
        //for(map<int, Agent*>::iterator pA=identities.begin(); pA!=identities.end() ; pA++)
        //{
        //    pA->second->m_present = 0.0;
        //}  
        //partner->m_present = 0.0;

        // check if last apparition was more than dThreshlodDisaparition ago


        if (tracked)
        {
            //Go through all skeletons
            for(deque<Player>::iterator p=joints.begin(); p!=joints.end(); p++)
            {
                //check if this skeletton is really tracked
                bool reallyTracked = false;
                for(map<string,Joint>::iterator jnt = p->skeleton.begin() ; jnt != p->skeleton.end() ; jnt++)
                {
                    if (jnt->second.x != 0 && jnt->second.y != 0 && jnt->second.z != 0)
                    {
                        reallyTracked = true; break;
                    }
                }
                if (reallyTracked)
                {
                    dSince = (clock() - dTimingLastApparition) / (double) CLOCKS_PER_SEC;
                    //yInfo() << " is REALLY tracked";
                    string playerName = partner_default_name;

                    //If the skeleton is tracked we dont identify
                    if (identities.find(p->ID) != identities.end())
                    {
                        playerName = identities[p->ID];
                    }
                    else
                    {   
                        //Check if we should learn this face
                        if (currentTrainingFace != "")
                        {
                            setIdentity(*p,currentTrainingFace);
                            currentTrainingFace = "";
                        }

                        //if (useFaceRecognition)
                        playerName = getIdentity(*p);
                    }

                    //We interact with OPC only if the calibration is done
                    if (isCalibrated)
                    {
                        //Retrieve this player in OPC or create if does not exist
                        opc->checkout();
                        partner = opc->addOrRetrieveEntity<Agent>(partner_default_name);
                        partner->m_present = 1.0;

                        // reset the timing.
                        dTimingLastApparition = clock();
                        
                        if (identities.find(p->ID) == identities.end())
                        {
                            cout<<"Assigning name "<<playerName<<" to skeleton "<<p->ID<<endl;

                            //Agent* specificAgent = opc->addEntity<Agent>(playerName);
                            Agent* specificAgent = opc->addOrRetrieveEntity<Agent>(playerName);
                            if(specificAgent == nullptr) {
                                yError() << "SHIT specificAgent";
                            } else {
                                identities[p->ID] = specificAgent->name();
                                specificAgent->m_present = 1.0;
                                yInfo() << " specific agent is commited";
                                opc->commit(specificAgent);
                                yInfo() << " specific agent is commited done";
                            }
                        }

//                        Relation r(partner->name(),"named",playerName);
//                        opc->addRelation(r,1.0);

//                        cout<<"Commiting : "<<r.toString()<<endl;
                        yarp::os::Bottle &skeleton = outputSkeletonPort.prepare();
                        skeleton.clear();
                        //Convert the skeleton into efaaHelpers body. We loose orientation in the process...
                        for(map<string,Joint>::iterator jnt = p->skeleton.begin() ; jnt != p->skeleton.end() ; jnt++)
                        {
                            Vector kPosition(4);
                            kPosition[0] = jnt->second.x;
                            kPosition[1] = jnt->second.y;
                            kPosition[2] = jnt->second.z;
                            kPosition[3] = 1;
                            Vector icubPos = kinect2icub * kPosition;
                            Vector irPos = icubPos.subVector(0,2);

                            if (isMounted)
                            {
                                irPos = transform2IR(irPos);
                                Bottle jntBtl;
                                jntBtl.clear();
                                jntBtl.addString(jnt->first);
                                jntBtl.addDouble(jnt->second.x);
                                jntBtl.addDouble(jnt->second.y);
                                jntBtl.addDouble(jnt->second.z);
                                skeleton.addList() = jntBtl;
                            }

                            if (jnt->first == EFAA_OPC_BODY_PART_TYPE_HEAD)
                            {
                                partner->m_ego_position = irPos;
                            }
                            partner->m_body.m_parts[jnt->first] = irPos;
                        }
                        opc->commit(partner);
//                        cout << skeleton.toString()<< endl;
                        outputSkeletonPort.write();
                        //opc->commit(agent);
                    }
//                    cout<<'1'<<endl;
                }
            }
        }
        else
        {
            if (dSince > dThresholdDisparition)
            {
                opc->checkout();
                partner = opc->addOrRetrieveEntity<Agent>(partner_default_name);
                partner->m_present = 0.0;
                opc->commit(partner);
            }
            else
            {
                //yInfo() << " clock is: " << clock() << "\t last apparition: " << dTimingLastApparition  << "\t dSince: " << dSince;
                //yInfo() << " agent dissapeared but not for too long.";
            }
        }
    }
    return true;
}

void AgentDetector::setIdentity(Player p, string name)
{
    if (useFaceRecognition)
    {
        Bottle bFeed;
        bFeed.clear();
        bFeed.addString("acquire");
        bFeed.addString(name.c_str());
        faceRecognizerModule.write(bFeed);
    }

    this->skeletonPatterns[name.c_str()] = getSkeletonPattern(p);
}

double distanceVector(Joint j1, Joint j2)
{
    return sqrt(pow((j1.x - j2.x),2.0) + pow((j1.y - j2.y),2.0) + pow((j1.z - j2.z),2.0));
}

Vector AgentDetector::getSkeletonPattern(Player p)
{
    //Create the skeleton pattern
    Vector pattern(5);

    pattern[0] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_ELBOW_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_L]);
    pattern[1] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_C]);
    pattern[2] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_C], p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_R]);
    pattern[3] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_C], p.skeleton[EFAA_OPC_BODY_PART_TYPE_SPINE]);
    pattern[4] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_C], p.skeleton[EFAA_OPC_BODY_PART_TYPE_HEAD]);

    //Arms
    //pattern[0] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_HAND_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_WRIST_L]);
    //pattern[1] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_WRIST_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_ELBOW_L]);
    //pattern[2] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_ELBOW_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_L]);
    //pattern[3] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_C]);
    //pattern[4] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_C], p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_R]);
    //pattern[5] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_SHOULDER_R], p.skeleton[EFAA_OPC_BODY_PART_TYPE_ELBOW_R]);
    //pattern[6] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_ELBOW_R], p.skeleton[EFAA_OPC_BODY_PART_TYPE_WRIST_R]);
    //pattern[7] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_WRIST_R], p.skeleton[EFAA_OPC_BODY_PART_TYPE_HAND_R]);

    //Legs
    //pattern[8] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_FOOT_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_ANKLE_L]);
    //pattern[9] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_ANKLE_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_KNEE_L]);
    //pattern[10] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_KNEE_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_HIP_L]);
    //pattern[11] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_HIP_L], p.skeleton[EFAA_OPC_BODY_PART_TYPE_HIP_C]);
    //pattern[12] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_HIP_C], p.skeleton[EFAA_OPC_BODY_PART_TYPE_HIP_R]);
    //pattern[13] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_HIP_R], p.skeleton[EFAA_OPC_BODY_PART_TYPE_KNEE_R]);
    //pattern[14] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_KNEE_R], p.skeleton[EFAA_OPC_BODY_PART_TYPE_ANKLE_R]);
    //pattern[15] = distanceVector(p.skeleton[EFAA_OPC_BODY_PART_TYPE_ANKLE_R], p.skeleton[EFAA_OPC_BODY_PART_TYPE_FOOT_R]); 

    return pattern;
}


string AgentDetector::getIdentity(Player p)
{
    //If this tracked ID has already been identified we don't try again
    if (identities.find(p.ID) != identities.end() )
    {
        return identities[p.ID];
    }
    if (useFaceRecognition)
    {
        Bottle* results = faceRecognizerModuleResults.read(false);
        if (results)
        {
            for(unsigned int i = 0; results->size() ; i++)
            {
                Bottle* bFace = results->get(i).asList();
                string name = bFace->get(0).asString().c_str();
                //todo check if the face location is the one of the actual guy
                return name;
            }
        }
    }
    else
    {
        if (skeletonPatterns.size()>0)
        {
            double bestVal = DBL_MAX;
            string bestName = partner_default_name;

            Vector currentPattern = getSkeletonPattern(p);
            for(map<string, Vector>::iterator patternIt = skeletonPatterns.begin(); patternIt != skeletonPatterns.end();patternIt++)
            {
                double distance = 0;
                for(size_t i=0; i<currentPattern.size();i++)
                {
                    distance += pow( patternIt->second[i] - currentPattern[i], 2.0);
                }
                distance = sqrt(distance);

                if (distance < bestVal)
                {
                    bestVal = distance;
                    bestName = patternIt->first;
                }
            }
            return bestName;
        }
        else
        {
            return partner_default_name;
        }

    }
    return partner_default_name;
}

Vector AgentDetector::transform2IR(Vector v)
{
    Agent* icub = opc->addOrRetrieveEntity<Agent>("icub");
    Vector Xs = icub->m_ego_position;
    double phi = icub->m_ego_orientation[2] * M_PI / 180.0;
    //cout<<"Robot position = "<<Xs.toString(3,3)<< " Orientation = "<<phi<<endl;
    // cout<<"Kinect position = "<<v.toString(3,3)<<endl;

    Matrix H(4,4);
    H(0,0) = cos(phi);  H(0,1) = -sin(phi);    H(0,2) = 0.0; H(0,3) = Xs[0];
    H(1,0) = sin(phi);  H(1,1) = cos(phi);     H(1,2) = 0.0; H(1,3) = Xs[1];
    H(2,0) = 0.0;       H(2,1) = 0.0;          H(2,2) = 1.0; H(2,3) = 0.0;
    H(3,0) = 0.0;       H(3,1) = 0.0;          H(3,2) = 0.0; H(3,3) = 1.0;

    Vector vIR = v;
    vIR.push_back(1.0);
    vIR = H * vIR;
    //cout<<"Kinect in IR = "<<vIR.toString(3,3)<<endl;
    return vIR;
}
